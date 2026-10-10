//! Tests of the download through the system `curl` (`build/prebuilt_curl.rs`) against a
//! canned HTTP server on 127.0.0.1.

use std::ffi::OsStr;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crate::prebuilt::Unavailable;
use crate::prebuilt_curl::*;

/// A server that answers by path and counts the connections it accepted.
struct Server {
    base: String,
    connections: Arc<AtomicUsize>,
}

impl Server {
    fn start() -> Server {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let base = format!("http://{}", listener.local_addr().expect("address"));
        let connections = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&connections);
        thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                counter.fetch_add(1, Ordering::SeqCst);
                thread::spawn(move || answer(stream));
            }
        });
        Server { base, connections }
    }

    fn url(&self, path: &str) -> String {
        format!("{}{path}", self.base)
    }

    fn connections(&self) -> usize {
        self.connections.load(Ordering::SeqCst)
    }
}

fn answer(mut stream: TcpStream) {
    let mut request = Vec::new();
    let mut byte = [0u8; 1];
    while !request.ends_with(b"\r\n\r\n") && stream.read(&mut byte).unwrap_or(0) == 1 {
        request.push(byte[0]);
    }
    let request = String::from_utf8_lossy(&request);
    let path = request.split_whitespace().nth(1).unwrap_or("/");
    let response: Vec<u8> = match path {
        "/ok" => ok(b"archive bytes"),
        "/redirect" => redirect("/ok"),
        "/loop" => redirect("/loop"),
        "/big" => ok(&[7u8; 2048]),
        "/big-unsized" => {
            let mut response = b"HTTP/1.1 200 OK\r\nConnection: close\r\n\r\n".to_vec();
            response.extend_from_slice(&[7u8; 2048]);
            response
        }
        "/silent" => {
            thread::sleep(Duration::from_secs(10));
            return;
        }
        _ => b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".to_vec(),
    };
    let _ = stream.write_all(&response);
}

fn ok(body: &[u8]) -> Vec<u8> {
    let mut response = format!(
        "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    )
    .into_bytes();
    response.extend_from_slice(body);
    response
}

fn redirect(to: &str) -> Vec<u8> {
    format!(
        "HTTP/1.1 302 Found\r\nLocation: {to}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
    )
    .into_bytes()
}

fn curl() -> &'static OsStr {
    OsStr::new("curl")
}

const SMALL: Limits = Limits {
    connect_s: 5,
    total_s: 20,
    max_bytes: 1024,
};

fn fetch(url: &str, limits: &Limits) -> Result<Vec<u8>, Unavailable> {
    download(curl(), url, false, limits)
}

fn assert_download_error(result: Result<Vec<u8>, Unavailable>, code: &str) {
    match result {
        Err(Unavailable::Download(message)) => {
            assert!(
                message.starts_with(&format!("curl exit {code}:")),
                "{message}"
            )
        }
        other => panic!("expected curl exit {code}, got {other:?}"),
    }
}

#[test]
fn a_body_is_returned() {
    let server = Server::start();
    assert_eq!(
        fetch(&server.url("/ok"), &SMALL),
        Ok(b"archive bytes".to_vec())
    );
}

#[test]
fn an_http_error_fails() {
    let server = Server::start();
    // 22: an HTTP status of 400 or more under `--fail`.
    assert_download_error(fetch(&server.url("/missing"), &SMALL), "22");
}

#[test]
fn a_redirect_is_followed_but_not_a_long_chain() {
    let server = Server::start();
    assert_eq!(
        fetch(&server.url("/redirect"), &SMALL),
        Ok(b"archive bytes".to_vec())
    );
    // 47: more than `--max-redirs` redirects.
    assert_download_error(fetch(&server.url("/loop"), &SMALL), "47");
}

#[test]
fn a_body_over_the_limit_is_too_large() {
    let server = Server::start();
    assert_eq!(
        fetch(&server.url("/big"), &SMALL),
        Err(Unavailable::TooLarge)
    );
    assert_eq!(
        fetch(&server.url("/big-unsized"), &SMALL),
        Err(Unavailable::TooLarge)
    );
}

#[test]
fn a_silent_server_times_out() {
    let server = Server::start();
    let limits = Limits {
        connect_s: 1,
        total_s: 2,
        max_bytes: 1024,
    };
    // 28: the operation timed out.
    assert_download_error(fetch(&server.url("/silent"), &limits), "28");
}

#[test]
fn https_only_refuses_http_without_a_request() {
    let server = Server::start();
    let result = download(curl(), &server.url("/ok"), true, &SMALL);
    assert!(
        matches!(result, Err(Unavailable::Download(_))),
        "{result:?}"
    );
    thread::sleep(Duration::from_millis(200));
    assert_eq!(server.connections(), 0);
}

#[test]
fn a_missing_program_is_no_curl() {
    let result = download(
        OsStr::new("oxijolt-no-curl"),
        "https://127.0.0.1:1/x",
        true,
        &SMALL,
    );
    assert_eq!(result, Err(Unavailable::NoCurl));
}

#[test]
fn the_override_url_is_checked() {
    let list = "https://github.com/o/r/releases/download/v1.2.0/";
    assert_eq!(
        base_url(list, None),
        Ok((
            "https://github.com/o/r/releases/download/v1.2.0".to_owned(),
            true
        ))
    );
    assert_eq!(base_url(list, Some("")).map(|(_, https)| https), Ok(true));
    assert_eq!(
        base_url(list, Some("http://127.0.0.1:8765/")),
        Ok(("http://127.0.0.1:8765".to_owned(), false))
    );
    for bad in [
        "http://user@host/x",
        "https://host/x?q",
        "https://host/x#f",
        "ftp://host/x",
        "https://host/a b",
        "127.0.0.1:8765",
    ] {
        assert_eq!(
            base_url(list, Some(bad)),
            Err(Unavailable::Download(
                "invalid JOLTC_PREBUILT_URL".to_owned()
            )),
            "{bad}"
        );
    }
    assert_eq!(
        archive_url("https://h/v1", "a-b"),
        "https://h/v1/a-b.tar.gz"
    );
}
