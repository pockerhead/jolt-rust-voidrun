//! The command line: `comparison <command> [options]`.

use std::process::ExitCode;

/// What `--help` prints.
pub const USAGE: &str = "\
usage: comparison <command> [options]

commands:
  help    print this text
";

/// Runs the command in `args` (without the program name) and returns the process status.
pub fn main(args: &[String]) -> ExitCode {
    match args.first().map(String::as_str) {
        None | Some("help" | "--help" | "-h") => {
            print!("{USAGE}");
            ExitCode::SUCCESS
        }
        Some(other) => {
            eprintln!("unknown command {other:?}\n\n{USAGE}");
            ExitCode::from(2)
        }
    }
}
