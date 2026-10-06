#!/usr/bin/env python3
"""Downloads the large test models listed in assets/models/models.tsv and checks every model.

Rows whose `where` is `download` are fetched from their pinned URL into --out (default
target/models) and kept only when their SHA-256 matches; a file already there with the right hash
is not downloaded again. Rows whose `where` is `commit` are checked in assets/models. Any mismatch
or failed download exits with status 1. Only the Python standard library is used.

    python scripts/fetch_models.py [--out DIR]
"""

import argparse
import hashlib
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TABLE = ROOT / "assets" / "models" / "models.tsv"
USER_AGENT = "oxijolt-dev (github.com/pockerhead/oxijolt)"
ATTEMPTS = 3


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def rows():
    lines = TABLE.read_text(encoding="utf-8").splitlines()
    header = lines[0].split("\t")
    for line in lines[1:]:
        if line.strip():
            yield dict(zip(header, line.split("\t")))


def download(url: str, target: Path) -> None:
    partial = target.with_name(target.name + ".part")
    for attempt in range(1, ATTEMPTS + 1):
        try:
            request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(request, timeout=120) as response, partial.open("wb") as out:
                while block := response.read(1 << 20):
                    out.write(block)
            partial.replace(target)
            return
        except OSError as error:
            print(f"  attempt {attempt} failed: {error}", file=sys.stderr)
            if attempt == ATTEMPTS:
                raise
            time.sleep(2 * attempt)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=ROOT / "target" / "models")
    out = parser.parse_args().out
    out.mkdir(parents=True, exist_ok=True)
    failures = 0
    for row in rows():
        name, expected = row["name"], row["sha256"]
        if row["where"] == "commit":
            path = ROOT / "assets" / "models" / row["file"]
        else:
            path = out / row["file"]
            if path.is_file() and sha256(path) == expected:
                print(f"{name}: present")
                continue
            print(f"{name}: downloading {row['url']}")
            try:
                download(row["url"], path)
            except OSError as error:
                print(f"{name}: download failed: {error}", file=sys.stderr)
                failures += 1
                continue
        actual = sha256(path) if path.is_file() else "missing"
        if actual == expected:
            print(f"{name}: ok")
        else:
            print(f"{name}: SHA-256 {actual}, expected {expected} ({path})", file=sys.stderr)
            failures += 1
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
