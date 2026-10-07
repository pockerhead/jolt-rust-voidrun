#!/usr/bin/env python3
"""Copies result tables from a results directory's summary.md into docs/comparison.md, between
`<!-- generated:NAME -->` and `<!-- /generated:NAME -->` markers, verbatim (the doc_tables test
checks that every Results row is in a committed summary.md).

Sections: timing-matched, timing-defaults, sweep, split, quality (the rows at --quality-threads).

usage: make_doc.py RESULTS [--doc docs/comparison.md] [--quality-threads 4]
"""
import argparse
import os
import re
import subprocess

SCENES = ["balls", "boxes", "capsules", "pyramid", "many_pyramids", "keva",
          "joint_ball", "joint_fixed", "joint_prismatic", "joint_revolute"]


def table_after(lines, start):
    rows = []
    for line in lines[start + 1:]:
        if line.startswith("|"):
            rows.append(line)
        elif rows or line.startswith("#"):
            break
    return rows


def section_bounds(lines, title):
    start = lines.index(f"## {title}")
    end = next((i for i in range(start + 1, len(lines)) if lines[i].startswith("## ")), len(lines))
    return start, end


def scene_tables(lines, title, profile):
    start, end = section_bounds(lines, title)
    out = []
    for scene in SCENES:
        heading = f"### {scene}, {profile}"
        if heading in lines[start:end]:
            rows = table_after(lines, lines.index(heading, start))
            out.append(f"**{scene}**\n\n" + "\n".join(rows))
    return "\n\n".join(out)


def main():
    root = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True,
                          check=True).stdout.strip()
    parser = argparse.ArgumentParser()
    parser.add_argument("results")
    parser.add_argument("--doc", default=os.path.join(root, "docs", "comparison.md"))
    parser.add_argument("--quality-threads", default="4")
    a = parser.parse_args()
    with open(os.path.join(a.results, "summary.md"), encoding="utf-8") as f:
        lines = f.read().replace("\r\n", "\n").split("\n")
    parts = {
        "timing-matched": scene_tables(lines, "Timing", "matched"),
        "timing-defaults": scene_tables(lines, "Timing", "defaults"),
        "sweep": scene_tables(lines, "Solver sweep", "matched"),
    }
    start, _ = section_bounds(lines, "Avian: physics schedule and the rest of the update")
    parts["split"] = "\n".join(table_after(lines, start))
    start, _ = section_bounds(lines, "Quality")
    quality = table_after(lines, start)
    keep = [r for r in quality[2:] if r.split("|")[4].strip() == a.quality_threads]
    parts["quality"] = "\n".join(quality[:2] + keep)
    with open(a.doc, encoding="utf-8") as f:
        doc = f.read().replace("\r\n", "\n")
    for name, body in parts.items():
        pattern = re.compile(rf"(<!-- generated:{name} -->\n).*?(\n<!-- /generated:{name} -->)", re.S)
        doc, count = pattern.subn(lambda m: m.group(1) + "\n" + body + "\n" + m.group(2), doc)
        if count != 1:
            raise SystemExit(f"marker generated:{name} not found once in {a.doc}")
    with open(a.doc, "w", encoding="utf-8", newline="\n") as f:
        f.write(doc)
    print("updated", a.doc)


if __name__ == "__main__":
    main()
