#!/usr/bin/env python3
"""Turns an lcov file into a coverview data directory.

Writes line.info (DA records), branch.info (BRDA records, only if there are
any), sources.txt and config.json to --out-dir. coverview shows one coverage
type per info file, so lines and branches are split. Function records are
dropped. The SF: paths in the lcov file have to be relative to --repo-root.
"""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

LINE_TAGS = ("DA:", "LF:", "LH:")
BRANCH_TAGS = ("BRDA:", "BRF:", "BRH:")


def split_info(info: str) -> tuple[list[str], list[str], list[str]]:
    """Returns the source files, the line records and the branch records.

    Files without a single instrumented line (headers with declarations only)
    are left out.
    """
    files, line, branch = [], [], []
    record_line, record_branch = [], []
    for raw in info.splitlines():
        if raw.startswith("SF:"):
            record_line, record_branch = [raw], [raw]
        elif raw == "end_of_record":
            if not any(r.startswith("DA:") for r in record_line):
                continue
            files.append(record_line[0][3:])
            line += ["TN:"] + record_line + [raw]
            if any(r.startswith("BRDA:") for r in record_branch):
                branch += ["TN:"] + record_branch + [raw]
        elif raw.startswith(LINE_TAGS):
            record_line.append(raw)
        elif raw.startswith(BRANCH_TAGS):
            record_branch.append(raw)
    return list(dict.fromkeys(files)), line, branch


def write_sources(out_path: Path, repo_root: Path, rel_paths: list[str]):
    with out_path.open("w") as f:
        for rel_path in rel_paths:
            full_path = repo_root / rel_path
            if not full_path.is_file():
                continue
            f.write(f"### FILE: {rel_path}\n")
            f.write(full_path.read_text(errors="replace"))
            f.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--info", required=True, type=Path)
    parser.add_argument("--repo-root", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--repo", required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--commit", required=True)
    args = parser.parse_args()

    files, line, branch = split_info(args.info.read_text())
    args.out_dir.mkdir(parents=True, exist_ok=True)
    types = {}
    for name, records in (("line", line), ("branch", branch)):
        if records:
            (args.out_dir / f"{name}.info").write_text("\n".join(records) + "\n")
            types[name] = f"{name}.info"
    write_sources(args.out_dir / "sources.txt", args.repo_root, files)

    config = {
        "title": args.title,
        "repo": args.repo,
        "branch": args.branch,
        "commit": args.commit,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "datasets": {args.dataset: types},
    }
    (args.out_dir / "config.json").write_text(json.dumps(config, indent=2))


if __name__ == "__main__":
    main()
