#!/usr/bin/env python3
"""
Merge IELTS speaking topic .docx files whose names start with 1-30.

This script uses docxcompose, so it appends each source .docx as a document
instead of rebuilding paragraphs by hand. That preserves formatting much better
for tables, images, styles, headers, and numbering.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from docx import Document
from docxcompose.composer import Composer


DEFAULT_SOURCE_DIR = Path("/media/cf/Data1/课程/课外课程/雅思/IELTS/口语topic")
DEFAULT_OUTPUT_NAME = "口语topic_1-30_合并.docx"
PREFIX_RE = re.compile(r"^\s*(\d+)\s*[-_ ]")


def collect_inputs(source_dir: Path, start: int, end: int, allow_missing: bool) -> list[Path]:
    numbered: dict[int, Path] = {}
    duplicates: dict[int, list[Path]] = {}

    for path in source_dir.iterdir():
        if not path.is_file():
            continue
        if path.name.startswith("~$") or path.suffix.lower() != ".docx":
            continue
        match = PREFIX_RE.match(path.name)
        if not match:
            continue
        number = int(match.group(1))
        if start <= number <= end:
            if number in numbered:
                duplicates.setdefault(number, [numbered[number]]).append(path)
            else:
                numbered[number] = path

    if duplicates:
        lines = ["发现重复编号，请先确认要保留哪一个："]
        for number, paths in sorted(duplicates.items()):
            lines.append(f"  {number}:")
            lines.extend(f"    - {p.name}" for p in paths)
        raise SystemExit("\n".join(lines))

    missing = [number for number in range(start, end + 1) if number not in numbered]
    if missing and not allow_missing:
        raise SystemExit(
            "缺少这些编号的 Word 文件："
            + ", ".join(map(str, missing))
            + "\n如果确认就是缺这些文件，可以加 --allow-missing 继续合并已有文件。"
        )

    return [numbered[number] for number in range(start, end + 1) if number in numbered]


def merge_docx(inputs: list[Path], output: Path) -> None:
    master = Document(str(inputs[0]))
    composer = Composer(master)

    for path in inputs[1:]:
        composer.doc.add_page_break()
        composer.append(Document(str(path)))

    output.parent.mkdir(parents=True, exist_ok=True)
    composer.save(str(output))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="按文件名前缀数字顺序合并 IELTS 口语 topic 的 Word 文档。"
    )
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCE_DIR)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=30)
    parser.add_argument("--allow-missing", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_dir = args.source_dir.expanduser()
    if not source_dir.is_dir():
        raise SystemExit(f"目录不存在：{source_dir}")

    output = args.output or source_dir / DEFAULT_OUTPUT_NAME
    output = output.expanduser()
    inputs = collect_inputs(source_dir, args.start, args.end, args.allow_missing)
    if not inputs:
        raise SystemExit("没有找到可合并的 .docx 文件")

    print("将按以下顺序合并：")
    for path in inputs:
        print(f"  - {path.name}")
    print(f"\n输出：{output}")

    merge_docx(inputs, output)
    print("完成")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
