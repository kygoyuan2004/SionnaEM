#!/usr/bin/env python3
"""Build a presentation-friendly PDF from parameter_report.md.

The source report is table-heavy and useful as an internal reference. This
converter keeps the report structure while removing operational columns that
make the PDF too wide for reading in a meeting.
"""

from __future__ import annotations

import argparse
import html
import os
import re
from pathlib import Path

import markdown
from weasyprint import HTML


DROP_HEADER_PATTERNS = (
    "代码位置",
    "公式位置",
    "公式/代码位置",
    "典型取值",
    "默认值",
    "推荐值",
    "取值",
    "对 MDS 结果的影响",
    "对结果的影响",
    "对仿真结果的影响",
    "对性能的影响",
    "对训练的影响",
)


CSS = r"""
@page {
    size: A4 landscape;
    margin: 14mm 13mm 15mm 13mm;
    @bottom-center {
        content: "SionnaEM 参数说明 · " counter(page) " / " counter(pages);
        color: #6b7280;
        font-size: 8.5pt;
    }
}

* {
    box-sizing: border-box;
}

html {
    font-family: "Noto Sans CJK SC", "Noto Sans CJK", "Noto Sans", "DejaVu Sans", sans-serif;
    color: #18202b;
}

body {
    margin: 0;
    font-size: 9.3pt;
    line-height: 1.55;
    background: #fff;
}

h1 {
    margin: 0 0 8mm;
    padding: 0 0 4mm;
    border-bottom: 2.2pt solid #2f6f73;
    color: #102a43;
    font-size: 22pt;
    line-height: 1.18;
    font-weight: 800;
}

h1 + blockquote {
    margin: -4mm 0 8mm;
    padding: 3.5mm 5mm;
    border-left: 4pt solid #2f6f73;
    background: #eef7f4;
    color: #334155;
    border-radius: 4px;
}

h2 {
    margin: 9mm 0 4mm;
    padding: 2.2mm 3mm;
    background: #102a43;
    color: #fff;
    border-radius: 4px;
    font-size: 14.5pt;
    line-height: 1.25;
    font-weight: 760;
    break-after: avoid;
}

h3 {
    margin: 6mm 0 2.8mm;
    color: #1f4f53;
    font-size: 11.2pt;
    font-weight: 760;
    break-after: avoid;
}

h4 {
    margin: 5mm 0 2mm;
    color: #334155;
    font-size: 10pt;
    break-after: avoid;
}

p {
    margin: 2.5mm 0;
}

ul, ol {
    margin: 2.5mm 0 4mm 6mm;
    padding-left: 4mm;
}

li {
    margin: 1mm 0;
}

blockquote {
    margin: 4mm 0;
    padding: 2.5mm 4mm;
    border-left: 3pt solid #7aa9a3;
    background: #f6faf9;
    color: #475569;
}

code {
    padding: 0.45mm 1.1mm;
    border-radius: 3px;
    background: #eef2f7;
    color: #243b53;
    font-family: "Noto Sans Mono", "DejaVu Sans Mono", monospace;
    font-size: 8.2pt;
}

pre {
    margin: 3mm 0 5mm;
    padding: 3mm 4mm;
    border: 0.8pt solid #d7dee8;
    border-radius: 5px;
    background: #f7f9fb;
    color: #1f2937;
    white-space: pre-wrap;
    word-break: break-word;
}

pre code {
    padding: 0;
    background: transparent;
    font-size: 8.1pt;
}

table {
    width: 100%;
    margin: 2.2mm 0 6mm;
    border-collapse: separate;
    border-spacing: 0;
    table-layout: fixed;
    border: 0.8pt solid #c8d3df;
    border-radius: 5px;
    overflow: hidden;
    break-inside: avoid;
}

thead {
    display: table-header-group;
}

tr {
    break-inside: avoid;
}

th {
    padding: 2.3mm 2.4mm;
    background: #2f6f73;
    color: #fff;
    border-right: 0.6pt solid rgba(255, 255, 255, 0.35);
    font-size: 8.7pt;
    font-weight: 760;
    text-align: left;
    vertical-align: middle;
}

td {
    padding: 2.1mm 2.4mm;
    border-top: 0.6pt solid #d9e2ec;
    border-right: 0.6pt solid #e1e8f0;
    vertical-align: top;
    word-break: break-word;
    overflow-wrap: anywhere;
}

td:last-child,
th:last-child {
    border-right: 0;
}

tbody tr:nth-child(even) {
    background: #f8fbfc;
}

tbody tr:nth-child(odd) {
    background: #fff;
}

strong {
    color: #0f3f46;
    font-weight: 760;
}

a {
    color: #245d63;
    text-decoration: none;
}

hr {
    margin: 6mm 0;
    border: 0;
    border-top: 0.8pt solid #d7dee8;
}

.toc, .toc ul {
    margin-left: 0;
    padding-left: 0;
    list-style: none;
}

.toc li {
    display: inline-block;
    width: 48%;
    margin: 0 1% 1.5mm 0;
    vertical-align: top;
}
"""


def split_table_row(line: str) -> list[str]:
    stripped = line.strip()
    if stripped.startswith("|"):
        stripped = stripped[1:]
    if stripped.endswith("|"):
        stripped = stripped[:-1]
    cells: list[str] = []
    current: list[str] = []
    in_code = False
    escaped = False

    for char in stripped:
        if escaped:
            current.append(char)
            escaped = False
            continue
        if char == "\\":
            current.append(char)
            escaped = True
            continue
        if char == "`":
            in_code = not in_code
            current.append(char)
            continue
        if char == "|" and not in_code:
            cells.append("".join(current).strip())
            current = []
            continue
        current.append(char)

    cells.append("".join(current).strip())
    return cells


def make_table_row(cells: list[str]) -> str:
    return "| " + " | ".join(cells) + " |"


def is_table_separator(line: str) -> bool:
    cells = split_table_row(line)
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell.strip()) for cell in cells)


def should_drop_header(header: str) -> bool:
    normalized = re.sub(r"\s+", " ", header.replace("**", "").replace("`", "")).strip()
    return any(pattern in normalized for pattern in DROP_HEADER_PATTERNS)


def filter_markdown_tables(md_text: str) -> str:
    lines = md_text.splitlines()
    output: list[str] = []
    i = 0

    while i < len(lines):
        if (
            i + 1 < len(lines)
            and lines[i].lstrip().startswith("|")
            and lines[i + 1].lstrip().startswith("|")
            and is_table_separator(lines[i + 1])
        ):
            header = split_table_row(lines[i])
            separator = split_table_row(lines[i + 1])
            drop_indexes = {idx for idx, name in enumerate(header) if should_drop_header(name)}

            if drop_indexes and len(drop_indexes) < len(header):
                keep_indexes = [idx for idx in range(len(header)) if idx not in drop_indexes]
                output.append(make_table_row([header[idx] for idx in keep_indexes]))
                output.append(make_table_row([separator[idx] for idx in keep_indexes]))
                i += 2
                while i < len(lines) and lines[i].lstrip().startswith("|"):
                    row = split_table_row(lines[i])
                    padded = row + [""] * max(0, len(header) - len(row))
                    output.append(make_table_row([padded[idx] for idx in keep_indexes]))
                    i += 1
                continue

        output.append(lines[i])
        i += 1

    return "\n".join(output) + "\n"


def render_pdf(input_path: Path, cleaned_md_path: Path, pdf_path: Path) -> None:
    md_text = input_path.read_text(encoding="utf-8")
    cleaned = filter_markdown_tables(md_text)
    cleaned_md_path.write_text(cleaned, encoding="utf-8")

    body = markdown.markdown(
        cleaned,
        extensions=["tables", "fenced_code", "toc", "sane_lists"],
        output_format="html5",
    )
    title = html.escape(input_path.stem)
    full_html = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>{title}</title>
  <style>{CSS}</style>
</head>
<body>{body}</body>
</html>
"""
    HTML(string=full_html, base_url=str(input_path.parent)).write_pdf(str(pdf_path))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input",
        nargs="?",
        default="docs/parameter_report.md",
        help="Source Markdown file.",
    )
    parser.add_argument(
        "--clean-md",
        default="docs/parameter_report_tables.md",
        help="Filtered Markdown output path.",
    )
    parser.add_argument(
        "--pdf",
        default="docs/parameter_report_tables.pdf",
        help="PDF output path.",
    )
    args = parser.parse_args()

    input_path = Path(args.input).resolve()
    cleaned_md_path = Path(args.clean_md).resolve()
    pdf_path = Path(args.pdf).resolve()

    if not input_path.exists():
        raise FileNotFoundError(input_path)

    os.makedirs(cleaned_md_path.parent, exist_ok=True)
    os.makedirs(pdf_path.parent, exist_ok=True)
    render_pdf(input_path, cleaned_md_path, pdf_path)
    print(f"cleaned markdown: {cleaned_md_path}")
    print(f"pdf: {pdf_path}")


if __name__ == "__main__":
    main()
