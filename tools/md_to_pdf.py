#!/usr/bin/env python3
"""Markdown → PDF converter using markdown + weasyprint"""
import sys, os
import markdown
from weasyprint import HTML

CSS = """
body {
    font-family: 'DejaVu Sans', 'Noto Sans CJK SC', sans-serif;
    font-size: 11pt;
    line-height: 1.6;
    max-width: 900px;
    margin: 40px auto;
    color: #222;
}
h1 { font-size: 20pt; border-bottom: 2px solid #333; padding-bottom: 6px; margin-top: 28px; }
h2 { font-size: 15pt; border-bottom: 1px solid #999; padding-bottom: 3px; margin-top: 24px; }
h3 { font-size: 12pt; margin-top: 18px; }
code {
    background: #f0f0f0; padding: 1px 4px; border-radius: 3px;
    font-family: 'DejaVu Sans Mono', monospace; font-size: 9.5pt;
}
pre {
    background: #f4f4f4; padding: 12px; border-radius: 4px;
    overflow-x: auto; border: 1px solid #ddd;
}
pre code { background: none; padding: 0; }
table { border-collapse: collapse; margin: 12px 0; width: 100%; }
th, td { border: 1px solid #ccc; padding: 6px 10px; text-align: left; }
th { background: #e8e8e8; }
blockquote { border-left: 4px solid #999; margin: 10px 0; padding: 4px 16px; color: #555; }
"""

def md_to_pdf(md_path, pdf_path=None):
    if pdf_path is None:
        pdf_path = os.path.splitext(md_path)[0] + ".pdf"

    with open(md_path, "r") as f:
        md_text = f.read()

    html_body = markdown.markdown(md_text, extensions=["tables", "fenced_code", "codehilite", "toc"])
    html = f"""<!DOCTYPE html><html><head><meta charset="utf-8"><style>{CSS}</style></head><body>{html_body}</body></html>"""

    HTML(string=html).write_pdf(pdf_path)
    print(f"  ✓ {md_path} → {pdf_path}")
    return pdf_path


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python md_to_pdf.py <file1.md> [file2.md ...]")
        sys.exit(1)

    for md_file in sys.argv[1:]:
        if not os.path.exists(md_file):
            print(f"  ✗ not found: {md_file}")
            continue
        try:
            md_to_pdf(md_file)
        except Exception as e:
            print(f"  ✗ {md_file}: {e}")
