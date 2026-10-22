#!/usr/bin/env python3
"""
Pack, minify & Gzip-compress main/ui_src/{style.css, body.html, app.js} (or index.html) into main/web_ui.h.
Supports bidirectional synchronization between the modular files (style.css, body.html, app.js) and index.html.
"""
import gzip
import os
import re

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_HTML = os.path.join(BASE_DIR, "index.html")
SRC_CSS = os.path.join(BASE_DIR, "style.css")
SRC_BODY = os.path.join(BASE_DIR, "body.html")
SRC_JS = os.path.join(BASE_DIR, "app.js")
OUT_H = os.path.abspath(os.path.join(BASE_DIR, "..", "web_ui.h"))

HTML_HEADER = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>ESP32-CAM Smart Meter &amp; Industrial AI Vision</title>
<style>
"""

def sync_modular_sources() -> str:
    has_parts = all(os.path.exists(p) for p in (SRC_CSS, SRC_BODY, SRC_JS))
    has_html = os.path.exists(SRC_HTML)

    if has_parts and has_html:
        html_mtime = os.path.getmtime(SRC_HTML)
        parts_mtime = max(os.path.getmtime(SRC_CSS), os.path.getmtime(SRC_BODY), os.path.getmtime(SRC_JS))
        if parts_mtime > html_mtime + 0.5:
            # Modular files were edited -> assemble into index.html
            css = open(SRC_CSS, "r", encoding="utf-8").read()
            body = open(SRC_BODY, "r", encoding="utf-8").read()
            js = open(SRC_JS, "r", encoding="utf-8").read()
            assembled = f"{HTML_HEADER}{css}</style>\n</head>\n<body>\n{body}<script>\n{js}</script>\n</body>\n</html>\n"
            with open(SRC_HTML, "w", encoding="utf-8", newline="\n") as f:
                f.write(assembled)
            return assembled
        else:
            # index.html is newer or equal -> sync modular files from index.html
            raw = open(SRC_HTML, "r", encoding="utf-8").read()
            try:
                s_start = raw.index("<style>") + len("<style>\n")
                s_end = raw.index("</style>")
                b_start = raw.index("<body>") + len("<body>\n")
                b_end = raw.index("<script>")
                j_start = raw.index("<script>") + len("<script>\n")
                j_end = raw.index("</script>")
                open(SRC_CSS, "w", encoding="utf-8", newline="\n").write(raw[s_start:s_end])
                open(SRC_BODY, "w", encoding="utf-8", newline="\n").write(raw[b_start:b_end])
                open(SRC_JS, "w", encoding="utf-8", newline="\n").write(raw[j_start:j_end])
            except ValueError:
                pass
            return raw

    with open(SRC_HTML, "r", encoding="utf-8") as f:
        return f.read()

def minify_html(text: str) -> str:
    text = re.sub(r"<!--[\s\S]*?-->", "", text)
    text = re.sub(r"/\*[\s\S]*?\*/", "", text)
    lines = []
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("//"):
            continue
        lines.append(s)
    return "\n".join(lines)

def main():
    raw_text = sync_modular_sources()
    raw_bytes = raw_text.encode("utf-8")
    min_text = minify_html(raw_text)
    min_bytes = min_text.encode("utf-8")

    gz_bytes = gzip.compress(min_bytes, compresslevel=9, mtime=0)
    raw_len = len(raw_bytes)
    min_len = len(min_bytes)
    gz_len = len(gz_bytes)
    ratio = (1.0 - (gz_len / raw_len)) * 100.0

    hex_lines = []
    for i in range(0, gz_len, 16):
        chunk = gz_bytes[i:i+16]
        hex_lines.append("  " + ", ".join(f"0x{b:02x}" for b in chunk))

    body_hex = ",\n".join(hex_lines)
    header_content = f"""#ifndef _WEB_UI_H_
#define _WEB_UI_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Auto-generated from main/ui_src/{{style.css, body.html, app.js}} by main/ui_src/pack_ui.py
 * Raw HTML size     : {raw_len} bytes
 * Minified size     : {min_len} bytes
 * Gzipped size      : {gz_len} bytes (Saved {raw_len - gz_len} bytes / -{ratio:.1f}%)
 */
static const size_t index_html_gz_len = {gz_len};
static const uint8_t index_html_gz[{gz_len}] = {{
{body_hex}
}};

#endif /* _WEB_UI_H_ */
"""
    with open(OUT_H, "w", encoding="utf-8") as f:
        f.write(header_content)

    print(f"[pack_ui] Raw: {raw_len} B -> Minified: {min_len} B -> Gzip: {gz_len} B (-{ratio:.1f}%). Wrote {OUT_H}")

if __name__ == "__main__":
    main()
