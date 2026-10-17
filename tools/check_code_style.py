#!/usr/bin/env python3
"""Static checks for Mini Drone's explicit Allman style. Uses Python stdlib only.

Run from the PlatformIO project root: python3 tools/check_code_style.py
This is a style check, not a C++ parser or a replacement for building firmware.
"""
from __future__ import annotations

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
FILES = sorted(f for f in (ROOT / "src").rglob("*") if f.suffix in (".cpp", ".h"))


def mask_noncode(source: str) -> str:
    """Preserve code positions/newlines, blank comments, literals, preprocessor."""
    buf = list(source)
    i = 0
    n = len(source)
    at_line_start = True
    while i < n:
        c = source[i]
        if c == "\n":
            at_line_start = True
            i += 1
            continue
        if at_line_start and c in " \t\r":
            i += 1
            continue
        if at_line_start and c == "#":
            end = i
            while end < n:
                if source[end] == "\n" and (end == 0 or source[end - 1] != "\\"):
                    break
                end += 1
            for k in range(i, end):
                if buf[k] != "\n":
                    buf[k] = " "
            i = end
            at_line_start = False
            continue
        at_line_start = False
        if source.startswith("//", i):
            end = source.find("\n", i)
            if end < 0:
                end = n
            for k in range(i, end):
                buf[k] = " "
            i = end
            continue
        if source.startswith("/*", i):
            end = source.find("*/", i + 2)
            end = n if end < 0 else end + 2
            for k in range(i, end):
                if buf[k] != "\n":
                    buf[k] = " "
            i = end
            continue
        if c in "\"'":
            quote = c
            end = i + 1
            while end < n:
                if source[end] == "\\":
                    end += 2
                    continue
                if source[end] == quote:
                    end += 1
                    break
                end += 1
            for k in range(i, min(end, n)):
                if buf[k] != "\n":
                    buf[k] = " "
            i = end
            continue
        i += 1
    return "".join(buf)


def matching_paren(mask: str, start: int) -> int | None:
    level = 0
    for i in range(start, len(mask)):
        if mask[i] == "(":
            level += 1
        elif mask[i] == ")":
            level -= 1
            if level == 0:
                return i
    return None


def enclosed(expression: str) -> bool:
    cleaned = expression.strip()
    if not cleaned.startswith("("):
        return False
    return matching_paren(cleaned, 0) == len(cleaned) - 1


def first_nonspace(mask: str, pos: int) -> int:
    while pos < len(mask) and mask[pos].isspace():
        pos += 1
    return pos


def check_file(file: Path) -> list[str]:
    src = file.read_text(encoding="utf-8")
    mask = mask_noncode(src)
    problems: list[str] = []

    def problem(pos: int, msg: str) -> None:
        line_no = src.count("\n", 0, pos) + 1
        problems.append(f"{file.relative_to(ROOT)}:{line_no}: {msg}")

    for match in re.finditer(r"^[ \t]*(?:case\s+[^\n]+:|default\s*:)[ \t]*$", mask, re.M):
        next_pos = first_nonspace(mask, match.end())
        if next_pos >= len(mask) or mask[next_pos] != "{":
            problem(match.start(), "case/default must open its own { } block")

    for match in re.finditer(r"\b(if|else|while|for|switch|do)\b", mask):
        word = match.group(1)
        cursor = first_nonspace(mask, match.end())
        if word == "else" and re.match(r"if\b", mask[cursor:]):
            continue
        if word in ("if", "while", "for", "switch"):
            if cursor >= len(mask) or mask[cursor] != "(":
                continue
            end = matching_paren(mask, cursor)
            if end is None:
                problem(match.start(), "unbalanced control expression")
                continue
            cursor = first_nonspace(mask, end + 1)
            # while (condition); following a do { } while (...) is valid C++.
            if word == "while" and cursor < len(mask) and mask[cursor] == ";":
                continue
        if cursor >= len(mask) or mask[cursor] != "{":
            problem(match.start(), f"{word} body must use explicit {{ }}")

    for match in re.finditer(r"\b(?:const\s+)?bool\s+\w+\s*=\s*", mask):
        pos = match.end()
        depth = 0
        end = None
        for j in range(pos, len(mask)):
            if mask[j] == "(":
                depth += 1
            elif mask[j] == ")":
                depth -= 1
                if depth < 0:
                    break
            elif mask[j] == ";" and depth == 0:
                end = j
                break
        if end is None:
            continue
        expr = src[pos:end].strip()
        if ("&&" in mask[pos:end] or "||" in mask[pos:end]) and not enclosed(expr):
            problem(pos, "boolean initialization with &&/|| needs whole-expression parentheses")

    if "\t" in mask:
        problem(mask.index("\t"), "tab character in source code")
    if re.search(r"\{\n[ \t]*\n", src):
        match = re.search(r"\{\n[ \t]*\n", src)
        if match:
            problem(match.start(), "avoid an empty line immediately after opening brace")
    if "\n\n\n" in src:
        problem(src.index("\n\n\n"), "limit between-block spacing to one empty line")
    return problems


def main() -> int:
    issues = [msg for path in FILES for msg in check_file(path)]
    if issues:
        print("\n".join(issues))
        print(f"Style check failed: {len(issues)} issue(s) in {len(FILES)} files.")
        return 1
    print(f"Style check passed: {len(FILES)} C++ / header files; explicit switch blocks and control braces verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
