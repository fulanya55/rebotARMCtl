"""A small S-expression reader and writer, enough for KiCad's file formats."""

from __future__ import annotations

BACKSLASH = chr(92)


class Sym(str):
    """An unquoted token. A plain str is written back quoted."""


def parse(text: str):
    i = 0
    n = len(text)
    stack: list[list] = []
    root = None
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c == "(":
            node: list = []
            if stack:
                stack[-1].append(node)
            stack.append(node)
            i += 1
        elif c == ")":
            root = stack.pop()
            i += 1
        elif c == '"':
            j = i + 1
            out = []
            while text[j] != '"':
                if text[j] == BACKSLASH:
                    j += 1
                    out.append({"n": chr(10), "t": chr(9), "r": chr(13)}.get(text[j], text[j]))
                else:
                    out.append(text[j])
                j += 1
            stack[-1].append("".join(out))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            stack[-1].append(Sym(text[i:j]))
            i = j
    return root


def quote(value: str) -> str:
    escaped = value.replace(BACKSLASH, BACKSLASH * 2).replace('"', BACKSLASH + '"').replace(chr(10), BACKSLASH + "n")
    return f'"{escaped}"'


def dump(node, indent: int = 0) -> str:
    if isinstance(node, Sym):
        return str(node)
    if isinstance(node, str):
        return quote(node)
    if isinstance(node, bool):
        return "yes" if node else "no"
    if isinstance(node, (int, float)):
        return f"{node:g}" if isinstance(node, float) else str(node)
    pad = chr(9) * indent
    simple = all(not isinstance(item, list) for item in node)
    if simple:
        return "(" + " ".join(dump(item) for item in node) + ")"
    parts = []
    inline = []
    for item in node:
        if isinstance(item, list):
            if inline:
                parts.append(" ".join(dump(x) for x in inline))
                inline = []
            parts.append(chr(10) + pad + chr(9) + dump(item, indent + 1))
        else:
            inline.append(item)
    if inline:
        parts.append(" ".join(dump(x) for x in inline))
    return "(" + "".join(parts) + chr(10) + pad + ")"


def find(node: list, name: str):
    for item in node:
        if isinstance(item, list) and item and item[0] == name:
            return item
    return None


def find_all(node: list, name: str) -> list:
    return [item for item in node if isinstance(item, list) and item and item[0] == name]
