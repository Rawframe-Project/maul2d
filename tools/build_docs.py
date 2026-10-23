#!/usr/bin/env python3
# Builds the documentation site: README.md as the home page,
# docs/README.md, every page of docs/user/, the API reference that
# tools/gen_api.py generates from the headers, and CHANGELOG.md, as
# _site/*.html inside one shared page shell. The Markdown stays plain for
# the repository view; this script owns the site view. A link between
# two published pages points at the other page; a link to anything else
# points at the repository.
# The same script serves every Maul library; it reads the library's name
# and title from CMakeLists.txt. A library with more generated reference
# pages adds tools/site_pages.py, whose pages() returns (output name,
# navigation title or None, Markdown) for each; they go after the API.
# Requires: pip install markdown.
#
# Usage: python3 tools/build_docs.py  (from the repository root)

import pathlib
import re
import sys

import markdown

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "_site"

sys.path.insert(0, str(ROOT / "tools"))
import gen_api  # noqa: E402


def cmake_setting(name):
    """A value the library's CMakeLists.txt sets with set(NAME value)."""
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"^set\(" + name + r" \"?([^\")]+)\"?\)", text, re.M)
    if not match:
        raise SystemExit("CMakeLists.txt does not set " + name)
    return match.group(1)


LIB = cmake_setting("MAUL_LIBRARY")
TITLE = cmake_setting("MAUL_TITLE")
REPOSITORY = "https://github.com/Rawframe-Project/" + LIB


def pages():
    """(output name, navigation title or None, source path, Markdown) of each page."""
    result = [("index", TITLE, ROOT / "README.md"),
              ("docs", "Documentation", ROOT / "docs" / "README.md")]
    for source in sorted((ROOT / "docs" / "user").glob("*.md")):
        result.append((source.stem, None, source))
    result.append(("changelog", "Changelog", ROOT / "CHANGELOG.md"))
    found = [(name, title, source, source.read_text(encoding="utf-8"))
             for name, title, source in result if source.exists()]
    generated = [("api", "API", ROOT / "api.md", gen_api.render())]
    if (ROOT / "tools" / "site_pages.py").exists():
        import site_pages
        generated += [(name, title, ROOT / (name + ".md"), text)
                      for name, title, text in site_pages.pages()]
    found[len(found) - 1:len(found) - 1] = generated
    return found


LINK = re.compile(r'href="([^":#]+)(#[^"]*)?"')

SHELL = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title} | {site}</title>
<style>
:root {{ color-scheme: light dark; }}
body {{ max-width: 46rem; margin: 0 auto; padding: 1rem 1.25rem 4rem;
       font: 16px/1.6 system-ui, sans-serif; }}
nav {{ padding: 0.75rem 0; border-bottom: 1px solid #8884; margin-bottom: 1.5rem; }}
nav a {{ margin-right: 1.25rem; text-decoration: none; font-weight: 600; }}
pre {{ overflow-x: auto; padding: 0.75rem; border: 1px solid #8884; border-radius: 6px; }}
code {{ font-size: 0.92em; }}
table {{ border-collapse: collapse; }}
td, th {{ border: 1px solid #8886; padding: 0.3rem 0.6rem; }}
h1, h2 {{ line-height: 1.25; }}
</style>
</head>
<body>
<nav>{nav}<a href="https://github.com/Rawframe-Project/{lib}">GitHub</a></nav>
{body}
</body>
</html>
"""


def main():
    OUT.mkdir(exist_ok=True)
    found = pages()
    names = {source.resolve(): name for name, _, source, _ in found}
    nav = "".join('<a href="%s.html">%s</a>' % (name, title) for name, title, _, _ in found if title)
    md = markdown.Markdown(extensions=["fenced_code", "tables", "toc"])

    for name, title, source, text in found:
        def relink(match):
            target = (source.parent / match.group(1)).resolve()
            anchor = match.group(2) or ""
            if target in names:
                return 'href="%s.html%s"' % (names[target], anchor)
            return 'href="%s/blob/main/%s%s"' % (REPOSITORY, target.relative_to(ROOT), anchor)

        body = LINK.sub(relink, md.reset().convert(text))
        heading = re.search(r"^# (.+)$", text, re.M)
        page = SHELL.format(title=title or (heading.group(1) if heading else name), site=TITLE,
                            nav=nav, lib=LIB, body=body)
        (OUT / (name + ".html")).write_text(page, encoding="utf-8")
        print("built", name + ".html")


if __name__ == "__main__":
    main()
