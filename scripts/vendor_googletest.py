"""Vendor a GoogleTest release into the repo (the only way third_party/googletest may change).

Downloads https://github.com/google/googletest/archive/refs/tags/<tag>.tar.gz (or reads a local
file given with --archive, which skips the download), prints the archive's SHA-256, extracts only
CMakeLists.txt, LICENSE, googletest/ and googlemock/ into --dest, and writes VENDORED.md there.
Refuses archive members with absolute paths or "..", and refuses a non-empty --dest without --force
(--force replaces the destination's contents).
Run: python scripts/vendor_googletest.py --tag v1.18.0
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import os
import re
import shutil
import sys
import tarfile
import tempfile
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
DEFAULT_DEST = os.path.join(REPO_ROOT, "third_party", "googletest")
URL_TEMPLATE = "https://github.com/google/googletest/archive/refs/tags/{tag}.tar.gz"
KEEP = ("CMakeLists.txt", "LICENSE", "googletest", "googlemock")
TAG_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class VendorError(Exception):
    pass


def archive_url(tag: str) -> str:
    if not TAG_PATTERN.match(tag):
        raise VendorError(f"invalid tag {tag!r}")
    return URL_TEMPLATE.format(tag=tag)


def sha256_of(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(url: str, target: str) -> None:
    try:
        with urllib.request.urlopen(url, timeout=60) as response, open(target, "wb") as out:
            shutil.copyfileobj(response, out)
    except OSError as e:
        raise VendorError(f"download failed: {url}: {e}") from e


def member_parts(name: str) -> list[str]:
    path = name.replace("\\", "/")
    if path.startswith("/") or re.match(r"^[A-Za-z]:", path):
        raise VendorError(f"unsafe archive member (absolute path): {name!r}")
    parts = [p for p in path.split("/") if p not in ("", ".")]
    if ".." in parts:
        raise VendorError(f"unsafe archive member ('..' in path): {name!r}")
    return parts


def select_members(tar: tarfile.TarFile) -> list[tuple[tarfile.TarInfo, list[str]]]:
    selected = []
    for member in tar.getmembers():
        parts = member_parts(member.name)
        if len(parts) < 2 or parts[1] not in KEEP:
            continue
        if not (member.isfile() or member.isdir()):
            raise VendorError(f"unsupported archive member (not a file or folder): {member.name!r}")
        selected.append((member, parts[1:]))
    return selected


def check_dest(dest: str, force: bool) -> None:
    if os.path.exists(dest) and not os.path.isdir(dest):
        raise VendorError(f"destination is not a folder: {dest}")
    if os.path.isdir(dest) and os.listdir(dest) and not force:
        raise VendorError(f"destination is not empty: {dest} (use --force to replace it)")


def extract(archive: str, dest: str) -> int:
    try:
        tar = tarfile.open(archive, "r:gz")
    except (tarfile.TarError, OSError) as e:
        raise VendorError(f"cannot read archive {archive}: {e}") from e
    with tar:
        try:
            selected = select_members(tar)
        except tarfile.TarError as e:
            raise VendorError(f"cannot read archive {archive}: {e}") from e
        present = {parts[0] for _, parts in selected}
        missing = [name for name in KEEP if name not in present]
        if missing:
            raise VendorError(f"archive is missing: {', '.join(missing)}")
        if os.path.isdir(dest):
            shutil.rmtree(dest)
        os.makedirs(dest)
        dest_real = os.path.realpath(dest)
        files = 0
        for member, parts in selected:
            target = os.path.realpath(os.path.join(dest, *parts))
            if os.path.commonpath([dest_real, target]) != dest_real:
                raise VendorError(f"unsafe archive member: {member.name!r}")
            if member.isdir():
                os.makedirs(target, exist_ok=True)
                continue
            os.makedirs(os.path.dirname(target), exist_ok=True)
            source = tar.extractfile(member)
            if source is None:
                raise VendorError(f"cannot read archive member: {member.name!r}")
            with source, open(target, "wb") as out:
                shutil.copyfileobj(source, out)
            files += 1
    return files


def write_record(dest: str, tag: str, url: str, sha256: str) -> None:
    today = datetime.datetime.now(datetime.timezone.utc).date().isoformat()
    text = (
        "# Vendored GoogleTest\n\n"
        f"- Tag: {tag}\n"
        f"- Archive: {url}\n"
        f"- SHA-256: {sha256}\n"
        f"- Date: {today}\n"
        "- Licence: BSD-3-Clause, see LICENSE\n\n"
        "Vendored by scripts/vendor_googletest.py; do not edit by hand.\n"
    )
    with open(os.path.join(dest, "VENDORED.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def run(tag: str, dest: str, archive: str | None, force: bool) -> None:
    url = archive_url(tag)
    check_dest(dest, force)
    with tempfile.TemporaryDirectory() as tmp:
        if archive is None:
            archive = os.path.join(tmp, "googletest.tar.gz")
            download(url, archive)
        elif not os.path.isfile(archive):
            raise VendorError(f"archive not found: {archive}")
        sha256 = sha256_of(archive)
        print(f"SHA-256: {sha256}")
        files = extract(archive, dest)
    write_record(dest, tag, url, sha256)
    print(f"Vendored {tag} into {dest} ({files} files).")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Vendor a GoogleTest release tag.")
    parser.add_argument("--tag", required=True, help="release tag, e.g. v1.18.0")
    parser.add_argument("--dest", default=DEFAULT_DEST, help="destination (default third_party/googletest)")
    parser.add_argument("--archive", help="use this local .tar.gz instead of downloading")
    parser.add_argument("--force", action="store_true", help="replace a non-empty destination")
    args = parser.parse_args(argv)
    try:
        run(args.tag, args.dest, args.archive, args.force)
    except VendorError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
