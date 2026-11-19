#!/usr/bin/env python3
"""Check this repo's rammp-alternative.yaml fragments.

This repo ships one real/mock pair (ui_bridge_node has no internal
simulation mode -- see rammp-alternative.mock.yaml's own comment), so this
is closer to rammp-kitchen-capability-engines's per-capability checker than
to rammp-orchestrator's from-scratch one: validate both fragments against
validate_fragment.py's generic schema, then check:

- both fragments' 'id' are unique
- both fragments' container.image is this repo's own GHCR image
- the mock's publishes/subscribes match the real one exactly (the whole
  point of a mock is that nothing downstream can tell the difference)
- the Dockerfile's FROM pins a versioned rammp-base/rammp-cuda image
- a `tier: demo-ready` fragment publishes a status-ish topic and pins its
  image

Usage: check_fragments.py
"""

import re
import sys
from pathlib import Path

import yaml
from validate_fragment import validate

ROOT = Path(__file__).resolve().parent.parent
IMAGE_PREFIX = "ghcr.io/rammp-org/rammp-ui-bridge"
# A release (2.0.0-jp6) or a dated nightly (nightly-20261027), never a moving tag.
PINNED_BASE = re.compile(r"^ghcr\.io/rammp-org/rammp-(base|cuda):(\d+\.\d+\.\d+\S*|nightly-\d{8})$")
FLOATING_TAGS = ("dev", "main", "latest")


def fragment_paths():
    return sorted(ROOT.glob("rammp-alternative*.yaml"))


def check_dockerfile():
    path = ROOT / "Dockerfile"
    errors = []
    stages = set()
    found = False
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts or parts[0].upper() != "FROM":
            continue
        found = True
        args = [p for p in parts[1:] if not p.startswith("--")]
        image = args[0]
        if image not in stages and not PINNED_BASE.match(image):
            errors.append(
                f"Dockerfile: FROM {image}: build from a versioned ghcr.io/rammp-org/rammp-base "
                "or rammp-cuda tag (e.g. rammp-base:2.0.0-jp6 or rammp-base:nightly-20261027), "
                "not a floating or upstream image"
            )
        if len(args) >= 3 and args[1].upper() == "AS":
            stages.add(args[2])
    if not found:
        errors.append("Dockerfile: no FROM line")
    return errors


def check_demo_ready(path, fragment):
    errors = []
    image = fragment["container"]["image"]
    tag = image.split("@")[0].rsplit(":", 1)[-1] if ":" in image.split("/")[-1] else ""
    if "@sha256:" not in image and (not tag or tag in FLOATING_TAGS):
        errors.append(f"{path}: tier demo-ready: image {image} must be pinned to a release tag or digest")
    if not any("status" in t for t in fragment.get("publishes", [])):
        errors.append(f"{path}: tier demo-ready: publishes no health/status-ish topic")
    return errors


def check_mock_matches_real(real_path, real, mock_path, mock):
    errors = []
    for key in ("publishes", "subscribes"):
        real_topics = sorted(real.get(key, []))
        mock_topics = sorted(mock.get(key, []))
        if real_topics != mock_topics:
            errors.append(
                f"{mock_path}: '{key}' {mock_topics} does not match {real_path}'s {real_topics} -- "
                "a mock must expose the real alternative's exact interface"
            )
    return errors


def main():
    paths = fragment_paths()
    if not paths:
        print("no rammp-alternative*.yaml fragments found", file=sys.stderr)
        return 1

    errors = []
    ids_seen = {}
    fragments = {}
    for path in paths:
        fragment = yaml.safe_load(path.read_text())
        fragments[path] = fragment
        frag_errors = [f"{path}: {e}" for e in validate(fragment)]
        if frag_errors:
            errors += frag_errors
            continue

        image = fragment["container"]["image"]
        if not image.startswith(f"{IMAGE_PREFIX}:") and not image.startswith(f"{IMAGE_PREFIX}@"):
            errors.append(f"{path}: image {image} must be {IMAGE_PREFIX}:<tag>")

        frag_id = fragment["id"]
        if frag_id in ids_seen:
            errors.append(f"{path}: id '{frag_id}' duplicates {ids_seen[frag_id]}")
        else:
            ids_seen[frag_id] = path

        if fragment["tier"] == "demo-ready":
            errors += check_demo_ready(path, fragment)

    real_path = ROOT / "rammp-alternative.yaml"
    mock_path = ROOT / "rammp-alternative.mock.yaml"
    if real_path in fragments and mock_path in fragments:
        errors += check_mock_matches_real(
            real_path, fragments[real_path], mock_path, fragments[mock_path]
        )

    errors += check_dockerfile()

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    for path in paths:
        print(f"{path.relative_to(ROOT)}: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
