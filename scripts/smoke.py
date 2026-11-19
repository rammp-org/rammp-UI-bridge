#!/usr/bin/env python3
"""Runtime smoke test for this repo's image, run once per
rammp-alternative*.yaml fragment (the real ui_bridge_node plus its mock
peer-pointed alternative -- see scripts/check_fragments.py's docstring for
why both reuse the exact same image/command).

Runs the image the way sheppyd will (no --init, host networking, the
fragment's command and its `params:` as a params file) and checks the two
things every other container relies on:

1. it stays up rather than exiting because a peer or device is absent (the
   UE WebSocket client retries in the background and never blocks startup
   -- see UnrealRemoteWebsocket's reconnect loop)
2. it exits on SIGTERM within the stop grace period

Adapted from rammp-orchestrator's scripts/smoke.py (itself adapted from
rammp-kitchen-capability-engines's).

Usage: smoke.py <image> <fragment.yaml> [<fragment.yaml> ...]
"""

import random
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

DISCOVERY_TIMEOUT_S = 10
GRACE_S = 5


def docker(*args, check=True):
    return subprocess.run(["docker", *args], capture_output=True, text=True, check=check).stdout


def fail(name, message):
    print(f"FAIL: {message}")
    print(docker("logs", "--tail", "20", name, check=False))
    return 1


def smoke_one(image, fragment_path):
    fragment = yaml.safe_load(fragment_path.read_text())
    frag_id = fragment["id"]
    command = fragment["container"].get("command", [])
    if isinstance(command, str):
        command = command.split()
    # A random domain keeps parallel smoke runs on one host from seeing each other.
    domain = str(random.randint(1, 100))
    name = f"rammp-smoke-{frag_id}-{domain}"

    # sheppy's params delivery: a file keyed by ros_node_name (or /**), and
    # --ros-args --params-file appended to the command.
    mount = []
    if fragment.get("params"):
        key = fragment.get("ros_node_name") or "/**"
        params = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        yaml.safe_dump({key: {"ros__parameters": fragment["params"]}}, params)
        params.close()
        mount = ["-v", f"{params.name}:/sheppy/params.yaml:ro"]
        command += ["--ros-args", "--params-file", "/sheppy/params.yaml"]

    docker(
        "run",
        "-d",
        "--rm",
        "--name",
        name,
        "--network",
        "host",
        "-e",
        f"ROS_DOMAIN_ID={domain}",
        *mount,
        image,
        *command,
    )
    try:
        deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
        while time.monotonic() < deadline:
            if docker("inspect", "-f", "{{.State.Running}}", name, check=False).strip() != "true":
                return fail(
                    name, f"{frag_id}: container exited on its own; never exit because a peer is absent"
                )
            time.sleep(1)
        if docker("inspect", "-f", "{{.State.Running}}", name, check=False).strip() != "true":
            return fail(name, f"{frag_id}: container exited on its own before the smoke window ended")

        start = time.monotonic()
        docker("stop", "-t", str(GRACE_S), name)
        elapsed = time.monotonic() - start
        if elapsed >= GRACE_S:
            print(f"FAIL: {frag_id} did not exit on SIGTERM within {GRACE_S}s ({elapsed:.1f}s)")
            print(
                "      A container that ignores its stop signal makes every 'sheppy woof' a stall."
            )
            return 1
    finally:
        docker("rm", "-f", name, check=False)

    print(f"PASS: {frag_id} stayed up and exited on SIGTERM in {elapsed:.1f}s")
    return 0


def main(args):
    if len(args) < 2:
        sys.exit(__doc__.strip().splitlines()[-1])
    image, *fragment_args = args
    failed = 0
    for fragment_arg in fragment_args:
        failed |= smoke_one(image, Path(fragment_arg))
    return failed


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
