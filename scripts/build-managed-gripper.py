#!/usr/bin/env python3
"""Build the capture-only hand client against an existing Polymetis build."""

import argparse
import os
import shlex
import subprocess
import tempfile
from pathlib import Path

POLY = Path("polymetis/polymetis")
TARGET = Path("CMakeFiles/franka_hand_client.dir")
OBJECT = TARGET / "src/clients/franka_panda_client/franka_hand_client.cpp.o"
OUTPUT = "franka_hand_capture_client"


def unique(values, label):
    matches = set(values)
    if len(matches) != 1:
        raise ValueError(f"Could not identify the original {label} in the Polymetis build")
    return matches.pop()


def build(repo, teleop, sdk, prefix):
    directory = teleop / POLY / "build"
    flags = {
        key.strip(): shlex.split(value)
        for line in (directory / TARGET / "flags.make").read_text().splitlines()
        if line.startswith(("CXX_DEFINES =", "CXX_INCLUDES =", "CXX_FLAGS ="))
        for key, value in [line.split("=", 1)]
    }
    includes = flags["CXX_INCLUDES"]
    link = shlex.split((directory / TARGET / "link.txt").read_text())
    if link[0] != "/usr/bin/c++" or link.count(str(OBJECT)) != 1:
        raise ValueError("Expected the Linux CMake franka_hand_client compile/link layout")
    output_index = link.index("-o") + 1
    if link[output_index] != "franka_hand_client":
        raise ValueError("Unexpected original linker output")
    original_teleop = unique(
        (x[2 : -len(f"/{POLY}/include")] for x in includes if x.endswith(f"/{POLY}/include")),
        "controller source tree",
    )
    original_sdk = unique(
        (str(Path(x).parent.parent) for x in link if "/build/libfranka.so." in x), "libfranka"
    )
    original_prefix = unique(
        (str(Path(x).parent.parent) for x in link if "/lib/libgrpc++.so." in x), "Polymetis prefix"
    )
    if not all(Path(x).is_absolute() for x in (original_teleop, original_sdk, original_prefix)):
        raise ValueError("Expected absolute original dependency locations")
    roots = {
        original_teleop: teleop,
        original_sdk: sdk,
        original_prefix: prefix,
    }

    def relocated(arg):
        flag, path = ("-I", arg[2:]) if arg.startswith("-I/") else ("", arg)
        for original, local in sorted(roots.items(), key=lambda pair: -len(pair[0])):
            if path.startswith(original + "/"):
                return flag + str(local / Path(path).relative_to(original))
        if path.startswith("/") and not path.startswith(("/usr/bin/", "/usr/lib/")):
            raise ValueError(f"Unexpected dependency outside the configured roots: {path}")
        return arg

    output_dir = repo / ".native"
    output_dir.mkdir(exist_ok=True)
    rpaths = (sdk / "build", prefix / "lib")
    with tempfile.TemporaryDirectory(prefix="build-", dir=output_dir) as temp:
        obj = Path(temp) / "managed_gripper.o"
        binary = Path(temp) / OUTPUT
        subprocess.run(
            [
                link[0],
                *flags["CXX_FLAGS"],
                *flags["CXX_DEFINES"],
                *(relocated(x) for x in includes),
                "-pthread",
                "-c",
                str(repo / "native/managed_gripper.cpp"),
                "-o",
                str(obj),
            ],
            cwd=directory,
            check=True,
        )
        link[link.index(str(OBJECT))] = str(obj)
        link[output_index] = str(binary)
        args = [
            x if x in (str(obj), str(binary)) else relocated(x)
            for x in link
            if not x.startswith("-Wl,-rpath,")
        ]
        # Preserve transitive lookup of the runtime's packaged gRPC dependencies.
        args.extend(("-Wl,--disable-new-dtags", "-Wl,-rpath," + ":".join(map(str, rpaths))))
        subprocess.run(args, cwd=directory, check=True)
        os.replace(binary, output_dir / OUTPUT)
    return output_dir / OUTPUT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teleop-root", required=True, type=Path)
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--polymetis-prefix", required=True, type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    try:
        binary = build(
            repo,
            args.teleop_root.resolve(strict=True),
            args.sdk_root.resolve(strict=True),
            args.polymetis_prefix.resolve(strict=True),
        )
    except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"Managed hand build failed: {exc}\n")
    print(f"Managed hand client built at {binary}. No hardware opened.")


if __name__ == "__main__":
    main()
