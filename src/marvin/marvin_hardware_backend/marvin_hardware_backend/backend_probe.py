"""Non-driving installation probe for the Marvin communication backend."""

from __future__ import annotations

import argparse
import json
import platform

from .marvin_hardware_bridge import MarvinHardwareBridge
from .sdk_loader import load_marvin_sdk, resolve_marvin_sdk


def main(args=None) -> int:
    parser = argparse.ArgumentParser(
        description="Check the Marvin backend without connecting to a robot."
    )
    parser.add_argument(
        "--resolve-only",
        action="store_true",
        help="Only locate SDK files; do not load libMarvinSDK.so.",
    )
    options = parser.parse_args(args)

    location = resolve_marvin_sdk()
    result = {
        "architecture": platform.machine(),
        "backend_import": "ok",
        "ros_bridge_class": MarvinHardwareBridge.__name__,
        "ros_bridge_import": "ok",
        "network_connect_attempted": False,
        "sdk_library": str(location.library),
        "sdk_python_root": str(location.python_root),
        "sdk_wrapper": str(location.wrapper),
    }
    if options.resolve_only:
        result["sdk_load"] = "skipped"
    else:
        dcss, robot_type, loaded_location = load_marvin_sdk()
        # Constructing these objects loads local code and the shared library.
        # It does not call Marvin_Robot.connect/OnLinkTo.
        robot = robot_type()
        dcss()
        result.update(
            {
                "sdk_load": "ok",
                "sdk_library": str(loaded_location.library),
                "sdk_connected": bool(
                    getattr(robot, "_connected", False)
                ),
            }
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
