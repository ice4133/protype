"""Locate the bundled Marvin SDK without connecting to hardware."""

from dataclasses import dataclass
import __future__
import importlib
import importlib.util
from pathlib import Path
import sys


class MarvinSdkNotFoundError(RuntimeError):
    """Raised when the bundled vendor runtime cannot be located."""


@dataclass(frozen=True)
class MarvinSdkLocation:
    python_root: Path
    wrapper: Path
    library: Path


def resolve_marvin_sdk() -> MarvinSdkLocation:
    spec = importlib.util.find_spec("marvin_sdk")
    if spec is None or not spec.submodule_search_locations:
        raise MarvinSdkNotFoundError(
            "The bundled marvin_sdk package is not installed. "
            "Rebuild marvin_hardware_backend."
        )
    package_dir = Path(next(iter(spec.submodule_search_locations))).resolve()
    wrapper = package_dir / "fx_robot.py"
    library = package_dir / "libMarvinSDK.so"
    if not wrapper.is_file() or not library.is_file():
        raise MarvinSdkNotFoundError(
            "The bundled marvin_sdk installation is incomplete: "
            f"expected {wrapper} and {library}"
        )
    return MarvinSdkLocation(package_dir.parent, wrapper, library)


def load_marvin_sdk():
    """Return ``(DCSS, Marvin_Robot, location)`` without network access."""
    location = resolve_marvin_sdk()
    module_name = "marvin_sdk.fx_robot"
    module = sys.modules.get(module_name)
    if module is None:
        package = importlib.import_module("marvin_sdk")
        spec = importlib.util.spec_from_file_location(
            module_name, location.wrapper
        )
        if spec is None or spec.loader is None:
            raise MarvinSdkNotFoundError(
                f"Cannot create an import spec for {location.wrapper}"
            )
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            source = location.wrapper.read_text(encoding="utf-8")
            code = compile(
                source,
                str(location.wrapper),
                "exec",
                flags=__future__.annotations.compiler_flag,
                dont_inherit=True,
            )
            exec(code, module.__dict__)
            setattr(package, "fx_robot", module)
        except BaseException:
            sys.modules.pop(module_name, None)
            raise
    imported = Path(module.__file__).resolve()
    if imported != location.wrapper:
        raise MarvinSdkNotFoundError(
            "A different marvin_sdk was already imported: "
            f"{imported}; expected {location.wrapper}"
        )
    return module.DCSS, module.Marvin_Robot, location
