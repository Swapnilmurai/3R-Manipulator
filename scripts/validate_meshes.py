"""Validate CAD mesh files and optionally convert STEP/STP to STL via FreeCAD CLI.

The script does not fabricate missing geometry. STEP conversion requires FreeCADCmd
(or FreeCAD on PATH) and imports each STEP file as a neutral CAD shape.
"""
from __future__ import annotations
import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

SUPPORTED = {".stl", ".obj", ".ply"}


def step_to_stl(source: Path, destination: Path, freecad: str) -> None:
    macro = f"""
import Part
shape = Part.read(r'{source.as_posix()}')
if shape.isNull():
    raise RuntimeError('Imported STEP shape is null')
Part.export([shape], r'{destination.as_posix()}')
"""
    with tempfile.NamedTemporaryFile("w", suffix=".FCMacro", delete=False) as f:
        f.write(macro)
        macro_path = Path(f.name)
    try:
        subprocess.run([freecad, str(macro_path)], check=True)
    finally:
        macro_path.unlink(missing_ok=True)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--input", type=Path, default=Path("assets/cad"))
    p.add_argument("--output", type=Path, default=Path("assets/meshes"))
    p.add_argument("--freecad", default=shutil.which("FreeCADCmd") or shutil.which("freecad"))
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    files = sorted(x for x in args.input.glob("*") if x.is_file())
    if not files:
        raise SystemExit(f"No CAD files found in {args.input}. Export assembly components first.")
    for src in files:
        ext = src.suffix.lower()
        if ext in SUPPORTED:
            dst = args.output / src.name
            shutil.copy2(src, dst)
            print(f"Copied mesh: {src} -> {dst}")
        elif ext in {".step", ".stp"}:
            if not args.freecad:
                print(f"SKIP {src}: STEP needs FreeCAD. Install FreeCAD and pass --freecad.")
                continue
            dst = args.output / (src.stem + ".stl")
            step_to_stl(src, dst, args.freecad)
            print(f"Converted: {src} -> {dst}")
        else:
            print(f"Unsupported format (left untouched): {src.name}")


if __name__ == "__main__":
    main()
