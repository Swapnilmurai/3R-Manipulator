"""MuJoCo smoke test. CAD model is default; --primitive selects the mesh-free fallback."""
from pathlib import Path
import argparse
import mujoco
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--primitive", action="store_true",
                        help="use the simple primitive model rather than CAD meshes")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    xml = Path(__file__).with_name("planar_3r.xml" if args.primitive else "planar_3r_cad.xml")
    if not args.primitive:
        required = ["Base_Ring.stl", "Base_Lid.stl", "Link1.stl", "Link2.stl", "Link3_PenHolder.stl"]
        mesh_dir = repo_root / "assets" / "meshes"
        missing = [name for name in required if not (mesh_dir / name).is_file()]
        if missing:
            names = ", ".join(missing)
            raise SystemExit(
                f"CAD meshes are missing from {mesh_dir}: {names}. "
                "Extract the CAD asset archive at the repository root, or run with --primitive."
            )

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)

    # Gentle bounded test pose. This is a simulation smoke test, never hardware control.
    targets = np.array([0.25, -0.45, 0.35], dtype=float)
    for _ in range(args.steps):
        data.ctrl[:] = targets
        mujoco.mj_step(model, data)

    tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool0")
    print(f"Model: {xml.name}")
    print(f"Simulation steps: {args.steps}")
    print(f"Joint position (rad): {np.round(data.qpos, 4)}")
    print(f"Tool position (m): {np.round(data.site_xpos[tip_id], 4)}")

    if not args.no_viewer:
        try:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    viewer.sync()
        except Exception as exc:
            print(f"Viewer not available ({exc}); smoke test completed.")


if __name__ == "__main__":
    main()
