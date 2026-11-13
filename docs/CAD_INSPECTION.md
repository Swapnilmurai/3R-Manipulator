# CAD inspection summary — 13 November 2026

Five supplied STL components were inspected as separate rigid meshes. Source STL coordinates are millimetres. After vertex welding, each component formed one watertight connected mesh.

| Component | Bounds (mm), X × Y × Z | Approx. volume (mm³) |
|---|---:|---:|
| Base_Ring | 154.0 × 140.0 × 36.0 | 50,482.9 |
| Base_Lid | 154.0 × 140.0 × 7.0 | 59,278.5 |
| Link1 | 150.0 × 48.0 × 8.0 | 32,488.1 |
| Link2 | 130.0 × 48.0 × 8.0 | 28,619.2 |
| Link3_PenHolder | 90.0 × 32.0 × 22.0 | 26,100.4 |

Top-view hole patterns indicate approximate proximal/distal pivot centres:
- Link1: x = 0 to 120 mm
- Link2: x = 0 to 100 mm
- Link3_PenHolder: x = 0 to 60 mm

The resulting working link spacings are 120 / 100 / 60 mm. These are geometry-derived estimates from individual parts, not a substitute for CAD assembly mate origins or a dimensioned drawing.

## Inertial estimates

STL does not encode material density. The following estimates use a placeholder uniform density of 1,200 kg/m³:

| Component | Estimated mass (kg) |
|---|---:|
| Base ring + lid, with lid stacked at z = 36 mm | 0.131714 |
| Link1 | 0.038986 |
| Link2 | 0.034343 |
| Link3_PenHolder | 0.031320 |

The link inertial tensors were calculated from the watertight mesh solids and transformed so each link frame is centred on its pivot-axis plane. They are still assumptions until measured part masses/material properties are supplied. Replace them before using dynamics results in research publications or hardware design.

## Assembly and simulation caveats

Individual STL exports do not encode the original CAD mate constraints, motor shaft axes, or component-to-component placement transforms. The MuJoCo/URDF starter assumes:
- Base_Lid stacked on Base_Ring at z = 36 mm
- Joint 1 axis at z = 47 mm
- Link planes and revolute axes aligned with the global z direction
- Tool point at a temporary offset relative to the Link3_PenHolder frame

These offsets must be checked against the native assembly or a dimensioned drawing. The current MuJoCo collision capsules are simplified proxies; use validated collision meshes/convex decomposition for obstacle-clearance and collision-safety claims.
