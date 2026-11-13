# CAD inspection summary

The five supplied STL files were inspected as separate rigid components. The STL coordinates are millimetres. Vertex welding produced one watertight connected mesh for each file.

| Component | Bounds (mm), X × Y × Z | Approx. volume (mm³) |
|---|---:|---:|
| Base_Ring | 154.0 × 140.0 × 36.0 | 50,482.9 |
| Base_Lid | 154.0 × 140.0 × 7.0 | 59,278.5 |
| Link1 | 150.0 × 48.0 × 8.0 | 32,488.1 |
| Link2 | 130.0 × 48.0 × 8.0 | 28,619.2 |
| Link3_PenHolder | 90.0 × 32.0 × 22.0 | 26,100.4 |

The link top views indicate proximal and distal pivot-hole centres approximately at:
- Link1: x = 0 to 120 mm
- Link2: x = 0 to 100 mm
- Link3_PenHolder: x = 0 to 60 mm

These are geometry-derived working dimensions from the component meshes, not an independently verified assembly drawing. Individual STL exports do not encode the original CAD mate constraints, motor shaft axes, or component-to-component placement transforms.

## Inertial caveat

CAD volume and centroid are available, but material density is not encoded in STL. The MuJoCo CAD model currently uses 1,200 kg/m³ as a documented placeholder density to derive mesh inertias. Replace it with measured link masses or material specifications before dynamics results are used in research publications or hardware design.

## Assembly caveat

The MuJoCo starter places the Base_Lid on top of the Base_Ring and locates joint 1 at z = 47 mm. This is a provisional stack-up, not verified from a native assembly. The pen-tip Z offset is also provisional.
