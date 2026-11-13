# URDF/Xacro CAD mapping

The robot description uses STL meshes in millimetres scaled by 0.001 to metres. The mesh archive must be extracted at the repository root to populate this package's `meshes/` directory.

The three joint origins use approximate proximal/distal hole-centre spacings measured from the individual STL components: 120 mm, 100 mm, and 60 mm. The exports do not contain CAD assembly mates, so the base height, joint stack-up, and pen tip offset still require confirmation from the source assembly or a dimensioned drawing.

Inertias were calculated from mesh geometry at an assumed uniform density of 1,200 kg/m³. They are placeholders until actual material/mass properties are available.
