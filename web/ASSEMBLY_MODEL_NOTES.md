# Browser assembly model notes

The Vercel browser app uses an articulated, CAD-proportioned visual assembly for three 28BYJ-48 5 V motor stations. It shows the Base Ring/Lid envelope, three link envelopes, motor body/flange/gearbox/connector envelopes, output shafts, pen holder and three revolute joints.

## CAD-derived joint datums

- J1 shaft centre: Base Lid local (0, 0) mm; mount-hole centres (-8, +17.5) and (-8, -17.5) mm.
- J2 shaft centre: Link 1 local (120, 0) mm; mount-hole centres (112, +17.5) and (112, -17.5) mm.
- J3 shaft centre: Link 2 local (100, 0) mm; mount-hole centres (92, +17.5) and (92, -17.5) mm.
- Distal tool axis: Link 3 local x = 60 mm.
- Nominal task-plane joint heights reconstructed from the sectioned mesh stack: Base Lid top 43 mm; Link 2 carrier plane 51 mm; Link 3 carrier plane 59 mm.

## Scope

The browser scene uses parametric visuals with the measured carrier/link envelopes and a standard 28BYJ-48 form-factor motor approximation. It is not an exact vendor STEP model, a scan of the particular motors, or a native CAD mates assembly. Original converted part meshes, source PLY files, and the mesh-based ROS 2 URDF are delivered separately in the downloadable CAD/URDF package.

MuJoCo provides three revolute joints and idealized position actuators. It does not simulate 5-wire coil commutation, ULN2003 electronics, missed steps, gearbox backlash, electrical current, motor thermal behaviour, or physical hardware. Browser view is a visual/physics digital-twin prototype pending runtime verification and physical fit validation.
