# RealGazebo Model Conventions — Unified Gazebo·Unreal Assets

In the RealGazebo pipeline, a single vehicle model is rendered from **the same mesh set** in both Gazebo (DAE) and Unreal (FBX).
This document defines the coordinate, SDF, and Unreal placement conventions that keep parts assembled and animated identically in both environments.
The LC-62 model is used as the reference case; apply the same rules when adding a new vehicle.

## Why these conventions exist

The RealGazebo Unreal plugin (`VehicleBasePawn`) consumes values streamed from Gazebo as follows:

| Stream | Plugin behavior | Source |
|---|---|---|
| Servo (control surface) link poses | Written **directly** into the relative location/rotation of `ControllableComponents[i]` | `VehicleBasePawn.cpp` `ApplyServoStates()` |
| Motor speeds | Spins `RotatingComponents[i]` about its **local Z axis (yaw) only** | `VehicleBasePawn.cpp` `ApplyMotorSpeeds()` |

Two constraints follow:

1. **A control surface mesh ends up at "streamed Gazebo link pose + mesh vertex coordinates."**
   If position information exists in both places it is applied twice, and parts explode outward
   to double their true distance from the origin.
   → Keep link poses ≈ 0 and bake positions into the mesh vertices.
2. **A rotating part's spin axis is the component's local Z.**
   → Model all propeller meshes as Z-spin; establish direction through placement
   (component/link rotation), never in the mesh itself.

## Coordinate conventions

### Baked — position lives in the mesh vertices

Applies to: **body, control surfaces (ailerons, elevators, rudder), decals (logos) — every non-rotating part**

- DAE/FBX vertex coordinates hold the **final position in the model frame**.
- Dropping the mesh at (0, 0, 0) in Blender/Unreal assembles the vehicle automatically.
- Example: the vertex centroid of `Aileron_L.dae` ≈ (-0.516, 0.820, 0.063) — the left wing position.

### Origin-centered — placement provides the position

Applies to: **rotating parts (lift props, pusher props)**

- Model the mesh with the hub (spin center) at the origin.
- **The spin axis must be +Z** (blades lying in the XY plane).
- Forward-thrust props (pushers) are also modeled Z-spin; orientation comes from the SDF link
  pose (pitch 1.57) / the Unreal component rotation (Pitch −90°). Never model the mesh
  facing forward.
- Identical prop shapes share one file across rotors (e.g. `Prop_1` ← rotors 0/2/5).
- Mirror-image left/right parts stay separate files. Do not use negative-scale mirroring.

## SDF conventions

### Control surface (servo) links

```xml
<link name="left_ail">
  <!-- No link pose = model origin. The streamed pose is ≈ identity. -->
  <inertial>
    <pose>-0.506204 0.819956 0.068762 0 0 0</pose>  <!-- hinge position -->
    ...
  </inertial>
  <visual name="left_ail_visual">
    <pose>0 0 0 0 0 0</pose>   <!-- mesh is baked, so zero -->
    ...
  </visual>
</link>

<joint name="left_ail_joint" type="revolute">
  <child>left_ail</child>
  <parent>base_link</parent>
  <pose>-0.506204 0.819956 0.068762 0 0 0</pose>  <!-- hinge goes in the joint pose -->
  ...
</joint>
```

- **No link pose** (stay at the origin). The hinge position goes in the **joint pose**.
- Visual pose is zero. When the link rotates about the hinge, that (rotation + slight
  translation) is streamed and reproduces the same hinge rotation in Unreal.
- ⚠️ Putting the link at the hinge (`<pose relative_to="base_link">`) works in Gazebo alone,
  but then the streamed value becomes the hinge coordinate and doubles the offset in Unreal.
  Do not use it.

### Rotor / pusher links

- Link pose = rotor mount position (required — the physics plugin applies thrust there).
- Visual pose = 0; the mesh is origin-centered, so the link pose is the placement.
- Pushers get pitch 1.57 in the link pose so the link Z axis points forward.
  Because the mesh is Z-spin, no visual compensation is needed.

### Other

- If the body link (base_link) pose is nonzero, compensate with its inverse in the body and
  decal visual poses so the baked mesh is applied exactly once
  (LC-62: base_link pose (-0.031204, -0.000834, 0.005588) →
  visual pose (0.031204, 0.000834, -0.005588)).
- Model name = folder name = URI (`model://<name>/...`). A mismatch means meshes are not found.
- Verify: for every baked part, **(link position in model frame + visual pose) = 0**.

### SDF ≥ 1.7 hygiene

- `<use_parent_model_frame>` was removed in SDF 1.7: gz warns and ignores it. Delete it.
- A joint on a rotated link that needs a model-frame axis (pushers, link pitch 1.57) uses
  `<xyz expressed_in="__model__">1 0 0</xyz>`.
- **Swept hinges**: a control surface whose leading edge is swept (the rudder on a swept fin)
  hinges along the sweep line, not the vertical; a vertical axis makes the surface shear away
  from the fin when deflected. Procedure (on the baked mesh): take a 5 cm slab at each span end,
  the leading-edge point of each slab is its most-forward vertex, axis = normalize(upper LE −
  lower LE) with the lateral component zeroed, sign chosen so "up the span" is positive; apply it
  when the sweep exceeds ~5° (LC-62 rudder: LE (-0.806, z 0.103) → (-0.968, z 0.353),
  `<xyz expressed_in="base_link">-0.544 0 0.839</xyz>`).
- Validate on the Gazebo machine: `GZ_SIM_RESOURCE_PATH=<models> gz sdf -k model.sdf` must print
  `Valid` with zero warnings.

### Model ↔ airframe pairing

`model.sdf` motor plugins and the PX4 airframe file are one tuning set: pusher `motorConstant`
and `SIM_GZ_EC_MAX7/8` determine max thrust (`motorConstant × EC_MAX²`). Ship them together and
check the thrust budget against cruise drag before release (LC-62 status: the pair
0.10e-4 / 3500 saturated at 24.7 m/s in transition — candidates: motorConstant 0.20e-4, or
VT_F_TRANS_THR 1.0 + FW_THR_MAX 0.9).

## Custom DAE generation — the empty-mesh pitfall

If DAE files are generated by script (required with Blender 5.0+, which removed Collada I/O),
**give every `<triangles>` input its own offset**:

```xml
<input semantic="VERTEX"   source="#V" offset="0"/>
<input semantic="NORMAL"   source="#N" offset="1"/>
<input semantic="TEXCOORD" source="#T" offset="2" set="0"/>
```

A layout where VERTEX and NORMAL share offset 0 is legal per the Collada spec, but the
Gazebo loader (gz-common) silently reads it as an **empty mesh** — no error, nothing rendered,
no highlight when selected. This was the cause of the invisible KARI logo decal on LC-62.
Always emit the standard separated layout (VERTEX=0, NORMAL=1, TEXCOORD=2) with `<p>`
index tuples expanded accordingly.

## Unreal placement conventions

| Part | Placement | Rotation |
|---|---|---|
| Body, decals, control surfaces | (0, 0, 0) — streaming/baking provide position | 0 |
| Lift props | Rotor position (SDF value ×100 cm, **Y sign flipped**) | 0 |
| Pusher props | Mount position (same conversion) | **Pitch −90°** (local Z → forward) |

- Register `RotatingComponents` in **Gazebo motor-number order**
  (LC-62: 0–5 lift rotors, 6 left pusher, 7 right pusher).
- Register control surfaces in `ControllableComponents` in servo order and leave their
  transforms at zero.
- For decals with alpha (transparent) textures, set the material Blend Mode to **Masked**
  after import and connect the texture's A channel to Opacity Mask (the FBX importer does
  not configure this automatically).

## Authoring pipeline (Blender)

- Keep a single `.blend` master file and export both DAE (Gazebo) and FBX (Unreal) from it.
  Never author per-format.
- FBX file names and object names **match the source DAE names**.
- ⚠️ Collada (.dae) import/export exists only up to Blender 4.5 LTS. On 5.0+, handle DAE via
  direct XML parsing/generation scripts (see the pitfall above) or keep 4.5 LTS installed
  alongside.
- Keep textures as separate PNG files (shared by DAE and FBX; updating the image updates both).

## Decal (logo) conventions

To attach a logo without UV-unwrapping the body mesh, use the same pattern as the x500's
NXP logo:

1. Build a thin patch mesh conforming to the target surface (shrinkwrap/raycast) and UV it.
   Offset 0.5–1 mm from the surface.
2. Put a background-transparent PNG under `materials/textures/`.
3. Add it to the SDF as a separate `<visual>` with a PBR `albedo_map`, pose = exactly the body's
   compensation pose (no extra lift — verified z-fight free in Gazebo at 0.95 m and 15 m with the
   0.6 mm modeled offset; a decal that does not render is the input-offset bug below, not depth).
   Do not touch the body file.
4. Bake the patch in the same frame as the body (it moves with the body).
5. In Unreal, after importing the FBX, set the decal material to Blend Mode **Masked** and wire
   the texture's A channel to Opacity Mask — otherwise the logo renders as a white box.

## LC-62 reference values

Hinges (model frame, m):

| Joint | X | Y | Z |
|---|---|---|---|
| left_ail | -0.506204 | 0.819956 | 0.068762 |
| right_ail | -0.506204 | -0.816144 | 0.068829 |
| left_elevator | 0.568796 | 0.517666 | -0.013702 |
| right_elevator | 0.568796 | -0.517834 | -0.013871 |
| rudder | -0.881204 | -0.000834 | 0.191168 |

Rotors / pushers (Unreal cm, Y sign flipped):

| Component | Mesh | X | Y | Z | Pitch |
|---|---|---|---|---|---|
| rotor_0 | Prop_1 | 8.72 | 72.50 | 7.76 | 0 |
| rotor_1 | Prop_2 | 8.77 | -72.50 | 7.76 | 0 |
| rotor_2 | Prop_1 | 109.28 | -71.86 | 7.73 | 0 |
| rotor_3 | Prop_2 | -101.22 | 71.89 | -10.06 | 0 |
| rotor_4 | Prop_2 | 109.27 | 71.87 | 7.73 | 0 |
| rotor_5 | Prop_1 | -101.26 | -71.90 | -10.06 | 0 |
| pusher_L | Pusher_L | -31.77 | -115.01 | 6.68 | -90 |
| pusher_R | Pusher_R | -31.77 | 115.01 | 6.73 | -90 |

## Checklist (adding or modifying a model)

- [ ] Work was done on a copy of the model folder; the delivered original is intact for diff/rollback
- [ ] Non-rotating parts: vertices baked in model-frame coordinates (dropping at 0,0,0 in Blender assembles the vehicle)
- [ ] Rotating parts: hub at origin, spin axis +Z
- [ ] Control surface links have no pose; hinges live in joint poses
- [ ] For every baked part, (link position + visual pose) = 0
- [ ] Model name = folder name = URI
- [ ] Script-generated DAE uses separated input offsets (VERTEX=0, NORMAL=1, TEXCOORD=2)
- [ ] FBX file names match the DAE names
- [ ] Parts assemble correctly when spawned in Gazebo / control surfaces stream in near identity in Unreal
