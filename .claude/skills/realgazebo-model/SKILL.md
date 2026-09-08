---
name: realgazebo-model
description: Use when creating, converting, or fixing a vehicle model (DAE meshes, model.sdf, FBX) for RealGazebo so Gazebo and Unreal render it identically — new vehicle from CAD, re-exporting meshes, adding a logo/decal, parts exploding or piling at the origin in Unreal, a mesh invisible in Gazebo, or Blender 5 without Collada support.
---

# RealGazebo model pipeline

One mesh set must serve Gazebo (DAE + SDF) and Unreal (FBX) through the RealGazebo bridge.
The bridge **writes streamed servo-link poses straight into Unreal components** and **spins motors
about local Z only** — every rule below follows from those two facts. Read
`reference/model-conventions.md` once for the why; this file is the how.

## Prerequisites (check, don't assume)

- `blender-mcp` connected (`get_scene_info` answers) and Blender ≥ 4.5. **Blender 5.0+ has no
  Collada I/O** — all DAE reading/writing goes through `scripts/`.
- `python` may be absent on Windows: run the stdlib scripts through Blender
  (`blender -b --python scripts/dae_tools.py -- <cmd>` or `exec(open(...).read())` in `execute_blender_code`).
  Each `execute_blender_code` call is a fresh namespace — re-`exec` the tool file at the top of every
  call — and pass **absolute paths** (headless Blender has no useful cwd). `bake.json` is written
  next to `model.sdf` by default.
- Work in a scene you create (`bpy.data.scenes.new`, then `bpy.context.window.scene = it`); never
  edit the user's open scenes.
- Input: the vehicle's DAE files, its `model.sdf`, the model name (= folder name = `model://<name>`).

## Procedure

0. **Work on a copy, never in place.** This pipeline overwrites DAEs (`rotate`, `bake`,
   `fix-offsets`), rewrites `model.sdf`, and deletes superseded mesh files. Before anything else,
   copy the whole model folder (`<name>/` with `meshes/`, `materials/`, `model.sdf`,
   `model.config`) to a scratch location — e.g. `<name>_work/` beside it — and run every step
   there. The untouched original is the rollback and the baseline for `compare`/diff. Copy the
   finished folder back over the deployed one only after the step 10 gate passes. Same rule for
   the PX4 airframe file if you edit it.
1. **Classify every part** with `dae_tools.py centroid meshes/*.dae` and the SDF joints:
   - rotates (prop, pusher, wheel) → **origin-centered**, spin axis must be +Z
   - everything else → **baked** (vertices in the model frame)
   Do not reclassify a part to make a step easier.
2. **Dedupe**: `dae_tools.py compare A.dae B.dae` — `SAME shape` → keep one file, point every
   rotor at it. `MIRROR image` (CW vs CCW props, left vs right surfaces) → separate files; a
   mirror is a different part even when the numbers look close.
3. **Spin axis**: a rotating part whose blades lie in XZ or YZ → `dae_tools.py rotate --axis y --deg -90`
   (positions AND normals) until the extent is smallest along Z (`compare` on an X-spin pair takes
   `--axis x` until then). Keep the SDF link pose that points link-Z along the true axis (pusher:
   pitch 1.57) and zero the old mesh compensation:
   `sdf_tools.py visual-pose model.sdf --visual <v> --pose "0 0 0 0 0 0"`. The pusher joint's own
   pose/axis is handled in step 6.
4. **SDF servo links** → `sdf_tools.py origin-links model.sdf --links <servo links> --emit-bake bake.json`
   (link pose removed, hinge into joint pose, visual 0). Then
   `sdf_tools.py body-compensate model.sdf --emit-bake bake.json` for base_link visuals.
5. **Bake** each mesh named in `bake.json`: `dae_tools.py bake --offset X Y Z file.dae`.
   Do this in Blender too if the master `.blend` is open (move vertices, not the object).
6. **SDF hygiene (SDF ≥ 1.7)**: `sdf_tools.py strip-deprecated model.sdf` (removes
   `use_parent_model_frame`); joints on a rotated link that need a model-frame axis (pushers):
   `sdf_tools.py axis model.sdf --joint <j> --xyz "1 0 0" --expressed-in __model__`.
   **Swept hinges**: a control surface whose leading edge is swept (rudder on a swept fin) hinges
   along the sweep, not the vertical — a vertical axis makes the surface shear away from the fin
   when deflected. After baking: `dae_tools.py hinge-axis meshes/Rudder.dae --span z` (`--span y`
   for ailerons/elevators); when it reports `swept` (≥ 5°), apply
   `sdf_tools.py axis model.sdf --joint <j> --xyz "<axis>" --expressed-in base_link`
   (LC-62 rudder: `-0.544 0 0.839`).
7. **Decal (optional)**: `make_alpha_png` → `make_decal(...)` on the body (0.5–1 mm off the
   surface, modeled into the mesh) → `write_dae_with_uv` →
   `sdf_tools.py add-visual model.sdf --name <n> --mesh meshes/<n>.dae --albedo materials/textures/<n>.png
   --pose "<exactly the body's compensation pose>"`. No extra lift in the SDF pose: verified in
   Gazebo at 0.95 m and 15 m — the modeled offset alone is z-fight free (an invisible decal means
   step 8's offset bug, not depth). **Frame rule**: build the decal on the *unbaked* body and bake
   it with the body's offset, or build it on the already-baked body with region coordinates in the
   model frame — mixing the two puts the logo off by the base_link offset. An existing decal DAE
   round-trips through `import_daes` with its UVs and texture, so its FBX carries the material. After the FBX import in Unreal the decal material MUST be
   switched to Blend Mode **Masked** with the texture's A channel wired to Opacity Mask — the
   importer never does this; without it the logo renders as a white box.
8. **Offsets**: `dae_tools.py fix-offsets meshes/*.dae` — any script-written DAE must have
   VERTEX=0 / NORMAL=1 / TEXCOORD=2. gz-common renders a shared-offset mesh as **nothing, no error**.
9. **Export FBX**: `export_fbx_parts(out_dir)` — one file per object, **same name as the DAE**.
   Import into an empty Blender file first: a name already in use becomes `Rudder.001` and the
   FBX inherits the suffix.
10. **Gate** (must PASS before handing over):
    - `sdf_tools.py verify model.sdf --meshes meshes` → every baked visual has link+visual = 0,
      every origin-centered visual sits on a posed link.
    - Re-import all DAEs into an empty Blender scene at identity: baked parts assemble, rotating
      parts sit at the origin.
    - On the Gazebo machine: `GZ_SIM_RESOURCE_PATH=<models dir> gz sdf -k model.sdf` → `Valid`
      with **zero warnings**; after spawning, the control-surface poses received in Unreal are ≈ 0.
11. **Hand over** `sdf_tools.py unreal-table model.sdf` (cm, Y flipped, pusher Pitch −90) and the
    RotatingComponents order = Gazebo motor numbers. Ship `model.sdf` **together with** the PX4
    airframe file: pusher `motorConstant` and `SIM_GZ_EC_MAX<n>` are one tuning pair
    (max thrust = motorConstant × EC_MAX²; check it against cruise drag before release).

## Quick reference

| Symptom | Cause | Fix |
|---|---|---|
| Unreal parts at 2× distance | link at hinge + baked mesh | step 4 (`origin-links`) |
| Parts piled at origin in Unreal/Blender | non-rotating mesh not baked | step 5 |
| Gazebo mesh invisible, no error, no highlight | shared `<input offset>` | step 8 |
| Pusher spins about the wrong axis in Unreal | mesh modeled X-spin | step 3 |
| Body and decal off by a few cm | base_link pose ≠ 0 not compensated | `body-compensate` |
| Logo shows white box in Unreal | FBX import ignores alpha | Masked material, A → Opacity Mask |

## Common mistakes

- **Editing the delivered folder in place.** Steps 3, 5 and 8 overwrite meshes and step 2
  deletes files; with no untouched copy there is nothing to diff against or roll back to. Step 0.
- **"Gazebo looks right, ship it."** Gazebo alone tolerates link-at-hinge; the Unreal stream does
  not. Run `verify` and the identity re-import, both.
- **Baking by object transform** in Blender. The FBX then carries a transform, not baked vertices.
  Move vertices (`v.co += offset`) or use `dae_tools.py bake`.
- **Baking a shared prop file** for one rotor. Rotating parts stay origin-centered; position lives
  in the link pose.
- **Editing the SDF with an XML library.** It strips comments and reorders attributes; use
  `sdf_tools.py` (text patches) or a text editor.
- **Trusting `<scale>` to fix size mismatches.** Every visual of the vehicle must carry the same
  scale or baked parts drift apart; fix the source instead.
- **Regenerating a DAE with the shared-offset layout** because "the spec allows it". It does; the
  loader doesn't. Always run step 8.
