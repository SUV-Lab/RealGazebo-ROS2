"""Blender-side tools (require bpy). Load into a running Blender through blender-mcp:

    exec(open(r"<skill>/scripts/blender_tools.py", encoding="utf-8").read())
    import_daes([r"...\\meshes\\Aileron_L.dae", ...])   # UVs + textured material are imported too
    export_fbx_parts(r"...\\unreal")                       # one FBX per object, same names as the DAEs
    make_alpha_png(r"logo.jpg", r"materials\\textures\\logo.png")
    ob = make_decal("LC62_Body", u_axis="y", u_range=(0.55, 0.24), v_axis="x", v_range=(-0.435, -0.335),
                    ray_axis="z", ray_from=0.5, offset=0.0006, image=r"materials\\textures\\logo.png")
    write_dae_with_uv(ob, r"meshes\\Logo.dae", "../materials/textures/logo.png")

Usage notes
- Every `execute_blender_code` call is a fresh namespace: put the exec(...) line at the top of EACH call.
- Functions bind to the CURRENT scene: set `bpy.context.window.scene = your_scene` first (create one with
  bpy.data.scenes.new so an existing file's objects don't collide and rename yours to `X.001`).
- Headless alternative: `blender -b --python this_file.py -- import <out.blend> a.dae b.dae ...`.

Blender 5.0+ has no Collada importer/exporter, so DAE goes through import_daes / write_dae_with_uv.
Baking, rotating and offset fixes on existing DAE files are done with dae_tools.py (no bpy needed).
"""
import os
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Vector

sys.dont_write_bytecode = True
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


# ----------------------------------------------------------------------------- import
def _float_array(mesh, source_ref):
    src = mesh.find(f"c:source[@id='{source_ref.lstrip('#')}']", NS)
    arr = src.find("c:float_array", NS) if src is not None else None
    return [float(x) for x in arr.text.split()] if arr is not None else None


def _texture_path(root, dae_path):
    img = root.find(".//c:library_images/c:image/c:init_from", NS)
    if img is None or not img.text:
        return None
    p = img.text.strip()
    if p.startswith("file://"):
        p = p[7:]
    p = os.path.normpath(os.path.join(os.path.dirname(dae_path), p)) if not os.path.isabs(p) else p
    return p if os.path.exists(p) else None


def import_daes(paths, collection=None, prefix=""):
    """Import DAE files: positions, faces, per-loop UVs (TEXCOORD) and, when the file references an
    image that exists, a Principled material with that image (alpha wired) — so an existing decal
    round-trips to FBX with its texture. Blender recomputes normals."""
    coll = None
    if collection:
        coll = bpy.data.collections.get(collection) or bpy.data.collections.new(collection)
        if coll.name not in bpy.context.scene.collection.children:
            bpy.context.scene.collection.children.link(coll)
    made = []
    for path in paths:
        root = ET.parse(path).getroot()
        tex = _texture_path(root, path)
        geoms = root.findall(".//c:library_geometries/c:geometry", NS)
        stem = os.path.splitext(os.path.basename(path))[0]
        for geom in geoms:
            # object name = DAE file name (the FBX naming rule); suffix only when a file holds several geometries
            gname = geom.get("name") or geom.get("id")
            name = prefix + (stem if len(geoms) == 1 else f"{stem}_{gname}")
            mesh = geom.find("c:mesh", NS)
            pos_ref = mesh.find("c:vertices", NS).find("c:input[@semantic='POSITION']", NS).get("source")
            f = _float_array(mesh, pos_ref)
            verts = [tuple(f[i:i + 3]) for i in range(0, len(f), 3)]
            faces, loop_uvs = [], []
            for prim in mesh.findall("c:triangles", NS) + mesh.findall("c:polylist", NS):
                inputs = prim.findall("c:input", NS)
                stride = max(int(i.get("offset")) for i in inputs) + 1
                voff = int(next(i for i in inputs if i.get("semantic") == "VERTEX").get("offset"))
                tin = next((i for i in inputs if i.get("semantic") == "TEXCOORD"), None)
                uv_arr = _float_array(mesh, tin.get("source")) if tin is not None else None
                toff = int(tin.get("offset")) if tin is not None else None
                idx = [int(x) for x in prim.find("c:p", NS).text.split()]
                if prim.tag.endswith("triangles"):
                    counts = [3] * (len(idx) // (3 * stride))
                else:
                    counts = [int(x) for x in prim.find("c:vcount", NS).text.split()]
                p = 0
                for vc in counts:
                    faces.append(tuple(idx[p + j * stride + voff] for j in range(vc)))
                    if uv_arr is not None:
                        for j in range(vc):
                            k = idx[p + j * stride + toff]
                            loop_uvs.append((uv_arr[2 * k], uv_arr[2 * k + 1]))
                    p += vc * stride
            me = bpy.data.meshes.new(name)
            me.from_pydata(verts, [], faces)
            me.update()
            if loop_uvs and len(loop_uvs) == len(me.loops):
                uv = me.uv_layers.new(name="UVMap")
                for li, (u, v) in enumerate(loop_uvs):
                    uv.data[li].uv = (u, v)
                if tex:
                    mat = bpy.data.materials.new(name + "_mat")
                    mat.use_nodes = True
                    bsdf = mat.node_tree.nodes["Principled BSDF"]
                    node = mat.node_tree.nodes.new("ShaderNodeTexImage")
                    node.image = bpy.data.images.load(tex, check_existing=True)
                    mat.node_tree.links.new(node.outputs["Color"], bsdf.inputs["Base Color"])
                    mat.node_tree.links.new(node.outputs["Alpha"], bsdf.inputs["Alpha"])
                    for attr, val in (("blend_method", "BLEND"), ("surface_render_method", "BLENDED")):
                        try:
                            setattr(mat, attr, val)
                        except Exception:
                            pass
                    me.materials.append(mat)
            ob = bpy.data.objects.new(name, me)
            (coll or bpy.context.collection).objects.link(ob)
            made.append(ob)
    return made


# ----------------------------------------------------------------------------- export
def export_fbx_parts(out_dir, names=None):
    """One FBX per object, file name == object name (the DAE naming rule)."""
    os.makedirs(out_dir, exist_ok=True)
    names = names or [o.name for o in bpy.context.scene.objects if o.type == "MESH"]
    for name in names:
        bpy.ops.object.select_all(action="DESELECT")
        bpy.data.objects[name].select_set(True)
        bpy.ops.export_scene.fbx(filepath=os.path.join(out_dir, name + ".fbx"), use_selection=True,
                                 path_mode="COPY", embed_textures=True, mesh_smooth_type="FACE")
    bpy.ops.object.select_all(action="DESELECT")
    return names


# ----------------------------------------------------------------------------- decal
def make_alpha_png(src, dst, white_cut=0.9):
    """White background -> transparent; all opaque pixels take the logo's mean color (no halo)."""
    import numpy as np
    img = bpy.data.images.load(src, check_existing=True)
    w, h = img.size
    px = np.array(img.pixels[:], dtype=np.float32).reshape(h, w, 4)
    dist = np.sqrt(((1.0 - px[:, :, :3]) ** 2).sum(axis=2))
    alpha = np.clip(dist / white_cut, 0, 1)
    color = px[:, :, :3][alpha > 0.95].mean(axis=0)
    out = np.empty_like(px)
    out[:, :, :3] = color
    out[:, :, 3] = alpha
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    new = bpy.data.images.new(os.path.basename(dst), width=w, height=h, alpha=True)
    new.pixels = out.ravel().tolist()
    new.filepath_raw = dst
    new.file_format = "PNG"
    new.save()
    return new


def make_decal(body_name, u_axis, u_range, v_axis, v_range, ray_axis, ray_from, offset, image,
               name="Decal", res=(49, 17)):
    """Thin patch conforming to the body surface.
    u_range runs along texture U (left->right as read), v_range along texture V (bottom->top).
    Rays are cast along ray_axis from coordinate ray_from toward the surface."""
    deps = bpy.context.evaluated_depsgraph_get()
    body = bpy.data.objects[body_name].evaluated_get(deps)
    inv = body.matrix_world.inverted()
    ax = {"x": 0, "y": 1, "z": 2}
    ui, vi, ri = ax[u_axis], ax[v_axis], ax[ray_axis]
    verts, last = [], None
    NU, NV = res
    for j in range(NV):
        t = j / (NV - 1)
        for i in range(NU):
            s = i / (NU - 1)
            p = [0.0, 0.0, 0.0]
            p[ui] = u_range[0] + (u_range[1] - u_range[0]) * s
            p[vi] = v_range[0] + (v_range[1] - v_range[0]) * t
            p[ri] = ray_from
            d = [0.0, 0.0, 0.0]
            d[ri] = -1.0 if ray_from > 0 else 1.0
            hit, loc, n, _ = body.ray_cast(inv @ Vector(p), Vector(d))
            if hit:
                w = body.matrix_world @ loc
                last = w[ri] - d[ri] * offset
            p[ri] = last if last is not None else ray_from
            verts.append(tuple(p))
    faces = []
    vid = lambda i, j: j * NU + i  # noqa: E731
    for j in range(NV - 1):
        for i in range(NU - 1):
            faces.append([vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)])
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    # normals must face the viewer (against the ray direction)
    toward = Vector([0, 0, 0]); toward[ri] = 1.0 if ray_from > 0 else -1.0
    if me.polygons and me.polygons[0].normal.dot(toward) < 0:
        me.flip_normals()
    uv = me.uv_layers.new(name="UVMap")
    for poly in me.polygons:
        for li in poly.loop_indices:
            co = me.vertices[me.loops[li].vertex_index].co
            uv.data[li].uv = ((co[ui] - u_range[0]) / (u_range[1] - u_range[0]),
                              (co[vi] - v_range[0]) / (v_range[1] - v_range[0]))
    mat = bpy.data.materials.new(name + "_mat")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(image, check_existing=True)
    mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    mat.node_tree.links.new(tex.outputs["Alpha"], bsdf.inputs["Alpha"])
    for attr, val in (("blend_method", "BLEND"), ("surface_render_method", "BLENDED")):
        try:
            setattr(mat, attr, val)
        except Exception:
            pass
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    return ob


def write_dae_with_uv(ob, out_path, texture_rel):
    """DAE with positions, normals, UVs and a textured material. Inputs use SEPARATE offsets
    (VERTEX=0, NORMAL=1, TEXCOORD=2) — gz-common loads shared offsets as an empty mesh."""
    me = ob.data
    me.calc_loop_triangles()
    pos = [c for v in me.vertices for c in v.co]
    nrm = [c for v in me.vertices for c in v.normal]
    uvl = me.uv_layers.active.data
    uvs, p, k = [], [], 0
    for tri in me.loop_triangles:
        for li in tri.loops:
            vi = me.loops[li].vertex_index
            uvs += list(uvl[li].uv)
            p += [vi, vi, k]
            k += 1
    fmt = lambda a: " ".join(f"{x:.6g}" for x in a)  # noqa: E731
    n = ob.name
    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="tex_png" name="tex_png"><init_from>{texture_rel}</init_from></image></library_images>
  <library_effects><effect id="{n}-effect"><profile_COMMON>
    <newparam sid="tex-surface"><surface type="2D"><init_from>tex_png</init_from></surface></newparam>
    <newparam sid="tex-sampler"><sampler2D><source>tex-surface</source></sampler2D></newparam>
    <technique sid="common"><lambert>
      <diffuse><texture texture="tex-sampler" texcoord="UVMap"/></diffuse>
      <transparent opaque="A_ONE"><texture texture="tex-sampler" texcoord="UVMap"/></transparent>
    </lambert></technique></profile_COMMON></effect></library_effects>
  <library_materials><material id="{n}-material" name="{n}"><instance_effect url="#{n}-effect"/></material></library_materials>
  <library_geometries><geometry id="{n}-mesh" name="{n}"><mesh>
    <source id="{n}-positions"><float_array id="{n}-positions-array" count="{len(pos)}">{fmt(pos)}</float_array><technique_common><accessor source="#{n}-positions-array" count="{len(pos)//3}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="{n}-normals"><float_array id="{n}-normals-array" count="{len(nrm)}">{fmt(nrm)}</float_array><technique_common><accessor source="#{n}-normals-array" count="{len(nrm)//3}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="{n}-uv"><float_array id="{n}-uv-array" count="{len(uvs)}">{fmt(uvs)}</float_array><technique_common><accessor source="#{n}-uv-array" count="{len(uvs)//2}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="{n}-vertices"><input semantic="POSITION" source="#{n}-positions"/></vertices>
    <triangles material="{n}-material" count="{len(me.loop_triangles)}">
      <input semantic="VERTEX" source="#{n}-vertices" offset="0"/>
      <input semantic="NORMAL" source="#{n}-normals" offset="1"/>
      <input semantic="TEXCOORD" source="#{n}-uv" offset="2" set="0"/>
      <p>{" ".join(map(str, p))}</p>
    </triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="Scene" name="Scene"><node id="{n}" name="{n}" type="NODE">
    <matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
    <instance_geometry url="#{n}-mesh" name="{n}"><bind_material><technique_common>
      <instance_material symbol="{n}-material" target="#{n}-material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material>
    </technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>'''
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(dae)
    return out_path
