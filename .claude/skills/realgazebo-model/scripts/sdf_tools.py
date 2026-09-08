"""SDF utilities for RealGazebo models. Standard library only; edits keep the file text intact
(targeted patches — comments and formatting survive).

  python sdf_tools.py inspect MODEL.sdf
  python sdf_tools.py origin-links MODEL.sdf --links left_ail,right_ail,rudder [--emit-bake bake.json]
        Convert servo links from "link at hinge" to the RealGazebo convention:
        link pose removed (origin) · inertial pose = hinge · visual pose = 0 · joint pose = hinge.
        Prints, per link, the offset to BAKE into its mesh (= old total visual position).
  python sdf_tools.py body-compensate MODEL.sdf [--link base_link] [--emit-bake bake.json]
        For visuals on base_link: visual pose = -(link pose), so a mesh baked in the MODEL frame lands once.
  python sdf_tools.py verify MODEL.sdf --meshes DIR
        For every mesh visual: (link position + visual pose) must be 0 when the mesh is baked,
        and the mesh must be origin-centered when it is not. Exit 1 on any failure.
  python sdf_tools.py unreal-table MODEL.sdf
        Rotor/pusher placement for Unreal (cm, Y sign flipped, pusher pitch -90).
  python sdf_tools.py visual-pose MODEL.sdf --visual NAME --pose "x y z r p y"
  python sdf_tools.py strip-deprecated MODEL.sdf        remove <use_parent_model_frame> (gone since SDF 1.7)
  python sdf_tools.py axis MODEL.sdf --joint NAME --xyz "x y z" [--expressed-in __model__|base_link]
  python sdf_tools.py add-visual MODEL.sdf --name NAME --mesh meshes/X.dae --albedo materials/textures/X.png
        [--link base_link] [--pose "x y z r p y"]      append a textured decal visual
"""
import json
import math
import os
import re
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dae_tools import centroid, geometries  # noqa: E402

POSE = re.compile(r"<pose\b([^>]*)>([^<]*)</pose>")
LINK_CHILD_TAGS = ("<inertial", "<visual", "<collision", "<sensor", "<gravity", "<velocity_decay",
                   "<self_collide", "<light", "<kinematic")


def _vec(text):
    v = [float(x) for x in text.split()]
    return v + [0.0] * (6 - len(v))


def _fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def _block(txt, tag, name):
    m = re.search(rf"<{tag}\s+name=[\"']{re.escape(name)}[\"'][^>]*>", txt)
    if not m:
        return None
    end = txt.index(f"</{tag}>", m.end()) + len(f"</{tag}>")
    return m.start(), end


def _block_by_child(txt, tag, child):
    for m in re.finditer(rf"<{tag}\b[^>]*>", txt):
        end = txt.index(f"</{tag}>", m.end()) + len(f"</{tag}>")
        if re.search(rf"<child>\s*{re.escape(child)}\s*</child>", txt[m.start():end]):
            return m.start(), end
    return None


def _link_level_pose(seg):
    """Match of the link's own <pose> (before any child element) or None."""
    cut = min([seg.find(t) for t in LINK_CHILD_TAGS if seg.find(t) != -1] or [len(seg)])
    m = POSE.search(seg, 0, cut)
    return m


def _model_pose_of_link(txt, link_name, base_link_pose):
    span = _block(txt, "link", link_name)
    if not span:
        raise SystemExit(f"link not found: {link_name}")
    seg = txt[span[0]:span[1]]
    m = _link_level_pose(seg)
    if not m:
        return [0.0] * 6
    v = _vec(m.group(2))
    if 'relative_to="base_link"' in m.group(1) or "relative_to='base_link'" in m.group(1):
        v = [a + b for a, b in zip(v[:3], base_link_pose[:3])] + v[3:]
    return v


def base_link_pose(txt):
    span = _block(txt, "link", "base_link")
    if not span:
        return [0.0] * 6
    m = _link_level_pose(txt[span[0]:span[1]])
    return _vec(m.group(2)) if m else [0.0] * 6


# ----------------------------------------------------------------------------- inspect
def iter_visuals(txt):
    for lm in re.finditer(r"<link\s+name=[\"']([^\"']+)[\"'][^>]*>", txt):
        lend = txt.index("</link>", lm.end())
        lseg = txt[lm.start():lend]
        for vm in re.finditer(r"<visual\s+name=[\"']([^\"']+)[\"'][^>]*>", lseg):
            vend = lseg.index("</visual>", vm.end())
            vseg = lseg[vm.start():vend]
            pm = POSE.search(vseg)
            um = re.search(r"<uri>([^<]+)</uri>", vseg)
            yield lm.group(1), vm.group(1), (_vec(pm.group(2)) if pm else [0.0] * 6), (um.group(1) if um else None)


def cmd_inspect(path):
    txt = open(path, encoding="utf-8").read()
    L = base_link_pose(txt)
    print(f"base_link pose: {_fmt(L)}")
    for lm in re.finditer(r"<link\s+name=[\"']([^\"']+)[\"'][^>]*>", txt):
        name = lm.group(1)
        lp = _model_pose_of_link(txt, name, L)
        print(f"link {name:22} model-frame pos=({lp[0]:.4f},{lp[1]:.4f},{lp[2]:.4f}) rpy=({lp[3]:.3f},{lp[4]:.3f},{lp[5]:.3f})")
    for link, vis, vp, uri in iter_visuals(txt):
        if uri:
            print(f"   visual {vis:26} pose=({vp[0]:.4f},{vp[1]:.4f},{vp[2]:.4f}) mesh={uri.split('/')[-1]}")
    for jm in re.finditer(r"<joint\s+name=[\"']([^\"']+)[\"'][^>]*>", txt):
        jend = txt.index("</joint>", jm.end())
        jseg = txt[jm.start():jend]
        child = re.search(r"<child>\s*([^<\s]+)\s*</child>", jseg)
        pm = POSE.search(jseg, 0, jseg.find("<axis") if "<axis" in jseg else len(jseg))
        print(f"joint {jm.group(1):24} child={child.group(1) if child else '?':18} pose={_fmt(_vec(pm.group(2))) if pm else '(none)'}")


# ----------------------------------------------------------------------------- patch helpers
def _set_first_pose(seg, new_vals, insert_after_tag=None, before_tag=None, keep_rpy=False):
    """Set the first <pose> in seg (optionally only before before_tag); insert one if missing."""
    limit = seg.find(before_tag) if before_tag and before_tag in seg else len(seg)
    m = POSE.search(seg, 0, limit)
    if m:
        vals = new_vals[:3] + (_vec(m.group(2))[3:] if keep_rpy else new_vals[3:])
        return seg[:m.start()] + f"<pose>{_fmt(vals)}</pose>" + seg[m.end():]
    anchor = re.search(insert_after_tag, seg) if insert_after_tag else None
    at = anchor.end() if anchor else seg.index(">") + 1
    indent = "\n" + " " * 8
    return seg[:at] + indent + f"<pose>{_fmt(new_vals)}</pose>" + seg[at:]


def _replace_span(txt, span, new_seg):
    return txt[:span[0]] + new_seg + txt[span[1]:]


def cmd_origin_links(path, links, emit=None):
    txt = open(path, encoding="utf-8").read()
    L = base_link_pose(txt)
    bake = {}
    for name in links:
        span = _block(txt, "link", name)
        if not span:
            raise SystemExit(f"link not found: {name}")
        seg = txt[span[0]:span[1]]
        hinge = _model_pose_of_link(txt, name, L)[:3]
        # 1) remove link-level pose
        m = _link_level_pose(seg)
        if m:
            seg = seg[:m.start()] + seg[m.end():]
            seg = re.sub(r"\n[ \t]*\n", "\n", seg, count=1)
        # 2) inertial pose = hinge
        im = re.search(r"<inertial>.*?</inertial>", seg, re.S)
        if im:
            iseg = _set_first_pose(im.group(0), hinge + [0, 0, 0], insert_after_tag=r"<inertial>")
            seg = seg[:im.start()] + iseg + seg[im.end():]
        # 3) each visual: record bake offset (= hinge + old visual pos), then pose = 0
        for vm in list(re.finditer(r"<visual\s+name=[\"']([^\"']+)[\"'][^>]*>.*?</visual>", seg, re.S))[::-1]:
            vseg = vm.group(0)
            pm = POSE.search(vseg)
            old = _vec(pm.group(2)) if pm else [0.0] * 6
            um = re.search(r"<uri>([^<]+)</uri>", vseg)
            if um:
                bake[um.group(1).split("/")[-1]] = [round(h + o, 6) for h, o in zip(hinge, old[:3])]
            vseg = _set_first_pose(vseg, [0, 0, 0, 0, 0, 0], insert_after_tag=r"<visual[^>]*>")
            seg = seg[:vm.start()] + vseg + seg[vm.end():]
        txt = _replace_span(txt, span, seg)
        # 4) joint whose child is this link: pose = hinge
        jspan = _block_by_child(txt, "joint", name)
        if not jspan:
            print(f"WARNING: no joint with child {name}")
        else:
            jseg = txt[jspan[0]:jspan[1]]
            jseg = _set_first_pose(jseg, hinge + [0, 0, 0], insert_after_tag=r"<child>[^<]*</child>", before_tag="<axis")
            txt = _replace_span(txt, jspan, jseg)
        print(f"{name}: hinge -> joint pose ({_fmt(hinge)}); bake offsets: "
              + ", ".join(f"{k}={v}" for k, v in bake.items() if k in txt))
    open(path, "w", encoding="utf-8").write(txt)
    if emit:
        _merge_json(emit, bake)
    return bake


def cmd_body_compensate(path, link="base_link", emit=None):
    txt = open(path, encoding="utf-8").read()
    L = base_link_pose(txt)[:3]
    span = _block(txt, "link", link)
    seg = txt[span[0]:span[1]]
    bake = {}
    for vm in list(re.finditer(r"<visual\s+name=[\"']([^\"']+)[\"'][^>]*>.*?</visual>", seg, re.S))[::-1]:
        vseg = vm.group(0)
        um = re.search(r"<uri>([^<]+)</uri>", vseg)
        if not um:
            continue
        pm = POSE.search(vseg)
        old = _vec(pm.group(2)) if pm else [0.0] * 6
        bake[um.group(1).split("/")[-1]] = [round(l + o, 6) for l, o in zip(L, old[:3])]
        vseg = _set_first_pose(vseg, [-x for x in L] + [0, 0, 0], insert_after_tag=r"<visual[^>]*>")
        seg = seg[:vm.start()] + vseg + seg[vm.end():]
    txt = _replace_span(txt, span, seg)
    open(path, "w", encoding="utf-8").write(txt)
    print(f"{link}: visual poses set to {_fmt([-x for x in L])}; bake offsets: {bake}")
    if emit:
        _merge_json(emit, bake)
    return bake


def default_bake(sdf_path):
    """bake.json lives next to model.sdf (absolute), so headless runs with an undefined cwd still find it."""
    return os.path.join(os.path.dirname(os.path.abspath(sdf_path)), "bake.json")


def _merge_json(path, data):
    cur = json.load(open(path)) if os.path.exists(path) else {}
    cur.update(data)
    json.dump(cur, open(path, "w"), indent=2)
    print(f"wrote {path}")


# ----------------------------------------------------------------------------- verify
def cmd_verify(path, mesh_dir, bake_json=None):
    """Baked visual: link+visual translation = 0 AND no rotation anywhere (link or visual).
    Origin-centered visual: must sit on a posed link. Which meshes are baked comes from bake.json
    (written by origin-links/body-compensate; default <model dir>/bake.json), else a centroid guess."""
    txt = open(path, encoding="utf-8").read()
    L = base_link_pose(txt)
    bake_json = bake_json or os.path.join(os.path.dirname(os.path.abspath(path)), "bake.json")
    baked_set = set(json.load(open(bake_json))) if os.path.exists(bake_json) else None
    print("baked-mesh list from:", bake_json if baked_set is not None else "centroid heuristic (no bake.json)")
    ok = True
    for link, vis, vp, uri in iter_visuals(txt):
        if not uri:
            continue
        fname = uri.split("/")[-1]
        mpath = os.path.join(mesh_dir, fname)
        if not os.path.exists(mpath):
            print(f"FAIL {vis}: mesh missing {mpath}"); ok = False; continue
        pts = [p for _, ps in geometries(mpath) for p in ps]
        c = centroid(pts)
        # bake.json is authoritative for what it lists; a mesh it doesn't know (e.g. a decal that was
        # authored already in the model frame) falls back to the centroid heuristic.
        baked = (baked_set is not None and fname in baked_set) or max(abs(x) for x in c) > 0.02
        lp = _model_pose_of_link(txt, link, L)
        total = [a + b for a, b in zip(lp[:3], vp[:3])]
        rot = max(abs(x) for x in lp[3:] + vp[3:])
        if baked:
            good = max(abs(x) for x in total) < 1e-6 and rot < 1e-6
            why = "" if good else ("  <- rotated: baked meshes take no link/visual rotation" if rot >= 1e-6 else "")
            print(f"{'OK  ' if good else 'FAIL'} {vis:26} baked mesh, link+visual=({total[0]:.6f},{total[1]:.6f},{total[2]:.6f}) (must be 0){why}")
        else:
            good = max(abs(x) for x in total) > 1e-6 or link == "base_link"
            print(f"{'OK  ' if good else 'FAIL'} {vis:26} origin-centered mesh placed at ({total[0]:.4f},{total[1]:.4f},{total[2]:.4f})"
                  + (f" rpy=({lp[3]:.2f},{lp[4]:.2f},{lp[5]:.2f})+vis({vp[3]:.2f},{vp[4]:.2f},{vp[5]:.2f})" if rot > 1e-6 else "")
                  + ("" if good else "  <- sits at the origin: bake it or give the link a pose"))
        ok &= good
    print("VERIFY:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def cmd_visual_pose(path, visual, pose):
    """Set one visual's pose, e.g. after re-modeling a pusher mesh to Z-spin: --pose "0 0 0 0 0 0"."""
    txt = open(path, encoding="utf-8").read()
    span = _block(txt, "visual", visual)
    if not span:
        raise SystemExit(f"visual not found: {visual}")
    seg = _set_first_pose(txt[span[0]:span[1]], _vec(pose), insert_after_tag=r"<visual[^>]*>")
    open(path, "w", encoding="utf-8").write(_replace_span(txt, span, seg))
    print(f"{visual}: pose = {pose}")


def cmd_strip_deprecated(path):
    """Remove <use_parent_model_frame> (deleted in SDF 1.7; gz warns and ignores it)."""
    txt = open(path, encoding="utf-8").read()
    new, n = re.subn(r"[ \t]*<use_parent_model_frame>[^<]*</use_parent_model_frame>[ \t]*\r?\n", "", txt)
    open(path, "w", encoding="utf-8").write(new)
    print(f"{path}: removed {n} <use_parent_model_frame>")


def cmd_axis(path, joint, xyz, expressed_in=None):
    """Set a joint's axis, e.g. pusher: --xyz "1 0 0" --expressed-in __model__ ;
    swept rudder hinge: --xyz "-0.544 0 0.839" --expressed-in base_link."""
    txt = open(path, encoding="utf-8").read()
    span = _block(txt, "joint", joint)
    if not span:
        raise SystemExit(f"joint not found: {joint}")
    attr = f' expressed_in="{expressed_in}"' if expressed_in else ""
    seg, n = re.subn(r"<xyz\b[^>]*>[^<]*</xyz>", f"<xyz{attr}>{xyz}</xyz>", txt[span[0]:span[1]], count=1)
    if n == 0:
        raise SystemExit(f"{joint}: no <xyz> in joint")
    open(path, "w", encoding="utf-8").write(_replace_span(txt, span, seg))
    print(f"{joint}: axis <xyz{attr}>{xyz}</xyz>")


def cmd_add_visual(path, link, name, mesh, albedo, pose):
    """Append a textured decal <visual> to a link (after its last <visual>).
    mesh/albedo are paths relative to the model folder; URIs use the model name from the file."""
    txt = open(path, encoding="utf-8").read()
    model = re.search(r"<model\s+name=[\"']([^\"']+)[\"']", txt).group(1)
    span = _block(txt, "link", link)
    if not span:
        raise SystemExit(f"link not found: {link}")
    seg = txt[span[0]:span[1]]
    last = seg.rfind("</visual>")
    if last == -1:
        raise SystemExit(f"{link}: no existing <visual> to anchor after")
    at = last + len("</visual>")
    nl = "\r\n" if "\r\n" in txt else "\n"
    block = nl.join([
        "", f"      <visual name='{name}'>",
        f"        <pose>{_fmt(_vec(pose))}</pose>",
        "        <geometry>", "          <mesh>", "            <scale>1 1 1</scale>",
        f"            <uri>model://{model}/{mesh.replace(os.sep, '/')}</uri>",
        "          </mesh>", "        </geometry>",
        "        <material>", "          <diffuse>1.0 1.0 1.0</diffuse>", "          <specular>0.2 0.2 0.2</specular>",
        "          <pbr>", "            <metal>",
        f"              <albedo_map>model://{model}/{albedo.replace(os.sep, '/')}</albedo_map>",
        "            </metal>", "          </pbr>", "        </material>", "      </visual>"])
    seg = seg[:at] + block + seg[at:]
    open(path, "w", encoding="utf-8").write(_replace_span(txt, span, seg))
    print(f"{link}: added visual {name} ({mesh}, albedo {albedo}, pose {pose})")


def cmd_unreal_table(path):
    txt = open(path, encoding="utf-8").read()
    L = base_link_pose(txt)
    print(f"{'link':22} {'mesh':16} {'X cm':>9} {'Y cm':>9} {'Z cm':>9}  Pitch")
    for link, vis, vp, uri in iter_visuals(txt):
        if not uri:
            continue
        lp = _model_pose_of_link(txt, link, L)
        pts = [p for _, ps in geometries(os.path.join(os.path.dirname(path), "meshes", uri.split("/")[-1])) for p in ps] \
            if os.path.exists(os.path.join(os.path.dirname(path), "meshes", uri.split("/")[-1])) else None
        if pts and max(abs(x) for x in centroid(pts)) > 0.02:
            continue  # baked parts go to 0,0,0
        pitch = -90 if abs(abs(lp[4]) - math.pi / 2) < 0.05 else 0
        print(f"{link:22} {uri.split('/')[-1]:16} {lp[0]*100:9.2f} {-lp[1]*100:9.2f} {lp[2]*100:9.2f}  {pitch}")
    print("(baked parts: place at 0,0,0 rotation 0)")
    motors = re.findall(r"<jointName>\s*([^<\s]+)\s*</jointName>.*?<motorNumber>\s*(\d+)\s*</motorNumber>", txt, re.S)
    if motors:
        print("RotatingComponents order (motorNumber):", ", ".join(f"{n}={j}" for j, n in sorted(motors, key=lambda t: int(t[1]))))
    servos = re.findall(r"JointPositionController.*?<joint_name>\s*([^<\s]+)\s*</joint_name>", txt, re.S)
    if servos:
        print("ControllableComponents order (JointPositionController):", ", ".join(servos), "- all at transform 0")


# ----------------------------------------------------------------------------- cli
def main(argv):
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    if not argv:
        print(__doc__); return 1
    cmd, path, rest = argv[0], argv[1], argv[2:]
    opt = lambda k, d=None: rest[rest.index(k) + 1] if k in rest else d  # noqa: E731
    if cmd == "inspect":
        cmd_inspect(path)
    elif cmd == "origin-links":
        cmd_origin_links(path, opt("--links").split(","), opt("--emit-bake", default_bake(path)))
    elif cmd == "body-compensate":
        cmd_body_compensate(path, opt("--link", "base_link"), opt("--emit-bake", default_bake(path)))
    elif cmd == "verify":
        return cmd_verify(path, opt("--meshes", os.path.join(os.path.dirname(os.path.abspath(path)), "meshes")), opt("--bake"))
    elif cmd == "unreal-table":
        cmd_unreal_table(path)
    elif cmd == "visual-pose":
        cmd_visual_pose(path, opt("--visual"), opt("--pose"))
    elif cmd == "strip-deprecated":
        cmd_strip_deprecated(path)
    elif cmd == "axis":
        cmd_axis(path, opt("--joint"), opt("--xyz"), opt("--expressed-in"))
    elif cmd == "add-visual":
        cmd_add_visual(path, opt("--link", "base_link"), opt("--name"), opt("--mesh"), opt("--albedo"), opt("--pose", "0 0 0 0 0 0"))
    else:
        print(__doc__); return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
