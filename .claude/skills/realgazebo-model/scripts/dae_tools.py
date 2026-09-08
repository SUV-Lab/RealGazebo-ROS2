"""DAE (Collada) utilities for RealGazebo models. Standard library only.

Runs under any Python 3, including Blender's:
  python dae_tools.py <command> ...
  blender -b --python dae_tools.py -- <command> ...

Commands
  centroid FILE...                    vertex centroid + extents per geometry (is it baked or origin-centered?)
  bake --offset X Y Z FILE...         translate vertex positions by (X,Y,Z) in place  (normals untouched)
  rotate --axis x|y|z --deg D FILE... rotate positions AND normals about an axis, in place
  fix-offsets FILE...                 give every <triangles>/<polylist> input its own offset
                                      (gz-common reads shared offsets as an EMPTY mesh)
  compare A.dae B.dae [--axis z]      are two meshes the same shape? (translation/rotation/mirror aware)
  hinge-axis FILE... [--span z|y]     swept-hinge axis of a BAKED control surface (leading-edge line)

All in-place commands keep the rest of the file byte-identical (regex patch on the float arrays).
"""
import math
import re
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True  # keep __pycache__ out of the skill folder when run headless
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
_FLOAT_ARRAY = re.compile(r'(<float_array id="([^"]*)"[^>]*>)([^<]+)(</float_array>)')


# ----------------------------------------------------------------------------- reading
def geometries(path):
    """Yield (name, positions[list of (x,y,z)]) for each geometry in a DAE."""
    root = ET.parse(path).getroot()
    for geom in root.findall(".//c:library_geometries/c:geometry", NS):
        mesh = geom.find("c:mesh", NS)
        verts = mesh.find("c:vertices", NS)
        src_id = verts.find("c:input[@semantic='POSITION']", NS).get("source").lstrip("#")
        arr = mesh.find(f"c:source[@id='{src_id}']/c:float_array", NS)
        f = [float(x) for x in arr.text.split()]
        yield (geom.get("name") or geom.get("id")), [tuple(f[i:i + 3]) for i in range(0, len(f), 3)]


def centroid(points):
    n = len(points)
    return tuple(sum(p[i] for p in points) / n for i in range(3))


def cmd_centroid(files):
    for path in files:
        for name, pts in geometries(path):
            c = centroid(pts)
            ext = [max(p[i] for p in pts) - min(p[i] for p in pts) for i in range(3)]
            m = max(abs(v) for v in c)
            kind = "origin-centered" if m < 0.002 else ("near origin (<2cm): centered OR baked with a small offset - check the SDF" if m < 0.02 else "BAKED")
            print(f"{path}: {name}: verts={len(pts)} centroid=({c[0]:.4f},{c[1]:.4f},{c[2]:.4f}) "
                  f"extent=({ext[0]:.3f},{ext[1]:.3f},{ext[2]:.3f}) -> {kind}")


# ----------------------------------------------------------------------------- patching
def array_ids(path):
    """(position float_array ids, normal float_array ids) resolved through the COLLADA inputs,
    so it works whatever the arrays are called."""
    root = ET.parse(path).getroot()
    pos, nrm = set(), set()
    for mesh in root.findall(".//c:mesh", NS):
        srcs = {s.get("id"): s for s in mesh.findall("c:source", NS)}

        def fa(ref):
            s = srcs.get(ref.lstrip("#"))
            a = s.find("c:float_array", NS) if s is not None else None
            return a.get("id") if a is not None else None

        for inp in mesh.findall("c:vertices/c:input", NS):
            if inp.get("semantic") == "POSITION":
                pos.add(fa(inp.get("source")))
        for prim in mesh.findall("c:triangles", NS) + mesh.findall("c:polylist", NS):
            for inp in prim.findall("c:input", NS):
                if inp.get("semantic") == "NORMAL":
                    nrm.add(fa(inp.get("source")))
    return pos, nrm


def _patch_arrays(path, selector, fn):
    """Apply fn(list[float]) -> list[float] to every float_array whose id matches selector(id)."""
    with open(path, encoding="utf-8") as fh:
        txt = fh.read()
    count = [0]

    def repl(m):
        if not selector(m.group(2)):
            return m.group(0)
        vals = fn([float(x) for x in m.group(3).split()])
        count[0] += 1
        return m.group(1) + " ".join(f"{v:.7g}" for v in vals) + m.group(4)

    new = _FLOAT_ARRAY.sub(repl, txt)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(new)
    return count[0]


def cmd_bake(offset, files):
    ox, oy, oz = offset
    for path in files:
        pos, _ = array_ids(path)
        n = _patch_arrays(path, lambda i: i in pos,
                          lambda v: [x + (ox, oy, oz)[k % 3] for k, x in enumerate(v)])
        print(f"{path}: baked offset ({ox},{oy},{oz}) into {n} position array(s)")
        if n == 0:
            raise SystemExit(f"ERROR: no position array found in {path}")


def rotation_matrix(axis, deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return {"x": [[1, 0, 0], [0, c, -s], [0, s, c]],
            "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
            "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]


def cmd_rotate(axis, deg, files):
    R = rotation_matrix(axis, deg)

    def rot(v):
        out = []
        for i in range(0, len(v), 3):
            x, y, z = v[i:i + 3]
            out += [R[r][0] * x + R[r][1] * y + R[r][2] * z for r in range(3)]
        return out

    for path in files:
        pos, nrm = array_ids(path)
        n = _patch_arrays(path, lambda i: i in pos or i in nrm, rot)
        print(f"{path}: rotated {deg} deg about {axis} ({n} arrays: positions+normals)")
        if n == 0:
            raise SystemExit(f"ERROR: no position/normal arrays found in {path}")


# ----------------------------------------------------------------------------- offsets
_PRIM = re.compile(r"(<(triangles|polylist)\b[^>]*>)(.*?)(</\2>)", re.S)
_INPUT = re.compile(r'<input\s+([^>]*?)/>')
_ATTR = re.compile(r'(\w+)="([^"]*)"')


def _fix_primitive(block):
    head, kind, body, tail = block.group(1), block.group(2), block.group(3), block.group(4)
    inputs = list(_INPUT.finditer(body))
    attrs = [dict(_ATTR.findall(m.group(1))) for m in inputs]
    old_offsets = [int(a.get("offset", "0")) for a in attrs]
    if len(set(old_offsets)) == len(old_offsets):
        return block.group(0)  # already unique
    old_stride = max(old_offsets) + 1
    new_offsets = list(range(len(inputs)))
    p_match = re.search(r"<p>([^<]*)</p>", body)
    idx = [int(x) for x in p_match.group(1).split()]
    out = []
    for i in range(0, len(idx), old_stride):
        tup = idx[i:i + old_stride]
        out += [tup[o] for o in old_offsets]
    new_body = body
    for m, a, no in reversed(list(zip(inputs, attrs, new_offsets))):
        a["offset"] = str(no)
        tag = "<input " + " ".join(f'{k}="{v}"' for k, v in a.items()) + "/>"
        new_body = new_body[:m.start()] + tag + new_body[m.end():]
    new_body = re.sub(r"<p>[^<]*</p>", "<p>" + " ".join(map(str, out)) + "</p>", new_body)
    return head + new_body + tail


def cmd_fix_offsets(files):
    for path in files:
        with open(path, encoding="utf-8") as fh:
            txt = fh.read()
        new = _PRIM.sub(_fix_primitive, txt)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(new)
        print(f"{path}: {'rewritten' if new != txt else 'already ok'}")


# ----------------------------------------------------------------------------- compare
def _nn_rms(a, b):
    s = 0.0
    for p in a:
        s += min((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2 for q in b)
    return math.sqrt(s / len(a))


def _sample(pts, n=600):
    step = max(1, len(pts) // n)
    return pts[::step]


def cmd_compare(a, b, axis="z"):
    A = _sample([tuple(p[i] - c for i, c in enumerate(centroid(pa))) for _, pa in geometries(a) for p in pa])
    B = _sample([tuple(p[i] - c for i, c in enumerate(centroid(pb))) for _, pb in geometries(b) for p in pb])
    def best_fit(mirror):
        best = (1e9, None)
        M = [tuple(-v if ax == mirror else v for ax, v in zip("xyz", p)) for p in A]
        for deg in range(0, 360, 10):
            R = rotation_matrix(axis, deg)
            Rm = [tuple(sum(R[r][k] * p[k] for k in range(3)) for r in range(3)) for p in M]
            d = _nn_rms(Rm, B)
            if d < best[0]:
                best = (d, deg)
        return best

    same = best_fit(None)
    mirrored = min((best_fit("x") + ("x",), best_fit("y") + ("y",)), key=lambda t: t[0])
    TOL = 0.006  # 6 mm nn-rms on a sampled cloud separates copies (~1-2 mm) from real differences (~9 mm+)
    if same[0] < TOL:
        verdict = f"SAME shape (rotated {same[1]} deg about {axis}) -> share ONE file"
    elif mirrored[0] < TOL:
        verdict = f"MIRROR image (mirror {mirrored[2]}, rot {mirrored[1]} deg) -> keep SEPARATE files (CW/CCW, left/right)"
    else:
        verdict = "DIFFERENT shapes"
    print(f"{a} vs {b}: same-fit={same[0]*1000:.2f}mm mirror-fit={mirrored[0]*1000:.2f}mm -> {verdict}")


# ----------------------------------------------------------------------------- hinge axis
def cmd_hinge_axis(files, span="z", slab=0.05):
    """Hinge axis of a swept control surface from its BAKED mesh: take a slab at each span end,
    the leading-edge point of each slab is its most-forward (max x) vertex, axis = normalized
    (high LE - low LE) with the lateral component zeroed. Use the swept axis when the sweep
    exceeds ~5 deg from the nominal span axis, else keep the nominal axis."""
    si = "xyz".index(span)
    lateral = {"z": 1, "y": 2}[span]
    for path in files:
        pts = [p for _, ps in geometries(path) for p in ps]
        lo, hi = min(p[si] for p in pts), max(p[si] for p in pts)
        le_lo = max((p for p in pts if p[si] <= lo + slab), key=lambda p: p[0])
        le_hi = max((p for p in pts if p[si] >= hi - slab), key=lambda p: p[0])
        d = [b - a for a, b in zip(le_lo, le_hi)]
        d[lateral] = 0.0
        n = math.sqrt(sum(x * x for x in d))
        axis = [x / n for x in d]
        sweep = math.degrees(math.acos(max(-1.0, min(1.0, axis[si]))))
        verdict = "swept -> use this axis" if sweep >= 5 else "nearly straight -> nominal axis is fine"
        print(f"{path}: LE low=({le_lo[0]:.3f},{le_lo[1]:.3f},{le_lo[2]:.3f}) high=({le_hi[0]:.3f},{le_hi[1]:.3f},{le_hi[2]:.3f}) "
              f"axis=({axis[0]:.3f} {axis[1]:.3f} {axis[2]:.3f}) sweep={sweep:.1f} deg  {verdict}")


# ----------------------------------------------------------------------------- cli
def main(argv):
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    if not argv:
        print(__doc__); return 1
    cmd, rest = argv[0], argv[1:]
    if cmd == "centroid":
        cmd_centroid(rest)
    elif cmd == "bake":
        i = rest.index("--offset"); off = tuple(float(x) for x in rest[i + 1:i + 4])
        cmd_bake(off, rest[:i] + rest[i + 4:])
    elif cmd == "rotate":
        ax = rest[rest.index("--axis") + 1]; deg = float(rest[rest.index("--deg") + 1])
        files = [f for f in rest if f.endswith(".dae")]
        cmd_rotate(ax, deg, files)
    elif cmd == "fix-offsets":
        cmd_fix_offsets(rest)
    elif cmd == "compare":
        ax = rest[rest.index("--axis") + 1] if "--axis" in rest else "z"
        files = [f for f in rest if f.endswith(".dae")]
        cmd_compare(files[0], files[1], ax)
    elif cmd == "hinge-axis":
        span = rest[rest.index("--span") + 1] if "--span" in rest else "z"
        slab = float(rest[rest.index("--slab") + 1]) if "--slab" in rest else 0.05
        cmd_hinge_axis([f for f in rest if f.endswith(".dae")], span, slab)
    else:
        print(__doc__); return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
