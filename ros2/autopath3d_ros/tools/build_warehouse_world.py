#!/usr/bin/env python3
"""Generate the AutoPath Warehouse Webots world from the official Webots `factory.wbt`.

Source: Webots R2025a, projects/samples/environments/factory/worlds/factory.wbt (Apache-2.0,
Cyberbotics). The building (floor, ceiling, 4 walls, doors, mezzanine, stairs, wall pipes and
valves, cabinets, oil barrels, signs, lights, background) is copied verbatim, except:
  * removed: the tabletop robots (Darwin-op, Aibo), the central table with its tools and the
    loose boxes/pallets of the original scene (replaced by the warehouse layout below);
  * the mezzanine platform (a visual-only Solid in the original) gets a boundingObject equal
    to its visual box, so the drone cannot fly through a visible platform.
Added (all with explicit boundingObjects): 8 pallet-rack segments in 4 rows split by a cross
aisle, cargo (official WoodenPallet / CardboardBox / WoodenBox PROTOs, static), steel columns,
an overhead ventilation duct with hangers, roof beams, workbenches (official Table PROTO), a
pallet stack, a fenced storage cage with a sliding gate, a parked forklift (official Forklift
PROTO, physics disabled, moved kinematically by the scene supervisor), a hidden "dropped load",
the Crazyflie (extern controller) and the AutoPath scene-manager supervisor robot.

Deterministic: no randomness. Usage:
    python3 build_warehouse_world.py <factory.wbt> <output.wbt>
"""

from __future__ import annotations

import math
import re
import sys

VER = "R2025a"
URL = f"https://raw.githubusercontent.com/cyberbotics/webots/{VER}/projects"

# ---------------------------------------------------------------- layout (metres, z up)
ROOM = dict(x=(-9.9, 9.9), y=(-12.4, 3.72), z=(0.0, 7.0))   # interior faces of the factory walls
RACK_ROWS = [(-9.0, -8.0), (-6.0, -5.0), (-3.0, -2.0), (0.0, 1.0)]   # y bands (depth 1.0 m)
RACK_SEGMENTS = [(-7.0, -1.0), (1.0, 7.0)]                          # x extents, cross aisle |x| < 1
RACK_HEIGHT = 3.0
RACK_LEVELS = [1.0, 2.0]          # beam/deck heights above the floor level
BAY = 3.0
POST = 0.08
COLUMNS = [(-8.4, -6.5), (-8.4, -0.5), (8.3, -7.6), (-4.0, -10.9), (4.0, -10.9)]
DUCT = dict(y=(-10.0, -9.6), z=(2.3, 2.7))       # overhead duct along x, full room width
HANGERS_X = [-8.0, -4.0, 0.0, 4.0, 8.0]
ROOF_BEAMS_Y = [-6.5, -0.5]                      # z 6.2..6.5, above the flight geofence
# Door 1.4 m wide: with the 0.37 m trajectory clearance at 0.1 m voxels a 1.0 m door has no
# certifiable lattice node (offline test, 2026-11-10). The gate (same width) parks south of the
# door and must stay inside the room wall at y = -12.4: door_y[0] - width - 0.02 >= -12.4.
CAGE = dict(x=(6.6, 9.9), y=(-12.4, -9.4), h=2.4, door_y=(-10.96, -9.56), t=0.04)
DRONE_START = (-8.5, -11.0)


def fmt(*v):
    return " ".join(f"{x:.6g}" for x in v)


def appearance(rgb, metal=0.3, rough=0.5, transparency=0.0):
    t = f" transparency {transparency:.3g}" if transparency else ""
    return f"PBRAppearance {{ baseColor {fmt(*rgb)} metalness {metal} roughness {rough}{t} }}"


def box_shape(center, size, app):
    return (f"Pose {{ translation {fmt(*center)} children [ Shape {{ appearance {app} "
            f"geometry Box {{ size {fmt(*size)} }} }} ] }}")


def box_bo(center, size):
    return f"Pose {{ translation {fmt(*center)} children [ Box {{ size {fmt(*size)} }} ] }}"


def solid(name, translation, shapes, bos, extra="", def_name=None):
    """Static Solid (no physics) whose boundingObject is the given list of boxes."""
    d = f"DEF {def_name} " if def_name else ""
    bo = f"Group {{ children [ {' '.join(bos)} ] }}" if bos else "NULL"
    return (f"{d}Solid {{\n  translation {fmt(*translation)}\n  name \"{name}\"\n"
            f"  children [\n    " + "\n    ".join(shapes) + f"\n  ]\n  boundingObject {bo}\n{extra}}}\n")


# ---------------------------------------------------------------- generated elements
STEEL_BLUE = appearance((0.08, 0.22, 0.55), 0.6, 0.35)
BEAM_ORANGE = appearance((0.95, 0.42, 0.05), 0.3, 0.45)
DECK_GREY = appearance((0.55, 0.57, 0.6), 0.8, 0.5)
COLUMN_YELLOW = appearance((0.95, 0.75, 0.1), 0.4, 0.5)
DUCT_METAL = appearance((0.75, 0.77, 0.8), 0.9, 0.3)
CAGE_MESH = appearance((0.65, 0.67, 0.72), 0.7, 0.4, transparency=0.55)
CAGE_FRAME = appearance((0.25, 0.27, 0.3), 0.7, 0.4)
GATE = appearance((0.95, 0.6, 0.0), 0.4, 0.5)


def rack(idx, x0, x1, y0, y1):
    """Pallet rack: upright frames every BAY metres, front/back beams and wire decks per level.
    boundingObject = posts + beams + decks (the real structural members). The planner treats
    the whole rack as occupied (its AABB envelope), a conservative superset."""
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    L, D = x1 - x0, y1 - y0
    shapes, bos = [], []
    nb = int(round(L / BAY))
    for k in range(nb + 1):
        x = -L / 2 + k * BAY
        x = min(max(x, -L / 2 + POST / 2), L / 2 - POST / 2)
        for y in (-D / 2 + POST / 2, D / 2 - POST / 2):
            c, s = (x, y, RACK_HEIGHT / 2), (POST, POST, RACK_HEIGHT)
            shapes.append(box_shape(c, s, STEEL_BLUE)); bos.append(box_bo(c, s))
        # frame bracing (visual; inside the rack envelope)
        for z in (0.5, 1.5, 2.5):
            shapes.append(box_shape((x, 0, z), (0.03, D - POST, 0.03), STEEL_BLUE))
    for z in RACK_LEVELS + [RACK_HEIGHT - 0.06]:
        for y in (-D / 2 + 0.025, D / 2 - 0.025):
            c, s = (0, y, z), (L, 0.05, 0.1)
            shapes.append(box_shape(c, s, BEAM_ORANGE)); bos.append(box_bo(c, s))
        if z < RACK_HEIGHT - 0.1:
            c, s = (0, 0, z + 0.06), (L - 0.02, D - 0.1, 0.02)
            shapes.append(box_shape(c, s, DECK_GREY)); bos.append(box_bo(c, s))
    return solid(f"rack {idx}", (cx, cy, 0), shapes, bos)


def cargo(idx, x0, x1, y0, y1):
    """Static official PROTO cargo on each bay level (deterministic pattern)."""
    out = []
    cy = 0.5 * (y0 + y1)
    nb = int(round((x1 - x0) / BAY))
    levels = [0.0] + [z + 0.07 for z in RACK_LEVELS]
    n = 0
    for b in range(nb):
        bx = x0 + (b + 0.5) * BAY
        for li, z in enumerate(levels):
            pattern = (idx + b + li) % 4
            if pattern == 3:
                continue                    # empty slot
            for side in (-0.7, 0.7):
                px = bx + side
                out.append(f"WoodenPallet {{ translation {fmt(px, cy, z)} rotation 0 0 1 1.5708 "
                           f"name \"pallet r{idx}b{b}l{li}{'ab'[side > 0]}\" size 0.9 1.1 0.14 }}")
                h = (0.55, 0.75, 0.45)[pattern] if side < 0 else (0.7, 0.4, 0.6)[pattern]
                kind = "CardboardBox" if (pattern + (side > 0)) % 2 == 0 else "WoodenBox"
                out.append(f"{kind} {{ translation {fmt(px, cy, z + 0.14 + h / 2)} "
                           f"name \"cargo {idx}-{n}\" size {fmt(0.8, 0.8, h)} }}")
                n += 1
    return out


def column(i, x, y):
    c, s = (0, 0, 3.5), (0.3, 0.3, 7.0)
    return solid(f"column {i}", (x, y, 0), [box_shape(c, s, COLUMN_YELLOW)], [box_bo(c, s)])


def duct():
    (y0, y1), (z0, z1) = DUCT["y"], DUCT["z"]
    L = ROOM["x"][1] - ROOM["x"][0]
    c, s = (0, 0.5 * (y0 + y1), 0.5 * (z0 + z1)), (L, y1 - y0, z1 - z0)
    shapes, bos = [box_shape(c, s, DUCT_METAL)], [box_bo(c, s)]
    for x in HANGERS_X:
        hc, hs = (x, 0.5 * (y0 + y1), 0.5 * (z1 + 7.0)), (0.04, 0.04, 7.0 - z1)
        shapes.append(box_shape(hc, hs, DECK_GREY)); bos.append(box_bo(hc, hs))
    return solid("ventilation duct", (0, 0, 0), shapes, bos)


def roof_beams():
    out = []
    for i, y in enumerate(ROOF_BEAMS_Y):
        c, s = (0, y, 6.35), (19.8, 0.25, 0.3)
        out.append(solid(f"roof beam {i}", (0, 0, 0), [box_shape(c, s, STEEL_BLUE)], [box_bo(c, s)]))
    return out


def cage():
    (x0, x1), (y0, y1), h, t = CAGE["x"], CAGE["y"], CAGE["h"], CAGE["t"]
    d0, d1 = CAGE["door_y"]
    panels = [  # (center, size): west wall split by the door, north wall, roof
        ((x0 + t / 2, 0.5 * (y0 + d0), h / 2), (t, d0 - y0, h)),
        ((x0 + t / 2, 0.5 * (d1 + y1), h / 2), (t, y1 - d1, h)),
        ((x0 + t / 2, 0.5 * (d0 + d1), h - 0.15), (t, d1 - d0, 0.3)),       # door lintel
        ((0.5 * (x0 + x1), y1 - t / 2, h / 2), (x1 - x0, t, h)),
        ((0.5 * (x0 + x1), 0.5 * (y0 + y1), h + t / 2), (x1 - x0, y1 - y0, t)),
    ]
    shapes, bos = [], []
    for c, s in panels:
        shapes.append(box_shape(c, s, CAGE_MESH)); bos.append(box_bo(c, s))
    for px, py in [(x0, y1), (x1, y1), (x0, d0), (x0, d1), (x0, y0)]:
        c = (px + (0.03 if px == x0 else -0.03), py + (-0.03 if py == y1 else 0.03 if py == y0 else 0), h / 2)
        shapes.append(box_shape(c, (0.06, 0.06, h), CAGE_FRAME)); bos.append(box_bo(c, (0.06, 0.06, h)))
    return solid("storage cage", (0, 0, 0), shapes, bos)


def gate():
    """Sliding gate, parked outside the cage wall south of the door; slides +y to close it."""
    (x0, _), (y0, _), h = CAGE["x"], CAGE["y"], CAGE["h"]
    d0, d1 = CAGE["door_y"]
    w = d1 - d0
    c, s = (0, 0, (h - 0.3) / 2), (0.05, w, h - 0.3)
    return solid("cage gate", (x0 - 0.06, d0 - w / 2 - 0.02, 0), [box_shape(c, s, GATE)], [box_bo(c, s)],
                 def_name="CAGE_GATE")


def dropped_load():
    """Hidden below the floor until the unsafe-stop scenario places it in an aisle."""
    s = (0.8, 0.8, 1.9)
    shapes = [f"WoodenPallet {{ name \"drop pallet\" size 0.8 0.8 0.14 boundingObject FALSE }}",
              f"CardboardBox {{ translation 0 0 0.52 name \"drop box a\" size 0.78 0.78 0.76 }}",
              f"CardboardBox {{ translation 0 0 1.28 name \"drop box b\" size 0.7 0.7 0.76 }}"]
    return solid("dropped load", (0, -7.0, -5.0), shapes, [box_bo((0, 0, s[2] / 2), s)], def_name="DROP_LOAD")


def floor_cargo():
    return [
        "WoodenPalletStack { translation 3.0 -11.9 0 name \"pallet stack staging\" palletNumber 9 }",
        "WoodenPallet { translation -4.2 -11.85 0 name \"staging pallet a\" }",
        "CardboardBox { translation -4.2 -11.85 0.44 name \"staging box a\" size 1.0 0.7 0.6 }",
        "WoodenPallet { translation -2.6 -11.85 0 name \"staging pallet b\" }",
        "WoodenBox { translation -2.6 -11.85 0.54 name \"staging box b\" size 1.0 0.7 0.8 }",
        "Table { translation -6.3 -11.95 0 name \"workbench a\" size 1.8 0.8 0.9 }",
        "Table { translation 0.9 -11.95 0 name \"workbench b\" size 1.6 0.8 0.9 }",
        "OilBarrel { translation 9.3 -8.6 0.44 name \"oil barrel east\" }",
    ]


def look_at_rotation(eye, target):
    """Webots Viewpoint orientation (axis-angle) for a camera looking along local +x, z up."""
    f = [t - e for t, e in zip(target, eye)]
    n = math.sqrt(sum(c * c for c in f)); f = [c / n for c in f]
    up = (0, 0, 1)
    l = [up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0]]
    n = math.sqrt(sum(c * c for c in l)); l = [c / n for c in l]
    u = [f[1] * l[2] - f[2] * l[1], f[2] * l[0] - f[0] * l[2], f[0] * l[1] - f[1] * l[0]]
    R = [[f[0], l[0], u[0]], [f[1], l[1], u[1]], [f[2], l[2], u[2]]]
    tr = R[0][0] + R[1][1] + R[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))
    ax = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    n = math.sqrt(sum(c * c for c in ax)) or 1.0
    return [c / n for c in ax] + [ang]


# ---------------------------------------------------------------- factory.wbt surgery
REMOVE = {"Darwin-op", "AiboErs7", "Table", "CardboardBox", "PaintBucket", "Screwdriver", "Wrench", "Hammer",
          "WoodenPalletStack", "WoodenBox", "WoodenPallet"}
EXTRA_PROTOS = [
    f"{URL}/robots/bitcraze/crazyflie/protos/Crazyflie.proto",
    f"{URL}/objects/factory/pallet/protos/WoodenPallet.proto",
    f"{URL}/objects/factory/pallet/protos/WoodenPalletStack.proto",
    f"{URL}/objects/factory/containers/protos/CardboardBox.proto",
    f"{URL}/objects/factory/containers/protos/WoodenBox.proto",
    f"{URL}/objects/factory/forklift/protos/Forklift.proto",
    f"{URL}/objects/tables/protos/Table.proto",
    f"{URL}/objects/obstacles/protos/OilBarrel.proto",
]
DROP_PROTOS = ["Darwin-op.proto", "AiboErs7.proto", "Wrench.proto", "Hammer.proto", "PaintBucket.proto",
               "Screwdriver.proto"]


def split_top_level(text):
    """Split a .wbt body into top-level node blocks (column-0 open line ... column-0 '}')."""
    blocks, cur = [], []
    for line in text.splitlines():
        if not cur:
            if line.strip():
                cur = [line]
                if line.rstrip().endswith("}") and line.count("{") == line.count("}"):
                    blocks.append("\n".join(cur)); cur = []
            continue
        cur.append(line)
        if line == "}":
            blocks.append("\n".join(cur)); cur = []
    if cur:
        raise ValueError("unterminated top-level node")
    return blocks


def node_type(block):
    m = re.match(r"\s*(?:DEF \S+ )?([A-Za-z0-9_\-]+) \{", block)
    return m.group(1) if m else None


MEZZANINE_RE = re.compile(r"translation 9\.25 -3\.95 3\.21")


def build(factory_path, out_path):
    src = open(factory_path, encoding="utf-8").read()
    header, body = [], []
    for line in src.splitlines():
        (header if line.startswith(("#VRML", "EXTERNPROTO")) else body).append(line)
    protos = [l for l in header if l.startswith("EXTERNPROTO") and not any(d in l for d in DROP_PROTOS)]
    have = {re.search(r'"(.+)"', l).group(1) for l in protos}
    protos += [f'EXTERNPROTO "{u}"' for u in EXTRA_PROTOS if u not in have]

    kept, removed = [], []
    for b in split_top_level("\n".join(body)):
        t = node_type(b)
        if t in ("WorldInfo", "Viewpoint"):
            continue
        if t in REMOVE:
            removed.append(t); continue
        if t == "Solid" and MEZZANINE_RE.search(b):
            # give the visual-only mezzanine platform a boundingObject equal to its box
            b = b[: b.rstrip().rfind("}")] + "  boundingObject Box {\n    size 1.5 0.12 2.6\n  }\n  name \"mezzanine platform\"\n}"
        kept.append(b)

    eye, target = (-13.5, -19.0, 11.5), (0.5, -4.5, 1.0)
    rot = look_at_rotation(eye, target)
    out = ["#VRML_SIM R2025a utf8", "# AutoPath Warehouse - generated by tools/build_warehouse_world.py; do not edit by hand.",
           "# Base: Webots R2025a projects/samples/environments/factory/worlds/factory.wbt (Apache-2.0).",
           "# Original work: Cyberbotics Ltd., licensed under the Apache License 2.0",
           "# (http://www.apache.org/licenses/LICENSE-2.0; copy in LICENSES/Apache-2.0.txt).",
           "# MODIFIED by the AutoPath-3D project (2026): loose robots, tools and props removed; racks, cargo,",
           "# columns, duct, roof beams, cage and gate, forklift, Crazyflie and scene manager added; the",
           "# mezzanine platform given a boundingObject. This file is Apache-2.0, not covered by the",
           "# AutoPath-3D MIT licence; see THIRD_PARTY_NOTICES.md.", ""]
    out += protos + [""]
    out.append('WorldInfo {\n  info [\n    "AutoPath Warehouse - Industrial Autonomous Drone Navigation"\n'
               '    "AutoPath-3D Phase 4 benchmark world built on the official Webots factory environment."\n  ]\n'
               '  title "AutoPath Warehouse"\n  basicTimeStep 16\n}')
    out.append(f"Viewpoint {{\n  orientation {fmt(*rot)}\n  position {fmt(*eye)}\n  exposure 1.2\n}}")
    out += kept
    for i, (y0, y1) in enumerate(RACK_ROWS):
        for j, (x0, x1) in enumerate(RACK_SEGMENTS):
            k = 2 * i + j
            out.append(rack(k, x0, x1, y0, y1))
            out += cargo(k, x0, x1, y0, y1)
    out += [column(i, x, y) for i, (x, y) in enumerate(COLUMNS)]
    out.append(duct())
    out += roof_beams()
    out.append(cage())
    out.append(gate())
    out += floor_cargo()
    out.append('DEF FORKLIFT Forklift {\n  translation -4.6 -4.0 0.81\n  name "forklift"\n  enablePhysics FALSE\n}')
    out.append(dropped_load())
    out.append("PointLight {\n  attenuation 0 0 1\n  intensity 8\n  location 0 -9 5.5\n}")
    out.append("PointLight {\n  attenuation 0 0 1\n  intensity 6\n  location 4 -2 5.5\n}")
    out.append(f'DEF DRONE Crazyflie {{\n  translation {fmt(DRONE_START[0], DRONE_START[1], 0.015)}\n'
               f'  name "Crazyflie"\n  controller "<extern>"\n}}')
    out.append('DEF START_PAD Pose {\n  translation -8.5 -11.0 0.002\n  children [\n    Shape {\n'
               '      appearance PBRAppearance { baseColor 0.1 0.7 0.2 roughness 0.6 metalness 0 }\n'
               '      geometry Cylinder { height 0.003 radius 0.35 }\n    }\n  ]\n}')
    out.append('DEF AUTOPATH_VIS Group {\n}')
    out.append('DEF SCENE_MANAGER Robot {\n  translation 0 0 -2\n  name "AutoPathSceneManager"\n'
               '  controller "<extern>"\n  supervisor TRUE\n}')
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"removed from factory.wbt: {sorted(set(removed))} ({len(removed)} nodes); kept {len(kept)} nodes")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    build(sys.argv[1], sys.argv[2])
