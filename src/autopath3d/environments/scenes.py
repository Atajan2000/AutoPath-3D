"""Deterministic synthetic 3D scenes built from analytic primitives.

Every scene is fully specified here (no randomness), so collision correctness can be
verified against exact ground-truth geometry. Units: metres, z up, ground at z = 0.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .primitives import Box, Primitive, VerticalCylinder, bounds_distance


@dataclass(frozen=True)
class Task:
    name: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]


@dataclass
class Scene:
    name: str
    description: str
    lo: np.ndarray
    hi: np.ndarray
    obstacles: list[Primitive]
    tasks: list[Task]
    unknown_regions: list[Box] = field(default_factory=list)

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Exact ground-truth distance to obstacles and bounds (<= 0 if invalid)."""
        p = np.atleast_2d(np.asarray(points, float))
        d = bounds_distance(p, self.lo, self.hi)
        for ob in self.obstacles:
            d = np.minimum(d, ob.distance(p))
        return d

    def task(self, name: str) -> Task:
        for t in self.tasks:
            if t.name == name:
                return t
        raise KeyError(name)


def _box(x0, y0, z0, x1, y1, z1, name="box") -> Box:
    return Box((x0, y0, z0), (x1, y1, z1), name)


def _slab_with_hole(x0, x1, y0, y1, z0, z1, hx0, hx1, hy0, hy1, name) -> list[Box]:
    """Horizontal slab [x0,x1]x[y0,y1]x[z0,z1] with rectangular hole (4 boxes)."""
    return [
        _box(x0, y0, z0, hx0, y1, z1, name + "_w"),
        _box(hx1, y0, z0, x1, y1, z1, name + "_e"),
        _box(hx0, y0, z0, hx1, hy0, z1, name + "_s"),
        _box(hx0, hy1, z0, hx1, y1, z1, name + "_n"),
    ]


def _wall_x_with_openings(x0, x1, y0, y1, z0, z1, openings, name) -> list[Box]:
    """Wall normal to x spanning [y0,y1]x[z0,z1]; openings are (ya, yb, za, zb)."""
    boxes = []
    ys = sorted({y0, y1, *[o[0] for o in openings], *[o[1] for o in openings]})
    for ya, yb in zip(ys[:-1], ys[1:]):
        zs = [(z0, z1)]
        for oy0, oy1, oz0, oz1 in openings:
            if oy0 <= ya and yb <= oy1:
                new = []
                for a, b in zs:
                    if oz0 > a:
                        new.append((a, min(b, oz0)))
                    if oz1 < b:
                        new.append((max(a, oz1), b))
                zs = [(a, b) for a, b in new if b > a]
        for a, b in zs:
            boxes.append(_box(x0, ya, a, x1, yb, b, f"{name}_{len(boxes)}"))
    return boxes


def warehouse() -> Scene:
    obs: list[Primitive] = []
    for k, yc in enumerate([4.0, 8.0, 12.0, 16.0]):
        y0, y1 = yc - 0.5, yc + 0.5
        if k in (1, 2):  # cross aisle through the middle rows
            obs += [_box(3, y0, 0, 14, y1, 6, f"rack{k}a"), _box(16, y0, 0, 27, y1, 6, f"rack{k}b")]
        else:
            obs.append(_box(3, y0, 0, 27, y1, 6, f"rack{k}"))
    obs += [
        _box(9.8, 4.5, 4.5, 10.2, 7.5, 4.8, "beam_a"),      # overhead beams across aisles
        _box(19.8, 12.5, 4.5, 20.2, 15.5, 4.8, "beam_b"),
        _box(21.8, 8.5, 3.0, 22.2, 11.5, 3.3, "beam_low"),
        _box(5, 1.0, 0, 25, 1.6, 1.0, "conveyor"),
        _box(12, 5.5, 0, 13.5, 6.5, 2.0, "pallets_a"),
        _box(18, 13.5, 0, 19.5, 14.5, 3.0, "pallets_b"),
        _box(6, 17.2, 0, 8, 18.8, 2.5, "pallets_c"),
    ]
    return Scene(
        "warehouse",
        "30x20x8 m hall: four 6 m rack rows, cross aisle, overhead beams, conveyor, pallets",
        np.array([0.0, 0, 0]), np.array([30.0, 20, 8]), obs,
        [Task("cross_warehouse", (1.5, 2.5, 1.5), (28.5, 18.0, 1.5)),
         Task("aisle_to_aisle", (8.0, 6.0, 1.0), (22.0, 14.0, 2.5))],
    )


_URBAN_HEIGHTS = [[12.0, 19.5, 8.0, 15.0],
                  [19.5, 10.0, 19.5, 6.0],
                  [9.0, 19.5, 14.0, 19.5],
                  [16.0, 7.0, 19.5, 11.0]]


def urban_canyon() -> Scene:
    obs: list[Primitive] = []
    blocks = [(2, 8), (12, 18), (22, 28), (32, 38)]
    for i, (x0, x1) in enumerate(blocks):
        for j, (y0, y1) in enumerate(blocks):
            obs.append(_box(x0, y0, 0, x1, y1, _URBAN_HEIGHTS[i][j], f"bldg_{i}{j}"))
    obs += [
        _box(8, 3, 6, 12, 7, 7.5, "skybridge_a"),
        _box(13, 18, 9, 17, 22, 10, "skybridge_b"),
        _box(28, 23, 12, 32, 27, 13, "skybridge_c"),
        _box(18, 14.8, 0, 22, 15.2, 6, "construction_wall"),
        _box(8, 24.97, 8.0, 12, 25.03, 8.06, "cable_a"),       # thin (6 cm) obstacles
        _box(28, 9.97, 5.0, 32, 10.03, 5.06, "cable_b"),
    ]
    return Scene(
        "urban_canyon",
        "40x40x20 m district: 16 buildings (6-19.5 m), skybridges, street wall, thin cables",
        np.array([0.0, 0, 0]), np.array([40.0, 40, 20]), obs,
        [Task("street_crossing", (1.0, 1.0, 2.0), (39.0, 39.0, 3.0)),
         Task("rooftop_landing", (1.0, 39.0, 2.0), (25.0, 5.0, 10.0))],
    )


def industrial() -> Scene:
    obs: list[Primitive] = []
    for x in (4.5, 10.0):
        for y in (6.0, 12.0, 18.0):
            obs.append(VerticalCylinder((x, y), 0.3, 0.0, 4.0, f"mezz_pillar_{x}_{y}"))
    for x in (20.0, 25.0):
        for y in (6.0, 12.0, 18.0):
            obs.append(VerticalCylinder((x, y), 0.3, 0.0, 10.0, f"pillar_{x}_{y}"))
    obs += _slab_with_hole(0, 14.85, 0, 24, 4.0, 4.3, 10.5, 13.5, 19, 22, "mezzanine")
    obs += _wall_x_with_openings(14.85, 15.15, 0, 24, 0, 10,
                                 [(5, 7, 0, 2.4), (17, 19, 6, 8)], "partition")
    obs += [
        _box(15.15, 11.95, 2.5, 30, 12.05, 2.6, "pipe_a"),   # thin (10 cm) pipes
        _box(15.15, 7.95, 3.5, 30, 8.05, 3.6, "pipe_b"),
        _box(21.95, 0, 6.0, 22.05, 24, 6.1, "pipe_c"),
        _box(7.45, 0, 2.8, 7.55, 24, 2.9, "pipe_d"),
        _box(17, 2, 0, 19.5, 5, 3, "machine_a"),
        _box(23, 14, 0, 27, 17, 4.5, "machine_b"),
        VerticalCylinder((26.0, 8.0), 1.5, 0.0, 7.0, "tank"),
    ]
    return Scene(
        "industrial",
        "30x24x10 m plant: mezzanine with stair hole, partition with door/window, pillars, thin pipes",
        np.array([0.0, 0, 0]), np.array([30.0, 24, 10]), obs,
        [Task("ground_to_mezzanine", (2.0, 2.0, 1.5), (5.0, 20.0, 6.5)),
         Task("cross_hall", (1.5, 22.0, 1.5), (29.0, 22.0, 1.5))],
    )


def multilevel() -> Scene:
    obs: list[Primitive] = []
    obs += _slab_with_hole(0, 20, 0, 20, 4.0, 4.3, 16, 19, 16, 19, "slab1")
    obs += _slab_with_hole(0, 20, 0, 20, 8.0, 8.3, 1, 4, 1, 4, "slab2")
    obs += _wall_x_with_openings(9.9, 10.1, 0, 20, 0, 4.0, [(2, 4, 0, 2.6)], "wall1")
    # Floor-2 wall normal to y: build as x-wall in swapped axes manually.
    obs += [_box(0, 9.9, 4.3, 15, 10.1, 8.0, "wall2_w"),
            _box(17, 9.9, 4.3, 20, 10.1, 8.0, "wall2_e"),
            _box(15, 9.9, 6.8, 17, 10.1, 8.0, "wall2_lintel")]
    obs += [VerticalCylinder((10.0, 10.0), 0.4, 8.3, 12.0, "col3_a"),
            VerticalCylinder((14.0, 14.0), 0.4, 8.3, 12.0, "col3_b")]
    return Scene(
        "multilevel",
        "20x20x12 m three-storey building: offset floor openings force vertical traversal",
        np.array([0.0, 0, 0]), np.array([20.0, 20, 12]), obs,
        [Task("ground_to_top", (2.0, 18.0, 1.5), (18.0, 18.0, 10.0)),
         Task("ground_to_second", (18.0, 2.0, 1.5), (2.0, 18.0, 6.0))],
    )


def narrow_passage(width: float) -> Scene:
    h = width / 2
    obs = _wall_x_with_openings(5.9, 6.1, 0, 8, 0, 4, [(4 - h, 4 + h, 2 - h, 2 + h)], "wall")
    return Scene(
        f"narrow_{int(round(width * 1000))}",
        f"12x8x4 m room split by a 0.2 m wall with one {width:.2f} m square window",
        np.array([0.0, 0, 0]), np.array([12.0, 8, 4]), obs,
        [Task("through_window", (2.0, 4.0, 2.0), (10.0, 4.0, 2.0))],
    )


SCENES = {
    "warehouse": warehouse,
    "urban_canyon": urban_canyon,
    "industrial": industrial,
    "multilevel": multilevel,
    "narrow_1200": lambda: narrow_passage(1.2),
    "narrow_900": lambda: narrow_passage(0.9),
    "narrow_500": lambda: narrow_passage(0.5),
}


def get_scene(name: str) -> Scene:
    if name not in SCENES:
        raise KeyError(f"unknown scene {name!r}; available: {sorted(SCENES)}")
    return SCENES[name]()
