#!/usr/bin/env python3
"""Generate a deterministic noisy icosphere OBJ for the kit/CLI parity test.

Coordinates are rounded to float32 before writing so the CLI (parsing as
double) and MeshDenoiserKit (Float input) see bit-identical positions.
"""
import math
import os
import struct


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


phi = (1 + 5 ** 0.5) / 2
verts = [normalize(v) for v in [
    (-1, phi, 0), (1, phi, 0), (-1, -phi, 0), (1, -phi, 0),
    (0, -1, phi), (0, 1, phi), (0, -1, -phi), (0, 1, -phi),
    (phi, 0, -1), (phi, 0, 1), (-phi, 0, -1), (-phi, 0, 1),
]]
faces = [
    (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
    (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
    (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
    (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
]

for _ in range(2):  # 2 subdivisions: 162 vertices, 320 faces
    midcache = {}

    def midpoint(a, b):
        key = (min(a, b), max(a, b))
        if key not in midcache:
            va, vb = verts[a], verts[b]
            verts.append(normalize(tuple((va[i] + vb[i]) / 2 for i in range(3))))
            midcache[key] = len(verts) - 1
        return midcache[key]

    new_faces = []
    for a, b, c in faces:
        ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
        new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
    faces = new_faces

state = 12345  # LCG for deterministic radial noise


def rand_unit():
    global state
    state = (state * 1103515245 + 12345) % (2 ** 31)
    return state / float(2 ** 31)


out_path = os.path.join(os.path.dirname(__file__), '..',
                        'Tests', 'MeshDenoiserKitTests', 'Fixtures', 'noisy_icosphere.obj')
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, 'w') as fh:
    for v in verts:
        r = 1.0 + (rand_unit() - 0.5) * 0.1  # +-5% radial noise
        fh.write('v %r %r %r\n' % tuple(f32(c * r) for c in v))
    for a, b, c in faces:
        fh.write('f %d %d %d\n' % (a + 1, b + 1, c + 1))
print('Wrote %s (%d vertices, %d faces)' % (out_path, len(verts), len(faces)))
