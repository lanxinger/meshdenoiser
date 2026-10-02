#!/usr/bin/env python3
"""Check numerical golden parity and GLB/glTF import using generated fixtures."""

import argparse
import base64
import copy
import json
import math
from pathlib import Path
import subprocess
import struct
import tempfile
import unittest


def read_obj(path):
    positions, faces = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            positions.append(tuple(float(value) for value in fields[1:4]))
        elif fields and fields[0] == "f":
            faces.append(tuple(int(value.split("/")[0]) - 1 for value in fields[1:]))
    return positions, faces


def write_obj(path, positions, faces):
    with path.open("w") as output:
        for point in positions:
            output.write(
                "v " + " ".join(format(value, ".17g") for value in point) + "\n"
            )
        for face in faces:
            output.write("f " + " ".join(str(index + 1) for index in face) + "\n")


def mesh_document(positions, faces, component_type=5123, interleaved=False):
    data = bytearray()
    for point in positions:
        if interleaved:
            data.extend(struct.pack("<f", 123))
        data.extend(struct.pack("<3f", *point))
    position_length = len(data)
    index_format = {5121: "B", 5123: "H", 5125: "I"}[component_type]
    indices = [index for face in faces for index in face]
    data.extend(struct.pack("<" + index_format * len(indices), *indices))
    document = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
        "buffers": [{"byteLength": len(data)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": position_length},
            {
                "buffer": 0,
                "byteOffset": position_length,
                "byteLength": len(data) - position_length,
            },
        ],
        "accessors": [
            {
                "bufferView": 0,
                "byteOffset": 4 if interleaved else 0,
                "componentType": 5126,
                "count": len(positions),
                "type": "VEC3",
                "min": [min(point[i] for point in positions) for i in range(3)],
                "max": [max(point[i] for point in positions) for i in range(3)],
            },
            {
                "bufferView": 1,
                "componentType": component_type,
                "count": len(indices),
                "type": "SCALAR",
            },
        ],
    }
    if interleaved:
        document["bufferViews"][0]["byteStride"] = 16
    return document, bytes(data)


def write_glb(path, document, data):
    encoded = json.dumps(document, separators=(",", ":")).encode()
    encoded += b" " * (-len(encoded) % 4)
    padded = data + b"\0" * (-len(data) % 4)
    path.write_bytes(
        struct.pack("<3I", 0x46546C67, 2, 28 + len(encoded) + len(padded))
        + struct.pack("<2I", len(encoded), 0x4E4F534A)
        + encoded
        + struct.pack("<2I", len(padded), 0x004E4942)
        + padded
    )


class CLIParityTests(unittest.TestCase):
    def run_cli(self, source, output):
        result = subprocess.run(
            [str(EXECUTABLE), str(source), str(output), "--deterministic"],
            capture_output=True,
            text=True,
            timeout=60,
        )
        self.assertNotIn("AddressSanitizer", result.stderr)
        self.assertNotIn("runtime error:", result.stderr)
        return result

    def test_matches_golden_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "denoised.obj"
            result = self.run_cli(FIXTURES / "noisy_icosphere.obj", output)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            actual, faces = read_obj(output)
            expected, expected_faces = read_obj(FIXTURES / "golden_denoised.obj")
            self.assertEqual(len(actual), 162)
            self.assertEqual(faces, expected_faces)
            self.assertTrue(
                all(math.isfinite(value) for point in actual for value in point)
            )
            self.assertEqual(len(actual), len(expected))
            max_error = max(
                abs(got - want)
                for point, target in zip(actual, expected)
                for got, want in zip(point, target)
            )
            self.assertLess(
                max_error, 1e-6, f"CLI/golden maximum coordinate error: {max_error}"
            )

    def assert_import_parity(self, directory, asset, positions, faces):
        source = directory / "reference.obj"
        write_obj(source, positions, faces)
        reference_output = directory / "reference-denoised.obj"
        actual_output = directory / "asset-denoised.obj"
        for mesh, output in [(source, reference_output), (asset, actual_output)]:
            result = self.run_cli(mesh, output)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        expected, expected_faces = read_obj(reference_output)
        actual, actual_faces = read_obj(actual_output)
        self.assertEqual(len(actual), len(expected))
        self.assertEqual(len(actual_faces), len(expected_faces))
        self.assertTrue(
            all(math.isfinite(value) for point in actual for value in point)
        )
        # GLB import assigns vertex handles in first-use order. Compare the
        # triangle corners to avoid mistaking that permutation for a change.
        max_error = max(
            abs(actual[got][axis] - expected[want][axis])
            for face, target in zip(actual_faces, expected_faces)
            for got, want in zip(face, target)
            for axis in range(3)
        )
        self.assertLess(
            max_error, 1e-6, f"glTF/OBJ maximum coordinate error: {max_error}"
        )

    def test_glb_index_widths_and_interleaved_positions(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        for component_type in [5121, 5123, 5125]:
            for interleaved in [False, True]:
                with self.subTest(
                    component_type=component_type, interleaved=interleaved
                ):
                    with tempfile.TemporaryDirectory() as name:
                        directory = Path(name)
                        document, data = mesh_document(
                            positions, faces, component_type, interleaved
                        )
                        asset = directory / "mesh.glb"
                        write_glb(asset, document, data)
                        self.assert_import_parity(directory, asset, positions, faces)

    def test_gltf_data_uri_and_external_buffer(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        for external in [False, True]:
            with self.subTest(external=external):
                with tempfile.TemporaryDirectory() as name:
                    directory = Path(name)
                    document, data = mesh_document(positions, faces)
                    if external:
                        (directory / "mesh.bin").write_bytes(data)
                        uri = "mesh.bin"
                    else:
                        uri = (
                            "data:application/octet-stream;base64,"
                            + base64.b64encode(data).decode()
                        )
                    document["buffers"][0]["uri"] = uri
                    asset = directory / "mesh.gltf"
                    asset.write_text(json.dumps(document))
                    self.assert_import_parity(directory, asset, positions, faces)

    def test_glb_column_major_matrix_nested_trs_and_default_scene(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        document, data = mesh_document(positions, faces)
        document["scene"] = 1
        document["scenes"] = [{"nodes": []}, {"nodes": [0]}]
        document["nodes"] = [
            {
                "matrix": [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 1.25, -2.5, 0.75, 1],
                "children": [1],
            },
            {"mesh": 0, "translation": [0.125, 0.25, -0.5], "scale": [2, 3, 4]},
        ]
        transformed = [
            (1 - 3 * y, 2 * x - 2.375, 4 * z + 0.25) for x, y, z in positions
        ]
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            asset = directory / "nested.glb"
            write_glb(asset, document, data)
            self.assert_import_parity(directory, asset, transformed, faces)

    def test_glb_without_scenes_visits_only_roots(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        document, data = mesh_document(positions, faces)
        del document["scenes"]
        del document["scene"]
        document["nodes"] = [{"translation": [3, 5, 7], "children": [1]}, {"mesh": 0}]
        transformed = [(x + 3, y + 5, z + 7) for x, y, z in positions]
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            asset = directory / "roots.glb"
            write_glb(asset, document, data)
            self.assert_import_parity(directory, asset, transformed, faces)

    def test_glb_mesh_instances_keep_separate_vertices(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        document, data = mesh_document(positions, faces)
        document["nodes"] = [{"mesh": 0}, {"mesh": 0, "translation": [4, 0, 0]}]
        document["scenes"][0]["nodes"] = [0, 1]
        combined_positions = positions + [(x + 4, y, z) for x, y, z in positions]
        combined_faces = faces + [
            tuple(index + len(positions) for index in face) for face in faces
        ]
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            asset = directory / "instances.glb"
            write_glb(asset, document, data)
            self.assert_import_parity(
                directory, asset, combined_positions, combined_faces
            )

    def test_glb_deep_hierarchy_does_not_recurse(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        document, data = mesh_document(positions, faces)
        document["nodes"] = [{"children": [i + 1]} for i in range(1024)] + [{"mesh": 0}]
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            asset = directory / "deep.glb"
            write_glb(asset, document, data)
            self.assert_import_parity(directory, asset, positions, faces)

    def test_malformed_glbs_fail_without_output(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        valid_document, valid_data = mesh_document(positions, faces)
        for problem in [
            "truncated_file",
            "truncated_indices",
            "truncated_positions",
            "invalid_vertex",
            "invalid_accessor",
            "cyclic_nodes",
            "repeated_nodes",
            "nonfinite_positions",
        ]:
            with self.subTest(problem=problem):
                with tempfile.TemporaryDirectory() as name:
                    directory = Path(name)
                    document, data = (
                        copy.deepcopy(valid_document),
                        bytearray(valid_data),
                    )
                    if problem == "truncated_indices":
                        document["bufferViews"][1]["byteLength"] = 2
                    elif problem == "truncated_positions":
                        document["bufferViews"][0]["byteLength"] = 4
                    elif problem == "nonfinite_positions":
                        struct.pack_into("<f", data, 0, math.nan)
                    elif problem == "invalid_vertex":
                        struct.pack_into(
                            "<H",
                            data,
                            document["bufferViews"][1]["byteOffset"],
                            len(positions),
                        )
                    elif problem == "invalid_accessor":
                        document["meshes"][0]["primitives"][0]["attributes"][
                            "POSITION"
                        ] = 42
                    elif problem == "cyclic_nodes":
                        document["nodes"][0]["children"] = [0]
                    elif problem == "repeated_nodes":
                        document["nodes"] = [
                            {"children": [1, 2]},
                            {"children": [2]},
                            {"mesh": 0},
                        ]
                    asset = directory / "invalid.glb"
                    write_glb(asset, document, data)
                    if problem == "truncated_file":
                        asset.write_bytes(asset.read_bytes()[:-7])
                    output = directory / "output.obj"
                    result = self.run_cli(asset, output)
                    self.assertGreater(
                        result.returncode, 0, result.stdout + result.stderr
                    )
                    self.assertFalse(output.exists())
                    self.assertIn("Error", result.stderr)

    def test_unsupported_nonindexed_and_sparse_primitives_fail_explicitly(self):
        positions, faces = read_obj(FIXTURES / "noisy_icosphere.obj")
        for sparse in [False, True]:
            with self.subTest(sparse=sparse):
                with tempfile.TemporaryDirectory() as name:
                    directory = Path(name)
                    document, data = mesh_document(positions, faces)
                    if sparse:
                        document["accessors"][0]["sparse"] = {
                            "count": 1,
                            "indices": {"bufferView": 1, "componentType": 5123},
                            "values": {"bufferView": 0},
                        }
                    else:
                        del document["meshes"][0]["primitives"][0]["indices"]
                    asset = directory / "unsupported.glb"
                    write_glb(asset, document, data)
                    output = directory / "output.obj"
                    result = self.run_cli(asset, output)
                    self.assertGreater(
                        result.returncode, 0, result.stdout + result.stderr
                    )
                    self.assertFalse(output.exists())
                    message = (
                        "Unsupported POSITION"
                        if sparse
                        else "Indexed primitives are required"
                    )
                    self.assertIn(message, result.stderr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--fixtures", type=Path, required=True)
    args = parser.parse_args()
    EXECUTABLE = args.executable.resolve()
    FIXTURES = args.fixtures.resolve()
    unittest.main(argv=[__file__], verbosity=2)
