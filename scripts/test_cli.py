#!/usr/bin/env python3
"""Check the CMake CLI against the committed numerical golden fixture."""

import argparse
import math
from pathlib import Path
import subprocess
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


class CLIParityTests(unittest.TestCase):
    def test_matches_golden_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "denoised.obj"
            result = subprocess.run(
                [str(EXECUTABLE), str(FIXTURES / "noisy_icosphere.obj"), str(output), "--deterministic"],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            actual, faces = read_obj(output)
            expected, expected_faces = read_obj(FIXTURES / "golden_denoised.obj")
            self.assertEqual(len(actual), 162)
            self.assertEqual(faces, expected_faces)
            self.assertTrue(all(math.isfinite(value) for point in actual for value in point))
            max_error = max(abs(got - want) for point, target in zip(actual, expected) for got, want in zip(point, target))
            self.assertLess(max_error, 1e-6, f"CLI/golden maximum coordinate error: {max_error}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--fixtures", type=Path, required=True)
    args = parser.parse_args()
    EXECUTABLE = args.executable.resolve()
    FIXTURES = args.fixtures.resolve()
    unittest.main(argv=[__file__], verbosity=2)
