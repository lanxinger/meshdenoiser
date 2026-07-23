#!/usr/bin/env bash
# Vendors Eigen and OpenMesh Core into the repo for the SwiftPM package.
# Re-run to upgrade; versions are pinned below.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

EIGEN_URL="https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
# SwiftPM vendors this immutable Core snapshot. The CMake build remains pinned
# separately to the stable OpenMesh 11.0 release.
OPENMESH_REVISION="cb4e95287240faad1af58d45d525b97e0c65fdee"
OPENMESH_SHA256="8b80cb41cc857e6c826c9085f4aee03ad453314ba738d361e165456061778b81"
OPENMESH_URL="https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh/-/archive/$OPENMESH_REVISION/OpenMesh-$OPENMESH_REVISION.tar.gz"

echo "Fetching Eigen..."
curl -fsSL "$EIGEN_URL" -o "$TMP/eigen.tar.gz"
tar -xzf "$TMP/eigen.tar.gz" -C "$TMP"
EIGEN_DIR="$(echo "$TMP"/eigen-*)"
rm -rf "$ROOT/Vendor/eigen"
mkdir -p "$ROOT/Vendor/eigen"
cp -R "$EIGEN_DIR/Eigen" "$ROOT/Vendor/eigen/Eigen"
cp "$EIGEN_DIR"/COPYING.* "$ROOT/Vendor/eigen/" 2>/dev/null || true

echo "Fetching OpenMesh..."
curl -fsSL "$OPENMESH_URL" -o "$TMP/openmesh.tar.gz"
printf '%s  %s\n' "$OPENMESH_SHA256" "$TMP/openmesh.tar.gz" | shasum -a 256 -c -
tar -xzf "$TMP/openmesh.tar.gz" -C "$TMP"
OM_DIR="$(echo "$TMP"/OpenMesh-*)"
OM_CORE="$OM_DIR/src/OpenMesh/Core"
DEST="$ROOT/Sources/OpenMeshCore"
rm -rf "$DEST/Headers" "$DEST/Core"
mkdir -p "$DEST/Headers/OpenMesh/Core" "$DEST/Core"

# Headers (tree preserved) -> Headers/OpenMesh/Core/...
# NOTE: "Headers", not SPM's default public-headers dir "include" — these must
# remain textual includes, never a Clang module (see Package.swift comment).
(cd "$OM_CORE" && find . \( -name '*.hh' -o -name '*.h' \) -print | while read -r f; do
  mkdir -p "$DEST/Headers/OpenMesh/Core/$(dirname "$f")"
  cp "$f" "$DEST/Headers/OpenMesh/Core/$f"
done)

# Full tree (sources + headers) -> Core/...
# Headers are duplicated here so OpenMesh's quoted relative includes
# (e.g. #include "PropertyCreator.hh") resolve next to their .cc files.
cp -R "$OM_CORE/." "$DEST/Core/"

# License texts ship with the vendored code (App Store / attribution requirement)
cp "$ROOT/LICENSES/OpenMesh-LICENSE.txt" "$DEST/LICENSE.txt"

echo "Done. Eigen -> Vendor/eigen, OpenMesh Core -> Sources/OpenMeshCore"
