#!/usr/bin/env bash
# Vendors Eigen and OpenMesh Core into the repo for the SwiftPM package.
# Re-run to upgrade; versions are pinned below.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

EIGEN_URL="https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
OPENMESH_URL="https://www.graphics.rwth-aachen.de/media/openmesh_static/Releases/11.0/OpenMesh-11.0.0.tar.gz"

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
tar -xzf "$TMP/openmesh.tar.gz" -C "$TMP"
OM_DIR="$(echo "$TMP"/OpenMesh-*)"
OM_CORE="$OM_DIR/src/OpenMesh/Core"
DEST="$ROOT/Sources/OpenMeshCore"
rm -rf "$DEST/include" "$DEST/Core"
mkdir -p "$DEST/include/OpenMesh/Core" "$DEST/Core"

# Headers (tree preserved) -> include/OpenMesh/Core/...
(cd "$OM_CORE" && find . \( -name '*.hh' -o -name '*.h' \) -print | while read -r f; do
  mkdir -p "$DEST/include/OpenMesh/Core/$(dirname "$f")"
  cp "$f" "$DEST/include/OpenMesh/Core/$f"
done)

# Sources (tree preserved) -> Core/...
(cd "$OM_CORE" && find . -name '*.cc' -print | while read -r f; do
  mkdir -p "$DEST/Core/$(dirname "$f")"
  cp "$f" "$DEST/Core/$f"
done)

# License texts ship with the vendored code (App Store / attribution requirement)
cp "$ROOT/LICENSES/OpenMesh-LICENSE.txt" "$DEST/LICENSE.txt"

echo "Done. Eigen -> Vendor/eigen, OpenMesh Core -> Sources/OpenMeshCore"
