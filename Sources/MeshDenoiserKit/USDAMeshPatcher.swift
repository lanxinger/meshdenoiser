#if os(macOS)
import Foundation

/// Rewrites mesh points, and optionally normals, in a `usdcat`-generated USDA layer.
///
/// Meshes are matched to denoised ModelIO meshes by their original positions rather than
/// by order, so prims ModelIO skipped, curves, or reordered prims cannot receive another
/// mesh's geometry. Parsing works on UTF-8 bytes because USDA files for scanned meshes
/// are routinely hundreds of megabytes.
enum USDAMeshPatcher {
    struct UnmatchedMeshes: Error {}

    static func patch(
        _ text: [UInt8],
        denoisedMeshes: [MeshAssetDenoiser.DenoisedMeshPatch],
        recomputesNormals: Bool
    ) throws -> [UInt8] {
        let source = USDAText(text)
        var edits = [(range: Range<Int>, replacement: [UInt8])]()
        var matched = Array(repeating: false, count: denoisedMeshes.count)
        var parsedMeshPoints = [[SIMD3<Float>]]()

        for (ordinal, block) in source.meshBlocks().enumerated() {
            guard let pointsArray = source.arrayValue(named: "point3f[] points", in: block),
                  let usdPoints = source.parseFloat3Array(pointsArray)
            else {
                continue
            }
            parsedMeshPoints.append(usdPoints)

            // Prefer the mesh at the same position in ModelIO's order, then any other.
            let candidates = [ordinal] + Array(denoisedMeshes.indices).filter { $0 != ordinal }
            var newPoints: [SIMD3<Float>]?
            for candidate in candidates where denoisedMeshes.indices.contains(candidate) && !matched[candidate] {
                if let mapped = mapDenoisedPositions(denoisedMeshes[candidate], onto: usdPoints) {
                    matched[candidate] = true
                    newPoints = mapped
                    break
                }
            }
            guard let newPoints else { continue }
            edits.append((pointsArray.values, USDAText.tupleArray(newPoints)))

            guard recomputesNormals,
                  let countsArray = source.arrayValue(named: "int[] faceVertexCounts", in: block),
                  let indicesArray = source.arrayValue(named: "int[] faceVertexIndices", in: block),
                  let faceVertexCounts = source.parseIntArray(countsArray),
                  let faceVertexIndices = source.parseIntArray(indicesArray)
            else {
                continue
            }
            let topology = FaceTopology(counts: faceVertexCounts, indices: faceVertexIndices, pointCount: newPoints.count)
            for name in ["normal3f[] normals", "normal3f[] primvars:normals"] {
                guard let normalsArray = source.arrayValue(named: name, in: block),
                      let normals = source.parseFloat3Array(normalsArray),
                      let topology
                else {
                    continue
                }
                var normalIndices: [Int]?
                if name.hasPrefix("normal3f[] primvars:") {
                    if let array = source.arrayValue(named: "int[] primvars:normals:indices", in: block) {
                        guard let parsed = source.parseIntArray(array) else { continue }
                        normalIndices = parsed
                    }
                }
                if let recomputed = recomputeNormals(
                    values: normals,
                    indices: normalIndices,
                    interpolation: normalsArray.interpolation ?? "vertex",
                    positions: newPoints,
                    topology: topology
                ) {
                    edits.append((normalsArray.values, USDAText.tupleArray(recomputed)))
                }
            }
        }

        // A mesh ModelIO loaded once per instance of a shared prototype matches a patched block.
        for index in denoisedMeshes.indices where !matched[index] {
            guard parsedMeshPoints.contains(where: { mapDenoisedPositions(denoisedMeshes[index], onto: $0) != nil }) else {
                throw UnmatchedMeshes()
            }
        }

        return source.applying(edits)
    }

    /// Maps ModelIO's per-vertex results onto the USD points, or returns nil when the
    /// mesh's original positions are not this USD mesh. ModelIO may split USD points
    /// along UV/normal seams; split copies are averaged back into their USD point.
    static func mapDenoisedPositions(
        _ mesh: MeshAssetDenoiser.DenoisedMeshPatch,
        onto targetPositions: [SIMD3<Float>]
    ) -> [SIMD3<Float>]? {
        let original = mesh.originalPositions
        guard original.count == mesh.denoisedPositions.count, !original.isEmpty else { return nil }

        if original.count == targetPositions.count,
           zip(original, targetPositions).allSatisfy({ nearlyEqual($0, $1) }) {
            return mesh.denoisedPositions
        }

        var targetIndexByKey = [PositionKey: Int](minimumCapacity: targetPositions.count)
        for (index, position) in targetPositions.enumerated() {
            guard let key = PositionKey(position) else { return nil }
            if targetIndexByKey[key] == nil {
                targetIndexByKey[key] = index
            }
        }

        var sums = Array(repeating: SIMD3<Float>(0, 0, 0), count: targetPositions.count)
        var counts = Array(repeating: 0, count: targetPositions.count)
        for (originalPosition, denoisedPosition) in zip(original, mesh.denoisedPositions) {
            guard let key = PositionKey(originalPosition), let targetIndex = targetIndexByKey[key] else {
                return nil
            }
            sums[targetIndex] += denoisedPosition
            counts[targetIndex] += 1
        }

        var mapped = targetPositions
        for index in mapped.indices where counts[index] > 0 {
            mapped[index] = sums[index] / Float(counts[index])
        }
        return mapped
    }

    private static func nearlyEqual(_ lhs: SIMD3<Float>, _ rhs: SIMD3<Float>) -> Bool {
        let difference = lhs - rhs
        let tolerance = 1e-6 * max(1, max(abs(rhs.x), abs(rhs.y), abs(rhs.z)))
        return abs(difference.x) <= tolerance && abs(difference.y) <= tolerance && abs(difference.z) <= tolerance
    }

    private struct PositionKey: Hashable {
        var x: Int64
        var y: Int64
        var z: Int64

        init?(_ position: SIMD3<Float>) {
            guard let x = Int64(exactly: (Double(position.x) * 1e6).rounded()),
                  let y = Int64(exactly: (Double(position.y) * 1e6).rounded()),
                  let z = Int64(exactly: (Double(position.z) * 1e6).rounded())
            else {
                return nil
            }
            self.x = x
            self.y = y
            self.z = z
        }
    }

    struct FaceTopology {
        /// Corner range of each face within faceVertexIndices.
        var faceStarts: [Int]
        var counts: [Int]
        var indices: [Int]

        init?(counts: [Int], indices: [Int], pointCount: Int) {
            var faceStarts = [Int]()
            faceStarts.reserveCapacity(counts.count)
            var cursor = 0
            for count in counts {
                guard count >= 0, cursor + count <= indices.count else { return nil }
                faceStarts.append(cursor)
                cursor += count
            }
            guard cursor == indices.count, indices.allSatisfy({ (0..<pointCount).contains($0) }) else {
                return nil
            }
            self.faceStarts = faceStarts
            self.counts = counts
            self.indices = indices
        }
    }

    /// Recomputes normals from denoised positions while keeping the authored smoothing
    /// structure: corners that shared a normal value (same index, or same authored value at
    /// the same point) are smoothed together, so hard edges stay hard. Returns nil, leaving
    /// the authored normals untouched, for unsupported interpolations or mismatched counts.
    static func recomputeNormals(
        values: [SIMD3<Float>],
        indices: [Int]?,
        interpolation: String,
        positions: [SIMD3<Float>],
        topology: FaceTopology
    ) -> [SIMD3<Float>]? {
        let faceCount = topology.counts.count
        let elementCount: Int
        switch interpolation {
        case "vertex", "varying": elementCount = positions.count
        case "faceVarying": elementCount = topology.indices.count
        case "uniform": elementCount = faceCount
        default: return nil
        }

        // Group id of each corner, and the group each output value takes its normal from.
        var groupOfCorner = [Int](repeating: 0, count: topology.indices.count)
        let groupCount: Int
        var valueGroup: [Int]
        if let indices {
            guard indices.count == elementCount, indices.allSatisfy({ values.indices.contains($0) }) else { return nil }
            groupCount = values.count
            valueGroup = Array(0..<values.count)
            for face in 0..<faceCount {
                for corner in topology.faceStarts[face]..<(topology.faceStarts[face] + topology.counts[face]) {
                    groupOfCorner[corner] = indices[element(of: corner, face: face, interpolation: interpolation, topology: topology)]
                }
            }
        } else {
            guard values.count == elementCount else { return nil }
            if interpolation == "faceVarying" {
                // Unindexed face-varying normals: corners at one point with the same authored
                // normal belonged to one smooth group.
                struct GroupKey: Hashable { var point: Int; var normal: SIMD3<Float> }
                var groupByKey = [GroupKey: Int]()
                valueGroup = [Int](repeating: 0, count: values.count)
                for corner in topology.indices.indices {
                    let key = GroupKey(point: topology.indices[corner], normal: values[corner])
                    let group: Int
                    if let existing = groupByKey[key] {
                        group = existing
                    } else {
                        group = groupByKey.count
                        groupByKey[key] = group
                    }
                    groupOfCorner[corner] = group
                    valueGroup[corner] = group
                }
                groupCount = groupByKey.count
            } else {
                groupCount = values.count
                valueGroup = Array(0..<values.count)
                for face in 0..<faceCount {
                    for corner in topology.faceStarts[face]..<(topology.faceStarts[face] + topology.counts[face]) {
                        groupOfCorner[corner] = element(of: corner, face: face, interpolation: interpolation, topology: topology)
                    }
                }
            }
        }

        var sums = [SIMD3<Float>](repeating: .zero, count: groupCount)
        for face in 0..<faceCount {
            let start = topology.faceStarts[face]
            let count = topology.counts[face]
            guard count >= 3 else { continue }
            // Area-weighted normal of the fan-triangulated polygon.
            let origin = positions[topology.indices[start]]
            var faceNormal = SIMD3<Float>.zero
            for k in 1..<(count - 1) {
                faceNormal += cross(
                    positions[topology.indices[start + k]] - origin,
                    positions[topology.indices[start + k + 1]] - origin
                )
            }
            for corner in start..<(start + count) {
                let group = groupOfCorner[corner]
                // Add each face once per group, even if the group repeats within the face.
                if !(start..<corner).contains(where: { groupOfCorner[$0] == group }) {
                    sums[group] += faceNormal
                }
            }
        }

        return values.indices.map { valueIndex in
            let sum = sums[valueGroup[valueIndex]]
            let length = (sum * sum).sum().squareRoot()
            return length > 0 && length.isFinite ? sum / length : values[valueIndex]
        }
    }

    private static func element(of corner: Int, face: Int, interpolation: String, topology: FaceTopology) -> Int {
        switch interpolation {
        case "faceVarying": return corner
        case "uniform": return face
        default: return topology.indices[corner]
        }
    }

    private static func cross(_ lhs: SIMD3<Float>, _ rhs: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3<Float>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x
        )
    }
}

/// Byte-level view of a USDA layer as written by `usdcat` (Sdf's text writer).
struct USDAText {
    struct ArrayValue {
        /// The `[...]` range.
        var values: Range<Int>
        /// `interpolation` metadata following the value, if any.
        var interpolation: String?
    }

    private let bytes: [UInt8]
    private var count: Int { bytes.count }

    init(_ text: [UInt8]) {
        bytes = text
    }

    /// Ranges from each `def Mesh` header to the next prim header. Sdf writes a prim's
    /// properties before its children, so this covers all of the mesh's own attributes.
    func meshBlocks() -> [Range<Int>] {
        var blocks = [Range<Int>]()
        var searchStart = 0
        while let header = find(Array("def Mesh \"".utf8), in: searchStart..<count) {
            let end = nextPrimHeader(after: header) ?? count
            blocks.append(header..<end)
            searchStart = end
        }
        return blocks
    }

    /// Finds `<name> = [` within `block` and returns the array range plus metadata.
    func arrayValue(named name: String, in block: Range<Int>) -> ArrayValue? {
        let needle = Array((name + " =").utf8)
        var searchStart = block.lowerBound
        while let hit = find(needle, in: searchStart..<block.upperBound) {
            searchStart = hit + needle.count
            // Must be a whole statement, not the tail of a longer attribute name.
            guard hit == 0 || isSpace(bytes[hit - 1]) else { continue }
            var index = hit + needle.count
            while index < count, bytes[index] == UInt8(ascii: " ") { index += 1 }
            guard index < count, bytes[index] == UInt8(ascii: "[") else { return nil }
            guard let close = matchingBracket(from: index) else { return nil }
            return ArrayValue(values: index..<(close + 1), interpolation: interpolation(after: close + 1))
        }
        return nil
    }

    func parseFloat3Array(_ array: ArrayValue) -> [SIMD3<Float>]? {
        guard let numbers = parseNumbers(in: array.values), numbers.count.isMultiple(of: 3) else { return nil }
        var result = [SIMD3<Float>]()
        result.reserveCapacity(numbers.count / 3)
        for offset in stride(from: 0, to: numbers.count, by: 3) {
            result.append(SIMD3<Float>(Float(numbers[offset]), Float(numbers[offset + 1]), Float(numbers[offset + 2])))
        }
        return result
    }

    func parseIntArray(_ array: ArrayValue) -> [Int]? {
        guard let numbers = parseNumbers(in: array.values) else { return nil }
        var result = [Int]()
        result.reserveCapacity(numbers.count)
        for number in numbers {
            guard let value = Int(exactly: number) else { return nil }
            result.append(value)
        }
        return result
    }

    /// Returns the text with each range replaced; ranges must not overlap.
    func applying(_ edits: [(range: Range<Int>, replacement: [UInt8])]) -> [UInt8] {
        var output = [UInt8]()
        output.reserveCapacity(count)
        var cursor = 0
        for edit in edits.sorted(by: { $0.range.lowerBound < $1.range.lowerBound }) {
            output.append(contentsOf: bytes[cursor..<edit.range.lowerBound])
            output.append(contentsOf: edit.replacement)
            cursor = edit.range.upperBound
        }
        output.append(contentsOf: bytes[cursor..<count])
        return output
    }

    static func tupleArray(_ values: [SIMD3<Float>]) -> [UInt8] {
        var output = [UInt8]()
        output.reserveCapacity(values.count * 36)
        output.append(UInt8(ascii: "["))
        for (index, value) in values.enumerated() {
            if index > 0 { output.append(contentsOf: [UInt8(ascii: ","), UInt8(ascii: " ")]) }
            output.append(UInt8(ascii: "("))
            appendFloat(value.x, to: &output)
            output.append(contentsOf: [UInt8(ascii: ","), UInt8(ascii: " ")])
            appendFloat(value.y, to: &output)
            output.append(contentsOf: [UInt8(ascii: ","), UInt8(ascii: " ")])
            appendFloat(value.z, to: &output)
            output.append(UInt8(ascii: ")"))
        }
        output.append(UInt8(ascii: "]"))
        return output
    }

    /// Shortest round-trip form (locale-independent), with integral values written as integers.
    private static func appendFloat(_ value: Float, to output: inout [UInt8]) {
        var text = value.description
        if text.hasSuffix(".0") {
            text.removeLast(2)
        }
        output.append(contentsOf: text.utf8)
    }

    private func find(_ needle: [UInt8], in range: Range<Int>) -> Int? {
        guard !needle.isEmpty, range.count >= needle.count else { return nil }
        return bytes.withUnsafeBytes { haystack in
            needle.withUnsafeBytes { pattern in
                guard let base = haystack.baseAddress,
                      let hit = memmem(base + range.lowerBound, range.count, pattern.baseAddress!, pattern.count)
                else {
                    return nil
                }
                return base.distance(to: UnsafeRawPointer(hit))
            }
        }
    }

    /// Start of the next line whose first token is `def`, `over` or `class`.
    private func nextPrimHeader(after start: Int) -> Int? {
        var index = start
        while let newline = find([UInt8(ascii: "\n")], in: index..<count) {
            var token = newline + 1
            while token < count, bytes[token] == UInt8(ascii: " ") || bytes[token] == UInt8(ascii: "\t") {
                token += 1
            }
            for keyword in ["def ", "over ", "class "] where hasPrefix(Array(keyword.utf8), at: token) {
                return token
            }
            index = newline + 1
        }
        return nil
    }

    private func hasPrefix(_ prefix: [UInt8], at index: Int) -> Bool {
        index + prefix.count <= count && bytes[index..<(index + prefix.count)].elementsEqual(prefix)
    }

    private func matchingBracket(from open: Int) -> Int? {
        var depth = 0
        var index = open
        while index < count {
            switch bytes[index] {
            case UInt8(ascii: "["): depth += 1
            case UInt8(ascii: "]"):
                depth -= 1
                if depth == 0 { return index }
            default: break
            }
            index += 1
        }
        return nil
    }

    /// Reads `interpolation = "..."` from a `( ... )` metadata block right after a value.
    private func interpolation(after index: Int) -> String? {
        var cursor = index
        while cursor < count, bytes[cursor] == UInt8(ascii: " ") || bytes[cursor] == UInt8(ascii: "\t") {
            cursor += 1
        }
        guard cursor < count, bytes[cursor] == UInt8(ascii: "(") else { return nil }

        // Find the closing parenthesis, skipping quoted strings.
        var end = cursor + 1
        var depth = 1
        var inString = false
        while end < count, depth > 0 {
            let byte = bytes[end]
            if byte == UInt8(ascii: "\"") {
                inString.toggle()
            } else if !inString {
                if byte == UInt8(ascii: "(") { depth += 1 }
                if byte == UInt8(ascii: ")") { depth -= 1 }
            }
            end += 1
        }

        let key = Array("interpolation = \"".utf8)
        guard let hit = find(key, in: cursor..<end) else { return nil }
        let valueStart = hit + key.count
        guard let quote = find([UInt8(ascii: "\"")], in: valueStart..<end) else { return nil }
        return String(decoding: bytes[valueStart..<quote], as: UTF8.self)
    }

    /// Parses every number in `range`, allowing only USDA array punctuation between them.
    /// `Double(_: String)` parses in the C locale, whatever the process locale is.
    private func parseNumbers(in range: Range<Int>) -> [Double]? {
        bytes.withUnsafeBufferPointer { buffer -> [Double]? in
            var numbers = [Double]()
            var index = range.lowerBound
            while index < range.upperBound {
                if isDelimiter(buffer[index]) {
                    index += 1
                    continue
                }
                var end = index
                while end < range.upperBound, !isDelimiter(buffer[end]) {
                    end += 1
                }
                // Short tokens stay in String's inline storage, so this does not allocate.
                guard let value = Double(String(decoding: UnsafeBufferPointer(rebasing: buffer[index..<end]), as: UTF8.self)) else {
                    return nil
                }
                numbers.append(value)
                index = end
            }
            return numbers
        }
    }

    private func isDelimiter(_ byte: UInt8) -> Bool {
        switch byte {
        case UInt8(ascii: " "), UInt8(ascii: "\t"), UInt8(ascii: "\n"), UInt8(ascii: "\r"),
             UInt8(ascii: ","), UInt8(ascii: "("), UInt8(ascii: ")"), UInt8(ascii: "["), UInt8(ascii: "]"):
            return true
        default:
            return false
        }
    }

    private func isSpace(_ byte: UInt8) -> Bool {
        byte == UInt8(ascii: " ") || byte == UInt8(ascii: "\t") || byte == UInt8(ascii: "\n") || byte == UInt8(ascii: "\r")
    }
}
#endif
