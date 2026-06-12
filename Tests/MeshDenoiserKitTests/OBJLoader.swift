import Foundation

/// Minimal OBJ reader for test fixtures: `v x y z` and `f a b c` (1-based, no slashes) only.
enum OBJLoader {
    static func load(_ url: URL) throws -> (positions: [SIMD3<Double>], indices: [UInt32]) {
        var positions: [SIMD3<Double>] = []
        var indices: [UInt32] = []
        for line in try String(contentsOf: url, encoding: .utf8).split(separator: "\n") {
            let parts = line.split(separator: " ")
            guard parts.count == 4 else { continue }
            if parts[0] == "v" {
                guard let x = Double(parts[1]), let y = Double(parts[2]), let z = Double(parts[3]) else {
                    throw CocoaError(.fileReadCorruptFile)
                }
                positions.append([x, y, z])
            } else if parts[0] == "f" {
                guard let a = UInt32(parts[1]), let b = UInt32(parts[2]), let c = UInt32(parts[3]) else {
                    throw CocoaError(.fileReadCorruptFile)
                }
                indices += [a - 1, b - 1, c - 1]
            }
        }
        return (positions, indices)
    }
}
