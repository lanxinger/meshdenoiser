/// Algorithm parameters for the native backend. Defaults match
/// `MeshDenoiserDefaults.txt`.
public struct NativeDenoiseParameters: Sendable, Equatable {
    public var lambda: Double = 0.15
    public var eta: Double = 2.2
    public var mu: Double = 0.2
    public var nu: Double = 0.25
    public var meshUpdateClosenessWeight: Double = 0.001
    public var meshUpdateIterations: Int = 20
    public var meshUpdateDisplacementEps: Double = 0.1
    public var outerIterations: Int = 1

    public init() {}

    var isValid: Bool {
        lambda > 0 && eta > 0 && mu > 0 && nu > 0
            && meshUpdateClosenessWeight >= 0
            && meshUpdateIterations > 0
            && meshUpdateDisplacementEps.isFinite
            && outerIterations > 0
    }
}

public enum NativeDenoiseError: Error, Equatable {
    case invalidInput
    case invalidParameters
    case solverFailed
    case cancelled
    case notImplemented
}

/// Compressed sparse rows: values[offsets[i]..<offsets[i+1]] belong to row i.
struct CSRAdjacency<Value: Sendable>: Sendable {
    var offsets: [UInt32]
    var values: [Value]

    func range(for row: Int) -> Range<Int> {
        Int(offsets[row])..<Int(offsets[row + 1])
    }
}

struct NativeMesh: Sendable {
    var positions: [SIMD3<Float>]
    var triangles: [SIMD3<UInt32>]
}
