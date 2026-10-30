import CMeshDenoiserCore
import Dispatch
import Foundation
import MeshDenoiserNative
import os

/// Bridges Task cancellation and progress callbacks across the C boundary.
private final class DenoiseSession: @unchecked Sendable {
    let progressHandler: (@Sendable (Double) -> Void)?
    private let cancelledLock = OSAllocatedUnfairLock(initialState: false)

    init(progressHandler: (@Sendable (Double) -> Void)?) {
        self.progressHandler = progressHandler
    }

    func cancel() { cancelledLock.withLock { $0 = true } }
    var isCancelled: Bool { cancelledLock.withLock { $0 } }
}

public enum MeshDenoiser {

    /// Denoises a triangle mesh, preserving vertex count and order.
    ///
    /// Runs the synchronous filter on a global queue; supports Task cancellation
    /// (checked between solver iterations) and reports fractional progress.
    /// - Returns: Denoised vertex positions, same count and order as `positions`.
    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters = MeshDenoiseParameters(),
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> [SIMD3<Float>] {
        let session = DenoiseSession(progressHandler: progress)
        let denoisedPositions = try await withTaskCancellationHandler {
            try await withCheckedThrowingContinuation { continuation in
                DispatchQueue.global(qos: .userInitiated).async {
                    continuation.resume(with: Result {
                        try runSync(positions: positions, indices: indices,
                                    parameters: parameters, session: session)
                    })
                }
            }
        } onCancel: {
            session.cancel()
        }
        try validateDenoisedPositions(denoisedPositions, expectedCount: positions.count)
        return denoisedPositions
    }

    private static func runSync(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters,
        session: DenoiseSession
    ) throws -> [SIMD3<Float>] {
        guard parameters.isValid else {
            throw MeshDenoiseError.invalidParameters
        }
        guard indices.count.isMultiple(of: 3) else {
            throw MeshDenoiseError.invalidInput
        }
        if session.isCancelled {
            throw CancellationError()
        }

        // ModelIO and glTF split vertices along UV/normal seams, and flat-shaded assets split
        // every triangle. Filter a welded copy so those seams are not treated as open
        // boundaries; keep the input topology if welding would make the mesh invalid.
        if let welded = WeldedMesh(positions: positions, indices: indices) {
            do {
                let denoised = try runBackend(
                    positions: welded.positions,
                    indices: welded.indices,
                    parameters: parameters,
                    session: session
                )
                return preserveCoincidentInputVertices(
                    welded.scatter(denoised),
                    originalPositions: positions
                )
            } catch MeshDenoiseError.invalidInput {
                // Fall back to filtering the mesh exactly as given.
            }
        }

        return preserveCoincidentInputVertices(
            try runBackend(positions: positions, indices: indices, parameters: parameters, session: session),
            originalPositions: positions
        )
    }

    private static func runBackend(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters,
        session: DenoiseSession
    ) throws -> [SIMD3<Float>] {
        switch parameters.backend {
        case .reference:
            return try runReferenceSync(
                positions: positions,
                indices: indices,
                parameters: parameters,
                session: session
            )
        case .nativeGPU where !NativeDenoiser.isGPUAvailable:
            throw MeshDenoiseError.gpuUnavailable
        case .automatic, .nativeGPU, .nativeCPU:
            let resolvedBackend = resolvedNativeBackend(
                for: parameters.backend,
                faceCount: indices.count / 3,
                isGPUAvailable: NativeDenoiser.isGPUAvailable
            )
            let useGPU = resolvedBackend == .nativeGPU
            var native = NativeDenoiseParameters()
            native.lambda = parameters.lambda
            native.eta = parameters.eta
            native.mu = parameters.mu
            native.nu = parameters.nu
            native.meshUpdateClosenessWeight = parameters.meshUpdateClosenessWeight
            native.meshUpdateIterations = parameters.meshUpdateIterations
            native.meshUpdateDisplacementEps = parameters.meshUpdateDisplacementEps
            native.outerIterations = parameters.outerIterations

            do {
                return try NativeDenoiser.denoise(
                    positions: positions,
                    indices: indices,
                    parameters: native,
                    useGPU: useGPU,
                    shouldCancel: { session.isCancelled },
                    progress: { completed, total in
                        if session.isCancelled { return false }
                        session.progressHandler?(Double(completed) / Double(total))
                        return true
                    }
                )
            } catch NativeDenoiseError.cancelled {
                throw CancellationError()
            } catch NativeDenoiseError.invalidInput {
                throw MeshDenoiseError.invalidInput
            } catch NativeDenoiseError.invalidParameters {
                throw MeshDenoiseError.invalidParameters
            } catch NativeDenoiseError.solverFailed {
                throw MeshDenoiseError.solverFailed
            } catch NativeDenoiseError.notImplemented {
                throw MeshDenoiseError.solverFailed
            }
        }
    }

    static func resolvedNativeBackend(
        for backend: MeshDenoiseParameters.Backend,
        faceCount _: Int,
        isGPUAvailable _: Bool
    ) -> MeshDenoiseParameters.Backend {
        switch backend {
        case .automatic:
            return .nativeCPU
        case .nativeGPU:
            return .nativeGPU
        case .nativeCPU:
            return .nativeCPU
        case .reference:
            return .reference
        }
    }

    private static func preserveCoincidentInputVertices(
        _ denoisedPositions: [SIMD3<Float>],
        originalPositions: [SIMD3<Float>],
        epsilon: Float = 1e-6
    ) -> [SIMD3<Float>] {
        guard denoisedPositions.count == originalPositions.count else {
            return denoisedPositions
        }

        // Representative (first) vertex of each coincidence group; summing in index order
        // keeps the result identical to averaging each group's members directly.
        var representativeByKey = [CoincidentPositionKey: Int](minimumCapacity: originalPositions.count)
        var representative = [Int](repeating: 0, count: originalPositions.count)
        var sums = [SIMD3<Float>](repeating: .zero, count: originalPositions.count)
        var counts = [Int](repeating: 0, count: originalPositions.count)
        for (index, position) in originalPositions.enumerated() {
            var group = index
            if let key = CoincidentPositionKey(position: position, epsilon: epsilon) {
                if let existing = representativeByKey[key] {
                    group = existing
                } else {
                    representativeByKey[key] = index
                }
            }
            representative[index] = group
            sums[group] += denoisedPositions[index]
            counts[group] += 1
        }

        var output = denoisedPositions
        for index in output.indices where counts[representative[index]] > 1 {
            output[index] = sums[representative[index]] / Float(counts[representative[index]])
        }
        return output
    }

    private static func validateDenoisedPositions(_ positions: [SIMD3<Float>], expectedCount: Int) throws {
        guard positions.count == expectedCount else {
            throw MeshDenoiseError.solverFailed
        }
        for position in positions {
            guard position.x.isFinite, position.y.isFinite, position.z.isFinite else {
                throw MeshDenoiseError.solverFailed
            }
        }
    }

    private static func runReferenceSync(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters,
        session: DenoiseSession
    ) throws -> [SIMD3<Float>] {
        var flat = [Float](repeating: 0, count: positions.count * 3)
        for (i, p) in positions.enumerated() {
            flat[3 * i] = p.x
            flat[3 * i + 1] = p.y
            flat[3 * i + 2] = p.z
        }
        var out = [Float](repeating: 0, count: flat.count)
        var cParams = parameters.cParams

        let callback: md_progress_fn = { completed, total, rawCtx in
            let session = Unmanaged<DenoiseSession>.fromOpaque(rawCtx!).takeUnretainedValue()
            if session.isCancelled { return false }
            session.progressHandler?(Double(completed) / Double(total))
            return true
        }
        let cancelCheck: md_cancel_fn = { rawCtx in
            Unmanaged<DenoiseSession>.fromOpaque(rawCtx!).takeUnretainedValue().isCancelled
        }

        let status = withExtendedLifetime(session) {
            md_denoise(flat, positions.count,
                       indices, indices.count / 3,
                       &cParams, &out,
                       callback, cancelCheck, Unmanaged.passUnretained(session).toOpaque())
        }

        switch status {
        case MD_OK:
            return (0..<positions.count).map {
                SIMD3<Float>(out[3 * $0], out[3 * $0 + 1], out[3 * $0 + 2])
            }
        case MD_CANCELLED:
            throw CancellationError()
        case MD_ERR_INVALID_INPUT:
            throw MeshDenoiseError.invalidInput
        case MD_ERR_INVALID_PARAMS:
            throw MeshDenoiseError.invalidParameters
        default:
            throw MeshDenoiseError.solverFailed
        }
    }
}

private struct CoincidentPositionKey: Hashable {
    var x: Int64
    var y: Int64
    var z: Int64

    /// Nil when the quantized position does not fit (non-finite or extreme coordinates).
    init?(position: SIMD3<Float>, epsilon: Float) {
        guard let x = Int64(exactly: (position.x / epsilon).rounded()),
              let y = Int64(exactly: (position.y / epsilon).rounded()),
              let z = Int64(exactly: (position.z / epsilon).rounded())
        else {
            return nil
        }
        self.x = x
        self.y = y
        self.z = z
    }
}

/// Vertices merged by exact position, plus the map back to the caller's vertices.
struct WeldedMesh {
    var positions: [SIMD3<Float>]
    var indices: [UInt32]
    /// Welded vertex index for each input vertex.
    var weldedIndexOfVertex: [UInt32]

    /// Returns nil when no vertices coincide, or when merging would leave a face with a
    /// repeated vertex (in which case the input is filtered as given).
    init?(positions inputPositions: [SIMD3<Float>], indices inputIndices: [UInt32]) {
        var indexOfPosition = [SIMD3<Float>: UInt32](minimumCapacity: inputPositions.count)
        var weldedPositions = [SIMD3<Float>]()
        weldedPositions.reserveCapacity(inputPositions.count)
        var weldedIndexOfVertex = [UInt32]()
        weldedIndexOfVertex.reserveCapacity(inputPositions.count)
        for position in inputPositions {
            if let existing = indexOfPosition[position] {
                weldedIndexOfVertex.append(existing)
            } else {
                let index = UInt32(weldedPositions.count)
                indexOfPosition[position] = index
                weldedPositions.append(position)
                weldedIndexOfVertex.append(index)
            }
        }
        guard weldedPositions.count < inputPositions.count else { return nil }

        var weldedIndices = [UInt32]()
        weldedIndices.reserveCapacity(inputIndices.count)
        for offset in stride(from: 0, to: inputIndices.count - 2, by: 3) {
            let a = inputIndices[offset], b = inputIndices[offset + 1], c = inputIndices[offset + 2]
            guard a < inputPositions.count, b < inputPositions.count, c < inputPositions.count else {
                return nil
            }
            let wa = weldedIndexOfVertex[Int(a)]
            let wb = weldedIndexOfVertex[Int(b)]
            let wc = weldedIndexOfVertex[Int(c)]
            guard wa != wb, wb != wc, wa != wc else { return nil }
            weldedIndices.append(wa)
            weldedIndices.append(wb)
            weldedIndices.append(wc)
        }

        positions = weldedPositions
        indices = weldedIndices
        self.weldedIndexOfVertex = weldedIndexOfVertex
    }

    /// Maps per-welded-vertex results back to the caller's vertex order.
    func scatter(_ weldedResult: [SIMD3<Float>]) -> [SIMD3<Float>] {
        weldedIndexOfVertex.map { weldedResult[Int($0)] }
    }
}
