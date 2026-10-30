import Foundation

enum FilterCPU {
    /// Faces per parallel work item. Fixed so the partition (and therefore the
    /// displacement reduction order) does not depend on the machine's core count.
    private static let chunkSize = 512

    static func run(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: FilterPrecompute,
        nu: Float,
        maxIterations: Int = 100,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        let faceCount = initialSignals.count
        guard faceCount > 0,
              faceCount == areaWeights.count,
              faceCount == precompute.weightedInitialSignals.count,
              nu.isFinite,
              nu > 0,
              maxIterations > 0
        else {
            throw NativeDenoiseError.invalidInput
        }
        guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }

        // Gather needs per-face rows. Streaming builds provide them; a precompute that
        // only carries pairs gets rows built here, in pair order.
        let rows: CSRAdjacency<FaceNeighbor>
        if rowsAreValid(precompute.faceNeighborRows, faceCount: faceCount, pairCount: precompute.staticWeights.count) {
            rows = precompute.faceNeighborRows
        } else {
            guard precompute.pairs.count == precompute.staticWeights.count,
                  precompute.pairs.allSatisfy({ Int($0.face0) < faceCount && Int($0.face1) < faceCount })
            else {
                throw NativeDenoiseError.invalidInput
            }
            rows = FilterPrecompute.makeFaceNeighborRows(faceCount: faceCount, pairs: precompute.pairs)
        }

        let h = Float(-0.5) / (nu * nu)
        let threshold = Double(precompute.convergenceThreshold)
        let chunkCount = ParallelChunks.chunkCount(for: faceCount, chunkSize: chunkSize)

        var current = initialSignals
        var next = [SIMD3<Float>](repeating: .zero, count: faceCount)
        var partialDisplacement = [Double](repeating: 0, count: chunkCount)
        // One exp per pair: each pair's weight is computed by the row of its first
        // face (neighbor index >= own index) and read by both rows.
        var dynamicWeights = [Float](repeating: 0, count: precompute.staticWeights.count)

        return try rows.offsets.withUnsafeBufferPointer { offsets in
            try rows.values.withUnsafeBufferPointer { neighbors in
                try precompute.staticWeights.withUnsafeBufferPointer { staticWeights in
                    try precompute.weightedInitialSignals.withUnsafeBufferPointer { weightedInitial in
                        try areaWeights.withUnsafeBufferPointer { areas in
                            for iteration in 1...maxIterations {
                                guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }

                                try current.withUnsafeBufferPointer { signals in
                                    try dynamicWeights.withUnsafeMutableBufferPointer { dynamicWeights in
                                        try ParallelChunks.forEachCancellable(
                                            count: faceCount,
                                            chunkSize: chunkSize,
                                            shouldCancel: shouldCancel
                                        ) { _, range in
                                            for face in range {
                                                let signal = signals[face]
                                                for entry in Int(offsets[face])..<Int(offsets[face + 1]) {
                                                    let neighbor = neighbors[entry]
                                                    let neighborFace = Int(neighbor.neighborFace)
                                                    guard neighborFace >= face else { continue }
                                                    let pairIndex = Int(neighbor.pairIndex)
                                                    let diff = signal - signals[neighborFace]
                                                    dynamicWeights[pairIndex] = staticWeights[pairIndex]
                                                        * Float(exp(Double(h * lengthSquared(diff))))
                                                }
                                            }
                                        }

                                        try next.withUnsafeMutableBufferPointer { filtered in
                                            try partialDisplacement.withUnsafeMutableBufferPointer { partial in
                                                try ParallelChunks.forEachCancellable(
                                                    count: faceCount,
                                                    chunkSize: chunkSize,
                                                    shouldCancel: shouldCancel
                                                ) { chunk, range in
                                                    var displacement = 0.0
                                                    for face in range {
                                                        let signal = signals[face]
                                                        var value = weightedInitial[face]
                                                        // Row order is pair order, so each face sums its
                                                        // neighbors in the same order as a pair-order scatter.
                                                        for entry in Int(offsets[face])..<Int(offsets[face + 1]) {
                                                            let neighbor = neighbors[entry]
                                                            value += signals[Int(neighbor.neighborFace)]
                                                                * dynamicWeights[Int(neighbor.pairIndex)]
                                                        }

                                                        let filteredSignal = normalized(value, fallback: signal)
                                                        filtered[face] = filteredSignal
                                                        displacement += Double(
                                                            areas[face] * lengthSquared(filteredSignal - signal)
                                                        )
                                                    }
                                                    partial[chunk] = displacement
                                                }
                                            }
                                        }
                                    }
                                }

                                swap(&current, &next)

                                var displacement = 0.0
                                for value in partialDisplacement {
                                    displacement += value
                                }
                                if displacement <= threshold {
                                    return (current, iteration, true)
                                }
                            }

                            return (current, maxIterations, false)
                        }
                    }
                }
            }
        }
    }

    /// Checks the CSR rows once so the hot loop can index without bounds checks.
    private static func rowsAreValid(
        _ rows: CSRAdjacency<FaceNeighbor>,
        faceCount: Int,
        pairCount: Int
    ) -> Bool {
        guard rows.offsets.count == faceCount + 1,
              rows.offsets[0] == 0,
              Int(rows.offsets[faceCount]) == rows.values.count,
              rows.values.count == 2 * pairCount
        else {
            return false
        }
        for face in 0..<faceCount where rows.offsets[face] > rows.offsets[face + 1] {
            return false
        }
        return rows.values.allSatisfy { neighbor in
            Int(neighbor.neighborFace) < faceCount && Int(neighbor.pairIndex) < pairCount
        }
    }
}
