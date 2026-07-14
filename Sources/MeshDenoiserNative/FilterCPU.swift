import Foundation

enum FilterCPU {
    static func run(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: FilterPrecompute,
        nu: Float,
        maxIterations: Int = 100,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        guard !initialSignals.isEmpty,
              initialSignals.count == areaWeights.count,
              initialSignals.count == precompute.weightedInitialSignals.count,
              precompute.pairs.count == precompute.staticWeights.count,
              nu.isFinite,
              nu > 0,
              maxIterations > 0
        else {
            throw NativeDenoiseError.invalidInput
        }
        guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }

        var signals = initialSignals
        let h = Float(-0.5) / (nu * nu)

        for iteration in 1...maxIterations {
            guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }
            let previous = signals

            var filtered = precompute.weightedInitialSignals
            for (pairIndex, pair) in precompute.pairs.enumerated() {
                if pairIndex.isMultiple(of: 4_096), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }
                let face0 = Int(pair.face0)
                let face1 = Int(pair.face1)
                let diff = signals[face0] - signals[face1]
                let dynamicWeight = precompute.staticWeights[pairIndex]
                    * Float(exp(Double(h * lengthSquared(diff))))
                filtered[face0] += signals[face1] * dynamicWeight
                filtered[face1] += signals[face0] * dynamicWeight
            }

            for faceIndex in filtered.indices {
                if faceIndex.isMultiple(of: 4_096), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }
                filtered[faceIndex] = normalized(filtered[faceIndex], fallback: signals[faceIndex])
            }

            signals = filtered

            var displacement: Float = 0
            for index in signals.indices {
                if index.isMultiple(of: 4_096), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }
                displacement += areaWeights[index] * lengthSquared(signals[index] - previous[index])
            }

            if displacement <= precompute.convergenceThreshold {
                return (signals, iteration, true)
            }
        }

        return (signals, maxIterations, false)
    }
}
