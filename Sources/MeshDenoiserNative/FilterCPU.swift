import Foundation

enum FilterCPU {
    static func run(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: FilterPrecompute,
        nu: Float,
        maxIterations: Int = 100
    ) -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        var signals = initialSignals
        let h = Float(-0.5) / (nu * nu)

        for iteration in 1...maxIterations {
            let previous = signals

            var filtered = precompute.weightedInitialSignals
            for (pairIndex, pair) in precompute.pairs.enumerated() {
                let face0 = Int(pair.face0)
                let face1 = Int(pair.face1)
                let diff = signals[face0] - signals[face1]
                let dynamicWeight = precompute.staticWeights[pairIndex]
                    * Float(exp(Double(h * lengthSquared(diff))))
                filtered[face0] += signals[face1] * dynamicWeight
                filtered[face1] += signals[face0] * dynamicWeight
            }

            for faceIndex in filtered.indices {
                filtered[faceIndex] = normalized(filtered[faceIndex], fallback: signals[faceIndex])
            }

            signals = filtered

            var displacement: Float = 0
            for index in signals.indices {
                displacement += areaWeights[index] * lengthSquared(signals[index] - previous[index])
            }

            if displacement <= precompute.convergenceThreshold {
                return (signals, iteration, true)
            }
        }

        return (signals, maxIterations, false)
    }
}
