import Foundation

struct FacePair: Sendable, Equatable {
    private var raw: SIMD2<UInt32>

    var face0: UInt32 { raw.x }
    var face1: UInt32 { raw.y }

    init(face0: UInt32, face1: UInt32) {
        raw = SIMD2<UInt32>(face0, face1)
    }
}

struct FaceNeighbor: Sendable, Equatable {
    private var raw: SIMD2<UInt32>

    var neighborFace: UInt32 { raw.x }
    var pairIndex: UInt32 { raw.y }

    init(neighborFace: UInt32, pairIndex: UInt32) {
        raw = SIMD2<UInt32>(neighborFace, pairIndex)
    }
}

struct FilterPrecompute: Sendable {
    var pairs: [FacePair]
    var staticWeights: [Float]
    var faceNeighborRows: CSRAdjacency<FaceNeighbor>
    var rescaledLambda: Float
    var weightedInitialSignals: [SIMD3<Float>]
    var convergenceThreshold: Float

    static func build(
        pairs: [NeighborPair],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters
    ) throws -> FilterPrecompute {
        guard guidanceNormals.count == initialNormals.count,
              initialNormals.count == areaWeights.count,
              etaPrime.isFinite,
              etaPrime > 0,
              parameters.isValid
        else {
            throw NativeDenoiseError.invalidInput
        }

        let faceCount = initialNormals.count
        guard !pairs.isEmpty else { throw NativeDenoiseError.invalidInput }

        var accumulation = Accumulation(
            faceCount: faceCount,
            estimatedPairCount: pairs.count,
            storesPairs: true
        )

        for pair in pairs {
            try append(
                pair: pair,
                guidanceNormals: guidanceNormals,
                areaWeights: areaWeights,
                etaPrime: etaPrime,
                mu: Float(parameters.mu),
                to: &accumulation
            )
        }

        return try finish(
            accumulation: accumulation,
            initialNormals: initialNormals,
            areaWeights: areaWeights,
            parameters: parameters
        )
    }

    static func build(
        centroids: [SIMD3<Float>],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters
    ) throws -> FilterPrecompute {
        guard centroids.count == initialNormals.count,
              guidanceNormals.count == initialNormals.count,
              initialNormals.count == areaWeights.count,
              etaPrime.isFinite,
              etaPrime > 0,
              parameters.isValid
        else {
            throw NativeDenoiseError.invalidInput
        }

        var accumulation = Accumulation(
            faceCount: initialNormals.count,
            estimatedPairCount: centroids.count * 12,
            storesPairs: true
        )

        try NeighborSearch.forEachPair(centroids: centroids, radius: 3 * etaPrime) { pair in
            try append(
                pair: pair,
                guidanceNormals: guidanceNormals,
                areaWeights: areaWeights,
                etaPrime: etaPrime,
                mu: Float(parameters.mu),
                to: &accumulation
            )
        }

        return try finish(
            accumulation: accumulation,
            initialNormals: initialNormals,
            areaWeights: areaWeights,
            parameters: parameters
        )
    }

    /// Streaming build for the CPU filter: CSR neighbor rows plus per-pair static
    /// weights, sized exactly by a counting pass. Pairs are not materialized.
    static func buildForCPU(
        centroids: [SIMD3<Float>],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> FilterPrecompute {
        try buildStreamingRows(
            centroids: centroids,
            guidanceNormals: guidanceNormals,
            initialNormals: initialNormals,
            areaWeights: areaWeights,
            etaPrime: etaPrime,
            parameters: parameters,
            shouldCancel: shouldCancel
        )
    }

    /// Same layout as `buildForCPU`. The production GPU path fills Metal buffers
    /// directly (see `FilterGPU.prepare`); this array form serves `FilterGPU.run(precompute:)`.
    static func buildForGPU(
        centroids: [SIMD3<Float>],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> FilterPrecompute {
        try buildStreamingRows(
            centroids: centroids,
            guidanceNormals: guidanceNormals,
            initialNormals: initialNormals,
            areaWeights: areaWeights,
            etaPrime: etaPrime,
            parameters: parameters,
            shouldCancel: shouldCancel
        )
    }

    private static func buildStreamingRows(
        centroids: [SIMD3<Float>],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)?
    ) throws -> FilterPrecompute {
        let layout = try makeNeighborLayout(
            centroids: centroids,
            guidanceNormals: guidanceNormals,
            initialNormals: initialNormals,
            areaWeights: areaWeights,
            etaPrime: etaPrime,
            parameters: parameters,
            shouldCancel: shouldCancel
        )

        var areaSpatialSum = 0.0
        var staticWeights = [Float]()
        let rowValues = try [FaceNeighbor](unsafeUninitializedCapacity: layout.rowEntryCount) { rows, rowCount in
            staticWeights = try [Float](unsafeUninitializedCapacity: layout.pairCount) { weights, weightCount in
                areaSpatialSum = try fillNeighborRows(
                    layout: layout,
                    rowValues: rows,
                    staticWeights: weights,
                    guidanceNormals: guidanceNormals,
                    areaWeights: areaWeights,
                    etaPrime: etaPrime,
                    mu: Float(parameters.mu),
                    shouldCancel: shouldCancel
                )
                weightCount = layout.pairCount
            }
            rowCount = layout.rowEntryCount
        }

        let scalars = try finishScalars(
            areaSpatialSum: areaSpatialSum,
            areaWeights: areaWeights,
            parameters: parameters
        )
        return FilterPrecompute(
            pairs: [],
            staticWeights: staticWeights,
            faceNeighborRows: CSRAdjacency(offsets: layout.rowOffsets, values: rowValues),
            rescaledLambda: scalars.rescaledLambda,
            weightedInitialSignals: makeWeightedInitialSignals(
                initialNormals: initialNormals,
                areaWeights: areaWeights,
                initialScale: scalars.initialScale
            ),
            convergenceThreshold: scalars.convergenceThreshold
        )
    }

    /// Exact CSR layout of the neighbor graph, from a parallel counting pass.
    struct NeighborLayout {
        let grid: NeighborSearch.Grid
        /// `faceCount + 1` entries; row `i` is `rowOffsets[i]..<rowOffsets[i + 1]`.
        let rowOffsets: [UInt32]

        var faceCount: Int { rowOffsets.count - 1 }
        var rowEntryCount: Int { Int(rowOffsets[faceCount]) }
        var pairCount: Int { rowEntryCount / 2 }
    }

    static func makeNeighborLayout(
        centroids: [SIMD3<Float>],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)?
    ) throws -> NeighborLayout {
        guard centroids.count == initialNormals.count,
              guidanceNormals.count == initialNormals.count,
              initialNormals.count == areaWeights.count,
              etaPrime.isFinite,
              etaPrime > 0,
              parameters.isValid
        else {
            throw NativeDenoiseError.invalidInput
        }

        let grid = try NeighborSearch.Grid(
            centroids: centroids,
            radius: 3 * etaPrime,
            shouldCancel: shouldCancel
        )
        let counts = try grid.neighborCounts(shouldCancel: shouldCancel)

        var offsets = [UInt32](repeating: 0, count: counts.count + 1)
        var total: UInt64 = 0
        for face in counts.indices {
            total += UInt64(counts[face])
            // Row offsets and pair indices are 32-bit on the GPU; refuse rather than trap.
            guard total <= UInt64(UInt32.max) else { throw NativeDenoiseError.invalidInput }
            offsets[face + 1] = UInt32(total)
        }
        guard total > 0, total.isMultiple(of: 2) else { throw NativeDenoiseError.invalidInput }

        return NeighborLayout(grid: grid, rowOffsets: offsets)
    }

    /// Second pass: enumerates pairs in canonical order, writing each face's row in
    /// pair order (so per-face sums match a pair-order scatter) and the static weight
    /// of every pair. Returns the area-spatial sum used to rescale lambda.
    static func fillNeighborRows(
        layout: NeighborLayout,
        rowValues: UnsafeMutableBufferPointer<FaceNeighbor>,
        staticWeights: UnsafeMutableBufferPointer<Float>,
        guidanceNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        mu: Float,
        shouldCancel: (@Sendable () -> Bool)?
    ) throws -> Double {
        guard rowValues.count == layout.rowEntryCount,
              staticWeights.count == layout.pairCount,
              guidanceNormals.count == layout.faceCount,
              areaWeights.count == layout.faceCount
        else {
            throw NativeDenoiseError.invalidInput
        }

        let hSpatial = Float(-0.5) / (etaPrime * etaPrime)
        let hGuidance = Float(-0.5) / (mu * mu)
        var cursor = Array(layout.rowOffsets.dropLast())
        var pairIndex = 0
        var areaSpatialSum = 0.0

        try layout.rowOffsets.withUnsafeBufferPointer { offsets in
            try cursor.withUnsafeMutableBufferPointer { cursor in
                try guidanceNormals.withUnsafeBufferPointer { guidance in
                    try areaWeights.withUnsafeBufferPointer { areas in
                        _ = try layout.grid.forEachPair(shouldCancel: shouldCancel) { pair in
                            let face0 = Int(pair.face0)
                            let face1 = Int(pair.face1)
                            guard pairIndex < staticWeights.count,
                                  face0 < areas.count,
                                  face1 < areas.count,
                                  cursor[face0] < offsets[face0 + 1],
                                  cursor[face1] < offsets[face1 + 1]
                            else {
                                throw NativeDenoiseError.invalidInput
                            }

                            let weights = pairWeights(
                                areaSum: areas[face0] + areas[face1],
                                distanceSquared: pair.distanceSquared,
                                guidanceDiff: guidance[face0] - guidance[face1],
                                hSpatial: hSpatial,
                                hGuidance: hGuidance
                            )
                            let pairID = UInt32(pairIndex)
                            staticWeights[pairIndex] = weights.staticWeight
                            areaSpatialSum += Double(weights.areaSpatial)

                            rowValues[Int(cursor[face0])] = FaceNeighbor(neighborFace: pair.face1, pairIndex: pairID)
                            cursor[face0] += 1
                            rowValues[Int(cursor[face1])] = FaceNeighbor(neighborFace: pair.face0, pairIndex: pairID)
                            cursor[face1] += 1
                            pairIndex += 1
                        }
                    }
                }
            }
        }

        // Every row is bounded by its offsets, so a matching pair count means every slot was written.
        guard pairIndex == layout.pairCount else { throw NativeDenoiseError.invalidInput }
        return areaSpatialSum
    }

    static func finishScalars(
        areaSpatialSum: Double,
        areaWeights: [Float],
        parameters: NativeDenoiseParameters
    ) throws -> (rescaledLambda: Float, initialScale: Float, convergenceThreshold: Float) {
        let totalArea = areaWeights.reduce(0.0) { $0 + Double($1) }
        guard areaSpatialSum.isFinite,
              areaSpatialSum > 0,
              totalArea.isFinite,
              totalArea > 0
        else {
            throw NativeDenoiseError.invalidInput
        }

        let rescaledLambda = Float(parameters.lambda * totalArea / areaSpatialSum)
        let initialScale = Float(2 * parameters.nu * parameters.nu) / rescaledLambda
        let avgDispEps = Float(2 * sin(0.1 * Double.pi / 180))
        return (rescaledLambda, initialScale, Float(totalArea) * avgDispEps * avgDispEps)
    }

    static func makeWeightedInitialSignals(
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        initialScale: Float
    ) -> [SIMD3<Float>] {
        initialNormals.indices.map { index in
            initialNormals[index] * areaWeights[index] * initialScale
        }
    }

    private static func pairWeights(
        areaSum: Float,
        distanceSquared: Float,
        guidanceDiff: SIMD3<Float>,
        hSpatial: Float,
        hGuidance: Float
    ) -> (staticWeight: Float, areaSpatial: Float) {
        let spatialTerm = hSpatial * distanceSquared
        let guidanceTerm = hGuidance * lengthSquared(guidanceDiff)
        return (
            areaSum * Float(exp(Double(guidanceTerm + spatialTerm))),
            areaSum * Float(exp(Double(spatialTerm)))
        )
    }

    private struct Accumulation {
        var facePairs: [FacePair]
        var staticWeights: [Float]
        var degree: [Int]
        // Double: tens of millions of pair terms would otherwise round away in a Float sum.
        var areaSpatialSum: Double = 0
        var storesPairs: Bool

        init(faceCount: Int, estimatedPairCount: Int, storesPairs: Bool) {
            facePairs = []
            staticWeights = []
            degree = [Int](repeating: 0, count: faceCount)
            self.storesPairs = storesPairs
            if storesPairs {
                facePairs.reserveCapacity(estimatedPairCount)
            }
            staticWeights.reserveCapacity(estimatedPairCount)
        }
    }

    private static func append(
        pair: NeighborPair,
        guidanceNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        mu: Float,
        to accumulation: inout Accumulation
    ) throws {
        let face0 = Int(pair.face0)
        let face1 = Int(pair.face1)
        guard face0 < areaWeights.count, face1 < areaWeights.count else {
            throw NativeDenoiseError.invalidInput
        }

        let weights = pairWeights(
            areaSum: areaWeights[face0] + areaWeights[face1],
            distanceSquared: pair.distanceSquared,
            guidanceDiff: guidanceNormals[face0] - guidanceNormals[face1],
            hSpatial: Float(-0.5) / (etaPrime * etaPrime),
            hGuidance: Float(-0.5) / (mu * mu)
        )

        if accumulation.storesPairs {
            accumulation.facePairs.append(FacePair(face0: pair.face0, face1: pair.face1))
        }
        accumulation.staticWeights.append(weights.staticWeight)
        accumulation.areaSpatialSum += Double(weights.areaSpatial)
        accumulation.degree[face0] += 1
        accumulation.degree[face1] += 1
    }

    private static func finish(
        accumulation: Accumulation,
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        parameters: NativeDenoiseParameters
    ) throws -> FilterPrecompute {
        let scalars = try finishScalars(
            areaSpatialSum: accumulation.areaSpatialSum,
            areaWeights: areaWeights,
            parameters: parameters
        )
        return FilterPrecompute(
            pairs: accumulation.facePairs,
            staticWeights: accumulation.staticWeights,
            faceNeighborRows: makeFaceNeighborRows(
                faceCount: initialNormals.count,
                pairs: accumulation.facePairs,
                degree: accumulation.degree
            ),
            rescaledLambda: scalars.rescaledLambda,
            weightedInitialSignals: makeWeightedInitialSignals(
                initialNormals: initialNormals,
                areaWeights: areaWeights,
                initialScale: scalars.initialScale
            ),
            convergenceThreshold: scalars.convergenceThreshold
        )
    }

    /// CSR rows for an explicit pair list, in pair order.
    static func makeFaceNeighborRows(
        faceCount: Int,
        pairs: [FacePair]
    ) -> CSRAdjacency<FaceNeighbor> {
        var degree = [Int](repeating: 0, count: faceCount)
        for pair in pairs {
            degree[Int(pair.face0)] += 1
            degree[Int(pair.face1)] += 1
        }
        return makeFaceNeighborRows(faceCount: faceCount, pairs: pairs, degree: degree)
    }

    private static func makeFaceNeighborRows(
        faceCount: Int,
        pairs: [FacePair],
        degree: [Int]
    ) -> CSRAdjacency<FaceNeighbor> {
        var offsets = [UInt32](repeating: 0, count: faceCount + 1)
        for face in 0..<faceCount {
            offsets[face + 1] = offsets[face] + UInt32(degree[face])
        }

        var values = [FaceNeighbor](
            repeating: FaceNeighbor(neighborFace: 0, pairIndex: 0),
            count: Int(offsets[faceCount])
        )
        var cursor = offsets

        for (pairIndex, pair) in pairs.enumerated() {
            let face0 = Int(pair.face0)
            let face1 = Int(pair.face1)
            let pairID = UInt32(pairIndex)

            let offset0 = Int(cursor[face0])
            values[offset0] = FaceNeighbor(neighborFace: pair.face1, pairIndex: pairID)
            cursor[face0] += 1

            let offset1 = Int(cursor[face1])
            values[offset1] = FaceNeighbor(neighborFace: pair.face0, pairIndex: pairID)
            cursor[face1] += 1
        }

        return CSRAdjacency(offsets: offsets, values: values)
    }
}
