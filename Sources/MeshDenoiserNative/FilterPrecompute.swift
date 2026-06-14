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

    static func buildForCPU(
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
            parameters: parameters,
            faceNeighborRows: CSRAdjacency(offsets: [], values: [])
        )
    }

    static func buildForGPU(
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
            storesPairs: false
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
            parameters: parameters,
            faceNeighborRows: makeFaceNeighborRows(
                faceCount: initialNormals.count,
                centroids: centroids,
                radius: 3 * etaPrime,
                degree: accumulation.degree
            )
        )
    }

    private struct Accumulation {
        var facePairs: [FacePair]
        var staticWeights: [Float]
        var degree: [Int]
        var areaSpatialSum: Float = 0
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

        let hSpatial = Float(-0.5) / (etaPrime * etaPrime)
        let hGuidance = Float(-0.5) / (mu * mu)
        let areaSum = areaWeights[face0] + areaWeights[face1]
        let distanceSquared = pair.distance * pair.distance
        let spatialTerm = hSpatial * distanceSquared
        let areaSpatial = areaSum * Float(exp(Double(spatialTerm)))
        let guidanceDiff = guidanceNormals[face0] - guidanceNormals[face1]
        let guidanceTerm = hGuidance * lengthSquared(guidanceDiff)

        if accumulation.storesPairs {
            accumulation.facePairs.append(FacePair(face0: pair.face0, face1: pair.face1))
        }
        accumulation.staticWeights.append(areaSum * Float(exp(Double(guidanceTerm + spatialTerm))))
        accumulation.areaSpatialSum += areaSpatial
        accumulation.degree[face0] += 1
        accumulation.degree[face1] += 1
    }

    private static func finish(
        accumulation: Accumulation,
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        parameters: NativeDenoiseParameters,
        faceNeighborRows explicitFaceNeighborRows: CSRAdjacency<FaceNeighbor>? = nil
    ) throws -> FilterPrecompute {
        let totalArea = areaWeights.reduce(0, +)
        guard accumulation.areaSpatialSum.isFinite,
              accumulation.areaSpatialSum > 0,
              totalArea.isFinite,
              totalArea > 0
        else {
            throw NativeDenoiseError.invalidInput
        }

        let rescaledLambda = Float(parameters.lambda) * totalArea / accumulation.areaSpatialSum
        let initialScale = Float(2 * parameters.nu * parameters.nu) / rescaledLambda
        let weightedInitialSignals = initialNormals.indices.map { index in
            initialNormals[index] * areaWeights[index] * initialScale
        }

        let avgDispEps = Float(2 * sin(0.1 * Double.pi / 180))
        return FilterPrecompute(
            pairs: accumulation.facePairs,
            staticWeights: accumulation.staticWeights,
            faceNeighborRows: explicitFaceNeighborRows ?? makeFaceNeighborRows(
                faceCount: initialNormals.count,
                pairs: accumulation.facePairs,
                degree: accumulation.degree
            ),
            rescaledLambda: rescaledLambda,
            weightedInitialSignals: weightedInitialSignals,
            convergenceThreshold: totalArea * avgDispEps * avgDispEps
        )
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

    private static func makeFaceNeighborRows(
        faceCount: Int,
        centroids: [SIMD3<Float>],
        radius: Float,
        degree: [Int]
    ) throws -> CSRAdjacency<FaceNeighbor> {
        var offsets = [UInt32](repeating: 0, count: faceCount + 1)
        for face in 0..<faceCount {
            offsets[face + 1] = offsets[face] + UInt32(degree[face])
        }

        var values = [FaceNeighbor](
            repeating: FaceNeighbor(neighborFace: 0, pairIndex: 0),
            count: Int(offsets[faceCount])
        )
        var cursor = offsets
        var pairIndex: UInt32 = 0

        let pairCount = try NeighborSearch.forEachPair(centroids: centroids, radius: radius) { pair in
            let face0 = Int(pair.face0)
            let face1 = Int(pair.face1)

            let offset0 = Int(cursor[face0])
            values[offset0] = FaceNeighbor(neighborFace: pair.face1, pairIndex: pairIndex)
            cursor[face0] += 1

            let offset1 = Int(cursor[face1])
            values[offset1] = FaceNeighbor(neighborFace: pair.face0, pairIndex: pairIndex)
            cursor[face1] += 1

            pairIndex += 1
        }

        guard pairIndex == UInt32(pairCount) else { throw NativeDenoiseError.invalidInput }
        return CSRAdjacency(offsets: offsets, values: values)
    }
}
