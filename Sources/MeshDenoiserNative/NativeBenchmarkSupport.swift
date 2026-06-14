import Darwin

public enum NativeBenchmarkSupport {
    public static func runVertexUpdate(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: NativeDenoiseParameters = NativeDenoiseParameters()
    ) throws -> [SIMD3<Float>] {
        guard parameters.isValid else { throw NativeDenoiseError.invalidParameters }

        var mesh = try MeshValidation.makeMesh(positions: positions, indices: indices)
        let normalized = try MeshNormalization.normalize(mesh.positions)
        mesh.positions = normalized.positions

        let connectivity = try MeshConnectivity.build(mesh: mesh)
        return try VertexUpdate.run(
            mesh: mesh,
            targetNormals: perturbedNormals(connectivity.faceGeometry.normals),
            parameters: parameters
        )
    }

    public static func runFilter(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        useGPU: Bool,
        parameters: NativeDenoiseParameters = NativeDenoiseParameters()
    ) throws -> [SIMD3<Float>] {
        guard parameters.isValid else { throw NativeDenoiseError.invalidParameters }

        var mesh = try MeshValidation.makeMesh(positions: positions, indices: indices)
        let normalized = try MeshNormalization.normalize(mesh.positions)
        mesh.positions = normalized.positions

        let connectivity = try MeshConnectivity.build(mesh: mesh)
        let guidanceNormals = GuidanceNormals.compute(connectivity: connectivity)
        let etaPrime = Float(parameters.eta) * connectivity.averageNeighborFaceCentroidDistance

        if useGPU {
            var precompute = try FilterPrecompute.buildForGPU(
                centroids: connectivity.faceGeometry.centroids,
                guidanceNormals: guidanceNormals,
                initialNormals: connectivity.faceGeometry.normals,
                areaWeights: connectivity.faceGeometry.areaWeights,
                etaPrime: etaPrime,
                parameters: parameters
            )
            return try FilterGPU.run(
                initialSignals: connectivity.faceGeometry.normals,
                areaWeights: connectivity.faceGeometry.areaWeights,
                precompute: &precompute,
                nu: Float(parameters.nu)
            ).signals
        }

        let precompute = try FilterPrecompute.buildForCPU(
            centroids: connectivity.faceGeometry.centroids,
            guidanceNormals: guidanceNormals,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: parameters
        )
        return FilterCPU.run(
            initialSignals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            precompute: precompute,
            nu: Float(parameters.nu)
        ).signals
    }

    private static func perturbedNormals(_ normals: [SIMD3<Float>]) -> [SIMD3<Float>] {
        normals.enumerated().map { index, normal in
            let phase = Float(index)
            let offset = SIMD3<Float>(
                sin(phase * 0.017) * 0.03,
                cos(phase * 0.011) * 0.03,
                sin(phase * 0.013) * 0.02
            )
            return normalized(normal + offset, fallback: normal)
        }
    }

    private static func normalized(_ vector: SIMD3<Float>, fallback: SIMD3<Float>) -> SIMD3<Float> {
        let magnitude = length(vector)
        guard magnitude.isFinite, magnitude > 0 else { return fallback }
        return vector / magnitude
    }
}
