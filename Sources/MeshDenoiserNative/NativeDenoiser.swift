import Metal

public enum NativeDenoiser {
    public static var isGPUAvailable: Bool {
        MTLCreateSystemDefaultDevice() != nil
    }

    /// Runs the native denoiser. The Metal path accelerates the fixed-point
    /// normal filter; preprocessing and vertex updates remain on CPU.
    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: NativeDenoiseParameters,
        useGPU: Bool,
        progress: (@Sendable (Int, Int) -> Bool)?
    ) throws -> [SIMD3<Float>] {
        guard parameters.isValid else { throw NativeDenoiseError.invalidParameters }

        var mesh = try MeshValidation.makeMesh(positions: positions, indices: indices)
        let normalized = try MeshNormalization.normalize(mesh.positions)
        mesh.positions = normalized.positions

        let stagesPerOuterIteration = 4
        let totalProgressStages = max(parameters.outerIterations * stagesPerOuterIteration, 1)

        for outerIteration in 0..<parameters.outerIterations {
            let progressBase = outerIteration * stagesPerOuterIteration
            let connectivity = try MeshConnectivity.build(mesh: mesh)
            let guidanceNormals = GuidanceNormals.compute(connectivity: connectivity)
            let etaPrime = Float(parameters.eta) * connectivity.averageNeighborFaceCentroidDistance
            try reportProgress(progressBase + 1, total: totalProgressStages, progress: progress)

            let filteredNormals: [SIMD3<Float>]
            if useGPU {
                var precompute = try FilterPrecompute.buildForGPU(
                    centroids: connectivity.faceGeometry.centroids,
                    guidanceNormals: guidanceNormals,
                    initialNormals: connectivity.faceGeometry.normals,
                    areaWeights: connectivity.faceGeometry.areaWeights,
                    etaPrime: etaPrime,
                    parameters: parameters
                )
                try reportProgress(progressBase + 2, total: totalProgressStages, progress: progress)
                filteredNormals = try FilterGPU.run(
                    initialSignals: connectivity.faceGeometry.normals,
                    areaWeights: connectivity.faceGeometry.areaWeights,
                    precompute: &precompute,
                    nu: Float(parameters.nu)
                ).signals
            } else {
                let precompute = try FilterPrecompute.buildForCPU(
                    centroids: connectivity.faceGeometry.centroids,
                    guidanceNormals: guidanceNormals,
                    initialNormals: connectivity.faceGeometry.normals,
                    areaWeights: connectivity.faceGeometry.areaWeights,
                    etaPrime: etaPrime,
                    parameters: parameters
                )
                try reportProgress(progressBase + 2, total: totalProgressStages, progress: progress)
                filteredNormals = FilterCPU.run(
                    initialSignals: connectivity.faceGeometry.normals,
                    areaWeights: connectivity.faceGeometry.areaWeights,
                    precompute: precompute,
                    nu: Float(parameters.nu)
                ).signals
            }
            try reportProgress(progressBase + 3, total: totalProgressStages, progress: progress)

            mesh.positions = try VertexUpdate.run(
                mesh: mesh,
                targetNormals: filteredNormals,
                parameters: parameters
            )

            try reportProgress(progressBase + 4, total: totalProgressStages, progress: progress)
        }

        return normalized.transform.restore(mesh.positions)
    }

    private static func reportProgress(
        _ completed: Int,
        total: Int,
        progress: (@Sendable (Int, Int) -> Bool)?
    ) throws {
        guard progress?(completed, total) ?? true else {
            throw NativeDenoiseError.cancelled
        }
    }
}
