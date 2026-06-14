#if os(macOS)
import Foundation
import ModelIO
import SceneKit
import SceneKit.ModelIO

public struct MeshAssetDenoiseOptions: Sendable, Equatable {
    public enum Preprocessing: Sendable, Equatable {
        case none
        case conservativeValidation(MeshPreprocessOptions)
    }

    public var parameters: MeshDenoiseParameters
    public var preprocessing: Preprocessing
    public var recomputesNormals: Bool

    public init(
        parameters: MeshDenoiseParameters = MeshDenoiseParameters(),
        preprocessing: Preprocessing = .none,
        recomputesNormals: Bool = true
    ) {
        self.parameters = parameters
        self.preprocessing = preprocessing
        self.recomputesNormals = recomputesNormals
    }
}

public struct MeshAssetDenoiseSummary: Sendable, Equatable {
    public var meshesProcessed: Int
    public var verticesProcessed: Int
    public var facesProcessed: Int
    public var preprocessingDiagnostics: [MeshPreprocessDiagnostics]

    public init(
        meshesProcessed: Int,
        verticesProcessed: Int,
        facesProcessed: Int,
        preprocessingDiagnostics: [MeshPreprocessDiagnostics] = []
    ) {
        self.meshesProcessed = meshesProcessed
        self.verticesProcessed = verticesProcessed
        self.facesProcessed = facesProcessed
        self.preprocessingDiagnostics = preprocessingDiagnostics
    }
}

public enum MeshAssetDenoiseError: LocalizedError, Sendable, Equatable {
    case unsupportedAsset(String)
    case preprocessingWouldChangeTopology
    case inputOutputURLsMustDiffer
    case exportFailed(String)

    public var errorDescription: String? {
        switch self {
        case .unsupportedAsset(let reason):
            return reason
        case .preprocessingWouldChangeTopology:
            return "Mesh preprocessing would change topology; topology-changing repair is not applied to asset files."
        case .inputOutputURLsMustDiffer:
            return "Input and output asset URLs must be different."
        case .exportFailed(let path):
            return "Could not export denoised asset to \(path)."
        }
    }
}

public enum MeshAssetDenoiser {
    private struct DenoisedMeshPatch {
        var originalPositions: [SIMD3<Float>]
        var denoisedPositions: [SIMD3<Float>]
    }

    private struct USDAArrayStatement {
        var statementRange: Range<String.Index>
        var arrayRange: Range<String.Index>
    }

    /// Inspects an asset file and returns the mesh counts and preprocessing
    /// diagnostics that processing would use, without denoising or exporting.
    public static func preflight(
        inputURL: URL,
        options: MeshAssetDenoiseOptions = MeshAssetDenoiseOptions()
    ) throws -> MeshAssetDenoiseSummary {
        try preflight(asset: MDLAsset(url: inputURL), options: options)
    }

    /// Inspects an already loaded asset and returns the mesh counts and
    /// preprocessing diagnostics that processing would use, without denoising
    /// or exporting.
    public static func preflight(
        asset: MDLAsset,
        options: MeshAssetDenoiseOptions = MeshAssetDenoiseOptions()
    ) throws -> MeshAssetDenoiseSummary {
        guard options.parameters.isValid else {
            throw MeshDenoiseError.invalidParameters
        }

        let meshes = asset.childObjects(of: MDLMesh.self).compactMap { $0 as? MDLMesh }
        guard !meshes.isEmpty else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no ModelIO meshes.")
        }

        var summary = MeshAssetDenoiseSummary(meshesProcessed: 0, verticesProcessed: 0, facesProcessed: 0)
        for mesh in meshes {
            let positions = try loadPositions(from: mesh)
            let indices = try loadTriangleIndices(from: mesh)
            guard !positions.isEmpty, !indices.isEmpty else {
                continue
            }

            let diagnostics = try validatePreprocessingIfNeeded(
                positions: positions,
                indices: indices,
                options: options.preprocessing
            )
            if let diagnostics {
                summary.preprocessingDiagnostics.append(diagnostics)
            }

            summary.meshesProcessed += 1
            summary.verticesProcessed += positions.count
            summary.facesProcessed += indices.count / 3
        }

        guard summary.meshesProcessed > 0 else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no triangle mesh data.")
        }
        return summary
    }

    /// Denoises all ModelIO meshes in a USD/USDZ-style asset while preserving
    /// vertex count, vertex order, submeshes, materials, UVs, and transforms.
    public static func process(
        inputURL: URL,
        outputURL: URL,
        options: MeshAssetDenoiseOptions = MeshAssetDenoiseOptions(),
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> MeshAssetDenoiseSummary {
        guard !sameFileURL(inputURL, outputURL) else {
            throw MeshAssetDenoiseError.inputOutputURLsMustDiffer
        }

        if inputURL.pathExtension.lowercased() == "usdz",
           outputURL.pathExtension.lowercased() == "usdz" {
            return try await processUSDZFile(
                inputURL: inputURL,
                outputURL: outputURL,
                options: options,
                progress: progress
            )
        }

        let asset = MDLAsset(url: inputURL)
        return try await process(
            asset: asset,
            outputURL: outputURL,
            options: options,
            progress: progress
        )
    }

    /// Denoises all ModelIO meshes in an already loaded asset and writes a new
    /// asset file. The input asset is mutated in place before export.
    public static func process(
        asset: MDLAsset,
        outputURL: URL,
        options: MeshAssetDenoiseOptions = MeshAssetDenoiseOptions(),
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> MeshAssetDenoiseSummary {
        guard options.parameters.isValid else {
            throw MeshDenoiseError.invalidParameters
        }

        let meshes = asset.childObjects(of: MDLMesh.self).compactMap { $0 as? MDLMesh }
        guard !meshes.isEmpty else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no ModelIO meshes.")
        }

        progress?(0)

        var summary = MeshAssetDenoiseSummary(meshesProcessed: 0, verticesProcessed: 0, facesProcessed: 0)
        for (meshIndex, mesh) in meshes.enumerated() {
            let positions = try loadPositions(from: mesh)
            let indices = try loadTriangleIndices(from: mesh)
            guard !positions.isEmpty, !indices.isEmpty else {
                continue
            }

            let diagnostics = try validatePreprocessingIfNeeded(
                positions: positions,
                indices: indices,
                options: options.preprocessing
            )
            if let diagnostics {
                summary.preprocessingDiagnostics.append(diagnostics)
            }

            let baseProgress = 0.9 * Double(meshIndex) / Double(meshes.count)
            let meshProgressScale = 0.9 / Double(meshes.count)
            let denoised = try await MeshDenoiser.denoise(
                positions: positions,
                indices: indices,
                parameters: options.parameters
            ) { meshProgress in
                progress?(baseProgress + meshProgress * meshProgressScale)
            }

            try writePositions(denoised, to: mesh)
            if options.recomputesNormals {
                mesh.addNormals(withAttributeNamed: MDLVertexAttributeNormal, creaseThreshold: 0.5)
            }

            summary.meshesProcessed += 1
            summary.verticesProcessed += positions.count
            summary.facesProcessed += indices.count / 3
            progress?(0.9 * Double(meshIndex + 1) / Double(meshes.count))
        }

        guard summary.meshesProcessed > 0 else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no triangle mesh data.")
        }

        progress?(0.95)
        try exportAtomically(to: outputURL) { temporaryURL in
            try exportDirect(asset: asset, to: temporaryURL)
        }
        progress?(1)
        return summary
    }

    static func exportAtomically(
        to outputURL: URL,
        export: (URL) throws -> Void
    ) throws {
        let fileManager = FileManager.default
        let temporaryURL = temporaryExportURL(for: outputURL)
        try? fileManager.removeItem(at: temporaryURL)

        do {
            try export(temporaryURL)
        } catch {
            try? fileManager.removeItem(at: temporaryURL)
            throw error
        }

        do {
            if fileManager.fileExists(atPath: outputURL.path) {
                _ = try fileManager.replaceItemAt(outputURL, withItemAt: temporaryURL)
            } else {
                try fileManager.moveItem(at: temporaryURL, to: outputURL)
            }
        } catch {
            try? fileManager.removeItem(at: temporaryURL)
            throw MeshAssetDenoiseError.exportFailed(outputURL.path)
        }
    }

    private static func exportDirect(asset: MDLAsset, to outputURL: URL) throws {
        if outputURL.pathExtension.lowercased() == "usdz" {
            let scene = SCNScene(mdlAsset: asset)
            guard scene.write(to: outputURL, options: nil, delegate: nil, progressHandler: nil) else {
                throw MeshAssetDenoiseError.exportFailed(outputURL.path)
            }
            return
        }

        do {
            try asset.export(to: outputURL)
        } catch {
            throw MeshAssetDenoiseError.exportFailed(outputURL.path)
        }
    }

    private static func processUSDZFile(
        inputURL: URL,
        outputURL: URL,
        options: MeshAssetDenoiseOptions,
        progress: (@Sendable (Double) -> Void)?
    ) async throws -> MeshAssetDenoiseSummary {
        guard options.parameters.isValid else {
            throw MeshDenoiseError.invalidParameters
        }

        let asset = MDLAsset(url: inputURL)
        let meshes = asset.childObjects(of: MDLMesh.self).compactMap { $0 as? MDLMesh }
        guard !meshes.isEmpty else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no ModelIO meshes.")
        }

        progress?(0)

        var denoisedMeshes = [DenoisedMeshPatch]()
        denoisedMeshes.reserveCapacity(meshes.count)
        var summary = MeshAssetDenoiseSummary(meshesProcessed: 0, verticesProcessed: 0, facesProcessed: 0)
        for (meshIndex, mesh) in meshes.enumerated() {
            let positions = try loadPositions(from: mesh)
            let indices = try loadTriangleIndices(from: mesh)
            guard !positions.isEmpty, !indices.isEmpty else {
                continue
            }

            let diagnostics = try validatePreprocessingIfNeeded(
                positions: positions,
                indices: indices,
                options: options.preprocessing
            )
            if let diagnostics {
                summary.preprocessingDiagnostics.append(diagnostics)
            }

            let baseProgress = 0.9 * Double(meshIndex) / Double(meshes.count)
            let meshProgressScale = 0.9 / Double(meshes.count)
            let denoised = try await MeshDenoiser.denoise(
                positions: positions,
                indices: indices,
                parameters: options.parameters
            ) { meshProgress in
                progress?(baseProgress + meshProgress * meshProgressScale)
            }

            denoisedMeshes.append(DenoisedMeshPatch(
                originalPositions: positions,
                denoisedPositions: denoised
            ))
            summary.meshesProcessed += 1
            summary.verticesProcessed += positions.count
            summary.facesProcessed += indices.count / 3
            progress?(0.9 * Double(meshIndex + 1) / Double(meshes.count))
        }

        guard summary.meshesProcessed > 0 else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no triangle mesh data.")
        }

        progress?(0.95)
        try exportAtomically(to: outputURL) { temporaryURL in
            try exportUSDZByPatchingPoints(
                inputURL: inputURL,
                outputURL: temporaryURL,
                denoisedMeshes: denoisedMeshes
            )
        }
        progress?(1)
        return summary
    }

    private static func exportUSDZByPatchingPoints(
        inputURL: URL,
        outputURL: URL,
        denoisedMeshes: [DenoisedMeshPatch]
    ) throws {
        let fileManager = FileManager.default
        let workingDirectory = fileManager.temporaryDirectory
            .appendingPathComponent("MeshDenoiserUSDZ-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: workingDirectory, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: workingDirectory) }

        try runTool("/usr/bin/ditto", arguments: ["-x", "-k", inputURL.path, workingDirectory.path])

        let rootURL = try rootUSDFile(in: workingDirectory)
        let usdaURL = workingDirectory.appendingPathComponent("denoised.usda")
        try runTool("/usr/bin/usdcat", arguments: [rootURL.path, "-o", usdaURL.path])
        try patchUSDAPoints(at: usdaURL, denoisedMeshes: denoisedMeshes)
        try runTool(
            "/usr/bin/usdzip",
            arguments: [outputURL.path, "--asset", usdaURL.lastPathComponent],
            currentDirectoryURL: workingDirectory
        )
    }

    private static func rootUSDFile(in directory: URL) throws -> URL {
        let contents = try FileManager.default.contentsOfDirectory(
            at: directory,
            includingPropertiesForKeys: [.isDirectoryKey],
            options: [.skipsHiddenFiles]
        )

        let usdFiles = contents.filter { url in
            let pathExtension = url.pathExtension.lowercased()
            return pathExtension == "usd" || pathExtension == "usda" || pathExtension == "usdc"
        }

        guard let root = usdFiles.sorted(by: { $0.lastPathComponent < $1.lastPathComponent }).first else {
            throw MeshAssetDenoiseError.exportFailed(directory.path)
        }
        return root
    }

    private static func patchUSDAPoints(at url: URL, denoisedMeshes: [DenoisedMeshPatch]) throws {
        var text = try String(contentsOf: url, encoding: .utf8)
        var searchStart = text.startIndex

        for mesh in denoisedMeshes {
            let faceVertexCountsStatement = try nextUSDAArrayStatement(
                named: "int[] faceVertexCounts =",
                in: text,
                from: searchStart
            )
            let faceVertexIndicesStatement = try nextUSDAArrayStatement(
                named: "int[] faceVertexIndices =",
                in: text,
                from: searchStart
            )
            let normalsStatement = nextUSDAArrayStatementIfPresent(
                named: "normal3f[] normals =",
                in: text,
                from: searchStart
            )
            let pointsStatement = try nextUSDAArrayStatement(
                named: "point3f[] points =",
                in: text,
                from: searchStart
            )
            let usdPositions = try parseUSDAPointArray(text[pointsStatement.arrayRange])
            let mappedPositions = try mapDenoisedPositionsToTargetGeometry(
                originalPositions: mesh.originalPositions,
                denoisedPositions: mesh.denoisedPositions,
                targetPositions: usdPositions
            )
            let faceVertexCounts = try parseUSDAIntArray(text[faceVertexCountsStatement.arrayRange])
            let faceVertexIndices = try parseUSDAIntArray(text[faceVertexIndicesStatement.arrayRange])
            let replacementNormals = try normalsStatement.flatMap { statement -> String? in
                let existingNormals = try parseUSDAPointArray(text[statement.arrayRange])
                guard existingNormals.count == mappedPositions.count else {
                    return nil
                }
                let normals = try recomputeVertexNormals(
                    positions: mappedPositions,
                    faceVertexCounts: faceVertexCounts,
                    faceVertexIndices: faceVertexIndices
                )
                return "normal3f[] normals = \(usdaTupleArray(normals))"
            }
            let replacement = "point3f[] points = \(usdaPointArray(mappedPositions))"
            let pointEndOffset = text.distance(from: text.startIndex, to: pointsStatement.statementRange.lowerBound)
                + replacement.count
            let normalOffsetDelta: Int
            if let normalsStatement, let replacementNormals {
                normalOffsetDelta = replacementNormals.count
                    - text.distance(from: normalsStatement.statementRange.lowerBound, to: normalsStatement.statementRange.upperBound)
            } else {
                normalOffsetDelta = 0
            }
            text.replaceSubrange(pointsStatement.statementRange, with: replacement)
            if let normalsStatement, let replacementNormals {
                text.replaceSubrange(normalsStatement.statementRange, with: replacementNormals)
            }
            searchStart = text.index(text.startIndex, offsetBy: pointEndOffset + normalOffsetDelta)
        }

        if nextUSDAArrayStatementIfPresent(named: "point3f[] points =", in: text, from: searchStart) != nil {
            throw MeshAssetDenoiseError.exportFailed(url.path)
        }

        try text.write(to: url, atomically: true, encoding: .utf8)
    }

    private static func nextUSDAArrayStatement(
        named name: String,
        in text: String,
        from start: String.Index
    ) throws -> USDAArrayStatement {
        guard let range = nextUSDAArrayStatementIfPresent(named: name, in: text, from: start) else {
            throw MeshAssetDenoiseError.exportFailed("USD array was not found: \(name)")
        }
        return range
    }

    private static func nextUSDAArrayStatementIfPresent(
        named name: String,
        in text: String,
        from start: String.Index
    ) -> USDAArrayStatement? {
        guard let statementStart = text.range(
            of: name,
            range: start..<text.endIndex
        )?.lowerBound else {
            return nil
        }

        let assignmentEnd = text.index(statementStart, offsetBy: name.count)
        guard let arrayStart = text[assignmentEnd...].firstIndex(of: "[") else {
            return nil
        }

        var depth = 0
        var index = arrayStart
        while index < text.endIndex {
            let character = text[index]
            if character == "[" {
                depth += 1
            } else if character == "]" {
                depth -= 1
                if depth == 0 {
                    let arrayEnd = text.index(after: index)
                    return USDAArrayStatement(
                        statementRange: statementStart..<arrayEnd,
                        arrayRange: arrayStart..<arrayEnd
                    )
                }
            }
            index = text.index(after: index)
        }
        return nil
    }

    private static func parseUSDAPointArray(_ arrayText: Substring) throws -> [SIMD3<Float>] {
        var positions = [SIMD3<Float>]()
        var index = arrayText.startIndex

        while let tupleStart = arrayText[index...].firstIndex(of: "(") {
            guard let tupleEnd = arrayText[tupleStart...].firstIndex(of: ")") else {
                throw MeshAssetDenoiseError.exportFailed("USD points array is malformed.")
            }

            let values = arrayText[arrayText.index(after: tupleStart)..<tupleEnd]
                .split(separator: ",")
                .map { value in
                    Float(value.trimmingCharacters(in: .whitespacesAndNewlines))
                }

            guard values.count == 3,
                  let x = values[0],
                  let y = values[1],
                  let z = values[2]
            else {
                throw MeshAssetDenoiseError.exportFailed("USD point tuple is malformed.")
            }

            positions.append(SIMD3<Float>(x, y, z))
            index = arrayText.index(after: tupleEnd)
        }

        guard !positions.isEmpty else {
            throw MeshAssetDenoiseError.exportFailed("USD points array is empty.")
        }
        return positions
    }

    private static func parseUSDAIntArray(_ arrayText: Substring) throws -> [Int] {
        let values = arrayText
            .dropFirst()
            .dropLast()
            .split(separator: ",")
            .map { value in
                Int(value.trimmingCharacters(in: .whitespacesAndNewlines))
            }
        guard !values.isEmpty, values.allSatisfy({ $0 != nil }) else {
            throw MeshAssetDenoiseError.exportFailed("USD integer array is malformed.")
        }
        return values.compactMap { $0 }
    }

    private static func usdaPointArray(_ positions: [SIMD3<Float>]) -> String {
        usdaTupleArray(positions)
    }

    private static func usdaTupleArray(_ positions: [SIMD3<Float>]) -> String {
        var points = [String]()
        points.reserveCapacity(positions.count)
        for position in positions {
            points.append("(\(usdaFloat(position.x)), \(usdaFloat(position.y)), \(usdaFloat(position.z)))")
        }
        return "[\(points.joined(separator: ", "))]"
    }

    private static func recomputeVertexNormals(
        positions: [SIMD3<Float>],
        faceVertexCounts: [Int],
        faceVertexIndices: [Int]
    ) throws -> [SIMD3<Float>] {
        var normals = Array(repeating: SIMD3<Float>(0, 0, 0), count: positions.count)
        var cursor = 0
        for faceVertexCount in faceVertexCounts {
            guard faceVertexCount >= 3, cursor + faceVertexCount <= faceVertexIndices.count else {
                throw MeshAssetDenoiseError.exportFailed("USD face index array is malformed.")
            }
            let baseIndex = faceVertexIndices[cursor]
            for localIndex in 1..<(faceVertexCount - 1) {
                let index0 = baseIndex
                let index1 = faceVertexIndices[cursor + localIndex]
                let index2 = faceVertexIndices[cursor + localIndex + 1]
                guard positions.indices.contains(index0),
                      positions.indices.contains(index1),
                      positions.indices.contains(index2)
                else {
                    throw MeshAssetDenoiseError.exportFailed("USD face index references missing points.")
                }
                let normal = cross(positions[index1] - positions[index0], positions[index2] - positions[index0])
                normals[index0] += normal
                normals[index1] += normal
                normals[index2] += normal
            }
            cursor += faceVertexCount
        }
        guard cursor == faceVertexIndices.count else {
            throw MeshAssetDenoiseError.exportFailed("USD face index array has trailing values.")
        }
        return normals.map { normal in
            let length = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z)
            guard length > 0 else { return SIMD3<Float>(0, 1, 0) }
            return normal / length
        }
    }

    private static func cross(_ lhs: SIMD3<Float>, _ rhs: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3<Float>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x
        )
    }

    private static func usdaFloat(_ value: Float) -> String {
        String(format: "%.9g", locale: Locale(identifier: "en_US_POSIX"), Double(value))
    }

    private static func runTool(
        _ executablePath: String,
        arguments: [String],
        currentDirectoryURL: URL? = nil
    ) throws {
        let process = Process()
        process.executableURL = URL(fileURLWithPath: executablePath)
        process.arguments = arguments
        process.currentDirectoryURL = currentDirectoryURL

        let outputPipe = Pipe()
        let errorPipe = Pipe()
        process.standardOutput = outputPipe
        process.standardError = errorPipe

        do {
            try process.run()
        } catch {
            throw MeshAssetDenoiseError.exportFailed(executablePath)
        }
        process.waitUntilExit()

        guard process.terminationStatus == 0 else {
            let errorData = errorPipe.fileHandleForReading.readDataToEndOfFile()
            let outputData = outputPipe.fileHandleForReading.readDataToEndOfFile()
            let message = String(data: errorData + outputData, encoding: .utf8) ?? executablePath
            throw MeshAssetDenoiseError.exportFailed(message.trimmingCharacters(in: .whitespacesAndNewlines))
        }
    }

    private static func mapDenoisedPositionsToTargetGeometry(
        originalPositions: [SIMD3<Float>],
        denoisedPositions: [SIMD3<Float>],
        targetPositions: [SIMD3<Float>]
    ) throws -> [SIMD3<Float>] {
        guard originalPositions.count == denoisedPositions.count else {
            throw MeshAssetDenoiseError.unsupportedAsset("Denoised positions cannot be mapped back to the original asset.")
        }
        if denoisedPositions.count == targetPositions.count {
            return denoisedPositions
        }

        var targetIndexByPosition = [PositionKey: Int](minimumCapacity: targetPositions.count)
        for (index, position) in targetPositions.enumerated() {
            targetIndexByPosition[PositionKey(position)] = index
        }

        var sums = Array(repeating: SIMD3<Float>(0, 0, 0), count: targetPositions.count)
        var counts = Array(repeating: 0, count: targetPositions.count)
        for (originalPosition, denoisedPosition) in zip(originalPositions, denoisedPositions) {
            guard let targetIndex = targetIndexByPosition[PositionKey(originalPosition)] else {
                throw MeshAssetDenoiseError.unsupportedAsset("ModelIO and USDZ position buffers could not be reconciled.")
            }
            sums[targetIndex] += denoisedPosition
            counts[targetIndex] += 1
        }

        var mappedPositions = targetPositions
        for index in mappedPositions.indices where counts[index] > 0 {
            mappedPositions[index] = sums[index] / Float(counts[index])
        }
        return mappedPositions
    }

    private struct PositionKey: Hashable {
        private static let scale: Float = 1_000_000

        var x: Int
        var y: Int
        var z: Int

        init(_ position: SIMD3<Float>) {
            x = Int((position.x * Self.scale).rounded())
            y = Int((position.y * Self.scale).rounded())
            z = Int((position.z * Self.scale).rounded())
        }
    }

    private static func temporaryExportURL(for outputURL: URL) -> URL {
        let directory = outputURL.deletingLastPathComponent()
        let baseName = outputURL.deletingPathExtension().lastPathComponent
        let fileExtension = outputURL.pathExtension
        let identifier = UUID().uuidString

        if fileExtension.isEmpty {
            return directory.appendingPathComponent(".\(baseName).\(identifier)")
        }
        return directory.appendingPathComponent(".\(baseName).\(identifier).\(fileExtension)")
    }

    private static func sameFileURL(_ lhs: URL, _ rhs: URL) -> Bool {
        lhs.standardizedFileURL.resolvingSymlinksInPath().path
            == rhs.standardizedFileURL.resolvingSymlinksInPath().path
    }

    private static func validatePreprocessingIfNeeded(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        options: MeshAssetDenoiseOptions.Preprocessing
    ) throws -> MeshPreprocessDiagnostics? {
        switch options {
        case .none:
            return nil
        case .conservativeValidation(let preprocessOptions):
            let diagnostics = try MeshPreprocessor.preservedTopologyDiagnostics(
                positions: positions,
                indices: indices,
                options: preprocessOptions
            )
            guard diagnostics.degenerateFacesBefore == 0,
                  diagnostics.nonManifoldEdgesBefore == 0,
                  !MeshPreprocessor.hasDuplicateFaces(indices),
                  !MeshPreprocessor.hasDuplicateDirectedEdges(indices)
            else {
                throw MeshAssetDenoiseError.preprocessingWouldChangeTopology
            }
            return diagnostics
        }
    }

    private static func loadPositions(from mesh: MDLMesh) throws -> [SIMD3<Float>] {
        guard let attributeData = mesh.vertexAttributeData(
            forAttributeNamed: MDLVertexAttributePosition,
            as: .float3
        ) else {
            throw MeshAssetDenoiseError.unsupportedAsset("Mesh is missing float3 positions.")
        }

        let stride = attributeData.stride
        let vertexCount = mesh.vertexCount
        let dataStart = attributeData.dataStart
        var positions = [SIMD3<Float>]()
        positions.reserveCapacity(vertexCount)

        for vertexIndex in 0..<vertexCount {
            let pointer = dataStart
                .advanced(by: vertexIndex * stride)
                .assumingMemoryBound(to: SIMD3<Float>.self)
            positions.append(pointer.pointee)
        }
        return positions
    }

    private static func writePositions(_ positions: [SIMD3<Float>], to mesh: MDLMesh) throws {
        guard positions.count == mesh.vertexCount,
              let attributeData = mesh.vertexAttributeData(
                forAttributeNamed: MDLVertexAttributePosition,
                as: .float3
              )
        else {
            throw MeshAssetDenoiseError.unsupportedAsset("Mesh positions cannot be written without changing topology.")
        }
        for position in positions {
            guard position.x.isFinite, position.y.isFinite, position.z.isFinite else {
                throw MeshAssetDenoiseError.unsupportedAsset("Denoised mesh contains non-finite positions.")
            }
        }

        let stride = attributeData.stride
        let dataStart = attributeData.dataStart
        for (vertexIndex, position) in positions.enumerated() {
            let pointer = dataStart
                .advanced(by: vertexIndex * stride)
                .assumingMemoryBound(to: SIMD3<Float>.self)
            pointer.pointee = position
        }
    }

    private static func loadTriangleIndices(from mesh: MDLMesh) throws -> [UInt32] {
        guard let submeshes = mesh.submeshes as? [MDLSubmesh], !submeshes.isEmpty else {
            throw MeshAssetDenoiseError.unsupportedAsset("Mesh contains no submeshes.")
        }

        var indices = [UInt32]()
        for submesh in submeshes {
            indices.append(contentsOf: try loadTriangleIndices(from: submesh))
        }
        return indices
    }

    private static func loadTriangleIndices(from submesh: MDLSubmesh) throws -> [UInt32] {
        let triangleSubmesh: MDLSubmesh
        if submesh.geometryType == .triangles {
            triangleSubmesh = submesh
        } else if let converted = MDLSubmesh(
            mdlSubmesh: submesh,
            indexType: .uInt32,
            geometryType: .triangles
        ) {
            triangleSubmesh = converted
        } else {
            throw MeshAssetDenoiseError.unsupportedAsset("Submesh cannot be converted to triangles.")
        }

        let indexBuffer = triangleSubmesh.indexBuffer(asIndexType: .uInt32)
        let map = indexBuffer.map()
        let pointer = map.bytes.assumingMemoryBound(to: UInt32.self)
        let count = triangleSubmesh.indexCount
        guard count.isMultiple(of: 3) else {
            throw MeshAssetDenoiseError.unsupportedAsset("Triangle index count is not divisible by 3.")
        }

        var indices = [UInt32]()
        indices.reserveCapacity(count)
        for index in 0..<count {
            indices.append(pointer[index])
        }
        return indices
    }

}
#endif
