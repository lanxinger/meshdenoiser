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

public struct MeshAssetDenoisePreviewMesh: Sendable, Equatable {
    public var originalPositions: [SIMD3<Float>]
    public var denoisedPositions: [SIMD3<Float>]
    public var indices: [UInt32]

    public init(
        originalPositions: [SIMD3<Float>],
        denoisedPositions: [SIMD3<Float>],
        indices: [UInt32]
    ) {
        self.originalPositions = originalPositions
        self.denoisedPositions = denoisedPositions
        self.indices = indices
    }
}

public struct MeshAssetDenoisePreview: Sendable, Equatable {
    public var summary: MeshAssetDenoiseSummary
    public var meshes: [MeshAssetDenoisePreviewMesh]

    public init(
        summary: MeshAssetDenoiseSummary,
        meshes: [MeshAssetDenoisePreviewMesh]
    ) {
        self.summary = summary
        self.meshes = meshes
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
    struct DenoisedMeshPatch {
        var originalPositions: [SIMD3<Float>]
        var denoisedPositions: [SIMD3<Float>]
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

    /// Denoises all ModelIO meshes in an asset and returns geometry buffers for
    /// fast preview without writing or mutating the source asset.
    public static func preview(
        inputURL: URL,
        options: MeshAssetDenoiseOptions = MeshAssetDenoiseOptions(),
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> MeshAssetDenoisePreview {
        try await preview(
            asset: MDLAsset(url: inputURL),
            options: options,
            progress: progress
        )
    }

    /// Denoises all ModelIO meshes in an already loaded asset and returns
    /// geometry buffers for fast preview without writing or mutating the asset.
    public static func preview(
        asset: MDLAsset,
        options: MeshAssetDenoiseOptions = MeshAssetDenoiseOptions(),
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> MeshAssetDenoisePreview {
        guard options.parameters.isValid else {
            throw MeshDenoiseError.invalidParameters
        }

        let meshes = asset.childObjects(of: MDLMesh.self).compactMap { $0 as? MDLMesh }
        guard !meshes.isEmpty else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no ModelIO meshes.")
        }

        progress?(0)

        var summary = MeshAssetDenoiseSummary(meshesProcessed: 0, verticesProcessed: 0, facesProcessed: 0)
        var previewMeshes = [MeshAssetDenoisePreviewMesh]()
        previewMeshes.reserveCapacity(meshes.count)

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

            let baseProgress = Double(meshIndex) / Double(meshes.count)
            let meshProgressScale = 1 / Double(meshes.count)
            let denoised = try await MeshDenoiser.denoise(
                positions: positions,
                indices: indices,
                parameters: options.parameters
            ) { meshProgress in
                progress?(baseProgress + meshProgress * meshProgressScale)
            }

            previewMeshes.append(MeshAssetDenoisePreviewMesh(
                originalPositions: positions,
                denoisedPositions: denoised,
                indices: indices
            ))
            summary.meshesProcessed += 1
            summary.verticesProcessed += positions.count
            summary.facesProcessed += indices.count / 3
            progress?(Double(meshIndex + 1) / Double(meshes.count))
        }

        guard summary.meshesProcessed > 0 else {
            throw MeshAssetDenoiseError.unsupportedAsset("Asset contains no triangle mesh data.")
        }
        progress?(1)
        return MeshAssetDenoisePreview(summary: summary, meshes: previewMeshes)
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
                denoisedMeshes: denoisedMeshes,
                recomputesNormals: options.recomputesNormals
            )
        }
        progress?(1)
        return summary
    }

    private static func exportUSDZByPatchingPoints(
        inputURL: URL,
        outputURL: URL,
        denoisedMeshes: [DenoisedMeshPatch],
        recomputesNormals: Bool
    ) throws {
        let fileManager = FileManager.default
        let workingDirectory = fileManager.temporaryDirectory
            .appendingPathComponent("MeshDenoiserUSDZ-\(UUID().uuidString)", isDirectory: true)
        try fileManager.createDirectory(at: workingDirectory, withIntermediateDirectories: true)
        defer { try? fileManager.removeItem(at: workingDirectory) }

        try runTool("/usr/bin/ditto", arguments: ["-x", "-k", inputURL.path, workingDirectory.path])

        // Write the patched layer next to the root layer so its relative asset paths still resolve.
        let rootPath = try rootLayerPath(ofUSDZ: inputURL, extractedTo: workingDirectory)
        let rootDirectory = (rootPath as NSString).deletingLastPathComponent
        var usdaName = "denoised.usda"
        if fileManager.fileExists(atPath: workingDirectory.appendingPathComponent(rootDirectory).appendingPathComponent(usdaName).path) {
            usdaName = "denoised-\(UUID().uuidString).usda"
        }
        let usdaPath = rootDirectory.isEmpty ? usdaName : rootDirectory + "/" + usdaName
        let rootURL = workingDirectory.appendingPathComponent(rootPath)
        let usdaURL = workingDirectory.appendingPathComponent(usdaPath)

        // Patch the root layer as authored; if meshes live in other layers (references,
        // sublayers), patch a flattened copy of the composed stage instead.
        try runTool("/usr/bin/usdcat", arguments: [rootURL.path, "-o", usdaURL.path])
        do {
            try patchUSDAPoints(at: usdaURL, denoisedMeshes: denoisedMeshes, recomputesNormals: recomputesNormals)
        } catch is USDAMeshPatcher.UnmatchedMeshes {
            try runTool("/usr/bin/usdcat", arguments: ["--flatten", rootURL.path, "-o", usdaURL.path])
            do {
                try patchUSDAPoints(at: usdaURL, denoisedMeshes: denoisedMeshes, recomputesNormals: recomputesNormals)
            } catch is USDAMeshPatcher.UnmatchedMeshes {
                throw MeshAssetDenoiseError.exportFailed("Denoised meshes could not be located in \(inputURL.lastPathComponent).")
            }
        }
        try runTool(
            "/usr/bin/usdzip",
            arguments: [outputURL.path, "--asset", usdaPath],
            currentDirectoryURL: workingDirectory
        )
    }

    /// The USDZ spec makes the first file in the archive the root layer, so read the
    /// first local file header rather than guessing from the extracted file names.
    static func rootLayerPath(ofUSDZ archiveURL: URL, extractedTo directory: URL) throws -> String {
        if let path = try firstArchiveEntryName(archiveURL),
           ["usd", "usda", "usdc"].contains((path as NSString).pathExtension.lowercased()),
           !path.hasPrefix("/"),
           !path.split(separator: "/").contains(".."),
           FileManager.default.fileExists(atPath: directory.appendingPathComponent(path).path) {
            return path
        }

        // Non-conforming archive: fall back to a top-level USD layer.
        let contents = try FileManager.default.contentsOfDirectory(
            at: directory,
            includingPropertiesForKeys: nil,
            options: [.skipsHiddenFiles]
        )
        guard let root = contents
            .map(\.lastPathComponent)
            .filter({ ["usd", "usda", "usdc"].contains(($0 as NSString).pathExtension.lowercased()) })
            .sorted()
            .first
        else {
            throw MeshAssetDenoiseError.exportFailed(directory.path)
        }
        return root
    }

    private static func firstArchiveEntryName(_ archiveURL: URL) throws -> String? {
        let handle = try FileHandle(forReadingFrom: archiveURL)
        defer { try? handle.close() }
        guard let header = try handle.read(upToCount: 30), header.count == 30 else { return nil }
        let bytes = [UInt8](header)
        func uint16(_ offset: Int) -> Int { Int(bytes[offset]) | Int(bytes[offset + 1]) << 8 }
        // Local file header signature "PK\u{3}\u{4}".
        guard bytes[0] == 0x50, bytes[1] == 0x4B, bytes[2] == 0x03, bytes[3] == 0x04 else { return nil }
        let nameLength = uint16(26)
        guard nameLength > 0, let name = try handle.read(upToCount: nameLength), name.count == nameLength else {
            return nil
        }
        return String(data: name, encoding: .utf8)
    }

    static func patchUSDAPoints(at url: URL, denoisedMeshes: [DenoisedMeshPatch], recomputesNormals: Bool) throws {
        let text = [UInt8](try Data(contentsOf: url))
        let patched = try USDAMeshPatcher.patch(
            text,
            denoisedMeshes: denoisedMeshes,
            recomputesNormals: recomputesNormals
        )
        try Data(patched).write(to: url, options: .atomic)
    }

    static func runTool(
        _ executablePath: String,
        arguments: [String],
        currentDirectoryURL: URL? = nil
    ) throws {
        let process = Process()
        process.executableURL = URL(fileURLWithPath: executablePath)
        process.arguments = arguments
        process.currentDirectoryURL = currentDirectoryURL

        let outputPipe = Pipe()
        process.standardOutput = outputPipe
        process.standardError = outputPipe

        do {
            try process.run()
        } catch {
            throw MeshAssetDenoiseError.exportFailed(executablePath)
        }
        // Drain while the child runs so either output stream can exceed pipe capacity.
        // Retain only bounded diagnostics; discard the rest after reading it.
        var diagnostics = Data()
        let diagnosticLimit = 64 * 1024
        while true {
            let chunk = outputPipe.fileHandleForReading.readData(ofLength: diagnosticLimit)
            if chunk.isEmpty { break }
            diagnostics.append(chunk.prefix(max(0, diagnosticLimit - diagnostics.count)))
        }
        process.waitUntilExit()

        guard process.terminationStatus == 0 else {
            let message = String(data: diagnostics, encoding: .utf8) ?? executablePath
            throw MeshAssetDenoiseError.exportFailed(message.trimmingCharacters(in: .whitespacesAndNewlines))
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

        return withExtendedLifetime(attributeData) {
            readFloat3Attribute(
                at: UnsafeRawPointer(attributeData.dataStart),
                stride: attributeData.stride,
                count: mesh.vertexCount
            )
        }
    }

    /// Reads packed float3 values. ModelIO's `.float3` is 12 bytes, whereas
    /// `SIMD3<Float>` is 16, so loading or storing a whole `SIMD3<Float>` would
    /// touch the first 4 bytes of the next attribute (or run past the buffer).
    static func readFloat3Attribute(at dataStart: UnsafeRawPointer, stride: Int, count: Int) -> [SIMD3<Float>] {
        var values = [SIMD3<Float>]()
        values.reserveCapacity(count)
        for index in 0..<count {
            let offset = index * stride
            values.append(SIMD3<Float>(
                dataStart.loadUnaligned(fromByteOffset: offset, as: Float.self),
                dataStart.loadUnaligned(fromByteOffset: offset + 4, as: Float.self),
                dataStart.loadUnaligned(fromByteOffset: offset + 8, as: Float.self)
            ))
        }
        return values
    }

    static func writeFloat3Attribute(_ values: [SIMD3<Float>], to dataStart: UnsafeMutableRawPointer, stride: Int) {
        for (index, value) in values.enumerated() {
            let offset = index * stride
            dataStart.storeBytes(of: value.x, toByteOffset: offset, as: Float.self)
            dataStart.storeBytes(of: value.y, toByteOffset: offset + 4, as: Float.self)
            dataStart.storeBytes(of: value.z, toByteOffset: offset + 8, as: Float.self)
        }
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

        withExtendedLifetime(attributeData) {
            writeFloat3Attribute(positions, to: attributeData.dataStart, stride: attributeData.stride)
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
        let count = triangleSubmesh.indexCount
        guard count.isMultiple(of: 3) else {
            throw MeshAssetDenoiseError.unsupportedAsset("Triangle index count is not divisible by 3.")
        }

        // The mapped bytes are only valid while the map object is alive.
        let map = indexBuffer.map()
        return withExtendedLifetime(map) {
            Array(UnsafeBufferPointer(start: map.bytes.assumingMemoryBound(to: UInt32.self), count: count))
        }
    }

}
#endif
