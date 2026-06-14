#if os(macOS)
import ModelIO
import SceneKit
import SceneKit.ModelIO
import XCTest

final class AssetProcessorCLITests: XCTestCase {

    func testProcessCLIWritesUSDZOutputAndSummary() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let inputURL = directory.appendingPathComponent("input.usdz")
        let outputURL = directory.appendingPathComponent("output.usdz")
        try writeUSDZ(asset: makeBoxAsset(), to: inputURL)

        let result = try runProcessor([
            "--input", inputURL.path,
            "--output", outputURL.path,
            "--backend", "nativeCPU",
        ])

        XCTAssertEqual(result.status, 0, result.stderr)
        XCTAssertTrue(FileManager.default.fileExists(atPath: outputURL.path))
        XCTAssertEqual(result.stdout.split(separator: "\n").map(String.init).first, "meshes,vertices,faces")
        XCTAssertTrue(result.stdout.contains("1,"))
    }

    private func runProcessor(_ arguments: [String]) throws -> (status: Int32, stdout: String, stderr: String) {
        let packageRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let executable = packageRoot
            .appendingPathComponent(".build/debug/MeshDenoiserProcess")

        let process = Process()
        process.executableURL = executable
        process.arguments = arguments

        let stdout = Pipe()
        let stderr = Pipe()
        process.standardOutput = stdout
        process.standardError = stderr
        try process.run()
        process.waitUntilExit()

        let stdoutText = String(data: stdout.fileHandleForReading.readDataToEndOfFile(), encoding: .utf8) ?? ""
        let stderrText = String(data: stderr.fileHandleForReading.readDataToEndOfFile(), encoding: .utf8) ?? ""
        return (process.terminationStatus, stdoutText, stderrText)
    }

    private func makeBoxAsset() -> MDLAsset {
        let mesh = MDLMesh(
            boxWithExtent: SIMD3<Float>(1, 1, 1),
            segments: SIMD3<UInt32>(1, 1, 1),
            inwardNormals: false,
            geometryType: .triangles,
            allocator: nil
        )
        let asset = MDLAsset()
        asset.add(mesh)
        return asset
    }

    private func writeUSDZ(asset: MDLAsset, to url: URL) throws {
        let scene = SCNScene(mdlAsset: asset)
        guard scene.write(to: url, options: nil, delegate: nil, progressHandler: nil) else {
            throw NSError(domain: "AssetProcessorCLITests", code: 1)
        }
    }
}
#endif
