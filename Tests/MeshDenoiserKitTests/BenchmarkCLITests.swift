import XCTest

final class BenchmarkCLITests: XCTestCase {

    func testNoReferenceCheckAllowsNativeRunWithoutErrorColumns() throws {
        let result = try runBench([
            "--faces", "64",
            "--backend", "nativeCPU",
            "--no-reference-check",
        ])

        XCTAssertEqual(result.status, 0, result.stderr)
        let lines = result.stdout.split(separator: "\n").map(String.init)
        XCTAssertEqual(lines.first, "backend,vertices,faces,total_secs,peak_memory_mb")
        XCTAssertEqual(lines.dropFirst().count, 1)
        guard lines.count > 1 else { return }
        XCTAssertTrue(lines[1].hasPrefix("nativeCPU,"))
    }

    private func runBench(_ arguments: [String]) throws -> (status: Int32, stdout: String, stderr: String) {
        let packageRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let executable = packageRoot
            .appendingPathComponent(".build/debug/MeshDenoiserBench")

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
}
