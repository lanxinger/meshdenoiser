import Darwin
import Foundation
import MeshDenoiserKit
import MeshDenoiserNative
#if os(macOS)
import ModelIO
#endif

struct Arguments {
    var faces = 20_000
    var backend = "all"
    var inputPath: String?
    var inputDirPath: String?
    var inputListPath: String?
    var mode = "denoise"
    var printsHeader = true
    var keepsGoing = false
    var failsOnError = false
    var maxErrorThreshold: Double?
    var meanErrorThreshold: Double?
    var maxTotalSecondsThreshold: Double?
    var maxMemoryMBThreshold: Double?
    var repairMode = "none"
    var tracesMemory = false
    var checksReference = true

    init(_ raw: [String]) throws {
        var index = 1
        while index < raw.count {
            switch raw[index] {
            case "--faces":
                guard index + 1 < raw.count, let value = Int(raw[index + 1]), value > 0 else {
                    throw BenchError.invalidArguments
                }
                faces = value
                index += 2
            case "--backend":
                guard index + 1 < raw.count else { throw BenchError.invalidArguments }
                backend = raw[index + 1]
                index += 2
            case "--input":
                guard index + 1 < raw.count else { throw BenchError.invalidArguments }
                inputPath = raw[index + 1]
                index += 2
            case "--input-dir":
                guard index + 1 < raw.count else { throw BenchError.invalidArguments }
                inputDirPath = raw[index + 1]
                index += 2
            case "--input-list":
                guard index + 1 < raw.count else { throw BenchError.invalidArguments }
                inputListPath = raw[index + 1]
                index += 2
            case "--mode":
                guard index + 1 < raw.count else { throw BenchError.invalidArguments }
                mode = raw[index + 1]
                index += 2
            case "--no-header":
                printsHeader = false
                index += 1
            case "--keep-going":
                keepsGoing = true
                index += 1
            case "--fail-on-error":
                failsOnError = true
                index += 1
            case "--max-error-threshold":
                guard index + 1 < raw.count, let value = Double(raw[index + 1]), value >= 0 else {
                    throw BenchError.invalidArguments
                }
                maxErrorThreshold = value
                index += 2
            case "--mean-error-threshold":
                guard index + 1 < raw.count, let value = Double(raw[index + 1]), value >= 0 else {
                    throw BenchError.invalidArguments
                }
                meanErrorThreshold = value
                index += 2
            case "--max-total-seconds-threshold":
                guard index + 1 < raw.count, let value = Double(raw[index + 1]), value >= 0 else {
                    throw BenchError.invalidArguments
                }
                maxTotalSecondsThreshold = value
                index += 2
            case "--max-memory-mb-threshold":
                guard index + 1 < raw.count, let value = Double(raw[index + 1]), value >= 0 else {
                    throw BenchError.invalidArguments
                }
                maxMemoryMBThreshold = value
                index += 2
            case "--repair":
                guard index + 1 < raw.count else { throw BenchError.invalidArguments }
                repairMode = raw[index + 1]
                index += 2
            case "--trace-memory":
                tracesMemory = true
                index += 1
            case "--no-reference-check":
                checksReference = false
                index += 1
            case "--help", "-h":
                throw BenchError.help
            default:
                throw BenchError.invalidArguments
            }
        }

        let inputSourceCount = [inputPath, inputDirPath, inputListPath].compactMap { $0 }.count
        guard inputSourceCount <= 1 else { throw BenchError.invalidArguments }
        guard repairMode == "none" || repairMode == "conservative" else { throw BenchError.invalidArguments }
        guard checksReference || (maxErrorThreshold == nil && meanErrorThreshold == nil) else {
            throw BenchError.invalidArguments
        }
    }
}

enum BenchError: Error {
    case help
    case invalidArguments
    case childFailed(String)
    case unsupportedInput(String)
    case batchFailures(Int)
}

struct Mesh {
    var positions: [SIMD3<Float>]
    var indices: [UInt32]
    var repairDiagnostics: MeshPreprocessDiagnostics?
}

struct MeshDiagnostics {
    var vertices: Int
    var faces: Int
    var boundaryEdges: Int
    var nonManifoldEdges: Int
    var degenerateFaces: Int
}

struct BatchDiagnostics {
    var mesh: MeshDiagnostics
    var repair: MeshPreprocessDiagnostics?
}

struct BenchmarkRow {
    var backend: String
    var vertices: Int
    var faces: Int
    var totalSeconds: Double
    var maxError: Double?
    var meanError: Double?
    var peakMemoryMB: Double
    var repairDiagnostics: MeshPreprocessDiagnostics?
}

struct VertexUpdateBenchmarkRow {
    var backend: String
    var vertices: Int
    var faces: Int
    var totalSeconds: Double
    var peakMemoryMB: Double
    var checksum: Double
}

@main
enum MeshDenoiserBench {
    static func main() async {
        do {
            let args = try Arguments(CommandLine.arguments)
            try await run(args: args)
        } catch BenchError.help {
            printUsage()
        } catch BenchError.batchFailures(let count) {
            fputs("error: \(count) benchmark row(s) failed\n", stderr)
            exit(1)
        } catch {
            printUsage()
            fputs("error: \(error)\n", stderr)
            exit(2)
        }
    }

    private static func run(args: Arguments) async throws {
        if args.mode == "vertex-update" {
            guard args.inputDirPath == nil, args.inputListPath == nil else {
                throw BenchError.invalidArguments
            }
            try runVertexUpdate(args: args)
            return
        }
        if args.mode == "filter" {
            guard args.inputDirPath == nil, args.inputListPath == nil else {
                throw BenchError.invalidArguments
            }
            try runFilter(args: args)
            return
        }
        guard args.mode == "denoise" else { throw BenchError.invalidArguments }

        if args.inputDirPath != nil || args.inputListPath != nil {
            try runBatchInputs(args: args)
            return
        }

        if args.backend == "all" {
            try runAllInChildProcesses(args: args)
            return
        }

        let mesh = try loadMesh(args: args)
        let backends = try selectedBackends(args.backend)
        traceMemory("mesh_ready", args: args)

        if args.printsHeader {
            Swift.print(benchmarkHeader(args: args))
            fflush(stdout)
        }

        for backend in backends {
            traceMemory("before_\(backend.name)", args: args)
            let run = try await measureDenoise(
                backend: backend,
                positions: mesh.positions,
                indices: mesh.indices
            )
            let backendMemory = peakMemoryMB()
            traceMemory("after_\(backend.name)", args: args)

            if backend == .reference {
                print(row: BenchmarkRow(
                    backend: "reference",
                    vertices: mesh.positions.count,
                    faces: mesh.indices.count / 3,
                    totalSeconds: run.elapsed,
                    maxError: args.checksReference ? 0 : nil,
                    meanError: args.checksReference ? 0 : nil,
                    peakMemoryMB: backendMemory,
                    repairDiagnostics: mesh.repairDiagnostics
                ))
                continue
            }

            let error: (max: Double, mean: Double)?
            if args.checksReference {
                traceMemory("before_reference_for_\(backend.name)", args: args)
                let referenceRun = try await measureDenoise(
                    backend: .reference,
                    positions: mesh.positions,
                    indices: mesh.indices
                )
                traceMemory("after_reference_for_\(backend.name)", args: args)
                error = compare(run.result, to: referenceRun.result)
            } else {
                error = nil
            }
            print(row: BenchmarkRow(
                backend: backend.name,
                vertices: mesh.positions.count,
                faces: mesh.indices.count / 3,
                totalSeconds: run.elapsed,
                maxError: error?.max,
                meanError: error?.mean,
                peakMemoryMB: backendMemory,
                repairDiagnostics: mesh.repairDiagnostics
            ))
        }
    }

    private static func runBatchInputs(args: Arguments) throws {
        let inputs = try batchInputURLs(args: args)
        guard !inputs.isEmpty else {
            throw BenchError.unsupportedInput("batch input did not contain any supported mesh files")
        }
        let backends = try selectedBackendNames(args.backend)

        if args.printsHeader {
            var header = "input,\(benchmarkHeader(args: args))"
            if args.keepsGoing {
                header += ",status,error"
            }
            Swift.print(header)
            fflush(stdout)
        }

        var failureCount = 0
        for input in inputs {
            let diagnostics = try loadDiagnostics(url: input, args: args)
            for backend in backends {
                let result = try runChildBackend(args: args, inputPath: input.path, backend: backend)
                if result.status == 0 {
                    if !result.stderr.isEmpty {
                        FileHandle.standardError.write(result.stderr)
                    }
                    for line in outputLines(result.stdout) {
                        var row = "\(csvField(input.path)),\(line)"
                        if let thresholdFailure = thresholdFailureMessage(line: line, args: args) {
                            failureCount += 1
                            if args.keepsGoing {
                                row += ",failed,\(csvField(thresholdFailure))"
                            } else {
                                throw BenchError.childFailed("\(backend): \(thresholdFailure)")
                            }
                        } else if args.keepsGoing {
                            row += ",ok,"
                        }
                        Swift.print(row)
                    }
                } else if args.keepsGoing {
                    Swift.print(failureBatchRow(
                        inputPath: input.path,
                        backend: backend,
                        result: result,
                        diagnostics: diagnostics,
                        args: args
                    ))
                    failureCount += 1
                } else {
                    throw BenchError.childFailed("\(backend): \(failureMessage(result))")
                }
            }
        }

        if args.keepsGoing {
            fputs(
                "batch summary: inputs=\(inputs.count) backend_runs=\(inputs.count * backends.count) failed=\(failureCount)\n",
                stderr
            )
        }
        if args.failsOnError, failureCount > 0 {
            throw BenchError.batchFailures(failureCount)
        }
    }

    private static func runVertexUpdate(args: Arguments) throws {
        guard args.backend == "all" else { throw BenchError.invalidArguments }

        let mesh = try loadMesh(args: args)
        if args.printsHeader {
            Swift.print("mode,backend,vertices,faces,total_secs,peak_memory_mb,checksum")
            fflush(stdout)
        }

        let start = DispatchTime.now().uptimeNanoseconds
        let output = try NativeBenchmarkSupport.runVertexUpdate(
            positions: mesh.positions,
            indices: mesh.indices
        )
        let elapsed = Double(DispatchTime.now().uptimeNanoseconds - start) / 1_000_000_000
        print(row: VertexUpdateBenchmarkRow(
            backend: "nativeCPU",
            vertices: mesh.positions.count,
            faces: mesh.indices.count / 3,
            totalSeconds: elapsed,
            peakMemoryMB: peakMemoryMB(),
            checksum: checksum(output)
        ))
    }

    private static func runFilter(args: Arguments) throws {
        let backends = try selectedBackends(args.backend)
            .filter { $0 == .nativeCPU || $0 == .nativeGPU }
        guard !backends.isEmpty else { throw BenchError.invalidArguments }

        let mesh = try loadMesh(args: args)
        if args.printsHeader {
            Swift.print("mode,backend,vertices,faces,total_secs,peak_memory_mb,checksum")
            fflush(stdout)
        }

        for backend in backends {
            let start = DispatchTime.now().uptimeNanoseconds
            let output = try NativeBenchmarkSupport.runFilter(
                positions: mesh.positions,
                indices: mesh.indices,
                useGPU: backend == .nativeGPU
            )
            let elapsed = Double(DispatchTime.now().uptimeNanoseconds - start) / 1_000_000_000
            Swift.print(
                "filter,\(backend.name),\(mesh.positions.count),\(mesh.indices.count / 3),"
                    + "\(elapsed),\(peakMemoryMB()),\(checksum(output))"
            )
        }
    }

    private static func runAllInChildProcesses(args: Arguments) throws {
        if args.printsHeader {
            Swift.print(benchmarkHeader(args: args))
            fflush(stdout)
        }

        try runAllInChildProcesses(args: args) { line in
            Swift.print(line)
        }
    }

    private static func runAllInChildProcesses(
        args: Arguments,
        printLine: (String) -> Void
    ) throws {

        for backend in ["reference", "nativeCPU", "nativeGPU"] {
            let result = try runChildBackend(args: args, inputPath: args.inputPath, backend: backend)
            if !result.stderr.isEmpty {
                FileHandle.standardError.write(result.stderr)
            }
            guard result.status == 0 else {
                throw BenchError.childFailed("\(backend): \(failureMessage(result))")
            }
            for line in outputLines(result.stdout) {
                printLine(line)
            }
        }
    }

    private static func runChildBackend(
        args: Arguments,
        inputPath: String?,
        backend: String
    ) throws -> ChildRunResult {
        let process = Process()
        process.executableURL = URL(fileURLWithPath: CommandLine.arguments[0])
        process.arguments = Arguments.childArguments(from: args, inputPath: inputPath, backend: backend)

        let stdout = Pipe()
        let stderr = Pipe()
        process.standardOutput = stdout
        process.standardError = stderr
        try process.run()
        let stdoutData = stdout.fileHandleForReading.readDataToEndOfFile()
        let stderrData = stderr.fileHandleForReading.readDataToEndOfFile()
        process.waitUntilExit()

        return ChildRunResult(
            status: process.terminationStatus,
            stdout: stdoutData,
            stderr: stderrData
        )
    }

    private static func measureDenoise(
        backend: MeshDenoiseParameters.Backend,
        positions: [SIMD3<Float>],
        indices: [UInt32]
    ) async throws -> (result: [SIMD3<Float>], elapsed: Double) {
        var params = MeshDenoiseParameters()
        params.backend = backend
        let start = DispatchTime.now().uptimeNanoseconds
        let result = try await withSuppressedStdout {
            try await MeshDenoiser.denoise(
                positions: positions,
                indices: indices,
                parameters: params
            )
        }
        let elapsed = Double(DispatchTime.now().uptimeNanoseconds - start) / 1_000_000_000
        return (result, elapsed)
    }

    private static func withSuppressedStdout<T>(
        _ body: () async throws -> T
    ) async throws -> T {
        fflush(stdout)
        let saved = dup(STDOUT_FILENO)
        guard saved >= 0 else { return try await body() }
        let null = open("/dev/null", O_WRONLY)
        guard null >= 0 else {
            close(saved)
            return try await body()
        }

        dup2(null, STDOUT_FILENO)
        close(null)
        defer {
            fflush(stdout)
            dup2(saved, STDOUT_FILENO)
            close(saved)
        }

        return try await body()
    }

    private static func selectedBackends(_ raw: String) throws -> [MeshDenoiseParameters.Backend] {
        switch raw {
        case "all":
            return [.reference, .nativeCPU, .nativeGPU]
        case "reference":
            return [.reference]
        case "nativeCPU":
            return [.nativeCPU]
        case "nativeGPU":
            return [.nativeGPU]
        default:
            throw BenchError.invalidArguments
        }
    }

    private static func selectedBackendNames(_ raw: String) throws -> [String] {
        _ = try selectedBackends(raw)
        if raw == "all" {
            return ["reference", "nativeCPU", "nativeGPU"]
        }
        return [raw]
    }

    private static func print(row: BenchmarkRow) {
        var fields = [
            row.backend,
            String(row.vertices),
            String(row.faces),
            String(row.totalSeconds),
        ]
        if let maxError = row.maxError, let meanError = row.meanError {
            fields += [String(maxError), String(meanError)]
        }
        fields.append(String(row.peakMemoryMB))
        if let repairDiagnostics = row.repairDiagnostics {
            fields += repairCSVFields(repairDiagnostics)
        }
        Swift.print(fields.joined(separator: ","))
    }

    private static func print(row: VertexUpdateBenchmarkRow) {
        Swift.print(
            "vertex-update,\(row.backend),\(row.vertices),\(row.faces),"
                + "\(row.totalSeconds),\(row.peakMemoryMB),\(row.checksum)"
        )
    }

    private static func compare(
        _ result: [SIMD3<Float>],
        to reference: [SIMD3<Float>]
    ) -> (max: Double, mean: Double) {
        guard result.count == reference.count, !result.isEmpty else {
            return (.infinity, .infinity)
        }

        var maxError = 0.0
        var sumError = 0.0
        for (got, want) in zip(result, reference) {
            let diff = got - want
            let error = Double((diff.x * diff.x + diff.y * diff.y + diff.z * diff.z).squareRoot())
            maxError = max(maxError, error)
            sumError += error
        }
        return (maxError, sumError / Double(result.count))
    }

    private static func checksum(_ positions: [SIMD3<Float>]) -> Double {
        positions.reduce(0) { partial, position in
            partial
                + Double(position.x) * 0.31
                + Double(position.y) * 0.37
                + Double(position.z) * 0.41
        }
    }

    private static func loadMesh(args: Arguments) throws -> Mesh {
        guard let inputPath = args.inputPath else {
            return try repairIfNeeded(generateNoisySphere(targetFaces: args.faces), args: args)
        }
        let url = fileURL(path: inputPath)
        let mesh: Mesh
        switch url.pathExtension.lowercased() {
        case "obj":
            mesh = try loadOBJ(url: url)
        case "usd", "usda", "usdc", "usdz":
            mesh = try loadUSD(url: url)
        default:
            throw BenchError.unsupportedInput("unsupported input extension: \(url.pathExtension)")
        }
        return try repairIfNeeded(mesh, args: args)
    }

    private static func repairIfNeeded(_ mesh: Mesh, args: Arguments) throws -> Mesh {
        traceMemory("raw_mesh_loaded", args: args)
        switch args.repairMode {
        case "none":
            return mesh
        case "conservative":
            let repaired = try MeshPreprocessor.repairForDenoising(
                positions: mesh.positions,
                indices: mesh.indices,
                options: .conservative()
            )
            let mesh = Mesh(
                positions: repaired.positions,
                indices: repaired.indices,
                repairDiagnostics: repaired.diagnostics
            )
            traceMemory("mesh_repaired", args: args)
            return mesh
        default:
            throw BenchError.invalidArguments
        }
    }

    private static func loadDiagnostics(url: URL, args: Arguments) throws -> BatchDiagnostics? {
        do {
            var rawArgs = ["MeshDenoiserBench", "--input", url.path]
            if args.repairMode != "none" {
                rawArgs += ["--repair", args.repairMode]
            }
            let mesh = try loadMesh(args: try Arguments(rawArgs))
            return BatchDiagnostics(
                mesh: MeshDiagnostics(mesh: mesh),
                repair: mesh.repairDiagnostics
            )
        } catch {
            return nil
        }
    }

    private static func batchInputURLs(args: Arguments) throws -> [URL] {
        if let inputDirPath = args.inputDirPath {
            return try discoverInputURLs(in: fileURL(path: inputDirPath))
        }
        if let inputListPath = args.inputListPath {
            return try loadInputList(url: fileURL(path: inputListPath))
        }
        return []
    }

    private static func discoverInputURLs(in directory: URL) throws -> [URL] {
        var isDirectory = ObjCBool(false)
        guard FileManager.default.fileExists(atPath: directory.path, isDirectory: &isDirectory),
              isDirectory.boolValue
        else {
            throw BenchError.unsupportedInput("input directory does not exist: \(directory.path)")
        }

        guard let enumerator = FileManager.default.enumerator(
            at: directory,
            includingPropertiesForKeys: [.isRegularFileKey, .isSymbolicLinkKey],
            options: [.skipsHiddenFiles]
        ) else {
            throw BenchError.unsupportedInput("cannot read input directory: \(directory.path)")
        }

        var urls = [URL]()
        for case let url as URL in enumerator {
            guard supportedInputExtensions.contains(url.pathExtension.lowercased()) else { continue }
            let values = try url.resourceValues(forKeys: [.isRegularFileKey, .isSymbolicLinkKey])
            if values.isRegularFile == true || values.isSymbolicLink == true {
                urls.append(url)
            }
        }
        return urls.sorted { $0.path.localizedStandardCompare($1.path) == .orderedAscending }
    }

    private static func loadInputList(url: URL) throws -> [URL] {
        let text = try String(contentsOf: url, encoding: .utf8)
        let base = url.deletingLastPathComponent()
        var urls = [URL]()
        for rawLine in text.split(separator: "\n") {
            let line = rawLine.trimmingCharacters(in: .whitespaces)
            guard !line.isEmpty, !line.hasPrefix("#") else { continue }
            let inputURL = line.hasPrefix("/")
                ? URL(fileURLWithPath: line)
                : base.appendingPathComponent(line)
            guard supportedInputExtensions.contains(inputURL.pathExtension.lowercased()) else {
                throw BenchError.unsupportedInput("unsupported input extension in list: \(line)")
            }
            urls.append(inputURL)
        }
        return urls
    }

    private static func loadOBJ(url: URL) throws -> Mesh {
        var positions = [SIMD3<Float>]()
        var indices = [UInt32]()

        for rawLine in try String(contentsOf: url, encoding: .utf8).split(separator: "\n") {
            let line = rawLine.split(separator: "#", maxSplits: 1, omittingEmptySubsequences: false)[0]
            let parts = line.split(whereSeparator: \.isWhitespace)
            guard let tag = parts.first else { continue }

            if tag == "v" {
                guard parts.count >= 4,
                      let x = Float(parts[1]),
                      let y = Float(parts[2]),
                      let z = Float(parts[3])
                else {
                    throw BenchError.unsupportedInput("invalid OBJ vertex line")
                }
                positions.append([x, y, z])
            } else if tag == "f" {
                let face = try parts.dropFirst().map(parseOBJVertexIndex)
                guard face.count >= 3 else {
                    throw BenchError.unsupportedInput("OBJ face has fewer than 3 vertices")
                }
                for index in 1..<(face.count - 1) {
                    indices += [face[0], face[index], face[index + 1]]
                }
            }
        }

        guard !positions.isEmpty, !indices.isEmpty else {
            throw BenchError.unsupportedInput("OBJ is missing vertices or faces")
        }
        let vertexCount = UInt32(positions.count)
        guard indices.allSatisfy({ $0 < vertexCount }) else {
            throw BenchError.unsupportedInput("OBJ face index out of range")
        }

        return Mesh(positions: positions, indices: indices)
    }

    private static func loadUSD(url: URL) throws -> Mesh {
        #if os(macOS)
        let asset = MDLAsset(url: url)
        let meshes = asset.childObjects(of: MDLMesh.self).compactMap { $0 as? MDLMesh }
        guard !meshes.isEmpty else {
            throw BenchError.unsupportedInput("USD contains no ModelIO meshes")
        }

        var positions = [SIMD3<Float>]()
        var indices = [UInt32]()

        for mesh in meshes {
            let baseVertex = UInt32(positions.count)
            let localPositions = try loadPositions(from: mesh)
            let transform = globalTransform(for: mesh)
            positions.reserveCapacity(positions.count + localPositions.count)
            for position in localPositions {
                positions.append(transformPosition(position, by: transform))
            }

            guard let submeshes = mesh.submeshes as? [MDLSubmesh], !submeshes.isEmpty else {
                continue
            }
            for submesh in submeshes {
                let submeshIndices = try loadTriangleIndices(from: submesh)
                for index in submeshIndices {
                    guard Int(index) < localPositions.count else {
                        throw BenchError.unsupportedInput("USD submesh index out of range")
                    }
                    indices.append(baseVertex + index)
                }
            }
        }

        guard !positions.isEmpty, !indices.isEmpty else {
            throw BenchError.unsupportedInput("USD is missing vertices or triangle faces")
        }
        return Mesh(positions: positions, indices: indices)
        #else
        throw BenchError.unsupportedInput("USD input is only available in the benchmark on macOS")
        #endif
    }

    #if os(macOS)
    private static func loadPositions(from mesh: MDLMesh) throws -> [SIMD3<Float>] {
        guard let attributeData = mesh.vertexAttributeData(
            forAttributeNamed: MDLVertexAttributePosition,
            as: .float3
        ) else {
            throw BenchError.unsupportedInput("USD mesh is missing float3 positions")
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
            throw BenchError.unsupportedInput("USD submesh cannot be converted to triangles")
        }

        let indexBuffer = triangleSubmesh.indexBuffer(asIndexType: .uInt32)
        let map = indexBuffer.map()
        let pointer = map.bytes.assumingMemoryBound(to: UInt32.self)
        let count = triangleSubmesh.indexCount
        guard count.isMultiple(of: 3) else {
            throw BenchError.unsupportedInput("USD triangle index count is not divisible by 3")
        }

        var indices = [UInt32]()
        indices.reserveCapacity(count)
        for index in 0..<count {
            indices.append(pointer[index])
        }
        return indices
    }

    private static func globalTransform(for object: MDLObject) -> simd_float4x4 {
        var chain = [MDLObject]()
        var current: MDLObject? = object
        while let item = current {
            chain.append(item)
            current = item.parent
        }

        var result = matrix_identity_float4x4
        for item in chain.reversed() {
            guard let transform = item.transform else { continue }
            if transform.resetsTransform {
                result = transform.localTransform?(atTime: 0) ?? transform.matrix
            } else {
                result = result * (transform.localTransform?(atTime: 0) ?? transform.matrix)
            }
        }
        return result
    }

    private static func transformPosition(
        _ position: SIMD3<Float>,
        by matrix: simd_float4x4
    ) -> SIMD3<Float> {
        let transformed = matrix * SIMD4<Float>(position.x, position.y, position.z, 1)
        return SIMD3<Float>(transformed.x, transformed.y, transformed.z)
    }
    #endif

    private static func parseOBJVertexIndex(_ token: Substring) throws -> UInt32 {
        guard let rawIndex = token.split(separator: "/", omittingEmptySubsequences: false).first,
              let oneBased = UInt32(rawIndex),
              oneBased > 0
        else {
            throw BenchError.unsupportedInput("only positive OBJ vertex indices are supported")
        }
        return oneBased - 1
    }

    private static func fileURL(path: String) -> URL {
        if path.hasPrefix("/") {
            return URL(fileURLWithPath: path)
        }
        return URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
            .appendingPathComponent(path)
    }

    private static func generateNoisySphere(targetFaces: Int) -> Mesh {
        let longitudeCount = max(8, Int((Double(targetFaces) / 2).squareRoot()))
        let latitudeCount = max(3, targetFaces / (2 * longitudeCount) + 1)
        var positions = [SIMD3<Float>]()
        positions.reserveCapacity(2 + (latitudeCount - 1) * longitudeCount)

        positions.append([0, 0, 1])
        for latitude in 1..<latitudeCount {
            let theta = Double.pi * Double(latitude) / Double(latitudeCount)
            let z = cos(theta)
            let ringRadius = sin(theta)
            for longitude in 0..<longitudeCount {
                let phi = 2 * Double.pi * Double(longitude) / Double(longitudeCount)
                let base = SIMD3<Float>(
                    Float(ringRadius * cos(phi)),
                    Float(ringRadius * sin(phi)),
                    Float(z)
                )
                positions.append(base * noiseScale(latitude: latitude, longitude: longitude))
            }
        }
        let bottom = UInt32(positions.count)
        positions.append([0, 0, -1])

        var indices = [UInt32]()
        indices.reserveCapacity(2 * longitudeCount * (latitudeCount - 1) * 3)

        func ringVertex(_ latitude: Int, _ longitude: Int) -> UInt32 {
            UInt32(1 + (latitude - 1) * longitudeCount + (longitude % longitudeCount))
        }

        for longitude in 0..<longitudeCount {
            indices += [0, ringVertex(1, longitude), ringVertex(1, longitude + 1)]
        }

        if latitudeCount > 2 {
            for latitude in 1..<(latitudeCount - 1) {
                for longitude in 0..<longitudeCount {
                    let a = ringVertex(latitude, longitude)
                    let b = ringVertex(latitude, longitude + 1)
                    let c = ringVertex(latitude + 1, longitude)
                    let d = ringVertex(latitude + 1, longitude + 1)
                    indices += [a, c, b, b, c, d]
                }
            }
        }

        for longitude in 0..<longitudeCount {
            indices += [
                ringVertex(latitudeCount - 1, longitude + 1),
                ringVertex(latitudeCount - 1, longitude),
                bottom,
            ]
        }

        return Mesh(positions: positions, indices: indices)
    }

    private static func noiseScale(latitude: Int, longitude: Int) -> Float {
        let value = sin(Double(latitude) * 12.9898 + Double(longitude) * 78.233) * 43_758.5453
        let fraction = value - floor(value)
        return Float(1.0 + (fraction - 0.5) * 0.06)
    }

    private static func peakMemoryMB() -> Double {
        var usage = rusage()
        guard getrusage(RUSAGE_SELF, &usage) == 0 else { return 0 }
        return Double(usage.ru_maxrss) / 1_048_576.0
    }

    private static func traceMemory(_ label: String, args: Arguments) {
        guard args.tracesMemory else { return }
        fputs("trace_memory,\(label),peak_memory_mb,\(peakMemoryMB())\n", stderr)
    }

    private static func printUsage() {
        Swift.print("""
        usage: swift run -c release MeshDenoiserBench [--faces N | --input mesh.obj|mesh.usdz | --input-dir meshes | --input-list files.txt] [--mode denoise|filter|vertex-update] [--backend all|reference|nativeCPU|nativeGPU] [--repair none|conservative] [--keep-going] [--fail-on-error] [--trace-memory] [--no-reference-check] [--max-error-threshold N] [--mean-error-threshold N] [--max-total-seconds-threshold N] [--max-memory-mb-threshold N]
        """)
    }

    private static func benchmarkHeader(args: Arguments) -> String {
        var header = "backend,vertices,faces,total_secs"
        if args.checksReference {
            header += ",max_error_vs_reference,mean_error_vs_reference"
        }
        header += ",peak_memory_mb"
        if args.repairMode != "none" {
            header += "," + repairCSVHeader
        }
        return header
    }

    private static var repairCSVHeader: String {
        [
            "repair_vertices_before",
            "repair_vertices_after",
            "repair_faces_before",
            "repair_faces_after",
            "repair_boundary_edges_before",
            "repair_boundary_edges_after",
            "repair_nonmanifold_edges_before",
            "repair_nonmanifold_edges_after",
            "repair_degenerate_faces_before",
            "repair_degenerate_faces_after",
            "repair_removed_degenerate_faces",
            "repair_removed_duplicate_faces",
            "repair_removed_nonmanifold_faces",
            "repair_removed_unreferenced_vertices",
        ].joined(separator: ",")
    }

    private static var supportedInputExtensions: Set<String> {
        ["obj", "usd", "usda", "usdc", "usdz"]
    }

    private static func outputLines(_ data: Data) -> [String] {
        guard let text = String(data: data, encoding: .utf8) else { return [] }
        return text.split(whereSeparator: \.isNewline).map(String.init)
    }

    private static func csvField(_ value: String) -> String {
        if value.contains(",") || value.contains("\"") || value.contains("\n") || value.contains("\r") {
            return "\"\(value.replacingOccurrences(of: "\"", with: "\"\""))\""
        }
        return value
    }

    private static func thresholdFailureMessage(line: String, args: Arguments) -> String? {
        guard args.maxErrorThreshold != nil
            || args.meanErrorThreshold != nil
            || args.maxTotalSecondsThreshold != nil
            || args.maxMemoryMBThreshold != nil
        else { return nil }

        let columns = line.split(separator: ",", omittingEmptySubsequences: false)
        let memoryColumn = args.checksReference ? 6 : 4
        guard columns.count > memoryColumn,
              let totalSeconds = Double(columns[3]),
              let memoryMB = Double(columns[memoryColumn])
        else {
            return "could not parse benchmark columns for threshold check"
        }

        var messages = [String]()
        if let threshold = args.maxTotalSecondsThreshold, totalSeconds > threshold {
            messages.append("total_secs \(totalSeconds) > \(threshold)")
        }
        if args.checksReference {
            guard let maxError = Double(columns[4]), let meanError = Double(columns[5]) else {
                return "could not parse benchmark columns for threshold check"
            }
            if let threshold = args.maxErrorThreshold, maxError > threshold {
                messages.append("max_error \(maxError) > \(threshold)")
            }
            if let threshold = args.meanErrorThreshold, meanError > threshold {
                messages.append("mean_error \(meanError) > \(threshold)")
            }
        }
        if let threshold = args.maxMemoryMBThreshold, memoryMB > threshold {
            messages.append("peak_memory_mb \(memoryMB) > \(threshold)")
        }
        return messages.isEmpty ? nil : messages.joined(separator: "; ")
    }

    private static func failureBatchRow(
        inputPath: String,
        backend: String,
        result: ChildRunResult,
        diagnostics: BatchDiagnostics?,
        args: Arguments
    ) -> String {
        var fields = [
            csvField(inputPath),
            backend,
            diagnostics.map { String($0.mesh.vertices) } ?? "0",
            diagnostics.map { String($0.mesh.faces) } ?? "0",
            "nan",
        ]
        if args.checksReference {
            fields += ["nan", "nan"]
        }
        fields.append("0")
        if args.repairMode != "none" {
            if let repairDiagnostics = diagnostics?.repair {
                fields += repairCSVFields(repairDiagnostics)
            } else {
                fields += Array(repeating: "", count: repairCSVFieldCount)
            }
        }
        fields += [
            "failed",
            csvField(failureMessage(result, diagnostics: diagnostics?.mesh)),
        ]
        return fields.joined(separator: ",")
    }

    private static func repairCSVFields(_ diagnostics: MeshPreprocessDiagnostics) -> [String] {
        [
            String(diagnostics.verticesBefore),
            String(diagnostics.verticesAfter),
            String(diagnostics.facesBefore),
            String(diagnostics.facesAfter),
            String(diagnostics.boundaryEdgesBefore),
            String(diagnostics.boundaryEdgesAfter),
            String(diagnostics.nonManifoldEdgesBefore),
            String(diagnostics.nonManifoldEdgesAfter),
            String(diagnostics.degenerateFacesBefore),
            String(diagnostics.degenerateFacesAfter),
            String(diagnostics.removedDegenerateFaces),
            String(diagnostics.removedDuplicateFaces),
            String(diagnostics.removedNonManifoldFaces),
            String(diagnostics.removedUnreferencedVertices),
        ]
    }

    private static var repairCSVFieldCount: Int {
        repairCSVHeader.split(separator: ",").count
    }

    private static func failureMessage(_ result: ChildRunResult, diagnostics: MeshDiagnostics? = nil) -> String {
        let stderrText = String(data: result.stderr, encoding: .utf8) ?? ""
        let stdoutText = String(data: result.stdout, encoding: .utf8) ?? ""
        let text = stderrText.isEmpty ? stdoutText : stderrText
        let line = text
            .split(whereSeparator: \.isNewline)
            .map(String.init)
            .first { !$0.trimmingCharacters(in: .whitespaces).isEmpty }
        let message = line ?? "child process exited with status \(result.status)"
        guard let diagnostics else { return message }
        return message + "; " + diagnostics.description
    }
}

private extension Arguments {
    static func childArguments(from args: Arguments, inputPath: String?, backend: String) -> [String] {
        var childArguments = [
            "--faces", String(args.faces),
            "--mode", args.mode,
            "--backend", backend,
            "--no-header",
        ]
        if let inputPath {
            childArguments += ["--input", inputPath]
        }
        if args.repairMode != "none" {
            childArguments += ["--repair", args.repairMode]
        }
        if args.tracesMemory {
            childArguments.append("--trace-memory")
        }
        if !args.checksReference {
            childArguments.append("--no-reference-check")
        }
        return childArguments
    }
}

private struct ChildRunResult {
    var status: Int32
    var stdout: Data
    var stderr: Data
}

private extension MeshDiagnostics {
    init(mesh: Mesh) {
        vertices = mesh.positions.count
        faces = mesh.indices.count / 3

        var edgeCounts = [EdgeKey: Int]()
        var degenerateFaceCount = 0
        for offset in stride(from: 0, to: mesh.indices.count, by: 3) {
            let a = mesh.indices[offset]
            let b = mesh.indices[offset + 1]
            let c = mesh.indices[offset + 2]
            if a == b || b == c || a == c {
                degenerateFaceCount += 1
                continue
            }
            edgeCounts[EdgeKey(a, b), default: 0] += 1
            edgeCounts[EdgeKey(b, c), default: 0] += 1
            edgeCounts[EdgeKey(c, a), default: 0] += 1
        }

        boundaryEdges = edgeCounts.values.filter { $0 == 1 }.count
        nonManifoldEdges = edgeCounts.values.filter { $0 > 2 }.count
        degenerateFaces = degenerateFaceCount
    }

    var description: String {
        "mesh vertices=\(vertices) faces=\(faces) boundary_edges=\(boundaryEdges) nonmanifold_edges=\(nonManifoldEdges) degenerate_faces=\(degenerateFaces)"
    }
}

private struct EdgeKey: Hashable {
    var a: UInt32
    var b: UInt32

    init(_ first: UInt32, _ second: UInt32) {
        if first < second {
            a = first
            b = second
        } else {
            a = second
            b = first
        }
    }
}

private extension MeshDenoiseParameters.Backend {
    var name: String {
        switch self {
        case .automatic: return "automatic"
        case .nativeGPU: return "nativeGPU"
        case .nativeCPU: return "nativeCPU"
        case .reference: return "reference"
        }
    }
}
