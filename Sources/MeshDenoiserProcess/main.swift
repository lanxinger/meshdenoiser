import Foundation
import MeshDenoiserKit

#if os(macOS)
@main
enum MeshDenoiserProcess {
    static func main() async {
        do {
            let args = try Arguments(CommandLine.arguments)
            let summary = try await MeshAssetDenoiser.process(
                inputURL: args.inputURL,
                outputURL: args.outputURL,
                options: args.options
            ) { progress in
                FileHandle.standardError.write(Data("progress=\(progress)\n".utf8))
            }

            print("meshes,vertices,faces")
            print("\(summary.meshesProcessed),\(summary.verticesProcessed),\(summary.facesProcessed)")
        } catch let error as LocalizedError {
            fputs("\(error.errorDescription ?? String(describing: error))\n", stderr)
            exit(1)
        } catch {
            fputs("\(error)\n", stderr)
            exit(1)
        }
    }
}

private struct Arguments {
    var inputURL: URL
    var outputURL: URL
    var options: MeshAssetDenoiseOptions

    init(_ rawArguments: [String]) throws {
        var inputPath: String?
        var outputPath: String?
        var backend = MeshDenoiseParameters.Backend.automatic
        var lambda: Double?
        var eta: Double?
        var meshUpdateIterations: Int?
        var meshUpdateClosenessWeight: Double?
        var meshUpdateDisplacementEps: Double?
        var outerIterations: Int?
        var preprocessing = MeshAssetDenoiseOptions.Preprocessing.none
        var recomputesNormals = true

        var index = 1
        while index < rawArguments.count {
            let argument = rawArguments[index]
            switch argument {
            case "--input":
                inputPath = try Self.value(after: argument, in: rawArguments, index: &index)
            case "--output":
                outputPath = try Self.value(after: argument, in: rawArguments, index: &index)
            case "--backend":
                backend = try Self.backend(try Self.value(after: argument, in: rawArguments, index: &index))
            case "--lambda":
                lambda = try Self.doubleValue(after: argument, in: rawArguments, index: &index)
            case "--eta":
                eta = try Self.doubleValue(after: argument, in: rawArguments, index: &index)
            case "--mesh-update-iterations":
                meshUpdateIterations = try Self.intValue(after: argument, in: rawArguments, index: &index)
            case "--mesh-update-closeness-weight":
                meshUpdateClosenessWeight = try Self.doubleValue(after: argument, in: rawArguments, index: &index)
            case "--mesh-update-displacement-eps":
                meshUpdateDisplacementEps = try Self.doubleValue(after: argument, in: rawArguments, index: &index)
            case "--outer-iterations":
                outerIterations = try Self.intValue(after: argument, in: rawArguments, index: &index)
            case "--repair":
                let value = try Self.value(after: argument, in: rawArguments, index: &index)
                preprocessing = try Self.preprocessing(value)
            case "--no-recompute-normals":
                recomputesNormals = false
                index += 1
            case "--help", "-h":
                throw ProcessCLIError.help
            default:
                throw ProcessCLIError.invalidArgument(argument)
            }
        }

        guard let inputPath, let outputPath else {
            throw ProcessCLIError.missingRequiredArguments
        }

        var parameters = MeshDenoiseParameters()
        parameters.backend = backend
        if let lambda {
            parameters.lambda = lambda
        }
        if let eta {
            parameters.eta = eta
        }
        if let meshUpdateIterations {
            parameters.meshUpdateIterations = meshUpdateIterations
        }
        if let meshUpdateClosenessWeight {
            parameters.meshUpdateClosenessWeight = meshUpdateClosenessWeight
        }
        if let meshUpdateDisplacementEps {
            parameters.meshUpdateDisplacementEps = meshUpdateDisplacementEps
        }
        if let outerIterations {
            parameters.outerIterations = outerIterations
        }

        inputURL = URL(fileURLWithPath: inputPath)
        outputURL = URL(fileURLWithPath: outputPath)
        options = MeshAssetDenoiseOptions(
            parameters: parameters,
            preprocessing: preprocessing,
            recomputesNormals: recomputesNormals
        )
    }

    private static func value(after option: String, in arguments: [String], index: inout Int) throws -> String {
        let valueIndex = index + 1
        guard valueIndex < arguments.count else {
            throw ProcessCLIError.missingValue(option)
        }
        index += 2
        return arguments[valueIndex]
    }

    private static func doubleValue(after option: String, in arguments: [String], index: inout Int) throws -> Double {
        let rawValue = try value(after: option, in: arguments, index: &index)
        guard let value = Double(rawValue), value.isFinite else {
            throw ProcessCLIError.invalidValue(option, rawValue)
        }
        return value
    }

    private static func intValue(after option: String, in arguments: [String], index: inout Int) throws -> Int {
        let rawValue = try value(after: option, in: arguments, index: &index)
        guard let value = Int(rawValue) else {
            throw ProcessCLIError.invalidValue(option, rawValue)
        }
        return value
    }

    private static func backend(_ value: String) throws -> MeshDenoiseParameters.Backend {
        switch value {
        case "automatic": return .automatic
        case "reference": return .reference
        case "nativeCPU": return .nativeCPU
        case "nativeGPU": return .nativeGPU
        default: throw ProcessCLIError.invalidBackend(value)
        }
    }

    private static func preprocessing(_ value: String) throws -> MeshAssetDenoiseOptions.Preprocessing {
        switch value {
        case "none": return .none
        case "conservative": return .conservativeValidation(.conservative())
        default: throw ProcessCLIError.invalidRepair(value)
        }
    }
}

private enum ProcessCLIError: LocalizedError {
    case help
    case missingRequiredArguments
    case missingValue(String)
    case invalidArgument(String)
    case invalidBackend(String)
    case invalidRepair(String)
    case invalidValue(String, String)

    var errorDescription: String? {
        switch self {
        case .help:
            return Self.usage
        case .missingRequiredArguments:
            return "Missing required --input and --output arguments.\n\(Self.usage)"
        case .missingValue(let option):
            return "Missing value for \(option).\n\(Self.usage)"
        case .invalidArgument(let argument):
            return "Unknown argument: \(argument).\n\(Self.usage)"
        case .invalidBackend(let value):
            return "Unknown backend: \(value). Use automatic, reference, nativeCPU, or nativeGPU."
        case .invalidRepair(let value):
            return "Unknown repair mode: \(value). Use none or conservative."
        case .invalidValue(let option, let value):
            return "Invalid value for \(option): \(value).\n\(Self.usage)"
        }
    }

    private static let usage = """
    usage: MeshDenoiserProcess --input model.usdz --output denoised.usdz [--backend automatic|reference|nativeCPU|nativeGPU] [--lambda value] [--eta value] [--mesh-update-iterations count] [--mesh-update-closeness-weight value] [--mesh-update-displacement-eps value] [--outer-iterations count] [--repair none|conservative] [--no-recompute-normals]
    """
}
#else
@main
enum MeshDenoiserProcess {
    static func main() {
        fputs("MeshDenoiserProcess is only supported on macOS.\n", stderr)
        exit(1)
    }
}
#endif
