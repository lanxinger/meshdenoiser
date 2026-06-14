// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "MeshDenoiserKit",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "MeshDenoiserKit", targets: ["MeshDenoiserKit"]),
        .executable(name: "MeshDenoiserBench", targets: ["MeshDenoiserBench"]),
        .executable(name: "MeshDenoiserProcess", targets: ["MeshDenoiserProcess"]),
    ],
    targets: [
        // OpenMesh headers deliberately live in "Headers", not the SPM default
        // "include": they must stay textual includes, not a Clang module —
        // some of them #include each other inside open namespaces, which
        // breaks when clang maps the #include to a module import.
        .target(
            name: "OpenMeshCore",
            path: "Sources/OpenMeshCore",
            cxxSettings: [
                .define("OM_STATIC_BUILD"),
                .headerSearchPath("Headers"),
            ]
        ),
        .target(
            name: "CMeshDenoiserCore",
            dependencies: ["OpenMeshCore"],
            path: "Sources/CMeshDenoiserCore",
            publicHeadersPath: "include",
            cxxSettings: [
                .headerSearchPath("../../src"),
                .headerSearchPath("../../Vendor/eigen"),
                .headerSearchPath("../OpenMeshCore/Headers"),
            ]
        ),
        .target(
            name: "MeshDenoiserNative",
            path: "Sources/MeshDenoiserNative",
            resources: [.copy("FilterKernels.metal")]
        ),
        .target(
            name: "MeshDenoiserKit",
            dependencies: ["CMeshDenoiserCore", "MeshDenoiserNative"],
            path: "Sources/MeshDenoiserKit",
            linkerSettings: [
                .linkedFramework("ModelIO", .when(platforms: [.macOS])),
                .linkedFramework("SceneKit", .when(platforms: [.macOS])),
            ]
        ),
        .executableTarget(
            name: "MeshDenoiserBench",
            dependencies: ["MeshDenoiserKit", "MeshDenoiserNative"],
            path: "Sources/MeshDenoiserBench",
            linkerSettings: [
                .linkedFramework("ModelIO", .when(platforms: [.macOS])),
            ]
        ),
        .executableTarget(
            name: "MeshDenoiserProcess",
            dependencies: ["MeshDenoiserKit"],
            path: "Sources/MeshDenoiserProcess"
        ),
        .testTarget(
            name: "MeshDenoiserKitTests",
            dependencies: ["MeshDenoiserKit", "MeshDenoiserNative"],
            path: "Tests/MeshDenoiserKitTests",
            resources: [.copy("Fixtures")]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
