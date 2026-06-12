// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "MeshDenoiserKit",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "MeshDenoiserKit", targets: ["MeshDenoiserKit"]),
    ],
    targets: [
        .target(
            name: "OpenMeshCore",
            path: "Sources/OpenMeshCore",
            publicHeadersPath: "include",
            cxxSettings: [
                .define("OM_STATIC_BUILD"),
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
            ]
        ),
        .target(
            name: "MeshDenoiserKit",
            dependencies: ["CMeshDenoiserCore"],
            path: "Sources/MeshDenoiserKit"
        ),
        .testTarget(
            name: "MeshDenoiserKitTests",
            dependencies: ["MeshDenoiserKit"],
            path: "Tests/MeshDenoiserKitTests",
            resources: [.copy("Fixtures")]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
