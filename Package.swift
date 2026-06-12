// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "MeshDenoiserKit",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "MeshDenoiserKit", targets: ["MeshDenoiserKit"]),
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
