// Intentionally empty. SwiftPM requires a public headers directory, but the
// OpenMesh headers themselves must NOT be exposed here: as a Clang module they
// fail to build (headers #include each other inside open namespaces). They are
// consumed textually via the "Headers" search path instead — see Package.swift.
