struct FaceGeometry: Sendable {
    var normals: [SIMD3<Float>]
    var centroids: [SIMD3<Float>]
    var areaWeights: [Float]
}

struct Edge: Hashable, Sendable {
    var a: UInt32
    var b: UInt32

    init(_ first: UInt32, _ second: UInt32) {
        if first < second {
            self.a = first
            self.b = second
        } else {
            self.a = second
            self.b = first
        }
    }
}

struct EdgeRecord: Sendable {
    var vertices: Edge
    var face0: UInt32
    var face1: UInt32?
}

enum MeshConnectivity {
    struct Result: Sendable {
        var triangles: [SIMD3<UInt32>]
        var faceGeometry: FaceGeometry
        var edges: [EdgeRecord]
        var vertexToFaces: CSRAdjacency<UInt32>
        var vertexToNonboundaryEdges: CSRAdjacency<UInt32>
        var averageNeighborFaceCentroidDistance: Float
    }

    static func build(mesh: NativeMesh) throws -> Result {
        let geometry = try faceGeometry(mesh: mesh)
        let edges = try edgeRecords(mesh: mesh)
        let vertexToFaces = makeVertexToFaces(mesh: mesh)
        let vertexToNonboundaryEdges = makeVertexToNonboundaryEdges(vertexCount: mesh.positions.count, edges: edges)
        let averageDistance = try averageNeighborFaceCentroidDistance(centroids: geometry.centroids, edges: edges)

        return Result(
            triangles: mesh.triangles,
            faceGeometry: geometry,
            edges: edges,
            vertexToFaces: vertexToFaces,
            vertexToNonboundaryEdges: vertexToNonboundaryEdges,
            averageNeighborFaceCentroidDistance: averageDistance
        )
    }

    static func faceGeometry(mesh: NativeMesh) throws -> FaceGeometry {
        var normals = [SIMD3<Float>]()
        var centroids = [SIMD3<Float>]()
        var rawAreas = [Float]()
        normals.reserveCapacity(mesh.triangles.count)
        centroids.reserveCapacity(mesh.triangles.count)
        rawAreas.reserveCapacity(mesh.triangles.count)

        for triangle in mesh.triangles {
            let p0 = mesh.positions[Int(triangle.x)]
            let p1 = mesh.positions[Int(triangle.y)]
            let p2 = mesh.positions[Int(triangle.z)]
            let normalArea = cross(p1 - p0, p2 - p0)
            let doubleArea = length(normalArea)
            guard doubleArea.isFinite, doubleArea > 0 else {
                throw NativeDenoiseError.invalidInput
            }

            normals.append(normalArea / doubleArea)
            centroids.append((p0 + p1 + p2) / 3)
            rawAreas.append(doubleArea * 0.5)
        }

        let meanArea = rawAreas.reduce(0, +) / Float(rawAreas.count)
        guard meanArea.isFinite, meanArea > 0 else {
            throw NativeDenoiseError.invalidInput
        }

        return FaceGeometry(
            normals: normals,
            centroids: centroids,
            areaWeights: rawAreas.map { $0 / meanArea }
        )
    }

    private static func edgeRecords(mesh: NativeMesh) throws -> [EdgeRecord] {
        var edgeIndexByVertices = [Edge: Int]()
        var edges = [EdgeRecord]()
        edges.reserveCapacity(mesh.triangles.count * 3 / 2)

        for (faceIndex, triangle) in mesh.triangles.enumerated() {
            let face = UInt32(faceIndex)
            for vertices in [(triangle.x, triangle.y), (triangle.y, triangle.z), (triangle.z, triangle.x)] {
                let edge = Edge(vertices.0, vertices.1)
                if let edgeIndex = edgeIndexByVertices[edge] {
                    guard edges[edgeIndex].face1 == nil else {
                        throw NativeDenoiseError.invalidInput
                    }
                    edges[edgeIndex].face1 = face
                } else {
                    edgeIndexByVertices[edge] = edges.count
                    edges.append(EdgeRecord(vertices: edge, face0: face, face1: nil))
                }
            }
        }

        return edges
    }

    private static func makeVertexToFaces(mesh: NativeMesh) -> CSRAdjacency<UInt32> {
        var rows = Array(repeating: [UInt32](), count: mesh.positions.count)
        for (faceIndex, triangle) in mesh.triangles.enumerated() {
            let face = UInt32(faceIndex)
            rows[Int(triangle.x)].append(face)
            rows[Int(triangle.y)].append(face)
            rows[Int(triangle.z)].append(face)
        }
        return makeCSR(rows: rows)
    }

    private static func makeVertexToNonboundaryEdges(
        vertexCount: Int,
        edges: [EdgeRecord]
    ) -> CSRAdjacency<UInt32> {
        var rows = Array(repeating: [UInt32](), count: vertexCount)
        for (edgeIndex, edge) in edges.enumerated() where edge.face1 != nil {
            let index = UInt32(edgeIndex)
            rows[Int(edge.vertices.a)].append(index)
            rows[Int(edge.vertices.b)].append(index)
        }
        return makeCSR(rows: rows)
    }

    private static func makeCSR(rows: [[UInt32]]) -> CSRAdjacency<UInt32> {
        var offsets = [UInt32]()
        var values = [UInt32]()
        offsets.reserveCapacity(rows.count + 1)
        offsets.append(0)

        for row in rows {
            values.append(contentsOf: row)
            offsets.append(UInt32(values.count))
        }

        return CSRAdjacency(offsets: offsets, values: values)
    }

    private static func averageNeighborFaceCentroidDistance(
        centroids: [SIMD3<Float>],
        edges: [EdgeRecord]
    ) throws -> Float {
        var sum: Float = 0
        var count: Float = 0

        for edge in edges {
            guard let face1 = edge.face1 else { continue }
            sum += length(centroids[Int(edge.face0)] - centroids[Int(face1)])
            count += 1
        }

        guard count > 0 else { throw NativeDenoiseError.invalidInput }
        return sum / count
    }
}

func length(_ vector: SIMD3<Float>) -> Float {
    lengthSquared(vector).squareRoot()
}

func lengthSquared(_ vector: SIMD3<Float>) -> Float {
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z
}

func cross(_ lhs: SIMD3<Float>, _ rhs: SIMD3<Float>) -> SIMD3<Float> {
    SIMD3(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    )
}
