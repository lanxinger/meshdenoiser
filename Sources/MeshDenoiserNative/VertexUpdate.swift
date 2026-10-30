enum VertexUpdate {
    private struct FacePositions {
        var first: SIMD3<Double>
        var second: SIMD3<Double>
        var third: SIMD3<Double>
    }

    /// Vertices or faces per parallel work item. Fixed so reductions are
    /// deterministic regardless of core count.
    private static let chunkSize = 2_048

    static func run(
        mesh: NativeMesh,
        targetNormals: [SIMD3<Float>],
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> [SIMD3<Float>] {
        guard targetNormals.count == mesh.triangles.count, parameters.isValid else {
            throw NativeDenoiseError.invalidInput
        }
        // Corner indices (3 * face + k) are stored as UInt32.
        guard !mesh.triangles.isEmpty, mesh.triangles.count <= Int(UInt32.max) / 3 else {
            throw NativeDenoiseError.invalidInput
        }

        let closenessWeight = parameters.meshUpdateClosenessWeight
            * Double(mesh.triangles.count) / Double(mesh.positions.count)
        let workspace = Workspace(mesh: mesh, closenessWeight: closenessWeight)

        try targetNormals.withUnsafeBufferPointer { targetNormals in
            for _ in 0..<parameters.meshUpdateIterations {
                guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }

                workspace.assembleRightHandSide(targetNormals: targetNormals)

                let tracksDisplacement = parameters.meshUpdateDisplacementEps > 0
                if tracksDisplacement {
                    _ = workspace.previous.update(fromContentsOf: workspace.x)
                }

                // Warm start: the current positions are the initial guess.
                try workspace.solve(shouldCancel: shouldCancel)

                if tracksDisplacement {
                    let rms = rootMeanSquareDisplacement(workspace.x, workspace.previous)
                    if rms <= parameters.meshUpdateDisplacementEps {
                        break
                    }
                }
            }
        }

        return workspace.x.map(toFloat)
    }

    /// Topology, Jacobi preconditioner and every per-iteration buffer, allocated once
    /// per `run`. The x/y/z systems share one matrix and are solved together as a
    /// three-column CG with per-column step sizes.
    private final class Workspace {
        let vertexCount: Int
        let faceCount: Int
        let closenessWeight: Double

        let triangles: UnsafeMutableBufferPointer<SIMD3<UInt32>>
        /// Vertex -> corner CSR; corner = 3 * face + k, in increasing face order.
        let cornerOffsets: UnsafeMutableBufferPointer<UInt32>
        let corners: UnsafeMutableBufferPointer<UInt32>
        let inverseDiagonal: UnsafeMutableBufferPointer<Double>

        let original: UnsafeMutableBufferPointer<SIMD3<Double>>
        let x: UnsafeMutableBufferPointer<SIMD3<Double>>
        let previous: UnsafeMutableBufferPointer<SIMD3<Double>>
        let rhs: UnsafeMutableBufferPointer<SIMD3<Double>>
        let r: UnsafeMutableBufferPointer<SIMD3<Double>>
        let z: UnsafeMutableBufferPointer<SIMD3<Double>>
        let p: UnsafeMutableBufferPointer<SIMD3<Double>>
        let ap: UnsafeMutableBufferPointer<SIMD3<Double>>
        let faceMeans: UnsafeMutableBufferPointer<SIMD3<Double>>
        let cornerTargets: UnsafeMutableBufferPointer<SIMD3<Double>>

        let vertexChunkCount: Int
        let partialA: UnsafeMutableBufferPointer<SIMD3<Double>>
        let partialB: UnsafeMutableBufferPointer<SIMD3<Double>>
        let partialC: UnsafeMutableBufferPointer<SIMD3<Double>>

        init(mesh: NativeMesh, closenessWeight: Double) {
            vertexCount = mesh.positions.count
            faceCount = mesh.triangles.count
            self.closenessWeight = closenessWeight

            triangles = Self.allocate(faceCount, SIMD3<UInt32>(repeating: 0))
            _ = triangles.update(fromContentsOf: mesh.triangles)

            cornerOffsets = Self.allocate(vertexCount + 1, UInt32(0))
            corners = Self.allocate(3 * faceCount, UInt32(0))
            for triangle in mesh.triangles {
                cornerOffsets[Int(triangle.x) + 1] += 1
                cornerOffsets[Int(triangle.y) + 1] += 1
                cornerOffsets[Int(triangle.z) + 1] += 1
            }
            for vertex in 0..<vertexCount {
                cornerOffsets[vertex + 1] += cornerOffsets[vertex]
            }
            var cursor = Array(cornerOffsets.dropLast())
            for (face, triangle) in mesh.triangles.enumerated() {
                let corner = UInt32(3 * face)
                corners[Int(cursor[Int(triangle.x)])] = corner
                cursor[Int(triangle.x)] += 1
                corners[Int(cursor[Int(triangle.y)])] = corner + 1
                cursor[Int(triangle.y)] += 1
                corners[Int(cursor[Int(triangle.z)])] = corner + 2
                cursor[Int(triangle.z)] += 1
            }

            // diag(A^T A + wI) = w + (2/3) * incident faces.
            inverseDiagonal = Self.allocate(vertexCount, 1.0)
            for vertex in 0..<vertexCount {
                let incident = Double(cornerOffsets[vertex + 1] - cornerOffsets[vertex])
                let diagonal = closenessWeight + incident * 2.0 / 3.0
                if diagonal.isFinite, diagonal > 0 {
                    inverseDiagonal[vertex] = 1 / diagonal
                }
            }

            original = Self.allocate(vertexCount, SIMD3<Double>.zero)
            for (vertex, position) in mesh.positions.enumerated() {
                original[vertex] = toDouble(position)
            }
            x = Self.allocate(vertexCount, SIMD3<Double>.zero)
            _ = x.update(fromContentsOf: original)
            previous = Self.allocate(vertexCount, SIMD3<Double>.zero)
            rhs = Self.allocate(vertexCount, SIMD3<Double>.zero)
            r = Self.allocate(vertexCount, SIMD3<Double>.zero)
            z = Self.allocate(vertexCount, SIMD3<Double>.zero)
            p = Self.allocate(vertexCount, SIMD3<Double>.zero)
            ap = Self.allocate(vertexCount, SIMD3<Double>.zero)
            faceMeans = Self.allocate(faceCount, SIMD3<Double>.zero)
            cornerTargets = Self.allocate(3 * faceCount, SIMD3<Double>.zero)

            vertexChunkCount = ParallelChunks.chunkCount(for: vertexCount, chunkSize: VertexUpdate.chunkSize)
            partialA = Self.allocate(vertexChunkCount, SIMD3<Double>.zero)
            partialB = Self.allocate(vertexChunkCount, SIMD3<Double>.zero)
            partialC = Self.allocate(vertexChunkCount, SIMD3<Double>.zero)
        }

        deinit {
            triangles.deallocate()
            cornerOffsets.deallocate()
            corners.deallocate()
            inverseDiagonal.deallocate()
            original.deallocate()
            x.deallocate()
            previous.deallocate()
            rhs.deallocate()
            r.deallocate()
            z.deallocate()
            p.deallocate()
            ap.deallocate()
            faceMeans.deallocate()
            cornerTargets.deallocate()
            partialA.deallocate()
            partialB.deallocate()
            partialC.deallocate()
        }

        private static func allocate<T>(_ count: Int, _ value: T) -> UnsafeMutableBufferPointer<T> {
            let buffer = UnsafeMutableBufferPointer<T>.allocate(capacity: count)
            buffer.initialize(repeating: value)
            return buffer
        }

        /// rhs = w * X0 + A^T B, with per-face targets computed in parallel and
        /// gathered per vertex in face order (the same order as a face-order scatter).
        func assembleRightHandSide(targetNormals: UnsafeBufferPointer<SIMD3<Float>>) {
            ParallelChunks.forEach(count: faceCount, chunkSize: VertexUpdate.chunkSize) { _, range in
                for face in range {
                    let triangle = self.triangles[face]
                    let facePositions = FacePositions(
                        first: self.x[Int(triangle.x)],
                        second: self.x[Int(triangle.y)],
                        third: self.x[Int(triangle.z)]
                    )
                    let centered = VertexUpdate.meanCentered(facePositions)
                    let currentNormal = VertexUpdate.faceNormal(facePositions)
                    let targetNormal = normalized(toDouble(targetNormals[face]), fallback: currentNormal)

                    let targetPositions: FacePositions
                    if dot(currentNormal, targetNormal) >= 0 {
                        targetPositions = VertexUpdate.projectToTargetPlane(
                            centeredPositions: centered,
                            targetNormal: targetNormal
                        )
                    } else {
                        targetPositions = VertexUpdate.projectToPrincipalLine(
                            centeredPositions: centered,
                            targetNormal: targetNormal
                        )
                    }

                    let centeredTargets = VertexUpdate.meanCentered(targetPositions)
                    self.cornerTargets[3 * face] = centeredTargets.first
                    self.cornerTargets[3 * face + 1] = centeredTargets.second
                    self.cornerTargets[3 * face + 2] = centeredTargets.third
                }
            }

            ParallelChunks.forEach(count: vertexCount, chunkSize: VertexUpdate.chunkSize) { _, range in
                for vertex in range {
                    var value = self.original[vertex] * self.closenessWeight
                    for entry in Int(self.cornerOffsets[vertex])..<Int(self.cornerOffsets[vertex + 1]) {
                        value += self.cornerTargets[Int(self.corners[entry])]
                    }
                    self.rhs[vertex] = value
                }
            }
        }

        /// result = (A^T A + wI) vector; returns per-column dot(vector, result).
        private func applySystem(
            to vector: UnsafeMutableBufferPointer<SIMD3<Double>>,
            into result: UnsafeMutableBufferPointer<SIMD3<Double>>
        ) -> SIMD3<Double> {
            ParallelChunks.forEach(count: faceCount, chunkSize: VertexUpdate.chunkSize) { _, range in
                for face in range {
                    let triangle = self.triangles[face]
                    self.faceMeans[face] = (vector[Int(triangle.x)] + vector[Int(triangle.y)] + vector[Int(triangle.z)]) / 3.0
                }
            }

            ParallelChunks.forEach(count: vertexCount, chunkSize: VertexUpdate.chunkSize) { chunk, range in
                var partialDot = SIMD3<Double>.zero
                for vertex in range {
                    let value = vector[vertex]
                    var sum = value * self.closenessWeight
                    for entry in Int(self.cornerOffsets[vertex])..<Int(self.cornerOffsets[vertex + 1]) {
                        sum += value - self.faceMeans[Int(self.corners[entry]) / 3]
                    }
                    result[vertex] = sum
                    partialDot += value * sum
                }
                self.partialA[chunk] = partialDot
            }

            return sumPartials(partialA)
        }

        private func sumPartials(_ partials: UnsafeMutableBufferPointer<SIMD3<Double>>) -> SIMD3<Double> {
            var sum = SIMD3<Double>.zero
            for value in partials {
                sum += value
            }
            return sum
        }

        /// Jacobi-preconditioned CG on the three coordinate columns at once,
        /// warm-started from `x`. A column stops once its unpreconditioned squared
        /// residual is <= max(1e-12, |b|^2 * 1e-20), the same criterion as before.
        func solve(shouldCancel: (@Sendable () -> Bool)?) throws {
            _ = applySystem(to: x, into: ap)
            ParallelChunks.forEach(count: vertexCount, chunkSize: VertexUpdate.chunkSize) { chunk, range in
                var rr = SIMD3<Double>.zero
                var rz = SIMD3<Double>.zero
                var bb = SIMD3<Double>.zero
                for vertex in range {
                    let b = self.rhs[vertex]
                    let residual = b - self.ap[vertex]
                    let preconditioned = residual * self.inverseDiagonal[vertex]
                    self.r[vertex] = residual
                    self.z[vertex] = preconditioned
                    self.p[vertex] = preconditioned
                    rr += residual * residual
                    rz += residual * preconditioned
                    bb += b * b
                }
                self.partialA[chunk] = rr
                self.partialB[chunk] = rz
                self.partialC[chunk] = bb
            }

            var residualNorm = sumPartials(partialA)
            var rz = sumPartials(partialB)
            let rhsNorm = sumPartials(partialC)
            var tolerance = SIMD3<Double>.zero
            var active = [false, false, false]
            for column in 0..<3 {
                tolerance[column] = max(1e-12, rhsNorm[column] * 1e-20)
                active[column] = residualNorm[column] > tolerance[column]
            }
            if !active.contains(true) {
                return
            }

            let maxIterations = max(100, vertexCount * 20)
            for iteration in 0..<maxIterations {
                if iteration.isMultiple(of: 16), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }

                let denominator = applySystem(to: p, into: ap)
                var alpha = SIMD3<Double>.zero
                for column in 0..<3 where active[column] {
                    guard denominator[column].isFinite, abs(denominator[column]) > 0 else {
                        throw NativeDenoiseError.solverFailed
                    }
                    alpha[column] = rz[column] / denominator[column]
                }
                let step = alpha

                ParallelChunks.forEach(count: vertexCount, chunkSize: VertexUpdate.chunkSize) { chunk, range in
                    var rr = SIMD3<Double>.zero
                    var rzNew = SIMD3<Double>.zero
                    for vertex in range {
                        self.x[vertex] += step * self.p[vertex]
                        let residual = self.r[vertex] - step * self.ap[vertex]
                        let preconditioned = residual * self.inverseDiagonal[vertex]
                        self.r[vertex] = residual
                        self.z[vertex] = preconditioned
                        rr += residual * residual
                        rzNew += residual * preconditioned
                    }
                    self.partialA[chunk] = rr
                    self.partialB[chunk] = rzNew
                }

                residualNorm = sumPartials(partialA)
                let rzNew = sumPartials(partialB)
                for column in 0..<3 where active[column] && residualNorm[column] <= tolerance[column] {
                    active[column] = false
                }
                if !active.contains(true) {
                    return
                }

                var beta = SIMD3<Double>.zero
                for column in 0..<3 where active[column] {
                    let value = rzNew[column] / rz[column]
                    guard value.isFinite else { throw NativeDenoiseError.solverFailed }
                    beta[column] = value
                }
                rz = rzNew
                let momentum = beta

                ParallelChunks.forEach(count: vertexCount, chunkSize: VertexUpdate.chunkSize) { _, range in
                    for vertex in range {
                        self.p[vertex] = self.z[vertex] + momentum * self.p[vertex]
                    }
                }
            }

            throw NativeDenoiseError.solverFailed
        }
    }

    /// Runs the production solver on an explicit right-hand side and initial guess.
    static func solveForTesting(
        triangles: [SIMD3<UInt32>],
        vertexCount: Int,
        closenessWeight: Double,
        rhs: [SIMD3<Double>],
        initialGuess: [SIMD3<Double>]
    ) throws -> [SIMD3<Double>] {
        guard rhs.count == vertexCount, initialGuess.count == vertexCount, !triangles.isEmpty else {
            throw NativeDenoiseError.invalidInput
        }
        let mesh = NativeMesh(
            positions: [SIMD3<Float>](repeating: .zero, count: vertexCount),
            triangles: triangles
        )
        let workspace = Workspace(mesh: mesh, closenessWeight: closenessWeight)
        _ = workspace.rhs.update(fromContentsOf: rhs)
        _ = workspace.x.update(fromContentsOf: initialGuess)
        try workspace.solve(shouldCancel: nil)
        return Array(workspace.x)
    }

    static func systemMatrixForTesting(
        vertexCount: Int,
        triangles: [SIMD3<UInt32>],
        closenessWeight: Double
    ) -> [[Double]] {
        var matrix = Array(
            repeating: Array(repeating: 0.0, count: vertexCount),
            count: vertexCount
        )

        for vertex in 0..<vertexCount {
            matrix[vertex][vertex] += closenessWeight
        }

        for triangle in triangles {
            let vertices = [Int(triangle.x), Int(triangle.y), Int(triangle.z)]
            for row in 0..<3 {
                for column in 0..<3 {
                    matrix[vertices[row]][vertices[column]] += row == column ? 2.0 / 3.0 : -1.0 / 3.0
                }
            }
        }

        return matrix
    }

    static func projectToTargetPlaneForTesting(
        centeredPositions: [SIMD3<Double>],
        targetNormal: SIMD3<Double>
    ) -> [SIMD3<Double>] {
        projectToTargetPlane(
            centeredPositions: centeredPositions,
            targetNormal: normalized(targetNormal, fallback: SIMD3<Double>(0, 0, 1))
        )
    }

    private static func projectToTargetPlane(
        centeredPositions: FacePositions,
        targetNormal: SIMD3<Double>
    ) -> FacePositions {
        FacePositions(
            first: centeredPositions.first
                - targetNormal * dot(targetNormal, centeredPositions.first),
            second: centeredPositions.second
                - targetNormal * dot(targetNormal, centeredPositions.second),
            third: centeredPositions.third
                - targetNormal * dot(targetNormal, centeredPositions.third)
        )
    }

    private static func projectToTargetPlane(
        centeredPositions: [SIMD3<Double>],
        targetNormal: SIMD3<Double>
    ) -> [SIMD3<Double>] {
        centeredPositions.map { position in
            position - targetNormal * dot(targetNormal, position)
        }
    }

    private static func projectToPrincipalLine(
        centeredPositions: FacePositions,
        targetNormal: SIMD3<Double>
    ) -> FacePositions {
        let basis = targetPlaneBasis(normal: targetNormal)
        var xx = 0.0
        var xy = 0.0
        var yy = 0.0

        var x = dot(centeredPositions.first, basis.u)
        var y = dot(centeredPositions.first, basis.v)
        xx += x * x
        xy += x * y
        yy += y * y

        x = dot(centeredPositions.second, basis.u)
        y = dot(centeredPositions.second, basis.v)
        xx += x * x
        xy += x * y
        yy += y * y

        x = dot(centeredPositions.third, basis.u)
        y = dot(centeredPositions.third, basis.v)
        xx += x * x
        xy += x * y
        yy += y * y

        let localDirection: SIMD2<Double>
        if abs(xy) <= 1e-14 {
            localDirection = xx >= yy ? SIMD2<Double>(1, 0) : SIMD2<Double>(0, 1)
        } else {
            let trace = xx + yy
            let determinant = xx * yy - xy * xy
            let discriminant = max(0, trace * trace * 0.25 - determinant).squareRoot()
            let lambda = trace * 0.5 + discriminant
            localDirection = normalized(SIMD2<Double>(xy, lambda - xx), fallback: SIMD2<Double>(1, 0))
        }

        let direction = normalized(
            basis.u * localDirection.x + basis.v * localDirection.y,
            fallback: basis.u
        )
        return FacePositions(
            first: direction * dot(direction, centeredPositions.first),
            second: direction * dot(direction, centeredPositions.second),
            third: direction * dot(direction, centeredPositions.third)
        )
    }

    private static func meanCentered(_ positions: FacePositions) -> FacePositions {
        let mean = ((positions.first + positions.second) + positions.third) / 3
        return FacePositions(
            first: positions.first - mean,
            second: positions.second - mean,
            third: positions.third - mean
        )
    }

    private static func faceNormal(_ positions: FacePositions) -> SIMD3<Double> {
        let normal = cross(
            positions.second - positions.first,
            positions.third - positions.first
        )
        return normalized(normal, fallback: SIMD3<Double>(0, 0, 1))
    }

    private static func targetPlaneBasis(normal: SIMD3<Double>) -> (u: SIMD3<Double>, v: SIMD3<Double>) {
        let helper = abs(normal.x) < 0.9 ? SIMD3<Double>(1, 0, 0) : SIMD3<Double>(0, 1, 0)
        let u = normalized(cross(normal, helper), fallback: SIMD3<Double>(0, 1, 0))
        let v = normalized(cross(normal, u), fallback: SIMD3<Double>(1, 0, 0))
        return (u, v)
    }

    private static func rootMeanSquareDisplacement(
        _ lhs: UnsafeMutableBufferPointer<SIMD3<Double>>,
        _ rhs: UnsafeMutableBufferPointer<SIMD3<Double>>
    ) -> Double {
        var sum = 0.0
        for index in lhs.indices {
            sum += lengthSquared(lhs[index] - rhs[index])
        }
        return (sum / Double(lhs.count)).squareRoot()
    }
}

private func toDouble(_ value: SIMD3<Float>) -> SIMD3<Double> {
    SIMD3<Double>(Double(value.x), Double(value.y), Double(value.z))
}

private func toFloat(_ value: SIMD3<Double>) -> SIMD3<Float> {
    SIMD3<Float>(Float(value.x), Float(value.y), Float(value.z))
}

private func dot(_ lhs: SIMD3<Double>, _ rhs: SIMD3<Double>) -> Double {
    lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z
}

private func length(_ vector: SIMD3<Double>) -> Double {
    lengthSquared(vector).squareRoot()
}

private func lengthSquared(_ vector: SIMD3<Double>) -> Double {
    dot(vector, vector)
}

private func cross(_ lhs: SIMD3<Double>, _ rhs: SIMD3<Double>) -> SIMD3<Double> {
    SIMD3<Double>(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    )
}

private func normalized(_ vector: SIMD3<Double>, fallback: SIMD3<Double>) -> SIMD3<Double> {
    let magnitude = length(vector)
    guard magnitude.isFinite, magnitude > 0 else { return fallback }
    return vector / magnitude
}

private func normalized(_ vector: SIMD2<Double>, fallback: SIMD2<Double>) -> SIMD2<Double> {
    let magnitude = (vector.x * vector.x + vector.y * vector.y).squareRoot()
    guard magnitude.isFinite, magnitude > 0 else { return fallback }
    return vector / magnitude
}
