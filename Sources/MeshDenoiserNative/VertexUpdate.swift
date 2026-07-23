enum VertexUpdate {
    private struct FacePositions {
        var first: SIMD3<Double>
        var second: SIMD3<Double>
        var third: SIMD3<Double>
    }

    static func run(
        mesh: NativeMesh,
        targetNormals: [SIMD3<Float>],
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> [SIMD3<Float>] {
        guard targetNormals.count == mesh.triangles.count, parameters.isValid else {
            throw NativeDenoiseError.invalidInput
        }

        let vertexCount = mesh.positions.count
        let faceCount = mesh.triangles.count
        let closenessWeight = parameters.meshUpdateClosenessWeight
            * Double(faceCount) / Double(vertexCount)
        let originalPositions = mesh.positions.map(toDouble)
        var positions = originalPositions

        for _ in 0..<parameters.meshUpdateIterations {
            guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }
            var rhs = originalPositions.map { $0 * closenessWeight }

            for (faceIndex, triangle) in mesh.triangles.enumerated() {
                let firstIndex = Int(triangle.x)
                let secondIndex = Int(triangle.y)
                let thirdIndex = Int(triangle.z)
                let facePositions = FacePositions(
                    first: positions[firstIndex],
                    second: positions[secondIndex],
                    third: positions[thirdIndex]
                )
                let centered = meanCentered(facePositions)
                let currentNormal = faceNormal(facePositions)
                let targetNormal = normalized(toDouble(targetNormals[faceIndex]), fallback: currentNormal)

                let targetPositions: FacePositions
                if dot(currentNormal, targetNormal) >= 0 {
                    targetPositions = projectToTargetPlane(
                        centeredPositions: centered,
                        targetNormal: targetNormal
                    )
                } else {
                    targetPositions = projectToPrincipalLine(
                        centeredPositions: centered,
                        targetNormal: targetNormal
                    )
                }

                let centeredTargets = meanCentered(targetPositions)
                rhs[firstIndex] += centeredTargets.first
                rhs[secondIndex] += centeredTargets.second
                rhs[thirdIndex] += centeredTargets.third
            }

            let previous = positions
            positions = try solveSystem(
                triangles: mesh.triangles,
                closenessWeight: closenessWeight,
                rhs: rhs,
                shouldCancel: shouldCancel
            )

            if parameters.meshUpdateDisplacementEps > 0 {
                let rms = rootMeanSquareDisplacement(positions, previous)
                if rms <= parameters.meshUpdateDisplacementEps {
                    break
                }
            }
        }

        return positions.map(toFloat)
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

    private static func solveSystem(
        triangles: [SIMD3<UInt32>],
        closenessWeight: Double,
        rhs: [SIMD3<Double>],
        shouldCancel: (@Sendable () -> Bool)?
    ) throws -> [SIMD3<Double>] {
        let x = try solveScalarSystem(
            triangles: triangles,
            closenessWeight: closenessWeight,
            rhs: rhs.map(\.x),
            shouldCancel: shouldCancel
        )
        let y = try solveScalarSystem(
            triangles: triangles,
            closenessWeight: closenessWeight,
            rhs: rhs.map(\.y),
            shouldCancel: shouldCancel
        )
        let z = try solveScalarSystem(
            triangles: triangles,
            closenessWeight: closenessWeight,
            rhs: rhs.map(\.z),
            shouldCancel: shouldCancel
        )

        return rhs.indices.map { SIMD3<Double>(x[$0], y[$0], z[$0]) }
    }

    private static func solveScalarSystem(
        triangles: [SIMD3<UInt32>],
        closenessWeight: Double,
        rhs: [Double],
        shouldCancel: (@Sendable () -> Bool)?
    ) throws -> [Double] {
        var x = [Double](repeating: 0, count: rhs.count)
        var r = rhs
        var p = r
        var rsOld = dot(r, r)
        let tolerance = max(1e-12, dot(rhs, rhs) * 1e-20)
        let maxIterations = max(100, rhs.count * 20)

        if rsOld <= tolerance {
            return x
        }

        for iteration in 0..<maxIterations {
            if iteration.isMultiple(of: 16), shouldCancel?() == true {
                throw NativeDenoiseError.cancelled
            }
            let ap = applySystem(
                vector: p,
                triangles: triangles,
                closenessWeight: closenessWeight
            )
            let denominator = dot(p, ap)
            guard denominator.isFinite, abs(denominator) > 0 else {
                throw NativeDenoiseError.solverFailed
            }

            let alpha = rsOld / denominator
            for index in x.indices {
                x[index] += alpha * p[index]
                r[index] -= alpha * ap[index]
            }

            let rsNew = dot(r, r)
            if rsNew <= tolerance {
                return x
            }

            let beta = rsNew / rsOld
            for index in p.indices {
                p[index] = r[index] + beta * p[index]
            }
            rsOld = rsNew
        }

        throw NativeDenoiseError.solverFailed
    }

    private static func applySystem(
        vector: [Double],
        triangles: [SIMD3<UInt32>],
        closenessWeight: Double
    ) -> [Double] {
        var result = vector.map { $0 * closenessWeight }

        for triangle in triangles {
            let a = Int(triangle.x)
            let b = Int(triangle.y)
            let c = Int(triangle.z)
            let mean = (vector[a] + vector[b] + vector[c]) / 3.0
            result[a] += vector[a] - mean
            result[b] += vector[b] - mean
            result[c] += vector[c] - mean
        }

        return result
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
        _ lhs: [SIMD3<Double>],
        _ rhs: [SIMD3<Double>]
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

private func dot(_ lhs: [Double], _ rhs: [Double]) -> Double {
    zip(lhs, rhs).reduce(0) { $0 + $1.0 * $1.1 }
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
