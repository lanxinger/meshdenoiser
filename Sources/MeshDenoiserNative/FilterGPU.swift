import Foundation
import Metal

enum FilterGPU {
    static var isAvailable: Bool {
        MTLCreateSystemDefaultDevice() != nil
    }

    static func faceNeighborRowsUseMetalUInt2LayoutForTesting() -> Bool {
        guard MemoryLayout<FaceNeighbor>.stride == MemoryLayout<SIMD2<UInt32>>.stride,
              MemoryLayout<FaceNeighbor>.alignment == MemoryLayout<SIMD2<UInt32>>.alignment
        else {
            return false
        }

        let neighbors = [
            FaceNeighbor(neighborFace: 3, pairIndex: 7),
            FaceNeighbor(neighborFace: 11, pairIndex: 13),
        ]
        let expected = [
            SIMD2<UInt32>(3, 7),
            SIMD2<UInt32>(11, 13),
        ]
        return neighbors.withUnsafeBytes { neighborBytes in
            expected.withUnsafeBytes { expectedBytes in
                Array(neighborBytes) == Array(expectedBytes)
            }
        }
    }

    static func displacementForTesting(
        previous: [SIMD3<Float>],
        current: [SIMD3<Float>],
        areaWeights: [Float]
    ) throws -> Float {
        guard let device = MTLCreateSystemDefaultDevice(),
              let commandQueue = device.makeCommandQueue(),
              previous.count == current.count,
              current.count == areaWeights.count,
              !current.isEmpty
        else {
            throw NativeDenoiseError.solverFailed
        }

        let library = try makeLibrary(device: device)
        guard let reductionFunction = library.makeFunction(name: "reduce_displacement") else {
            throw NativeDenoiseError.solverFailed
        }
        let reductionPipeline = try device.makeComputePipelineState(function: reductionFunction)
        let width = reductionThreadgroupWidth(for: reductionPipeline)
        let partialCount = partialDisplacementCount(faceCount: current.count, threadgroupWidth: width)

        let previousSignals = previous.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        let currentSignals = current.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        guard let previousBuffer = makeBuffer(device: device, values: previousSignals),
              let currentBuffer = makeBuffer(device: device, values: currentSignals),
              let areaWeightBuffer = makeBuffer(device: device, values: areaWeights),
              let partialBuffer = device.makeBuffer(
                length: partialCount * MemoryLayout<Float>.stride,
                options: [.storageModeShared]
              ),
              let commandBuffer = commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeComputeCommandEncoder()
        else {
            throw NativeDenoiseError.solverFailed
        }

        encodeReduction(
            encoder,
            pipeline: reductionPipeline,
            previousBuffer: previousBuffer,
            currentBuffer: currentBuffer,
            areaWeightBuffer: areaWeightBuffer,
            partialBuffer: partialBuffer,
            faceCount: current.count,
            threadgroupWidth: width
        )
        encoder.endEncoding()
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
        guard commandBuffer.error == nil else { throw NativeDenoiseError.solverFailed }

        return readPartialDisplacement(from: partialBuffer, count: partialCount)
    }

    static func run(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: inout FilterPrecompute,
        nu: Float,
        maxIterations: Int = 100
    ) throws -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        guard let device = MTLCreateSystemDefaultDevice(),
              let commandQueue = device.makeCommandQueue(),
              !initialSignals.isEmpty,
              initialSignals.count == areaWeights.count,
              nu.isFinite,
              nu > 0
        else {
            throw NativeDenoiseError.solverFailed
        }

        let library = try makeLibrary(device: device)
        guard let gatherFunction = library.makeFunction(name: "gather_filter_signals"),
              let reductionFunction = library.makeFunction(name: "reduce_displacement")
        else {
            throw NativeDenoiseError.solverFailed
        }

        let gatherPipeline = try device.makeComputePipelineState(function: gatherFunction)
        let reductionPipeline = try device.makeComputePipelineState(function: reductionFunction)
        let reductionWidth = reductionThreadgroupWidth(for: reductionPipeline)
        let partialCount = partialDisplacementCount(
            faceCount: initialSignals.count,
            threadgroupWidth: reductionWidth
        )
        let faceCount = initialSignals.count
        let convergenceThreshold = precompute.convergenceThreshold
        precompute.pairs.removeAll(keepingCapacity: false)

        guard let rowOffsetBuffer = makeBufferAndRelease(
            device: device,
            values: &precompute.faceNeighborRows.offsets
        ) else {
            throw NativeDenoiseError.solverFailed
        }
        guard let rowValueBuffer = makeBufferAndRelease(
            device: device,
            values: &precompute.faceNeighborRows.values
        ) else {
            throw NativeDenoiseError.solverFailed
        }

        guard let staticWeightBuffer = makeBufferAndRelease(device: device, values: &precompute.staticWeights) else {
            throw NativeDenoiseError.solverFailed
        }

        var weightedInitial = precompute.weightedInitialSignals.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        precompute.weightedInitialSignals.removeAll(keepingCapacity: false)
        guard let weightedInitialBuffer = makeBufferAndRelease(device: device, values: &weightedInitial) else {
            throw NativeDenoiseError.solverFailed
        }

        var signals = initialSignals.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        guard let signalsBuffer = makeBufferAndRelease(device: device, values: &signals) else {
            throw NativeDenoiseError.solverFailed
        }

        guard let filteredBuffer = device.makeBuffer(
            length: faceCount * MemoryLayout<SIMD4<Float>>.stride,
            options: [.storageModeShared]
        ) else {
            throw NativeDenoiseError.solverFailed
        }

        guard let areaWeightBuffer = makeBuffer(device: device, values: areaWeights),
              let partialDisplacementBuffer = device.makeBuffer(
                length: partialCount * MemoryLayout<Float>.stride,
                options: [.storageModeShared]
              )
        else {
            throw NativeDenoiseError.solverFailed
        }

        var hDynamic = Float(-0.5) / (nu * nu)

        var inputBuffer = signalsBuffer
        var outputBuffer = filteredBuffer
        var latestBuffer = signalsBuffer

        for iteration in 1...maxIterations {
            guard let commandBuffer = commandQueue.makeCommandBuffer(),
                  let gatherEncoder = commandBuffer.makeComputeCommandEncoder()
            else {
                throw NativeDenoiseError.solverFailed
            }

            gatherEncoder.setComputePipelineState(gatherPipeline)
            gatherEncoder.setBuffer(inputBuffer, offset: 0, index: 0)
            gatherEncoder.setBuffer(weightedInitialBuffer, offset: 0, index: 1)
            gatherEncoder.setBuffer(rowOffsetBuffer, offset: 0, index: 2)
            gatherEncoder.setBuffer(rowValueBuffer, offset: 0, index: 3)
            gatherEncoder.setBuffer(staticWeightBuffer, offset: 0, index: 4)
            gatherEncoder.setBuffer(outputBuffer, offset: 0, index: 5)
            gatherEncoder.setBytes(&hDynamic, length: MemoryLayout<Float>.stride, index: 6)
            dispatch(gatherEncoder, pipeline: gatherPipeline, count: faceCount)
            gatherEncoder.endEncoding()

            guard let reductionEncoder = commandBuffer.makeComputeCommandEncoder() else {
                throw NativeDenoiseError.solverFailed
            }
            encodeReduction(
                reductionEncoder,
                pipeline: reductionPipeline,
                previousBuffer: inputBuffer,
                currentBuffer: outputBuffer,
                areaWeightBuffer: areaWeightBuffer,
                partialBuffer: partialDisplacementBuffer,
                faceCount: faceCount,
                threadgroupWidth: reductionWidth
            )
            reductionEncoder.endEncoding()

            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()
            guard commandBuffer.error == nil else { throw NativeDenoiseError.solverFailed }

            latestBuffer = outputBuffer
            let displacement = readPartialDisplacement(from: partialDisplacementBuffer, count: partialCount)

            if displacement <= convergenceThreshold {
                return (readFloat3Signals(from: outputBuffer, count: faceCount), iteration, true)
            }

            swap(&inputBuffer, &outputBuffer)
        }

        return (readFloat3Signals(from: latestBuffer, count: faceCount), maxIterations, false)
    }

    private static func makeLibrary(device: MTLDevice) throws -> MTLLibrary {
        let url = Bundle.module.url(forResource: "FilterKernels", withExtension: "metal")
        guard let url, let source = try? String(contentsOf: url, encoding: .utf8) else {
            throw NativeDenoiseError.solverFailed
        }
        return try device.makeLibrary(source: source, options: nil)
    }

    private static func makeBuffer<T>(
        device: MTLDevice,
        values: [T]
    ) -> MTLBuffer? {
        values.withUnsafeBytes { bytes in
            guard let baseAddress = bytes.baseAddress else { return nil }
            return device.makeBuffer(
                bytes: baseAddress,
                length: bytes.count,
                options: [.storageModeShared]
            )
        }
    }

    private static func makeBufferAndRelease<T>(
        device: MTLDevice,
        values: inout [T]
    ) -> MTLBuffer? {
        guard let buffer = makeBuffer(device: device, values: values) else { return nil }
        values.removeAll(keepingCapacity: false)
        return buffer
    }

    private static func dispatch(
        _ encoder: MTLComputeCommandEncoder,
        pipeline: MTLComputePipelineState,
        count: Int
    ) {
        let width = max(1, min(pipeline.maxTotalThreadsPerThreadgroup, 256))
        encoder.dispatchThreads(
            MTLSize(width: count, height: 1, depth: 1),
            threadsPerThreadgroup: MTLSize(width: width, height: 1, depth: 1)
        )
    }

    private static func encodeReduction(
        _ encoder: MTLComputeCommandEncoder,
        pipeline: MTLComputePipelineState,
        previousBuffer: MTLBuffer,
        currentBuffer: MTLBuffer,
        areaWeightBuffer: MTLBuffer,
        partialBuffer: MTLBuffer,
        faceCount: Int,
        threadgroupWidth: Int
    ) {
        var count = UInt32(faceCount)
        let partialCount = partialDisplacementCount(
            faceCount: faceCount,
            threadgroupWidth: threadgroupWidth
        )

        encoder.setComputePipelineState(pipeline)
        encoder.setBuffer(previousBuffer, offset: 0, index: 0)
        encoder.setBuffer(currentBuffer, offset: 0, index: 1)
        encoder.setBuffer(areaWeightBuffer, offset: 0, index: 2)
        encoder.setBuffer(partialBuffer, offset: 0, index: 3)
        encoder.setBytes(&count, length: MemoryLayout<UInt32>.stride, index: 4)
        encoder.setThreadgroupMemoryLength(
            threadgroupWidth * MemoryLayout<Float>.stride,
            index: 0
        )
        encoder.dispatchThreads(
            MTLSize(width: partialCount * threadgroupWidth, height: 1, depth: 1),
            threadsPerThreadgroup: MTLSize(width: threadgroupWidth, height: 1, depth: 1)
        )
    }

    private static func reductionThreadgroupWidth(for pipeline: MTLComputePipelineState) -> Int {
        max(1, min(pipeline.maxTotalThreadsPerThreadgroup, 256))
    }

    private static func partialDisplacementCount(
        faceCount: Int,
        threadgroupWidth: Int
    ) -> Int {
        (faceCount + threadgroupWidth - 1) / threadgroupWidth
    }

    private static func readPartialDisplacement(from buffer: MTLBuffer, count: Int) -> Float {
        let pointer = buffer.contents().bindMemory(to: Float.self, capacity: count)
        return UnsafeBufferPointer(start: pointer, count: count).reduce(0, +)
    }

    private static func readFloat3Signals(from buffer: MTLBuffer, count: Int) -> [SIMD3<Float>] {
        let pointer = buffer.contents().bindMemory(to: SIMD4<Float>.self, capacity: count)
        return UnsafeBufferPointer(start: pointer, count: count).map {
            SIMD3<Float>($0.x, $0.y, $0.z)
        }
    }
}
