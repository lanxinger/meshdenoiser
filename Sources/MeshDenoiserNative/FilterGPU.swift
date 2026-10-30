import Foundation
import Metal

enum FilterGPU {
    private final class Context: @unchecked Sendable {
        let device: MTLDevice
        let commandQueue: MTLCommandQueue
        let gatherReductionPipeline: MTLComputePipelineState
        let reductionPipeline: MTLComputePipelineState

        init?() {
            guard let device = MTLCreateSystemDefaultDevice(),
                  let commandQueue = device.makeCommandQueue(),
                  let url = Bundle.module.url(forResource: "FilterKernels", withExtension: "metal"),
                  let source = try? String(contentsOf: url, encoding: .utf8),
                  let library = try? device.makeLibrary(source: source, options: nil),
                  let gatherReductionFunction = library.makeFunction(
                    name: "gather_filter_signals_and_reduce_displacement"
                  ),
                  let reductionFunction = library.makeFunction(name: "reduce_displacement"),
                  let gatherReductionPipeline = try? device.makeComputePipelineState(
                    function: gatherReductionFunction
                  ),
                  let reductionPipeline = try? device.makeComputePipelineState(function: reductionFunction)
            else {
                return nil
            }

            self.device = device
            self.commandQueue = commandQueue
            self.gatherReductionPipeline = gatherReductionPipeline
            self.reductionPipeline = reductionPipeline
        }
    }

    private static let sharedContext = Context()

    static var isAvailable: Bool {
        sharedContext != nil
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
        guard let context = sharedContext,
              previous.count == current.count,
              current.count == areaWeights.count,
              !current.isEmpty
        else {
            throw NativeDenoiseError.solverFailed
        }

        let width = reductionThreadgroupWidth(for: context.reductionPipeline)
        let partialCount = partialDisplacementCount(faceCount: current.count, threadgroupWidth: width)

        let previousSignals = previous.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        let currentSignals = current.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        guard let previousBuffer = makeBuffer(device: context.device, values: previousSignals),
              let currentBuffer = makeBuffer(device: context.device, values: currentSignals),
              let areaWeightBuffer = makeBuffer(device: context.device, values: areaWeights),
              let partialBuffer = context.device.makeBuffer(
                length: partialCount * MemoryLayout<Float>.stride,
                options: [.storageModeShared]
              ),
              let commandBuffer = context.commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeComputeCommandEncoder()
        else {
            throw NativeDenoiseError.solverFailed
        }

        encodeReduction(
            encoder,
            pipeline: context.reductionPipeline,
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

    /// Static filter inputs resident in Metal buffers.
    struct PreparedFilter {
        let faceCount: Int
        let rowOffsetBuffer: MTLBuffer
        let rowValueBuffer: MTLBuffer
        let staticWeightBuffer: MTLBuffer
        let weightedInitialBuffer: MTLBuffer
        let convergenceThreshold: Float
    }

    /// Builds the filter precompute straight into shared Metal buffers: a counting
    /// pass sizes the neighbor rows exactly, and the fill pass writes rows and
    /// weights into buffer memory, so no Swift-array copy coexists with the buffers.
    static func prepare(
        centroids: [SIMD3<Float>],
        guidanceNormals: [SIMD3<Float>],
        initialNormals: [SIMD3<Float>],
        areaWeights: [Float],
        etaPrime: Float,
        parameters: NativeDenoiseParameters,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> PreparedFilter {
        guard let context = sharedContext else { throw NativeDenoiseError.solverFailed }
        guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }

        let layout = try FilterPrecompute.makeNeighborLayout(
            centroids: centroids,
            guidanceNormals: guidanceNormals,
            initialNormals: initialNormals,
            areaWeights: areaWeights,
            etaPrime: etaPrime,
            parameters: parameters,
            shouldCancel: shouldCancel
        )
        let faceCount = layout.faceCount
        guard faceCount <= UInt32.max,
              let rowOffsetBuffer = makeBuffer(device: context.device, values: layout.rowOffsets),
              let rowValueBuffer = context.device.makeBuffer(
                length: layout.rowEntryCount * MemoryLayout<FaceNeighbor>.stride,
                options: [.storageModeShared]
              ),
              let staticWeightBuffer = context.device.makeBuffer(
                length: layout.pairCount * MemoryLayout<Float>.stride,
                options: [.storageModeShared]
              ),
              let weightedInitialBuffer = context.device.makeBuffer(
                length: faceCount * MemoryLayout<SIMD4<Float>>.stride,
                options: [.storageModeShared]
              )
        else {
            throw NativeDenoiseError.solverFailed
        }

        let areaSpatialSum = try FilterPrecompute.fillNeighborRows(
            layout: layout,
            rowValues: boundBuffer(rowValueBuffer, as: FaceNeighbor.self, count: layout.rowEntryCount),
            staticWeights: boundBuffer(staticWeightBuffer, as: Float.self, count: layout.pairCount),
            guidanceNormals: guidanceNormals,
            areaWeights: areaWeights,
            etaPrime: etaPrime,
            mu: Float(parameters.mu),
            shouldCancel: shouldCancel
        )
        let scalars = try FilterPrecompute.finishScalars(
            areaSpatialSum: areaSpatialSum,
            areaWeights: areaWeights,
            parameters: parameters
        )

        let weightedInitial = boundBuffer(weightedInitialBuffer, as: SIMD4<Float>.self, count: faceCount)
        for face in 0..<faceCount {
            let value = initialNormals[face] * areaWeights[face] * scalars.initialScale
            weightedInitial[face] = SIMD4<Float>(value.x, value.y, value.z, 0)
        }

        return PreparedFilter(
            faceCount: faceCount,
            rowOffsetBuffer: rowOffsetBuffer,
            rowValueBuffer: rowValueBuffer,
            staticWeightBuffer: staticWeightBuffer,
            weightedInitialBuffer: weightedInitialBuffer,
            convergenceThreshold: scalars.convergenceThreshold
        )
    }

    /// Array-precompute entry point (tests, benchmarks). Moves the arrays into
    /// Metal buffers one at a time, releasing each array after its copy.
    static func run(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: inout FilterPrecompute,
        nu: Float,
        maxIterations: Int = 100,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }
        guard let context = sharedContext,
              !initialSignals.isEmpty,
              initialSignals.count == areaWeights.count,
              initialSignals.count == precompute.weightedInitialSignals.count,
              initialSignals.count <= UInt32.max,
              nu.isFinite,
              nu > 0,
              maxIterations > 0
        else {
            throw NativeDenoiseError.solverFailed
        }

        let faceCount = initialSignals.count
        let convergenceThreshold = precompute.convergenceThreshold
        precompute.pairs.removeAll(keepingCapacity: false)

        guard let rowOffsetBuffer = makeBufferAndRelease(
            device: context.device,
            values: &precompute.faceNeighborRows.offsets
        ) else {
            throw NativeDenoiseError.solverFailed
        }
        guard let rowValueBuffer = makeBufferAndRelease(
            device: context.device,
            values: &precompute.faceNeighborRows.values
        ) else {
            throw NativeDenoiseError.solverFailed
        }

        guard let staticWeightBuffer = makeBufferAndRelease(
            device: context.device,
            values: &precompute.staticWeights
        ) else {
            throw NativeDenoiseError.solverFailed
        }

        var weightedInitial = precompute.weightedInitialSignals.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        precompute.weightedInitialSignals.removeAll(keepingCapacity: false)
        guard let weightedInitialBuffer = makeBufferAndRelease(
            device: context.device,
            values: &weightedInitial
        ) else {
            throw NativeDenoiseError.solverFailed
        }

        return try run(
            prepared: PreparedFilter(
                faceCount: faceCount,
                rowOffsetBuffer: rowOffsetBuffer,
                rowValueBuffer: rowValueBuffer,
                staticWeightBuffer: staticWeightBuffer,
                weightedInitialBuffer: weightedInitialBuffer,
                convergenceThreshold: convergenceThreshold
            ),
            initialSignals: initialSignals,
            areaWeights: areaWeights,
            nu: nu,
            maxIterations: maxIterations,
            shouldCancel: shouldCancel
        )
    }

    static func run(
        prepared: PreparedFilter,
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        nu: Float,
        maxIterations: Int = 100,
        shouldCancel: (@Sendable () -> Bool)? = nil
    ) throws -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }
        guard let context = sharedContext,
              !initialSignals.isEmpty,
              initialSignals.count == prepared.faceCount,
              initialSignals.count == areaWeights.count,
              initialSignals.count <= UInt32.max,
              nu.isFinite,
              nu > 0,
              maxIterations > 0
        else {
            throw NativeDenoiseError.solverFailed
        }

        let faceCount = prepared.faceCount
        let reductionWidth = reductionThreadgroupWidth(for: context.gatherReductionPipeline)
        let partialCount = partialDisplacementCount(
            faceCount: faceCount,
            threadgroupWidth: reductionWidth
        )

        var signals = initialSignals.map { SIMD4<Float>($0.x, $0.y, $0.z, 0) }
        guard let signalsBuffer = makeBufferAndRelease(device: context.device, values: &signals) else {
            throw NativeDenoiseError.solverFailed
        }

        guard let filteredBuffer = context.device.makeBuffer(
            length: faceCount * MemoryLayout<SIMD4<Float>>.stride,
            options: [.storageModeShared]
        ) else {
            throw NativeDenoiseError.solverFailed
        }

        guard let areaWeightBuffer = makeBuffer(device: context.device, values: areaWeights),
              let partialDisplacementBuffer = context.device.makeBuffer(
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
            guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }
            guard let commandBuffer = context.commandQueue.makeCommandBuffer(),
                  let encoder = commandBuffer.makeComputeCommandEncoder()
            else {
                throw NativeDenoiseError.solverFailed
            }

            encodeGatherAndReduction(
                encoder,
                pipeline: context.gatherReductionPipeline,
                inputBuffer: inputBuffer,
                weightedInitialBuffer: prepared.weightedInitialBuffer,
                rowOffsetBuffer: prepared.rowOffsetBuffer,
                rowValueBuffer: prepared.rowValueBuffer,
                staticWeightBuffer: prepared.staticWeightBuffer,
                areaWeightBuffer: areaWeightBuffer,
                outputBuffer: outputBuffer,
                partialBuffer: partialDisplacementBuffer,
                hDynamic: &hDynamic,
                faceCount: faceCount,
                threadgroupWidth: reductionWidth
            )
            encoder.endEncoding()

            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()
            guard commandBuffer.error == nil else { throw NativeDenoiseError.solverFailed }
            guard shouldCancel?() != true else { throw NativeDenoiseError.cancelled }

            latestBuffer = outputBuffer
            let displacement = readPartialDisplacement(from: partialDisplacementBuffer, count: partialCount)

            if displacement <= prepared.convergenceThreshold {
                return (readFloat3Signals(from: outputBuffer, count: faceCount), iteration, true)
            }

            swap(&inputBuffer, &outputBuffer)
        }

        return (readFloat3Signals(from: latestBuffer, count: faceCount), maxIterations, false)
    }

    private static func boundBuffer<T>(
        _ buffer: MTLBuffer,
        as type: T.Type,
        count: Int
    ) -> UnsafeMutableBufferPointer<T> {
        UnsafeMutableBufferPointer(
            start: buffer.contents().bindMemory(to: type, capacity: count),
            count: count
        )
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

    private static func encodeGatherAndReduction(
        _ encoder: MTLComputeCommandEncoder,
        pipeline: MTLComputePipelineState,
        inputBuffer: MTLBuffer,
        weightedInitialBuffer: MTLBuffer,
        rowOffsetBuffer: MTLBuffer,
        rowValueBuffer: MTLBuffer,
        staticWeightBuffer: MTLBuffer,
        areaWeightBuffer: MTLBuffer,
        outputBuffer: MTLBuffer,
        partialBuffer: MTLBuffer,
        hDynamic: inout Float,
        faceCount: Int,
        threadgroupWidth: Int
    ) {
        var count = UInt32(faceCount)
        let partialCount = partialDisplacementCount(
            faceCount: faceCount,
            threadgroupWidth: threadgroupWidth
        )

        encoder.setComputePipelineState(pipeline)
        encoder.setBuffer(inputBuffer, offset: 0, index: 0)
        encoder.setBuffer(weightedInitialBuffer, offset: 0, index: 1)
        encoder.setBuffer(rowOffsetBuffer, offset: 0, index: 2)
        encoder.setBuffer(rowValueBuffer, offset: 0, index: 3)
        encoder.setBuffer(staticWeightBuffer, offset: 0, index: 4)
        encoder.setBuffer(areaWeightBuffer, offset: 0, index: 5)
        encoder.setBuffer(outputBuffer, offset: 0, index: 6)
        encoder.setBuffer(partialBuffer, offset: 0, index: 7)
        encoder.setBytes(&hDynamic, length: MemoryLayout<Float>.stride, index: 8)
        encoder.setBytes(&count, length: MemoryLayout<UInt32>.stride, index: 9)
        encoder.setThreadgroupMemoryLength(
            threadgroupMemoryLength(threadgroupWidth: threadgroupWidth),
            index: 0
        )
        encoder.dispatchThreads(
            MTLSize(width: partialCount * threadgroupWidth, height: 1, depth: 1),
            threadsPerThreadgroup: MTLSize(width: threadgroupWidth, height: 1, depth: 1)
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
            threadgroupMemoryLength(threadgroupWidth: threadgroupWidth),
            index: 0
        )
        encoder.dispatchThreads(
            MTLSize(width: partialCount * threadgroupWidth, height: 1, depth: 1),
            threadsPerThreadgroup: MTLSize(width: threadgroupWidth, height: 1, depth: 1)
        )
    }

    private static func reductionThreadgroupWidth(for pipeline: MTLComputePipelineState) -> Int {
        reductionThreadgroupWidth(maxTotalThreadsPerThreadgroup: pipeline.maxTotalThreadsPerThreadgroup)
    }

    /// The kernels' tree reduction halves the stride each step, so the width must be
    /// a power of two: largest power of two <= min(limit, 256).
    static func reductionThreadgroupWidth(maxTotalThreadsPerThreadgroup: Int) -> Int {
        let limit = max(1, min(maxTotalThreadsPerThreadgroup, 256))
        return 1 << (Int.bitWidth - 1 - limit.leadingZeroBitCount)
    }

    /// Threadgroup memory lengths must be multiples of 16 bytes.
    private static func threadgroupMemoryLength(threadgroupWidth: Int) -> Int {
        let bytes = threadgroupWidth * MemoryLayout<Float>.stride
        return (bytes + 15) / 16 * 16
    }

    private static func partialDisplacementCount(
        faceCount: Int,
        threadgroupWidth: Int
    ) -> Int {
        (faceCount + threadgroupWidth - 1) / threadgroupWidth
    }

    private static func readPartialDisplacement(from buffer: MTLBuffer, count: Int) -> Float {
        let pointer = buffer.contents().bindMemory(to: Float.self, capacity: count)
        return Float(UnsafeBufferPointer(start: pointer, count: count).reduce(0.0) { $0 + Double($1) })
    }

    private static func readFloat3Signals(from buffer: MTLBuffer, count: Int) -> [SIMD3<Float>] {
        let pointer = buffer.contents().bindMemory(to: SIMD4<Float>.self, capacity: count)
        return UnsafeBufferPointer(start: pointer, count: count).map {
            SIMD3<Float>($0.x, $0.y, $0.z)
        }
    }
}
