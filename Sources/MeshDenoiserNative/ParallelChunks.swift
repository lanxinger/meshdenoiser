import Dispatch

/// Splits `0..<count` into fixed-size chunks and runs them concurrently.
///
/// The partition depends only on `count` and `chunkSize`, never on the number of
/// threads, so per-chunk partial results combined in chunk order are deterministic.
enum ParallelChunks {
    static func chunkCount(for count: Int, chunkSize: Int) -> Int {
        precondition(chunkSize > 0)
        return count <= 0 ? 0 : (count + chunkSize - 1) / chunkSize
    }

    static func range(chunk: Int, count: Int, chunkSize: Int) -> Range<Int> {
        let start = chunk * chunkSize
        return start..<min(start + chunkSize, count)
    }

    /// Runs `body(chunkIndex, range)` for every chunk. Bodies must only write to
    /// memory owned by their own chunk.
    static func forEach(
        count: Int,
        chunkSize: Int,
        _ body: (_ chunk: Int, _ range: Range<Int>) -> Void
    ) {
        let chunks = chunkCount(for: count, chunkSize: chunkSize)
        if chunks <= 1 {
            if chunks == 1 { body(0, 0..<count) }
            return
        }
        DispatchQueue.concurrentPerform(iterations: chunks) { chunk in
            body(chunk, range(chunk: chunk, count: count, chunkSize: chunkSize))
        }
    }

    /// Like `forEach`, but each chunk first polls `shouldCancel` and skips its work
    /// when cancellation was requested. Throws `.cancelled` after all chunks return
    /// if any chunk observed cancellation. Per-chunk flags avoid shared writes.
    static func forEachCancellable(
        count: Int,
        chunkSize: Int,
        shouldCancel: (@Sendable () -> Bool)?,
        _ body: (_ chunk: Int, _ range: Range<Int>) -> Void
    ) throws {
        guard let shouldCancel else {
            forEach(count: count, chunkSize: chunkSize, body)
            return
        }

        let chunks = chunkCount(for: count, chunkSize: chunkSize)
        var cancelledFlags = [Bool](repeating: false, count: chunks)
        cancelledFlags.withUnsafeMutableBufferPointer { flags in
            forEach(count: count, chunkSize: chunkSize) { chunk, range in
                if shouldCancel() {
                    flags[chunk] = true
                    return
                }
                body(chunk, range)
            }
        }
        if cancelledFlags.contains(true) {
            throw NativeDenoiseError.cancelled
        }
    }
}
