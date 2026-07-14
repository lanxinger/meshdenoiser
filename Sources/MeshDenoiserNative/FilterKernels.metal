#include <metal_stdlib>
using namespace metal;

kernel void gather_filter_signals_and_reduce_displacement(
    device const float4 *signals [[buffer(0)]],
    device const float4 *weightedInitialSignals [[buffer(1)]],
    device const uint *rowOffsets [[buffer(2)]],
    device const uint2 *rowValues [[buffer(3)]],
    device const float *staticWeights [[buffer(4)]],
    device const float *areaWeights [[buffer(5)]],
    device float4 *filteredSignals [[buffer(6)]],
    device float *partialDisplacement [[buffer(7)]],
    constant float &hDynamic [[buffer(8)]],
    constant uint &faceCount [[buffer(9)]],
    threadgroup float *scratch [[threadgroup(0)]],
    uint id [[thread_position_in_grid]],
    uint localId [[thread_position_in_threadgroup]],
    uint groupId [[threadgroup_position_in_grid]],
    uint groupSize [[threads_per_threadgroup]]
) {
    float displacement = 0.0f;
    if (id < faceCount) {
        float3 value = weightedInitialSignals[id].xyz;
        float3 signal = signals[id].xyz;
        uint start = rowOffsets[id];
        uint end = rowOffsets[id + 1];

        for (uint offset = start; offset < end; ++offset) {
            uint2 neighbor = rowValues[offset];
            float3 neighborSignal = signals[neighbor.x].xyz;
            float3 diff = signal - neighborSignal;
            float dynamicWeight = staticWeights[neighbor.y] * exp(hDynamic * dot(diff, diff));
            value += neighborSignal * dynamicWeight;
        }

        float magnitude = length(value);
        float3 filteredSignal = signal;
        if (isfinite(magnitude) && magnitude > 0.0f) {
            filteredSignal = value / magnitude;
        }
        filteredSignals[id] = float4(filteredSignal, 0.0f);

        float3 diff = filteredSignal - signal;
        displacement = areaWeights[id] * dot(diff, diff);
    }

    scratch[localId] = displacement;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint stride = groupSize / 2; stride > 0; stride >>= 1) {
        if (localId < stride) {
            scratch[localId] += scratch[localId + stride];
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (localId == 0) {
        partialDisplacement[groupId] = scratch[0];
    }
}

kernel void reduce_displacement(
    device const float4 *previousSignals [[buffer(0)]],
    device const float4 *currentSignals [[buffer(1)]],
    device const float *areaWeights [[buffer(2)]],
    device float *partialDisplacement [[buffer(3)]],
    constant uint &faceCount [[buffer(4)]],
    threadgroup float *scratch [[threadgroup(0)]],
    uint id [[thread_position_in_grid]],
    uint localId [[thread_position_in_threadgroup]],
    uint groupId [[threadgroup_position_in_grid]],
    uint groupSize [[threads_per_threadgroup]]
) {
    float value = 0.0f;
    if (id < faceCount) {
        float3 diff = currentSignals[id].xyz - previousSignals[id].xyz;
        value = areaWeights[id] * dot(diff, diff);
    }

    scratch[localId] = value;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint stride = groupSize / 2; stride > 0; stride >>= 1) {
        if (localId < stride) {
            scratch[localId] += scratch[localId + stride];
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (localId == 0) {
        partialDisplacement[groupId] = scratch[0];
    }
}
