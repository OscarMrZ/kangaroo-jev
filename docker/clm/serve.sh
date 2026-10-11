#!/bin/bash
# Qwen3-8B pooling encoder (last-token pooling + prefix cache, the setup the CLM head was trained against), then
# the CLM API on :8700. Qwen3-8B's bf16 weights (16.4 GB) do not fit a 16 GB card: OFFLOAD_GB of them stay in
# CPU memory, which keeps the embeddings exact at some latency cost. One request (8 states, ~4.5k tokens) is one
# forward pass, so the offloaded weights cross PCIe once per decision. VLLM_ARGS: extra vllm serve flags.
set -e
vllm serve Qwen/Qwen3-8B --served-model-name qwen3-8b --runner pooling --enforce-eager --enable-prefix-caching \
  --max-model-len "${MAXLEN:-2048}" --gpu-memory-utilization "${UTIL:-0.9}" --cpu-offload-gb "${OFFLOAD_GB:-0}" \
  --max-num-seqs 16 --max-num-batched-tokens 8192 --port 8090 ${VLLM_ARGS:-} &
until curl -sf localhost:8090/health >/dev/null; do
  kill -0 $! 2>/dev/null || { echo "vllm exited"; exit 1; }
  sleep 5
done
exec clm-serve --no-ui --port 8700 --max-tokens "${MAXLEN:-2048}" ${CLM_ARGS:-}
