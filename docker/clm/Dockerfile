# CLM-8B (Contrastive-LM) behind its TypeSafe-compatible API: vLLM serves the Qwen3-8B encoder on the GPU,
# clm-serve runs the projection heads on the CPU.
FROM vllm/vllm-openai:v0.31.0

ARG CLM_SHA=d5f9ef0fd9bde185df0ceaad4f4ecc6cfe8c34f6
RUN pip install --no-cache-dir --no-deps https://github.com/Contrastive-LM/CLM/archive/${CLM_SHA}.tar.gz

COPY docker/clm/serve.sh /serve.sh
ENTRYPOINT ["/serve.sh"]
