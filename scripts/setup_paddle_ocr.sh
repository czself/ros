#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OCR_ENV="${OCR_ENV:-$HOME/.venvs/community-ocr}"
python3 -m venv "$OCR_ENV"
"$OCR_ENV/bin/python" -m pip install --upgrade pip
"$OCR_ENV/bin/python" -m pip install --timeout 30 --retries 1 \
  --index-url "${OCR_PIP_INDEX:-https://mirrors.aliyun.com/pypi/simple}" \
  -r "$ROOT_DIR/models/ocr/paddle/requirements-lock.txt"
echo "独立OCR环境：$OCR_ENV；本地模型：$ROOT_DIR/models/ocr/paddle/PP-OCRv5_mobile_rec_infer"
