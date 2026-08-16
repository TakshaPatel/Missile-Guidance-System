#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

cargo run --release
python3 visualize.py
