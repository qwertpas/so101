#!/bin/sh
set -eu
cd "$(dirname "$0")"
UV_PROJECT_ENVIRONMENT="$(pwd)/../.venv" uv sync --locked
