#!/bin/bash
set -e
script_dir="$(cd "$(dirname "$0")" && pwd)"
cd "$script_dir"
docker compose up --build
