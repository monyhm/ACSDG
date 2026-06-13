#!/usr/bin/env bash
# Start the Household Budget Tracker.
#   ./run.sh                 # http://0.0.0.0:8000
#   PORT=9000 ./run.sh
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -f config.json ]; then
  echo "No config.json found — copying config.example.json."
  echo "Edit config.json to add your cards before going live."
  cp config.example.json config.json
fi

PORT="${PORT:-8000}"
exec uvicorn app.main:app --host 0.0.0.0 --port "$PORT"
