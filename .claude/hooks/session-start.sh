#!/bin/bash
set -euo pipefail

# Ensure dolt sql-server is running for beads_turtlebot-maze (port 3308)
DOLT_DATA_DIR="$CLAUDE_PROJECT_DIR/.beads/dolt"
DOLT_DB_DIR="$DOLT_DATA_DIR/beads_turtlebot-maze"
DOLT_PORT=3308
DOLT_LOG="/tmp/dolt-turtlebot-maze.log"

if command -v dolt &>/dev/null && [ -d "$DOLT_DB_DIR" ]; then
  if ! ss -tlnp 2>/dev/null | grep -q ":${DOLT_PORT} "; then
    nohup dolt sql-server \
      --data-dir="${DOLT_DATA_DIR}" \
      --port="${DOLT_PORT}" \
      --host=127.0.0.1 \
      --loglevel=warning \
      > "${DOLT_LOG}" 2>&1 &
    # Wait up to 5s for server to be ready
    for i in 1 2 3 4 5; do
      sleep 1
      ss -tlnp 2>/dev/null | grep -q ":${DOLT_PORT} " && break
    done
  fi
fi

# Only run the rest in remote (Claude Code on the web) environments
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

# Install pre-commit and Python linting/formatting tools
pip install --quiet pre-commit codespell yamllint

# Install pre-commit hook environments (downloads/caches all hook deps)
cd "$CLAUDE_PROJECT_DIR"
pre-commit install-hooks

# Install beads issue tracker (bd)
if ! command -v bd &>/dev/null; then
  npm install -g @beads/bd 2>/dev/null || \
    (curl -fsSL https://raw.githubusercontent.com/steveyegge/beads/main/scripts/install.sh | bash) 2>/dev/null || \
    true
fi

# Initialize beads database if not already set up
if command -v bd &>/dev/null && [ ! -d "$CLAUDE_PROJECT_DIR/.beads/dolt" ]; then
  cd "$CLAUDE_PROJECT_DIR"
  bd init --skip-hooks --quiet 2>/dev/null || true
fi
