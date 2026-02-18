#!/bin/bash
set -euo pipefail

# Only run in remote (Claude Code on the web) environments
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
