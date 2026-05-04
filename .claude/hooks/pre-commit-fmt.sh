#!/usr/bin/env bash
# Block `git commit` if `cargo fmt` would rewrite anything.
#
# Wired in .claude/settings.json as a PreToolUse hook on the Bash matcher.
# The hook receives the tool-call JSON on stdin; we extract the proposed
# command, short-circuit when it has nothing to do with `git commit`, and
# only invoke cargo fmt when a commit is actually about to fire.
#
# Communicates back to Claude Code via the documented JSON shape on stdout
# (hookSpecificOutput.permissionDecision = "deny"), so the user sees the
# reason directly in the permission-denied surface.
#
# Why a hook at all: CI runs `cargo fmt --all -- --check` and fails the
# build on any drift. We've burned cycles pushing fmt-dirty branches and
# then patching them; doing the same check ~300 ms locally before every
# commit catches it before it leaves the machine.

set -uo pipefail

# Pull the proposed shell command out of the hook payload. Python rather
# than jq so the hook works on any box with a Rust toolchain and a base
# Python install, no extra system package required.
cmd=$(python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
print(data.get("tool_input", {}).get("command", ""))
')

# Match `git commit` anywhere in the command — `git add ... && git commit
# ...` and `git commit --amend` and similar all need the gate. Skip the
# fmt run for any other Bash invocation; this hook fires on every Bash
# call so the fast-path matters.
case "$cmd" in
    *"git commit"*) ;;
    *) exit 0 ;;
esac

# `cargo fmt --all -- --check` is the same invocation the CI fmt job
# runs. It exits 0 on clean, 1 on drift. Either way it's quiet on
# stdout/stderr unless asked, which keeps the hook output a clean JSON
# blob.
if cargo fmt --all -- --check >/dev/null 2>&1; then
    exit 0
fi

# Drift detected — block the commit. The reason text is what the user
# sees in the deny surface; keep it actionable (one shell command they
# can copy) rather than a long explanation.
cat <<'JSON'
{
  "hookSpecificOutput": {
    "hookEventName": "PreToolUse",
    "permissionDecision": "deny",
    "permissionDecisionReason": "cargo fmt would rewrite files. Run `cargo fmt --all` from the repo root, then retry the commit. (CI's fmt job rejects formatting drift; this hook catches it before the push.)"
  }
}
JSON
exit 0
