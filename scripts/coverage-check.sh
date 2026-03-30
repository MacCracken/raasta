#!/usr/bin/env bash
set -euo pipefail

# Check test coverage meets the minimum threshold.
# Usage: ./scripts/coverage-check.sh [threshold]
# Default threshold: 80%

THRESHOLD="${1:-80}"

echo "Running coverage analysis (threshold: ${THRESHOLD}%)..."

# Run tarpaulin and capture the summary line
OUTPUT=$(cargo tarpaulin --all-features --skip-clean 2>&1)
COVERAGE=$(echo "$OUTPUT" | grep -oP '[\d.]+(?=% coverage)' | tail -1)

if [ -z "$COVERAGE" ]; then
    echo "ERROR: Could not parse coverage percentage"
    echo "$OUTPUT" | tail -5
    exit 1
fi

echo "Coverage: ${COVERAGE}%"

# Compare using awk for float comparison
PASS=$(awk "BEGIN { print ($COVERAGE >= $THRESHOLD) }")

if [ "$PASS" = "1" ]; then
    echo "PASS: ${COVERAGE}% >= ${THRESHOLD}%"
    exit 0
else
    echo "FAIL: ${COVERAGE}% < ${THRESHOLD}%"
    exit 1
fi
