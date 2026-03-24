#!/usr/bin/env bash
set -euo pipefail

# Bump version in VERSION file and Cargo.toml
# Usage: ./scripts/version-bump.sh 0.2.0

NEW_VERSION="${1:?Usage: version-bump.sh <new-version>}"
CURRENT_VERSION=$(cat VERSION | tr -d '[:space:]')

echo "Bumping version: ${CURRENT_VERSION} → ${NEW_VERSION}"

# Update VERSION file
echo -n "$NEW_VERSION" > VERSION

# Update Cargo.toml
sed -i "s/^version = \"${CURRENT_VERSION}\"/version = \"${NEW_VERSION}\"/" Cargo.toml

echo "Done. Updated VERSION and Cargo.toml to ${NEW_VERSION}"
echo "Don't forget to update CHANGELOG.md"
