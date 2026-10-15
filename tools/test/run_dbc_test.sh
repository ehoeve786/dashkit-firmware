#!/usr/bin/env bash
# Run the DBC engine unit test (tools/test/test_dbc.c) against main/can/dbc.
# Compiled with plain gcc against the ESP-IDF shim headers in tools/test/shim,
# so no ESP-IDF or mbedTLS is needed.
#
# Requires: gcc.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK="${REPO_ROOT}/build/dbc-host-test"
mkdir -p "${WORK}"

echo "==> Compiling DBC test"
cc -std=c99 -Wall -Wextra \
   -I "${SCRIPT_DIR}/shim" \
   -I "${REPO_ROOT}/main/can" \
   -I "${REPO_ROOT}/main/can/dbc" \
   "${SCRIPT_DIR}/test_dbc.c" \
   "${REPO_ROOT}/main/can/dbc/dbc.c" \
   "${REPO_ROOT}/main/can/dbc/dbc_generated.c" \
   -lm -o "${WORK}/test_dbc"

echo "==> Running DBC test"
"${WORK}/test_dbc"
