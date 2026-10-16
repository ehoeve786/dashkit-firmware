#!/usr/bin/env bash
# Run the CAN filter unit test (tools/test/test_can_filter.c) against
# main/can/can_filter.c. Plain gcc with the ESP-IDF shim headers in tools/test/shim.
#
# Requires: gcc.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK="${REPO_ROOT}/build/can-filter-host-test"
mkdir -p "${WORK}"

echo "==> Compiling CAN filter test"
cc -std=c99 -Wall -Wextra \
   -I "${SCRIPT_DIR}/shim" \
   -I "${SCRIPT_DIR}/shim/freertos" \
   -I "${REPO_ROOT}/main/can" \
   "${SCRIPT_DIR}/test_can_filter.c" \
   "${REPO_ROOT}/main/can/can_filter.c" \
   -o "${WORK}/test_can_filter"

echo "==> Running CAN filter test"
"${WORK}/test_can_filter"
