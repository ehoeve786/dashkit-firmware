#!/usr/bin/env bash
# Host tests for the CAN path: the
# chunked MCP251xFD receive/transmit core against a chip model, and the software
# routing against the generated manifest tables. Also runs the DBC engine test.
#
# Requires: gcc.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "${OUT}"' EXIT

INC=(-I "${SCRIPT_DIR}/shim" -I "${REPO_ROOT}/main/can" -I "${REPO_ROOT}/main/can/dbc")

echo "==> DBC engine"
cc -std=gnu11 -Wall -Wextra "${INC[@]}" \
   "${SCRIPT_DIR}/test_dbc.c" \
   "${REPO_ROOT}/main/can/dbc/dbc.c" \
   "${REPO_ROOT}/main/can/dbc/dbc_generated.c" \
   -lm -o "${OUT}/test_dbc"
"${OUT}/test_dbc" | tail -1

echo "==> CAN stream core + routing"
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -g "${INC[@]}" \
   "${SCRIPT_DIR}/test_can_stream.c" \
   "${REPO_ROOT}/main/can/mcp251xfd_stream.c" \
   "${REPO_ROOT}/main/can/can_route.c" \
   "${REPO_ROOT}/main/can/dbc/dbc_generated.c" \
   -o "${OUT}/test_can_stream"
"${OUT}/test_can_stream"
