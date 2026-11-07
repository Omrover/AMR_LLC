#!/usr/bin/env bash
# Compile and run the firmware logic on the host (no board, no Zephyr).
#   ./run_host_tests.sh            -> tests YOUR src/student.c
#   ./run_host_tests.sh solution   -> tests src/student_solution.c
set -euo pipefail
cd "$(dirname "$0")"
student=../src/student.c
[[ "${1:-}" == "solution" ]] && student=../src/student_solution.c
out=$(mktemp)
trap 'rm -f "$out"' EXIT
gcc -std=c11 -Wall -Wextra -Werror -O1 host_test.c ../src/lab_logic.c "$student" -lm -o "$out"
"$out"
