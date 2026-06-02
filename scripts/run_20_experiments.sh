#!/usr/bin/env bash
set -u -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

REPEATS="${REPEATS:-5}"
PAUSE_SECONDS="${PAUSE_SECONDS:-2}"
CONTINUE_ON_FAILURE="${CONTINUE_ON_FAILURE:-0}"

if [[ -n "${FLEXJOINT_EXE:-}" ]]; then
  EXE="${FLEXJOINT_EXE}"
elif [[ -x "${PROJECT_ROOT}/build/flexjoint_vs" ]]; then
  EXE="${PROJECT_ROOT}/build/flexjoint_vs"
elif [[ -x "./flexjoint_vs" ]]; then
  EXE="$(pwd)/flexjoint_vs"
else
  EXE="${PROJECT_ROOT}/build/flexjoint_vs"
fi

CONFIGS=(
  "${PROJECT_ROOT}/config/robot_config.yaml"
  "${PROJECT_ROOT}/config/robot_config_proposed_no_fast.yaml"
  "${PROJECT_ROOT}/config/robot_config_baseline_pd.yaml"
  "${PROJECT_ROOT}/config/robot_config_baseline_pd_no_fast.yaml"
)

LABELS=(
  "proposed"
  "proposed_no_fast"
  "baseline_pd"
  "baseline_pd_no_fast"
)

if [[ ! -x "${EXE}" ]]; then
  echo "Error: executable not found or not executable: ${EXE}" >&2
  echo "Set FLEXJOINT_EXE=/path/to/flexjoint_vs if your binary is elsewhere." >&2
  exit 1
fi

if ! [[ "${REPEATS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: REPEATS must be a positive integer, got: ${REPEATS}" >&2
  exit 1
fi

total=$(( ${#CONFIGS[@]} * REPEATS ))
done_count=0
failed_count=0

echo "Executable: ${EXE}"
echo "Repeats per config: ${REPEATS}"
echo "Total runs: ${total}"
echo

for i in "${!CONFIGS[@]}"; do
  config="${CONFIGS[$i]}"
  label="${LABELS[$i]}"

  if [[ ! -f "${config}" ]]; then
    echo "Error: config not found: ${config}" >&2
    exit 1
  fi

  for run_idx in $(seq 1 "${REPEATS}"); do
    done_count=$((done_count + 1))
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Run ${done_count}/${total}: ${label} (${run_idx}/${REPEATS})"
    echo "  config: ${config}"

    "${EXE}" "${config}"
    status=$?

    if [[ ${status} -ne 0 ]]; then
      failed_count=$((failed_count + 1))
      echo "  failed with exit code ${status}" >&2
      if [[ "${CONTINUE_ON_FAILURE}" != "1" ]]; then
        echo "Stopping. Set CONTINUE_ON_FAILURE=1 to keep running after failures." >&2
        exit "${status}"
      fi
    else
      echo "  completed"
    fi

    if [[ "${done_count}" -lt "${total}" && "${PAUSE_SECONDS}" != "0" ]]; then
      sleep "${PAUSE_SECONDS}"
    fi
    echo
  done
done

echo "Batch finished: ${done_count} runs attempted, ${failed_count} failed."
