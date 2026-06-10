# shellcheck shell=bash
# Pick Python for CRP/LeRobot verify scripts.
# Priority: explicit PYTHON > active conda (if 3.12+) > .venv (if 3.12+ and has draccus) > conda lerobot > python3.12.

_python_ok() {
  local py="$1"
  [[ -x "${py}" ]] || return 1
  "${py}" -c "import sys; exit(0 if sys.version_info >= (3, 12) else 1)" 2>/dev/null || return 1
  return 0
}

_python_has_lerobot_deps() {
  local py="$1"
  "${py}" -c "import draccus" 2>/dev/null
}

_resolve_lerobot_python() {
  local root="${1:?root}"

  if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python" ]]; then
    if _python_ok "${CONDA_PREFIX}/bin/python"; then
      echo "${CONDA_PREFIX}/bin/python"
      return
    fi
  fi

  if [[ -x "${root}/.venv/bin/python" ]] && _python_ok "${root}/.venv/bin/python"; then
    if _python_has_lerobot_deps "${root}/.venv/bin/python"; then
      echo "${root}/.venv/bin/python"
      return
    fi
  fi

  if [[ -x "${HOME}/miniforge3/envs/lerobot/bin/python" ]] && _python_ok "${HOME}/miniforge3/envs/lerobot/bin/python"; then
    echo "${HOME}/miniforge3/envs/lerobot/bin/python"
    return
  fi

  if [[ -x "${root}/.venv/bin/python" ]] && _python_ok "${root}/.venv/bin/python"; then
    echo "${root}/.venv/bin/python"
    return
  fi

  if command -v python3.12 >/dev/null 2>&1 && _python_ok "$(command -v python3.12)"; then
    command -v python3.12
    return
  fi

  command -v python3
}
