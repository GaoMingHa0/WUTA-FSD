#!/usr/bin/env bash
set -euo pipefail

EXPECTED_PACKAGES=16
ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${SCRIPT_DIR}"

usage() {
  cat <<'EOF'
Usage: ./build_ws.sh [--clean]

Build the WUTA-FSD ROS 2 workspace reproducibly.
The script resolves the workspace from its own location and automatically
cleans stale CMake/colcon output when the repository has been moved.

Options:
  --clean   Remove build/, install/, and log/ before building.
EOF
}

CLEAN=0
for arg in "$@"; do
  case "$arg" in
    --clean)
      CLEAN=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd git
require_cmd colcon
require_cmd pkg-config
require_cmd cmake

stale_build_cache() {
  local cache cache_dir cached_dir cached_home

  [[ -d build ]] || return 1

  while IFS= read -r cache; do
    cache_dir="$(cd "$(dirname "${cache}")" && pwd -P)"
    cached_dir="$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' "${cache}" | tail -n 1)"
    cached_home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "${cache}" | tail -n 1)"

    if [[ -n "${cached_dir}" && "${cached_dir}" != "${cache_dir}" ]]; then
      echo "Stale CMake cache detected: ${cache}" >&2
      echo "  cached build dir: ${cached_dir}" >&2
      echo "  current build dir: ${cache_dir}" >&2
      return 0
    fi

    if [[ "${cached_home}" == */ros2_ws/* &&
          "${cached_home}" != "${WS_DIR}/"* &&
          "${cached_home}" != "${cache_dir}/"* ]]; then
      echo "Stale CMake source path detected: ${cache}" >&2
      echo "  cached source dir: ${cached_home}" >&2
      echo "  current workspace: ${WS_DIR}" >&2
      return 0
    fi
  done < <(find build -name CMakeCache.txt -print)

  return 1
}

stale_install_prefix() {
  local current_install cached_install

  [[ -f install/setup.sh ]] || return 1

  current_install="$(cd install && pwd -P)"
  cached_install="$(
    sed -n 's/^_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=//p' install/setup.sh \
      | head -n 1 \
      | sed 's/^"//; s/"$//'
  )"

  if [[ -n "${cached_install}" && "${cached_install}" != "${current_install}" ]]; then
    echo "Stale colcon install prefix detected: install/setup.sh" >&2
    echo "  cached install dir: ${cached_install}" >&2
    echo "  current install dir: ${current_install}" >&2
    return 0
  fi

  return 1
}

if ! pkg-config --exists ompi-c ompi-cxx; then
  echo "Missing OpenMPI pkg-config metadata: ompi-c and/or ompi-cxx" >&2
  echo "Install OpenMPI development packages, for example: sudo apt install libopenmpi-dev" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 setup file not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  echo "Install ROS 2 ${ROS_DISTRO} or set ROS_DISTRO to the installed distro." >&2
  exit 1
fi

cd "${REPO_ROOT}"
git submodule update --init --recursive

cd "${WS_DIR}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

package_count="$(colcon list --base-paths src --names-only | wc -l)"
if [[ "${package_count}" -ne "${EXPECTED_PACKAGES}" ]]; then
  echo "Expected ${EXPECTED_PACKAGES} ROS packages, found ${package_count}:" >&2
  colcon list --base-paths src --names-only >&2
  exit 1
fi

if [[ "${CLEAN}" -eq 0 ]]; then
  if stale_build_cache || stale_install_prefix; then
    echo "Repository path changed; removing build/, install/, and log/ before rebuilding." >&2
    CLEAN=1
  fi
fi

if [[ "${CLEAN}" -eq 1 ]]; then
  rm -rf build install log
fi

colcon build --symlink-install \
  --cmake-args "-DCMAKE_MODULE_PATH=${WS_DIR}/cmake"

set +u
source install/setup.bash
set -u
echo "Build complete: ${EXPECTED_PACKAGES} packages finished."
