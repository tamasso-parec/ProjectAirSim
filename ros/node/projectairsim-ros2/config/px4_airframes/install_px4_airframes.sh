#!/usr/bin/env bash
#
# Copyright (C) Microsoft Corporation.
# Copyright (C) 2025 IAMAI CONSULTING CORP
# MIT License.
#
# Install Project AirSim's PX4 airframes into a PX4-Autopilot checkout.
#
# PX4 airframes live inside the firmware's ROMFS and are compiled in, so they
# have to be copied into the PX4 source tree and PX4 rebuilt. This script does
# the copy and the CMakeLists registration; it does not build.
#
# Usage:
#   install_px4_airframes.sh /path/to/PX4-Autopilot
#
# Then rebuild PX4:
#   cd /path/to/PX4-Autopilot && make px4_sitl_default
#
# The script is idempotent: running it twice leaves the tree unchanged.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -ne 1 ]]; then
    echo "usage: $(basename "$0") /path/to/PX4-Autopilot" >&2
    exit 2
fi

px4_root="$1"
airframes_dir="${px4_root}/ROMFS/px4fmu_common/init.d-posix/airframes"
cmake_file="${airframes_dir}/CMakeLists.txt"

if [[ ! -d "${px4_root}" ]]; then
    echo "PX4 checkout not found: ${px4_root}" >&2
    exit 1
fi

if [[ ! -d "${airframes_dir}" ]]; then
    echo "Not a PX4 checkout, no airframes directory: ${airframes_dir}" >&2
    exit 1
fi

if [[ ! -f "${cmake_file}" ]]; then
    echo "Airframe CMakeLists not found: ${cmake_file}" >&2
    exit 1
fi

# Refuse to touch a PX4 tree that is currently flying something. Rebuilding
# underneath a running SITL instance corrupts whatever it is doing.
if pgrep -x px4 >/dev/null 2>&1; then
    echo "A px4 process is running. Installing airframes means rebuilding PX4," >&2
    echo "which would disturb it. Stop PX4 first, then re-run this script." >&2
    exit 1
fi

installed=()
for airframe_path in "${script_dir}"/[0-9]*_*; do
    [[ -f "${airframe_path}" ]] || continue
    airframe="$(basename "${airframe_path}")"

    install -m 644 "${airframe_path}" "${airframes_dir}/${airframe}"

    # Register the airframe so PX4's build includes it. Each entry is a bare
    # indented filename inside the CMake list.
    if ! grep -qE "^[[:space:]]*${airframe}[[:space:]]*$" "${cmake_file}"; then
        tmp_file="$(mktemp)"
        awk -v entry="${airframe}" '
            /^\)[[:space:]]*$/ && !inserted { print "\t" entry; inserted = 1 }
            { print }
        ' "${cmake_file}" > "${tmp_file}"
        mv "${tmp_file}" "${cmake_file}"
    fi

    installed+=("${airframe}")
done

if [[ ${#installed[@]} -eq 0 ]]; then
    echo "No airframe files found in ${script_dir}" >&2
    exit 1
fi

echo "Installed into ${airframes_dir}:"
for airframe in "${installed[@]}"; do
    echo "  ${airframe}"
done
echo
echo "Now rebuild PX4:"
echo "  cd ${px4_root} && make px4_sitl_default"
