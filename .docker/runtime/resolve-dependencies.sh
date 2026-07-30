#!/usr/bin/env bash

set -euo pipefail

source_dir="${1:?source directory is required}"
skip_file="${2:?skip-key file is required}"
output_file="${3:?output file is required}"

test -d "${source_dir}"
test -s "${skip_file}"

raw_keys="$(mktemp)"
skip_keys="$(mktemp)"
resolved_packages="$(mktemp)"
trap 'rm -f "${raw_keys}" "${skip_keys}" "${resolved_packages}"' EXIT

grep -vE '^[[:space:]]*(#|$)' "${skip_file}" | sort -u > "${skip_keys}"
test -s "${skip_keys}"

env \
  -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u ROS_PACKAGE_PATH \
  rosdep keys \
    --from-paths "${source_dir}" \
    --ignore-src \
    --rosdistro jazzy \
    --dependency-types exec \
  | sort -u > "${raw_keys}"

test -s "${raw_keys}"

while IFS= read -r skip_key; do
  if ! grep -Fxq "${skip_key}" "${raw_keys}"; then
    echo "Runtime skip key is not declared by the workspace: ${skip_key}" >&2
    exit 1
  fi
done < "${skip_keys}"

while IFS= read -r rosdep_key; do
  if grep -Fxq "${rosdep_key}" "${skip_keys}"; then
    continue
  fi

  resolution="$(rosdep resolve --rosdistro jazzy "${rosdep_key}")"
  installer="$(sed -n '1p' <<< "${resolution}")"
  packages="$(sed '1d;/^[[:space:]]*$/d' <<< "${resolution}")"

  if [ "${installer}" != "#apt" ] || [ -z "${packages}" ]; then
    echo "Runtime rosdep key did not resolve to apt: ${rosdep_key}" >&2
    printf '%s\n' "${resolution}" >&2
    exit 1
  fi

  for package in ${packages}; do
    if [[ ! "${package}" =~ ^[a-z0-9][a-z0-9+.-]*$ ]]; then
      echo "Invalid apt package from rosdep key ${rosdep_key}: ${package}" >&2
      exit 1
    fi
    printf '%s\n' "${package}"
  done
done < "${raw_keys}" > "${resolved_packages}"

sort -u "${resolved_packages}" > "${output_file}"
test -s "${output_file}"

printf 'Resolved runtime apt packages:\n'
sed -n '1,200p' "${output_file}"
