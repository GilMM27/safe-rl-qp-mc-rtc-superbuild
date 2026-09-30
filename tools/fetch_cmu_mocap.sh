#!/usr/bin/env bash

set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [output-directory]" >&2
  exit 2
fi

output_dir="${1:-${PWD}/data/cmu/subject14}"
mkdir -p "${output_dir}"

base_url="http://mocap.cs.cmu.edu/subjects/14"
curl --fail --location --retry 3 "${base_url}/14.asf" --output "${output_dir}/14.asf"
curl --fail --location --retry 3 "${base_url}/14_07.amc" --output "${output_dir}/14_07.amc"

cat <<EOF
Downloaded CMU subject 14, trial 14_07:
  skeleton: ${output_dir}/14.asf
  motion:   ${output_dir}/14_07.amc

Description: jump up to grab, reach for, tiptoe
Source: http://mocap.cs.cmu.edu/search.php?subjectnumber=14&motion=%25%25%25&maincat=%25&subcat=%25&subtext=yes

Review the CMU database terms before redistributing these files. The repository
does not commit or automatically install the recording.
EOF
