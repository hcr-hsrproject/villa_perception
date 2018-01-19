#!/usr/bin/env bash
ids=(
'0B10UkB-gUiRudHZMdll1YW80c28' # Gender weights
'0B10UkB-gUiRucExhX1ZDU2RWYmc' # Age weights
)

force="$1"

for id in "${ids[@]}"; do
	gdrive download --recursive $force $id || true
done

