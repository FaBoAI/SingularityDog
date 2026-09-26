#!/usr/bin/env bash
set -euo pipefail

readonly package_dir=/home/jetson/singularitydog-tests/id7-photo-pair-20260926-r1
readonly rear_device=/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0
readonly uids_file=/home/jetson/singularitydog-tests/fr-toward-stance-r11/expected-uids.json
readonly output_dir=/home/jetson/singularitydog-logs/RO-id7-photo-pair-$(date +%Y%m%d-%H%M%S)-$$

echo 'ID7写真A/Bと角度読値を対応付けます。モーターは脱力したまま、支持台上で行ってください。'
echo '各姿勢を写真に撮り、動かさずにEnterを押すと、ID7の角度を3回読みます。自動駆動・校正変更はありません。'
PYTHONPATH="$package_dir" exec python3 -B -m singularitydog_hw.id7_photo_pair \
  --execute-readonly --port "$rear_device" \
  --expected-uids "$uids_file" --output "$output_dir"
