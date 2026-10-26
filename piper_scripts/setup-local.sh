#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
env_path="$HOME/.venvs/lerobot-piper"
python_bin="${PYTHON_BIN:-$HOME/.local/bin/python3.11}"
if [ ! -x "$env_path/bin/python" ]; then
  "$python_bin" -m venv "$env_path"
fi
"$env_path/bin/python" -m pip install --timeout 90 torch==2.6.0 torchvision==0.21.0 \
  --index-url https://download.pytorch.org/whl/cpu --extra-index-url https://pypi.org/simple
"$env_path/bin/python" -m pip install --timeout 90 -e '.[piper]' -c piper_scripts/constraints-local.txt
ffmpeg_bin="$("$env_path/bin/python" -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')"
ln -sf "$ffmpeg_bin" "$env_path/bin/ffmpeg"
"$env_path/bin/python" -m pip check
printf 'Ready: source %s/bin/activate\n' "$env_path"
