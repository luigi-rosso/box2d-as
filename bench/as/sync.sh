#!/bin/sh
# Copies lib and the verify scenes into this project, which rive-cli builds from its own directory
HERE=$(cd "$(dirname "$0")" && pwd)
rsync -a --delete "$HERE/../../lib/" "$HERE/lib/"
mkdir -p "$HERE/scenes" && sed "s|\"../../lib/|\"../lib/|" "$HERE/../../verify/scenes/scenes.as" > "$HERE/scenes/scenes.as"
