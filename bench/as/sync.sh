#!/bin/sh
# Copies lib and the verify scenes into this project, which rive-cli builds from its own directory
HERE=$(cd "$(dirname "$0")" && pwd)
rsync -a --delete "$HERE/../../lib/" "$HERE/lib/"
mkdir -p "$HERE/scenes"
for f in scenes record; do
	sed "s|\"../../lib/|\"../lib/|" "$HERE/../../verify/scenes/$f.as" > "$HERE/scenes/$f.as"
done
