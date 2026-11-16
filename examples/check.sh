#!/bin/bash
# Runs every example through examples/tests.as with rive --test, in a project copy that holds lib/ beside
# examples/, since rive-cli resolves imports only inside the project.
#   examples/check.sh          interpreted
#   examples/check.sh --aot    compiled ahead of time, with RIVE_WAMRC
# verify/toolchain.sh picks the CLI; any arguments go to it.
here="$(cd "$(dirname "$0")" && pwd)"
source "$here/../verify/toolchain.sh"
project=$(mktemp -d)
trap 'rm -rf "$project"' EXIT

mkdir -p "$project/examples"
rsync -a "$here/../lib/" "$project/lib/"
cp "$here"/*.as "$project/examples/"
cat > "$project/rive.yaml" << 'YAML'
name: box2d_examples
scripting: wasm
YAML

# The CLI compiles ahead of time whenever RIVE_WAMRC is set, so only --aot passes it on
wamrc=(-u RIVE_WAMRC)
[[ " $* " == *" --aot "* ]] && wamrc=(RIVE_WAMRC="$RIVE_WAMRC")
env "${wamrc[@]}" perl -e 'alarm shift; exec @ARGV' 600 "$RIVE" "$project" --test "$@" 2>&1 | grep -v "warning"
exit "${PIPESTATUS[0]}"
