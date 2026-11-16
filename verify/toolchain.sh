# Sourced by the verify and bench scripts. RIVE_TOOLCHAIN names a rive checkout at master 4a861ff147 or later with release
# builds of rasc and rive-cli, else the first of rive-box2d-master and rive-master that has rasc. RASC, AS_STD, RIVE
# and RIVE_WAMRC override each tool; wamrc defaults to the one built from the checkout's patched WAMR into the CLI's
# out directory.
if [ -z "$RIVE_TOOLCHAIN" ]; then
	for root in "$HOME/Projects/rive-box2d-master" "$HOME/Projects/rive-master"; do
		if [ -x "$root/packages/rasc/animascript/out/release/rasc" ]; then
			RIVE_TOOLCHAIN=$root
			break
		fi
	done
fi
RASC=${RASC:-$RIVE_TOOLCHAIN/packages/rasc/animascript/out/release/rasc}
if [ ! -x "$RASC" ]; then
	echo "no rasc found; set RIVE_TOOLCHAIN, or RASC and AS_STD" >&2
	exit 2
fi
AS_STD=${AS_STD:-$(cd "$(dirname "$RASC")/../.." && pwd)/std}
RIVE=${RIVE:-$RIVE_TOOLCHAIN/packages/rive-cli/out/release/rive}
RIVE_WAMRC=${RIVE_WAMRC:-$RIVE_TOOLCHAIN/packages/rive-cli/out/wamrc}
