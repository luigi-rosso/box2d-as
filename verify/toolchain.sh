# Sourced by the verify scripts: RASC and AS_STD from the environment, else the first rasc build
# found in rive-pinefall, then rive-master.
if [ -z "$RASC" ]; then
	for root in "$HOME/Projects/rive-pinefall" "$HOME/Projects/rive-master"; do
		if [ -x "$root/packages/rasc/animascript/out/release/rasc" ]; then
			RASC="$root/packages/rasc/animascript/out/release/rasc"
			AS_STD=${AS_STD:-$root/packages/rasc/animascript/std}
			break
		fi
	done
fi
if [ -z "$RASC" ] || [ ! -x "$RASC" ]; then
	echo "no rasc found; set RASC and AS_STD" >&2
	exit 2
fi
AS_STD=${AS_STD:-$(cd "$(dirname "$RASC")/../.." && pwd)/std}
