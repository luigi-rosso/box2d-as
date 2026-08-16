// Minimal-runtime cadence probe: step-level control so the host can sweep
// how often it collects. The bake itself runs the stub runtime and rewinds
// instead; this quantifies the TCMS alternative.

export { benchStart, benchStep, benchFinish, benchRun } from "../bench";

export function benchCollect(): void {
    __collect();
}
