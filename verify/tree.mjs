import { runModule } from './lib.mjs';

// The tree and the broad phase live in module globals; a collection every few ops moves their arrays.
// nanEqual: b2OffsetAABB adds in double, where wasm returns the canonical NaN and arm64 keeps the payload.
await runModule('tree', (e, ctx) => {
  e.aabbChecks();
  ctx.frameBoundary();

  e.treeBegin();
  for (let round = 0; round < 18; ++round) {
    const ops = e.treeStart(round);
    for (let op = 0; op < ops; ++op) {
      e.treeOp();
      if (op % 25 === 0) ctx.frameBoundary();
    }
    e.treeEnd();
    ctx.frameBoundary();
  }

  e.broadBegin();
  for (let round = 0; round < 8; ++round) {
    const steps = e.broadStart(round);
    for (let step = 0; step < steps; ++step) {
      e.broadStep();
      ctx.frameBoundary();
    }
    e.broadEnd();
    ctx.frameBoundary();
  }

  e.sortChecks();
}, { nanEqual: true });
