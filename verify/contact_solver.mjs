import { runModule } from './lib.mjs';

// One scenario per call, with a collection between them.
await runModule('contact_solver', (e, ctx) => {
  e.begin();
  for (let i = 0; i < 400; ++i) {
    e.scenario();
    ctx.frameBoundary();
  }
});
