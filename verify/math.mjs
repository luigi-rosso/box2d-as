import { runModule } from './lib.mjs';

// The C driver is built at -O2 as the reference build is, and clang commutes operations whose
// operands are both NaN, so NaN payloads are not part of the contract here. NAN_EXACT=1 shows them.
await runModule('math', (e) => e.run(), { nanEqual: process.env.NAN_EXACT !== '1' });
