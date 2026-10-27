import { runModule } from './lib.mjs';

// Special lane values feed arithmetic whose NaN payload clang may pick from either operand.
await runModule('simd', (e) => e.run(), { nanEqual: process.env.NAN_EXACT !== '1' });
