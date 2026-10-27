// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/bitset.as, mirrors verify/cref/bitset.c

import { vtag, vi32, vu32, vu64, vbool, vend, vseed, vrand32, vrandint } from "./common";
import {
  b2BitSet,
  b2CreateBitSet,
  b2DestroyBitSet,
  b2SetBitCountAndClear,
  b2InPlaceUnion,
  b2GrowBitSet,
  b2CountSetBits,
  b2SetBit,
  b2SetBitGrow,
  b2ClearBit,
  b2GetBit,
  b2GetBitSetBytes,
} from "../../lib/bitset";

export { frameBoundary } from "./common";

function emitSet(tag: string, set: b2BitSet): void {
  vtag(tag);
  vu32(set.blockCapacity);
  vu32(set.blockCount);
  vi32(b2GetBitSetBytes(set));
  vi32(b2CountSetBits(set));
  for (let i: u32 = 0; i < set.blockCapacity; ++i) {
    vu64(set.bits[<i32>i]);
  }
  vend();
}

export function run(): void {
  vseed(3);

  for (let round = 0; round < 40; ++round) {
    let capacity = vrand32() % 700;
    let a = b2CreateBitSet(capacity);
    let b = b2CreateBitSet(vrand32() % 300);
    emitSet("create", a);

    let bitCount = vrand32() % 900;
    b2SetBitCountAndClear(a, bitCount);
    b2SetBitCountAndClear(b, bitCount);
    emitSet("count", a);

    for (let phase = 0; phase < 8; ++phase) {
      let ops = vrandint(1, 120);
      for (let k = 0; k < ops; ++k) {
        let op = vrand32() % 6;
        let limit: u32 = a.blockCount * 64 + 200;
        let bit = vrand32() % limit;
        if (op == 0 && a.blockCount > 0) {
          let inside = vrand32() % (a.blockCount * 64);
          b2SetBit(a, inside);
        } else if (op == 1) {
          b2SetBitGrow(a, bit);
        } else if (op == 2) {
          b2ClearBit(a, bit);
        } else if (op == 3) {
          vtag("get");
          vu32(bit);
          vbool(b2GetBit(a, bit));
          vend();
        } else if (op == 4) {
          b2SetBitGrow(b, vrand32() % (a.blockCount * 64 + 64));
        } else {
          b2ClearBit(b, bit);
        }
      }
      emitSet("a", a);
      emitSet("b", b);

      if (phase == 4) {
        b2GrowBitSet(a, a.blockCount + 1 + vrand32() % 5);
        emitSet("grow", a);
      }

      if (b.blockCount < a.blockCount) {
        b2GrowBitSet(b, a.blockCount);
      }
      if (a.blockCount < b.blockCount) {
        b2GrowBitSet(a, b.blockCount);
      }
      b2InPlaceUnion(a, b);
      emitSet("union", a);
    }

    b2SetBitCountAndClear(a, vrand32() % 2000);
    emitSet("recount", a);

    b2DestroyBitSet(a);
    b2DestroyBitSet(b);
    emitSet("destroy", a);
  }
}
