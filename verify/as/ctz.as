// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/ctz.as, mirrors verify/cref/ctz.c

import { vtag, vi32, vu32, vu64, vbool, vend, vseed, vrand32 } from "./common";
import {
  b2CTZ32,
  b2CLZ32,
  b2CTZ64,
  b2PopCount64,
  b2IsPowerOf2,
  b2BoundingPowerOf2,
  b2RoundUpPowerOf2,
} from "../../lib/ctz";

export { frameBoundary } from "./common";

function pickU32(): u32 {
  let kind = vrand32() % 4;
  let shift = vrand32() % 32;
  if (kind == 0) {
    return (<u32>1) << shift;
  }
  if (kind == 1) {
    return ((<u32>1) << shift) + 1;
  }
  if (kind == 2) {
    return ((<u32>1) << shift) - 1;
  }
  return vrand32() >> (vrand32() % 32);
}

export function run(): void {
  vseed(1);
  for (let i = 0; i < 20000; ++i) {
    let a = pickU32();
    let b = (<u64>pickU32() << <u64>(vrand32() % 33)) | <u64>pickU32();
    vtag("u");
    vu32(a);
    vu64(b);
    if (a != 0) {
      vu32(b2CTZ32(a));
      vu32(b2CLZ32(a));
    }
    if (b != 0) {
      vu32(b2CTZ64(b));
    }
    vi32(b2PopCount64(b));
    vend();

    let x = <i32>(a & 0x3fffffff);
    vtag("p");
    vi32(x);
    vbool(b2IsPowerOf2(x));
    vi32(b2BoundingPowerOf2(x));
    vi32(b2RoundUpPowerOf2(x));
    vend();
  }

  // Negative and boundary inputs
  let edges: StaticArray<i32> = [
    -2147483648,
    -5,
    -1,
    0,
    1,
    2,
    3,
    0x40000000,
    0x7fffffff
  ];
  for (let i = 0; i < edges.length; ++i) {
    let x = edges[i];
    vtag("e");
    vi32(x);
    vbool(b2IsPowerOf2(x));
    vi32(b2BoundingPowerOf2(x));
    if (x <= 0x40000000) {
      vi32(b2RoundUpPowerOf2(x));
    }
    vend();
  }
}
