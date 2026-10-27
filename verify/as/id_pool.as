// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/id_pool.as, mirrors verify/cref/id_pool.c

import { vtag, vi32, vend, vseed, vrand32, vrandint } from "./common";
import {
  b2IdPool,
  b2CreateIdPool,
  b2DestroyIdPool,
  b2AllocId,
  b2FreeId,
  b2GetIdCount,
  b2GetIdCapacity,
  b2GetIdBytes,
} from "../../lib/id_pool";

export { frameBoundary } from "./common";

function emitPool(pool: b2IdPool): void {
  vtag("s");
  vi32(b2GetIdCount(pool));
  vi32(b2GetIdCapacity(pool));
  vi32(b2GetIdBytes(pool));
  vi32(pool.freeArray.length);
  vend();

  vtag("free");
  for (let i = 0; i < pool.freeArray.length; ++i) {
    vi32(pool.freeArray[i]);
  }
  vend();
}

export function run(): void {
  vseed(2);
  let live = new Array<i32>();
  let pool = b2CreateIdPool();
  emitPool(pool);

  for (let phase = 0; phase < 60; ++phase) {
    let ops = vrandint(1, 300);
    for (let k = 0; k < ops; ++k) {
      let r = vrand32() % 5;
      if (r < 3 || live.length == 0) {
        let id = b2AllocId(pool);
        live.push(id);
        vtag("a");
        vi32(id);
        vend();
      } else {
        let index = vrandint(0, live.length);
        let id = live[index];
        live[index] = live[live.length - 1];
        live.pop();
        b2FreeId(pool, id);
        vtag("f");
        vi32(id);
        vend();
      }
    }

    emitPool(pool);
  }

  b2DestroyIdPool(pool);
  emitPool(pool);

  for (let k = 0; k < 40; ++k) {
    let id = b2AllocId(pool);
    vtag("a");
    vi32(id);
    vend();
    if (k % 3 == 0) {
      b2FreeId(pool, id / 2);
    }
  }
  emitPool(pool);
}
