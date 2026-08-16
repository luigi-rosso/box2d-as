// Ported from box2d include/box2d/id.h. The index-plus-generation handle
// scheme is load-bearing for every pool in later phases; the packed
// store/load layouts match the C bit layout exactly.

export class b2WorldId {
    index1: u16 = 0;
    generation: u16 = 0;

    constructor(index1: u16 = 0, generation: u16 = 0) {
        this.index1 = index1;
        this.generation = generation;
    }
}

export class b2BodyId {
    index1: i32 = 0;
    world0: u16 = 0;
    generation: u16 = 0;

    constructor(index1: i32 = 0, world0: u16 = 0, generation: u16 = 0) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

export class b2ShapeId {
    index1: i32 = 0;
    world0: u16 = 0;
    generation: u16 = 0;

    constructor(index1: i32 = 0, world0: u16 = 0, generation: u16 = 0) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

export class b2ChainId {
    index1: i32 = 0;
    world0: u16 = 0;
    generation: u16 = 0;

    constructor(index1: i32 = 0, world0: u16 = 0, generation: u16 = 0) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

export class b2JointId {
    index1: i32 = 0;
    world0: u16 = 0;
    generation: u16 = 0;

    constructor(index1: i32 = 0, world0: u16 = 0, generation: u16 = 0) {
        this.index1 = index1;
        this.world0 = world0;
        this.generation = generation;
    }
}

export class b2ContactId {
    index1: i32 = 0;
    world0: u16 = 0;
    padding: i16 = 0;
    generation: u32 = 0;

    constructor(index1: i32 = 0, world0: u16 = 0, generation: u32 = 0) {
        this.index1 = index1;
        this.world0 = world0;
        this.padding = 0;
        this.generation = generation;
    }
}

export function b2StoreWorldId(id: b2WorldId): u32 {
    return (<u32>id.index1 << 16) | <u32>id.generation;
}

export function b2LoadWorldId(x: u32): b2WorldId {
    return new b2WorldId(<u16>(x >> 16), <u16>x);
}

export function b2StoreBodyId(id: b2BodyId): u64 {
    return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

export function b2LoadBodyId(x: u64): b2BodyId {
    return new b2BodyId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
}

export function b2StoreShapeId(id: b2ShapeId): u64 {
    return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

export function b2LoadShapeId(x: u64): b2ShapeId {
    return new b2ShapeId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
}

export function b2StoreChainId(id: b2ChainId): u64 {
    return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

export function b2LoadChainId(x: u64): b2ChainId {
    return new b2ChainId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
}

export function b2StoreJointId(id: b2JointId): u64 {
    return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

export function b2LoadJointId(x: u64): b2JointId {
    return new b2JointId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
}

export function b2StoreContactId(id: b2ContactId, values: Uint32Array): void {
    values[0] = <u32>id.index1;
    values[1] = <u32>id.world0;
    values[2] = id.generation;
}

export function b2LoadContactId(values: Uint32Array): b2ContactId {
    return new b2ContactId(<i32>values[0], <u16>values[1], values[2]);
}
