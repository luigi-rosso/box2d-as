// Ported from box2d src/array.h/.c. The C macros generate one typed array per
// element type; here b2IntArray covers the int instantiation and b2Array<T>
// covers struct element types. Growth semantics are exact:
// reserve only grows, push/add expand to capacity < 2 ? 2 : capacity * 1.5.
// C's Array_Add returns a pointer to the raw new slot; b2Array<T>.add returns
// a default-constructed element placed in the new slot.

export class b2IntArray {
    data: Int32Array;
    count: i32;
    capacity: i32;

    constructor() {
        this.data = new Int32Array(0);
        this.count = 0;
        this.capacity = 0;
    }
}

export function b2IntArray_Create(capacity: i32): b2IntArray {
    let a = new b2IntArray();
    if (capacity > 0) {
        a.data = new Int32Array(capacity);
        a.capacity = capacity;
    }
    return a;
}

export function b2IntArray_Reserve(a: b2IntArray, newCapacity: i32): void {
    if (newCapacity <= a.capacity) {
        return;
    }
    let newData = new Int32Array(newCapacity);
    newData.set(a.data);
    a.data = newData;
    a.capacity = newCapacity;
}

export function b2IntArray_Destroy(a: b2IntArray): void {
    a.data = new Int32Array(0);
    a.count = 0;
    a.capacity = 0;
}

export function b2IntArray_Resize(a: b2IntArray, count: i32): void {
    b2IntArray_Reserve(a, count);
    a.count = count;
}

export function b2IntArray_Get(a: b2IntArray, index: i32): i32 {
    assert(0 <= index && index < a.count);
    return unchecked(a.data[index]);
}

// C's Add returns a slot pointer; here the new slot's index.
export function b2IntArray_Add(a: b2IntArray): i32 {
    if (a.count == a.capacity) {
        let newCapacity = a.capacity < 2 ? 2 : a.capacity + (a.capacity >> 1);
        b2IntArray_Reserve(a, newCapacity);
    }
    a.count += 1;
    return a.count - 1;
}

export function b2IntArray_Push(a: b2IntArray, value: i32): void {
    if (a.count == a.capacity) {
        let newCapacity = a.capacity < 2 ? 2 : a.capacity + (a.capacity >> 1);
        b2IntArray_Reserve(a, newCapacity);
    }
    unchecked(a.data[a.count] = value);
    a.count += 1;
}

export function b2IntArray_Set(a: b2IntArray, index: i32, value: i32): void {
    assert(0 <= index && index < a.count);
    unchecked(a.data[index] = value);
}

export function b2IntArray_RemoveSwap(a: b2IntArray, index: i32): i32 {
    assert(0 <= index && index < a.count);
    let movedIndex = -1;
    if (index != a.count - 1) {
        movedIndex = a.count - 1;
        unchecked(a.data[index] = a.data[movedIndex]);
    }
    a.count -= 1;
    return movedIndex;
}

export function b2IntArray_Pop(a: b2IntArray): i32 {
    assert(a.count > 0);
    let value = unchecked(a.data[a.count - 1]);
    a.count -= 1;
    return value;
}

export function b2IntArray_Clear(a: b2IntArray): void {
    a.count = 0;
}

export function b2IntArray_ByteCount(a: b2IntArray): i32 {
    return a.capacity * 4;
}

// Generic instantiation for struct element types. T must have a parameterless
// constructor so add() can fill the raw slot C leaves uninitialized.
export class b2Array<T> {
    data: Array<T | null>;
    count: i32;
    capacity: i32;

    constructor(capacity: i32 = 0) {
        this.data = new Array<T | null>(capacity > 0 ? capacity : 0);
        this.count = 0;
        this.capacity = capacity > 0 ? capacity : 0;
    }

    reserve(newCapacity: i32): void {
        if (newCapacity <= this.capacity) {
            return;
        }
        let newData = new Array<T | null>(newCapacity);
        for (let i = 0; i < this.count; i++) {
            unchecked(newData[i] = this.data[i]);
        }
        this.data = newData;
        this.capacity = newCapacity;
    }

    resize(count: i32): void {
        this.reserve(count);
        this.count = count;
    }

    get(index: i32): T {
        assert(0 <= index && index < this.count);
        return unchecked(this.data[index])!;
    }

    add(): T {
        if (this.count == this.capacity) {
            let newCapacity = this.capacity < 2 ? 2 : this.capacity + (this.capacity >> 1);
            this.reserve(newCapacity);
        }
        let value = instantiate<T>();
        unchecked(this.data[this.count] = value);
        this.count += 1;
        return value;
    }

    push(value: T): void {
        if (this.count == this.capacity) {
            let newCapacity = this.capacity < 2 ? 2 : this.capacity + (this.capacity >> 1);
            this.reserve(newCapacity);
        }
        unchecked(this.data[this.count] = value);
        this.count += 1;
    }

    set(index: i32, value: T): void {
        assert(0 <= index && index < this.count);
        unchecked(this.data[index] = value);
    }

    removeSwap(index: i32): i32 {
        assert(0 <= index && index < this.count);
        let movedIndex = -1;
        if (index != this.count - 1) {
            movedIndex = this.count - 1;
            unchecked(this.data[index] = this.data[movedIndex]);
        }
        this.count -= 1;
        return movedIndex;
    }

    pop(): T {
        assert(this.count > 0);
        let value = unchecked(this.data[this.count - 1])!;
        this.count -= 1;
        return value;
    }

    clear(): void {
        this.count = 0;
    }
}
