// Ported from box2d src/constraint_graph.h/.c. C's memcpy of sims into graph
// slots becomes a reference push; every caller destroys the source right after.

import { B2_NULL_INDEX } from "./core";
import { B2_GRAPH_COLOR_COUNT } from "./constants";
import { b2MaxInt } from "./math_functions";
import { b2Array } from "./array";
import { b2BitSet, b2CreateBitSet, b2SetBitCountAndClear, b2GetBit, b2SetBitGrow, b2ClearBit } from "./bitset";
import { b2World, b2_awakeSet } from "./physics_world";
import { b2Body } from "./body";
import { b2Contact, b2ContactSim, b2_simTouchingFlag, b2_contactTouchingFlag } from "./contact";
import { b2Joint, b2JointSim } from "./joint";
import { b2ContactConstraintSIMD, b2ContactConstraint } from "./contact_solver";
import { b2_dynamicBody, b2_staticBody } from "./broad_phase";

// This holds constraints that cannot fit the graph color limit.
export const B2_OVERFLOW_INDEX: i32 = B2_GRAPH_COLOR_COUNT - 1;

// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
export const B2_DYNAMIC_COLOR_COUNT: i32 = B2_GRAPH_COLOR_COUNT - 4;

export class b2GraphColor {
    // This bitset is indexed by bodyId so this is over-sized to encompass static bodies.
    // Unused on the overflow color.
    bodySet: b2BitSet = new b2BitSet();
    // cache friendly arrays
    contactSims: b2Array<b2ContactSim> = new b2Array<b2ContactSim>();
    jointSims: b2Array<b2JointSim> = new b2Array<b2JointSim>();
    // transient (C union). simdConstraints holds reference copies into the
    // step's shared constraint array (C slices the allocation with pointer
    // arithmetic; the slots alias the same objects here).
    simdConstraints: StaticArray<b2ContactConstraintSIMD> | null = null;
    overflowConstraints: StaticArray<b2ContactConstraint> | null = null;
}

function makeColors(): StaticArray<b2GraphColor> {
    let colors = new StaticArray<b2GraphColor>(B2_GRAPH_COLOR_COUNT);
    for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
        colors[i] = new b2GraphColor();
    }
    return colors;
}

export class b2ConstraintGraph {
    // including overflow at the end
    colors: StaticArray<b2GraphColor> = makeColors();
}

export function b2CreateGraph(graph: b2ConstraintGraph, bodyCapacity: i32): void {
    assert(B2_GRAPH_COLOR_COUNT >= 2);
    assert(B2_OVERFLOW_INDEX == B2_GRAPH_COLOR_COUNT - 1);
    assert(B2_DYNAMIC_COLOR_COUNT >= 2);

    graph.colors = makeColors();

    bodyCapacity = b2MaxInt(bodyCapacity, 8);

    // Initialize graph color bit set.
    // No bitset for overflow color.
    for (let i = 0; i < B2_OVERFLOW_INDEX; ++i) {
        let color = graph.colors[i];
        color.bodySet = b2CreateBitSet(bodyCapacity);
        b2SetBitCountAndClear(color.bodySet, bodyCapacity);
    }
}

export function b2DestroyGraph(graph: b2ConstraintGraph): void {
    graph.colors = makeColors();
}

// Contacts are always created as non-touching. They get moved into the constraint
// graph once they are found to be touching.
export function b2AddContactToGraph(world: b2World, contactSim: b2ContactSim, contact: b2Contact): void {
    assert(contactSim.manifold.pointCount > 0);
    assert((contactSim.simFlags & b2_simTouchingFlag) != 0);
    assert((contact.flags & b2_contactTouchingFlag) != 0);

    let graph = world.constraintGraph;
    let colorIndex = B2_OVERFLOW_INDEX;

    let bodyIdA = contact.edges[0].bodyId;
    let bodyIdB = contact.edges[1].bodyId;
    let bodyA = world.bodies.get(bodyIdA);
    let bodyB = world.bodies.get(bodyIdB);

    let typeA = bodyA.type;
    let typeB = bodyB.type;
    assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

    if (typeA == b2_dynamicBody && typeB == b2_dynamicBody) {
        // Dynamic constraint colors cannot encroach on colors reserved for static constraints
        for (let i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i) {
            let color = graph.colors[i];
            if (b2GetBit(color.bodySet, bodyIdA) || b2GetBit(color.bodySet, bodyIdB)) {
                continue;
            }

            b2SetBitGrow(color.bodySet, bodyIdA);
            b2SetBitGrow(color.bodySet, bodyIdB);
            colorIndex = i;
            break;
        }
    } else if (typeA == b2_dynamicBody) {
        // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
        for (let i = B2_OVERFLOW_INDEX - 1; i >= 1; --i) {
            let color = graph.colors[i];
            if (b2GetBit(color.bodySet, bodyIdA)) {
                continue;
            }

            b2SetBitGrow(color.bodySet, bodyIdA);
            colorIndex = i;
            break;
        }
    } else if (typeB == b2_dynamicBody) {
        // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
        for (let i = B2_OVERFLOW_INDEX - 1; i >= 1; --i) {
            let color = graph.colors[i];
            if (b2GetBit(color.bodySet, bodyIdB)) {
                continue;
            }

            b2SetBitGrow(color.bodySet, bodyIdB);
            colorIndex = i;
            break;
        }
    }

    let color = graph.colors[colorIndex];
    contact.colorIndex = colorIndex;
    contact.localIndex = color.contactSims.count;

    let newContact = contactSim;
    color.contactSims.push(newContact);

    // todo perhaps skip this if the contact is already awake

    if (typeA == b2_staticBody) {
        newContact.bodySimIndexA = B2_NULL_INDEX;
        newContact.invMassA = 0.0;
        newContact.invIA = 0.0;
    } else {
        assert(bodyA.setIndex == b2_awakeSet);
        let awakeSet = world.solverSets.get(b2_awakeSet);

        let localIndex = bodyA.localIndex;
        newContact.bodySimIndexA = localIndex;

        let bodySimA = awakeSet.bodySims.get(localIndex);
        newContact.invMassA = bodySimA.invMass;
        newContact.invIA = bodySimA.invInertia;
    }

    if (typeB == b2_staticBody) {
        newContact.bodySimIndexB = B2_NULL_INDEX;
        newContact.invMassB = 0.0;
        newContact.invIB = 0.0;
    } else {
        assert(bodyB.setIndex == b2_awakeSet);
        let awakeSet = world.solverSets.get(b2_awakeSet);

        let localIndex = bodyB.localIndex;
        newContact.bodySimIndexB = localIndex;

        let bodySimB = awakeSet.bodySims.get(localIndex);
        newContact.invMassB = bodySimB.invMass;
        newContact.invIB = bodySimB.invInertia;
    }
}

export function b2RemoveContactFromGraph(world: b2World, bodyIdA: i32, bodyIdB: i32, colorIndex: i32, localIndex: i32): void {
    let graph = world.constraintGraph;

    assert(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
    let color = graph.colors[colorIndex];

    if (colorIndex != B2_OVERFLOW_INDEX) {
        // This might clear a bit for a kinematic or static body, but this has no effect
        b2ClearBit(color.bodySet, bodyIdA);
        b2ClearBit(color.bodySet, bodyIdB);
    }

    let movedIndex = color.contactSims.removeSwap(localIndex);
    if (movedIndex != B2_NULL_INDEX) {
        // Fix index on swapped contact
        let movedContactSim = color.contactSims.get(localIndex);

        // Fix moved contact
        let movedId = movedContactSim.contactId;
        let movedContact = world.contacts.get(movedId);
        assert(movedContact.setIndex == b2_awakeSet);
        assert(movedContact.colorIndex == colorIndex);
        assert(movedContact.localIndex == movedIndex);
        movedContact.localIndex = localIndex;
    }
}

function b2AssignJointColor(graph: b2ConstraintGraph, bodyIdA: i32, bodyIdB: i32, typeA: i32, typeB: i32): i32 {
    assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

    if (typeA == b2_dynamicBody && typeB == b2_dynamicBody) {
        // Dynamic constraint colors cannot encroach on colors reserved for static constraints
        for (let i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i) {
            let color = graph.colors[i];
            if (b2GetBit(color.bodySet, bodyIdA) || b2GetBit(color.bodySet, bodyIdB)) {
                continue;
            }

            b2SetBitGrow(color.bodySet, bodyIdA);
            b2SetBitGrow(color.bodySet, bodyIdB);
            return i;
        }
    } else if (typeA == b2_dynamicBody) {
        // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
        for (let i = B2_OVERFLOW_INDEX - 1; i >= 1; --i) {
            let color = graph.colors[i];
            if (b2GetBit(color.bodySet, bodyIdA)) {
                continue;
            }

            b2SetBitGrow(color.bodySet, bodyIdA);
            return i;
        }
    } else if (typeB == b2_dynamicBody) {
        // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
        for (let i = B2_OVERFLOW_INDEX - 1; i >= 1; --i) {
            let color = graph.colors[i];
            if (b2GetBit(color.bodySet, bodyIdB)) {
                continue;
            }

            b2SetBitGrow(color.bodySet, bodyIdB);
            return i;
        }
    }

    return B2_OVERFLOW_INDEX;
}

export function b2CreateJointInGraph(world: b2World, joint: b2Joint): b2JointSim {
    let graph = world.constraintGraph;

    let bodyIdA = joint.edges[0].bodyId;
    let bodyIdB = joint.edges[1].bodyId;
    let bodyA = world.bodies.get(bodyIdA);
    let bodyB = world.bodies.get(bodyIdB);

    let colorIndex = b2AssignJointColor(graph, bodyIdA, bodyIdB, bodyA.type, bodyB.type);

    let jointSim = graph.colors[colorIndex].jointSims.add();

    joint.colorIndex = colorIndex;
    joint.localIndex = graph.colors[colorIndex].jointSims.count - 1;
    return jointSim;
}

export function b2AddJointToGraph(world: b2World, jointSim: b2JointSim, joint: b2Joint): void {
    // C copies jointSim into a fresh graph slot; move the reference into the
    // slot the color assignment just created.
    b2CreateJointInGraph(world, joint);
    let color = world.constraintGraph.colors[joint.colorIndex];
    color.jointSims.set(joint.localIndex, jointSim);
}

export function b2RemoveJointFromGraph(world: b2World, bodyIdA: i32, bodyIdB: i32, colorIndex: i32, localIndex: i32): void {
    let graph = world.constraintGraph;

    assert(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
    let color = graph.colors[colorIndex];

    if (colorIndex != B2_OVERFLOW_INDEX) {
        // May clear static bodies, no effect
        b2ClearBit(color.bodySet, bodyIdA);
        b2ClearBit(color.bodySet, bodyIdB);
    }

    let movedIndex = color.jointSims.removeSwap(localIndex);
    if (movedIndex != B2_NULL_INDEX) {
        // Fix moved joint
        let movedJointSim = color.jointSims.get(localIndex);
        let movedId = movedJointSim.jointId;
        let movedJoint = world.joints.get(movedId);
        assert(movedJoint.setIndex == b2_awakeSet);
        assert(movedJoint.colorIndex == colorIndex);
        assert(movedJoint.localIndex == movedIndex);
        movedJoint.localIndex = localIndex;
    }
}
