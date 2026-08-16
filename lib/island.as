// Ported from box2d src/island.h/.c. Validation is compiled out under NDEBUG
// like the reference build; b2ValidateIsland stays a no-op.

import { B2_NULL_INDEX } from "./core";
import { b2World, b2ValidateIsland, b2_awakeSet, b2_disabledSet, b2_staticSet, b2_firstSleepingSet } from "./physics_world";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2WakeSolverSet } from "./solver_set";
import { b2Contact, b2_contactTouchingFlag } from "./contact";
import { b2Joint } from "./joint";
import { b2Body } from "./body";
import { b2_dynamicBody } from "./broad_phase";

export class b2Island {
    setIndex: i32 = 0;
    localIndex: i32 = 0;
    islandId: i32 = 0;
    headBody: i32 = 0;
    tailBody: i32 = 0;
    bodyCount: i32 = 0;
    headContact: i32 = 0;
    tailContact: i32 = 0;
    contactCount: i32 = 0;
    headJoint: i32 = 0;
    tailJoint: i32 = 0;
    jointCount: i32 = 0;
    constraintRemoveCount: i32 = 0;
}

export class b2IslandSim {
    islandId: i32 = 0;
}

export function b2CreateIsland(world: b2World, setIndex: i32): b2Island {
    assert(setIndex == b2_awakeSet || setIndex >= b2_firstSleepingSet);

    let islandId = b2AllocId(world.islandIdPool);

    if (islandId == world.islands.count) {
        world.islands.add();
    } else {
        assert(world.islands.get(islandId).setIndex == B2_NULL_INDEX);
    }

    let set = world.solverSets.get(setIndex);

    let island = world.islands.get(islandId);
    island.setIndex = setIndex;
    island.localIndex = set.islandSims.count;
    island.islandId = islandId;
    island.headBody = B2_NULL_INDEX;
    island.tailBody = B2_NULL_INDEX;
    island.bodyCount = 0;
    island.headContact = B2_NULL_INDEX;
    island.tailContact = B2_NULL_INDEX;
    island.contactCount = 0;
    island.headJoint = B2_NULL_INDEX;
    island.tailJoint = B2_NULL_INDEX;
    island.jointCount = 0;
    island.constraintRemoveCount = 0;

    let islandSim = set.islandSims.add();
    islandSim.islandId = islandId;

    return island;
}

export function b2DestroyIsland(world: b2World, islandId: i32): void {
    if (world.splitIslandId == islandId) {
        world.splitIslandId = B2_NULL_INDEX;
    }

    // assume island is empty
    let island = world.islands.get(islandId);
    let set = world.solverSets.get(island.setIndex);
    let movedIndex = set.islandSims.removeSwap(island.localIndex);
    if (movedIndex != B2_NULL_INDEX) {
        // Fix index on moved element
        let movedElement = set.islandSims.get(island.localIndex);
        let movedId = movedElement.islandId;
        let movedIsland = world.islands.get(movedId);
        assert(movedIsland.localIndex == movedIndex);
        movedIsland.localIndex = island.localIndex;
    }

    // Free island and id (preserve island revision)
    island.islandId = B2_NULL_INDEX;
    island.setIndex = B2_NULL_INDEX;
    island.localIndex = B2_NULL_INDEX;
    b2FreeId(world.islandIdPool, islandId);
}

function b2MergeIslands(world: b2World, islandIdA: i32, islandIdB: i32): i32 {
    if (islandIdA == islandIdB) {
        return islandIdA;
    }

    if (islandIdA == B2_NULL_INDEX) {
        assert(islandIdB != B2_NULL_INDEX);
        return islandIdB;
    }

    if (islandIdB == B2_NULL_INDEX) {
        assert(islandIdA != B2_NULL_INDEX);
        return islandIdA;
    }

    let islandA = world.islands.get(islandIdA);
    let islandB = world.islands.get(islandIdB);

    // Keep the biggest island to reduce cache misses
    let big: b2Island;
    let small: b2Island;
    if (islandA.bodyCount >= islandB.bodyCount) {
        big = islandA;
        small = islandB;
    } else {
        big = islandB;
        small = islandA;
    }

    let bigId = big.islandId;

    // remap island indices
    let bodyId = small.headBody;
    while (bodyId != B2_NULL_INDEX) {
        let body = world.bodies.get(bodyId);
        body.islandId = bigId;
        bodyId = body.islandNext;
    }

    let contactId = small.headContact;
    while (contactId != B2_NULL_INDEX) {
        let contact = world.contacts.get(contactId);
        contact.islandId = bigId;
        contactId = contact.islandNext;
    }

    let jointId = small.headJoint;
    while (jointId != B2_NULL_INDEX) {
        let joint = world.joints.get(jointId);
        joint.islandId = bigId;
        jointId = joint.islandNext;
    }

    // connect body lists
    assert(big.tailBody != B2_NULL_INDEX);
    let tailBody = world.bodies.get(big.tailBody);
    assert(tailBody.islandNext == B2_NULL_INDEX);
    tailBody.islandNext = small.headBody;

    assert(small.headBody != B2_NULL_INDEX);
    let headBody = world.bodies.get(small.headBody);
    assert(headBody.islandPrev == B2_NULL_INDEX);
    headBody.islandPrev = big.tailBody;

    big.tailBody = small.tailBody;
    big.bodyCount += small.bodyCount;

    // connect contact lists
    if (big.headContact == B2_NULL_INDEX) {
        // Big island has no contacts
        assert(big.tailContact == B2_NULL_INDEX && big.contactCount == 0);
        big.headContact = small.headContact;
        big.tailContact = small.tailContact;
        big.contactCount = small.contactCount;
    } else if (small.headContact != B2_NULL_INDEX) {
        // Both islands have contacts
        assert(small.tailContact != B2_NULL_INDEX && small.contactCount > 0);
        assert(big.tailContact != B2_NULL_INDEX && big.contactCount > 0);

        let tailContact = world.contacts.get(big.tailContact);
        assert(tailContact.islandNext == B2_NULL_INDEX);
        tailContact.islandNext = small.headContact;

        let headContact = world.contacts.get(small.headContact);
        assert(headContact.islandPrev == B2_NULL_INDEX);
        headContact.islandPrev = big.tailContact;

        big.tailContact = small.tailContact;
        big.contactCount += small.contactCount;
    }

    if (big.headJoint == B2_NULL_INDEX) {
        // Root island has no joints
        assert(big.tailJoint == B2_NULL_INDEX && big.jointCount == 0);
        big.headJoint = small.headJoint;
        big.tailJoint = small.tailJoint;
        big.jointCount = small.jointCount;
    } else if (small.headJoint != B2_NULL_INDEX) {
        // Both islands have joints
        assert(small.tailJoint != B2_NULL_INDEX && small.jointCount > 0);
        assert(big.tailJoint != B2_NULL_INDEX && big.jointCount > 0);

        let tailJoint = world.joints.get(big.tailJoint);
        assert(tailJoint.islandNext == B2_NULL_INDEX);
        tailJoint.islandNext = small.headJoint;

        let headJoint = world.joints.get(small.headJoint);
        assert(headJoint.islandPrev == B2_NULL_INDEX);
        headJoint.islandPrev = big.tailJoint;

        big.tailJoint = small.tailJoint;
        big.jointCount += small.jointCount;
    }

    // Track removed constraints
    big.constraintRemoveCount += small.constraintRemoveCount;

    small.bodyCount = 0;
    small.contactCount = 0;
    small.jointCount = 0;
    small.headBody = B2_NULL_INDEX;
    small.headContact = B2_NULL_INDEX;
    small.headJoint = B2_NULL_INDEX;
    small.tailBody = B2_NULL_INDEX;
    small.tailContact = B2_NULL_INDEX;
    small.tailJoint = B2_NULL_INDEX;
    small.constraintRemoveCount = 0;

    b2DestroyIsland(world, small.islandId);

    b2ValidateIsland(world, bigId);

    return bigId;
}

function b2AddContactToIsland(world: b2World, islandId: i32, contact: b2Contact): void {
    assert(contact.islandId == B2_NULL_INDEX);
    assert(contact.islandPrev == B2_NULL_INDEX);
    assert(contact.islandNext == B2_NULL_INDEX);

    let island = world.islands.get(islandId);

    if (island.headContact != B2_NULL_INDEX) {
        contact.islandNext = island.headContact;
        let headContact = world.contacts.get(island.headContact);
        headContact.islandPrev = contact.contactId;
    }

    island.headContact = contact.contactId;
    if (island.tailContact == B2_NULL_INDEX) {
        island.tailContact = island.headContact;
    }

    island.contactCount += 1;
    contact.islandId = islandId;

    b2ValidateIsland(world, islandId);
}

// Link a contact into an island.
export function b2LinkContact(world: b2World, contact: b2Contact): void {
    assert((contact.flags & b2_contactTouchingFlag) != 0);

    let bodyIdA = contact.edges[0].bodyId;
    let bodyIdB = contact.edges[1].bodyId;

    let bodyA = world.bodies.get(bodyIdA);
    let bodyB = world.bodies.get(bodyIdB);

    assert(bodyA.setIndex != b2_disabledSet && bodyB.setIndex != b2_disabledSet);
    assert(bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet);

    // Wake bodyB if bodyA is awake and bodyB is sleeping
    if (bodyA.setIndex == b2_awakeSet && bodyB.setIndex >= b2_firstSleepingSet) {
        b2WakeSolverSet(world, bodyB.setIndex);
    }

    // Wake bodyA if bodyB is awake and bodyA is sleeping
    if (bodyB.setIndex == b2_awakeSet && bodyA.setIndex >= b2_firstSleepingSet) {
        b2WakeSolverSet(world, bodyA.setIndex);
    }

    let islandIdA = bodyA.islandId;
    let islandIdB = bodyB.islandId;

    // Static bodies have null island indices.
    assert(bodyA.setIndex != b2_staticSet || islandIdA == B2_NULL_INDEX);
    assert(bodyB.setIndex != b2_staticSet || islandIdB == B2_NULL_INDEX);
    assert(islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX);

    // Merge islands. This will destroy one of the islands.
    let finalIslandId = b2MergeIslands(world, islandIdA, islandIdB);

    // Add contact to the island that survived
    b2AddContactToIsland(world, finalIslandId, contact);
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
export function b2UnlinkContact(world: b2World, contact: b2Contact): void {
    assert(contact.islandId != B2_NULL_INDEX);

    // remove from island
    let islandId = contact.islandId;
    let island = world.islands.get(islandId);

    if (contact.islandPrev != B2_NULL_INDEX) {
        let prevContact = world.contacts.get(contact.islandPrev);
        assert(prevContact.islandNext == contact.contactId);
        prevContact.islandNext = contact.islandNext;
    }

    if (contact.islandNext != B2_NULL_INDEX) {
        let nextContact = world.contacts.get(contact.islandNext);
        assert(nextContact.islandPrev == contact.contactId);
        nextContact.islandPrev = contact.islandPrev;
    }

    if (island.headContact == contact.contactId) {
        island.headContact = contact.islandNext;
    }

    if (island.tailContact == contact.contactId) {
        island.tailContact = contact.islandPrev;
    }

    assert(island.contactCount > 0);
    island.contactCount -= 1;
    island.constraintRemoveCount += 1;

    contact.islandId = B2_NULL_INDEX;
    contact.islandPrev = B2_NULL_INDEX;
    contact.islandNext = B2_NULL_INDEX;

    b2ValidateIsland(world, islandId);
}

function b2AddJointToIsland(world: b2World, islandId: i32, joint: b2Joint): void {
    assert(joint.islandId == B2_NULL_INDEX);
    assert(joint.islandPrev == B2_NULL_INDEX);
    assert(joint.islandNext == B2_NULL_INDEX);

    let island = world.islands.get(islandId);

    if (island.headJoint != B2_NULL_INDEX) {
        joint.islandNext = island.headJoint;
        let headJoint = world.joints.get(island.headJoint);
        headJoint.islandPrev = joint.jointId;
    }

    island.headJoint = joint.jointId;
    if (island.tailJoint == B2_NULL_INDEX) {
        island.tailJoint = island.headJoint;
    }

    island.jointCount += 1;
    joint.islandId = islandId;

    b2ValidateIsland(world, islandId);
}

export function b2LinkJoint(world: b2World, joint: b2Joint): void {
    let bodyA = world.bodies.get(joint.edges[0].bodyId);
    let bodyB = world.bodies.get(joint.edges[1].bodyId);

    assert(bodyA.type == b2_dynamicBody || bodyB.type == b2_dynamicBody);

    if (bodyA.setIndex == b2_awakeSet && bodyB.setIndex >= b2_firstSleepingSet) {
        b2WakeSolverSet(world, bodyB.setIndex);
    } else if (bodyB.setIndex == b2_awakeSet && bodyA.setIndex >= b2_firstSleepingSet) {
        b2WakeSolverSet(world, bodyA.setIndex);
    }

    let islandIdA = bodyA.islandId;
    let islandIdB = bodyB.islandId;

    assert(islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX);

    // Merge islands. This will destroy one of the islands.
    let finalIslandId = b2MergeIslands(world, islandIdA, islandIdB);

    // Add joint the island that survived
    b2AddJointToIsland(world, finalIslandId, joint);
}

export function b2UnlinkJoint(world: b2World, joint: b2Joint): void {
    if (joint.islandId == B2_NULL_INDEX) {
        return;
    }

    // remove from island
    let islandId = joint.islandId;
    let island = world.islands.get(islandId);

    if (joint.islandPrev != B2_NULL_INDEX) {
        let prevJoint = world.joints.get(joint.islandPrev);
        assert(prevJoint.islandNext == joint.jointId);
        prevJoint.islandNext = joint.islandNext;
    }

    if (joint.islandNext != B2_NULL_INDEX) {
        let nextJoint = world.joints.get(joint.islandNext);
        assert(nextJoint.islandPrev == joint.jointId);
        nextJoint.islandPrev = joint.islandPrev;
    }

    if (island.headJoint == joint.jointId) {
        island.headJoint = joint.islandNext;
    }

    if (island.tailJoint == joint.jointId) {
        island.tailJoint = joint.islandPrev;
    }

    assert(island.jointCount > 0);
    island.jointCount -= 1;
    island.constraintRemoveCount += 1;

    joint.islandId = B2_NULL_INDEX;
    joint.islandPrev = B2_NULL_INDEX;
    joint.islandNext = B2_NULL_INDEX;

    b2ValidateIsland(world, islandId);
}

export function b2SplitIsland(world: b2World, baseId: i32): void {
    let baseIsland = world.islands.get(baseId);
    let setIndex = baseIsland.setIndex;

    if (setIndex != b2_awakeSet) {
        // can only split awake island
        return;
    }

    if (baseIsland.constraintRemoveCount == 0) {
        // this island doesn't need to be split
        return;
    }

    b2ValidateIsland(world, baseId);

    let bodyCount = baseIsland.bodyCount;

    let stack = new Int32Array(bodyCount);
    let bodyIds = new Int32Array(bodyCount);

    // Build array containing all body indices from base island. These
    // serve as seed bodies for the depth first search (DFS).
    let index = 0;
    let nextBody = baseIsland.headBody;
    while (nextBody != B2_NULL_INDEX) {
        bodyIds[index++] = nextBody;
        let body = world.bodies.get(nextBody);

        nextBody = body.islandNext;
    }
    assert(index == bodyCount);

    // Each island is found as a depth first search starting from a seed body
    for (let i = 0; i < bodyCount; ++i) {
        let seedIndex = bodyIds[i];
        let seed = world.bodies.get(seedIndex);
        assert(seed.setIndex == setIndex);

        if (seed.islandId != baseId) {
            // The body has already been visited
            continue;
        }

        let stackCount = 0;
        stack[stackCount++] = seedIndex;

        // Create new island
        let island = b2CreateIsland(world, setIndex);

        let islandId = island.islandId;
        seed.islandId = islandId;

        // Perform a depth first search (DFS) on the constraint graph.
        while (stackCount > 0) {
            // Grab the next body off the stack and add it to the island.
            let bodyId = stack[--stackCount];
            let body = world.bodies.get(bodyId);
            assert(body.setIndex == b2_awakeSet);
            assert(body.islandId == islandId);

            // Add body to island
            if (island.tailBody != B2_NULL_INDEX) {
                world.bodies.get(island.tailBody).islandNext = bodyId;
            }
            body.islandPrev = island.tailBody;
            body.islandNext = B2_NULL_INDEX;
            island.tailBody = bodyId;

            if (island.headBody == B2_NULL_INDEX) {
                island.headBody = bodyId;
            }

            island.bodyCount += 1;

            // Search all contacts connected to this body.
            let contactKey = body.headContactKey;
            while (contactKey != B2_NULL_INDEX) {
                let contactId = contactKey >> 1;
                let edgeIndex = contactKey & 1;

                let contact = world.contacts.get(contactId);
                assert(contact.contactId == contactId);

                // Next key
                contactKey = contact.edges[edgeIndex].nextKey;

                // Has this contact already been added to this island?
                if (contact.islandId == islandId) {
                    continue;
                }

                // Is this contact enabled and touching?
                if ((contact.flags & b2_contactTouchingFlag) == 0) {
                    continue;
                }

                let otherEdgeIndex = edgeIndex ^ 1;
                let otherBodyId = contact.edges[otherEdgeIndex].bodyId;
                let otherBody = world.bodies.get(otherBodyId);

                // Maybe add other body to stack
                if (otherBody.islandId != islandId && otherBody.setIndex != b2_staticSet) {
                    assert(stackCount < bodyCount);
                    stack[stackCount++] = otherBodyId;

                    // Need to update the body's island id immediately so it is not traversed again
                    otherBody.islandId = islandId;
                }

                // Add contact to island
                contact.islandId = islandId;
                if (island.tailContact != B2_NULL_INDEX) {
                    let tailContact = world.contacts.get(island.tailContact);
                    tailContact.islandNext = contactId;
                }
                contact.islandPrev = island.tailContact;
                contact.islandNext = B2_NULL_INDEX;
                island.tailContact = contactId;

                if (island.headContact == B2_NULL_INDEX) {
                    island.headContact = contactId;
                }

                island.contactCount += 1;
            }

            // Search all joints connect to this body.
            let jointKey = body.headJointKey;
            while (jointKey != B2_NULL_INDEX) {
                let jointId = jointKey >> 1;
                let edgeIndex = jointKey & 1;

                let joint = world.joints.get(jointId);
                assert(joint.jointId == jointId);

                // Next key
                jointKey = joint.edges[edgeIndex].nextKey;

                // Has this joint already been added to this island?
                if (joint.islandId == islandId) {
                    continue;
                }

                if (joint.setIndex == b2_disabledSet) {
                    continue;
                }

                let otherEdgeIndex = edgeIndex ^ 1;
                let otherBodyId = joint.edges[otherEdgeIndex].bodyId;
                let otherBody = world.bodies.get(otherBodyId);

                // Don't simulate joints connected to disabled bodies.
                if (otherBody.setIndex == b2_disabledSet) {
                    continue;
                }

                // At least one body must be dynamic
                if (body.type != b2_dynamicBody && otherBody.type != b2_dynamicBody) {
                    continue;
                }

                // Maybe add other body to stack
                if (otherBody.islandId != islandId && otherBody.setIndex == b2_awakeSet) {
                    assert(stackCount < bodyCount);
                    stack[stackCount++] = otherBodyId;

                    // Need to update the body's island id immediately so it is not traversed again
                    otherBody.islandId = islandId;
                }

                // Add joint to island
                joint.islandId = islandId;
                if (island.tailJoint != B2_NULL_INDEX) {
                    let tailJoint = world.joints.get(island.tailJoint);
                    tailJoint.islandNext = jointId;
                }
                joint.islandPrev = island.tailJoint;
                joint.islandNext = B2_NULL_INDEX;
                island.tailJoint = jointId;

                if (island.headJoint == B2_NULL_INDEX) {
                    island.headJoint = jointId;
                }

                island.jointCount += 1;
            }
        }

        b2ValidateIsland(world, islandId);
    }

    // Done with the base split island. This is delayed because the baseId is used as a marker and it
    // should not be recycled in while splitting.
    b2DestroyIsland(world, baseId);
}

// Single threaded port of b2SplitIslandTask.
export function b2SplitIslandTask(startIndex: i32, endIndex: i32, threadIndex: u32, world: b2World): void {
    assert(world.splitIslandId != B2_NULL_INDEX);
    b2SplitIsland(world, world.splitIslandId);
}
