// Ported from box2d src/solver_set.h/.c. C's memcpy transfers become
// reference moves: every transfer site removes or destroys the source slot
// right after, so no aliasing survives (see README Phase C notes).

import { B2_NULL_INDEX } from "./core";
import { B2_GRAPH_COLOR_COUNT } from "./constants";
import { b2Array } from "./array";
import { b2ClearBit } from "./bitset";
import { b2AllocId, b2FreeId } from "./id_pool";
import {
    b2World, b2ValidateSolverSets, b2_staticSet, b2_disabledSet, b2_awakeSet, b2_firstSleepingSet,
} from "./physics_world";
import { b2Body, b2BodySim, b2BodyState, b2_isFast, b2_isSpeedCapped, b2_hadTimeOfImpact } from "./body";
import { b2Contact, b2ContactSim, b2_contactTouchingFlag, b2_simTouchingFlag } from "./contact";
import { b2Joint, b2JointSim } from "./joint";
import { b2AddJointToGraph, b2RemoveJointFromGraph, b2AddContactToGraph, B2_OVERFLOW_INDEX } from "./constraint_graph";
import { b2IslandSim } from "./island";

export class b2SolverSet {
    // Body array. Empty for unused set.
    bodySims: b2Array<b2BodySim> = new b2Array<b2BodySim>();
    // Body state only exists for active set
    bodyStates: b2Array<b2BodyState> = new b2Array<b2BodyState>();
    // This holds sleeping/disabled joints. Empty for static/active set.
    jointSims: b2Array<b2JointSim> = new b2Array<b2JointSim>();
    // This holds all contacts for sleeping sets.
    // This holds non-touching contacts for the awake set.
    contactSims: b2Array<b2ContactSim> = new b2Array<b2ContactSim>();
    // The awake set has an array of islands. Sleeping sets normally have a single island.
    islandSims: b2Array<b2IslandSim> = new b2Array<b2IslandSim>();
    // Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
    setIndex: i32 = 0;
}

export function b2DestroySolverSet(world: b2World, setIndex: i32): void {
    let set = world.solverSets.get(setIndex);
    set.bodySims = new b2Array<b2BodySim>();
    set.bodyStates = new b2Array<b2BodyState>();
    set.contactSims = new b2Array<b2ContactSim>();
    set.jointSims = new b2Array<b2JointSim>();
    set.islandSims = new b2Array<b2IslandSim>();
    b2FreeId(world.solverSetIdPool, setIndex);
    set.setIndex = B2_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
export function b2WakeSolverSet(world: b2World, setIndex: i32): void {
    assert(setIndex >= b2_firstSleepingSet);
    let set = world.solverSets.get(setIndex);
    let awakeSet = world.solverSets.get(b2_awakeSet);
    let disabledSet = world.solverSets.get(b2_disabledSet);

    let bodyCount = set.bodySims.count;
    for (let i = 0; i < bodyCount; ++i) {
        let simSrc = set.bodySims.get(i);

        let body = world.bodies.get(simSrc.bodyId);
        assert(body.setIndex == setIndex);
        body.setIndex = b2_awakeSet;
        body.localIndex = awakeSet.bodySims.count;

        // Reset sleep timer
        body.sleepTime = 0.0;

        awakeSet.bodySims.push(simSrc);

        let state = awakeSet.bodyStates.add();
        state.flags = body.flags;

        // move non-touching contacts from disabled set to awake set
        let contactKey = body.headContactKey;
        while (contactKey != B2_NULL_INDEX) {
            let edgeIndex = contactKey & 1;
            let contactId = contactKey >> 1;

            let contact = world.contacts.get(contactId);

            contactKey = contact.edges[edgeIndex].nextKey;

            if (contact.setIndex != b2_disabledSet) {
                assert(contact.setIndex == b2_awakeSet || contact.setIndex == setIndex);
                continue;
            }

            let localIndex = contact.localIndex;
            let contactSim = disabledSet.contactSims.get(localIndex);

            assert((contact.flags & b2_contactTouchingFlag) == 0 && contactSim.manifold.pointCount == 0);

            contact.setIndex = b2_awakeSet;
            contact.localIndex = awakeSet.contactSims.count;
            awakeSet.contactSims.push(contactSim);

            let movedLocalIndex = disabledSet.contactSims.removeSwap(localIndex);
            if (movedLocalIndex != B2_NULL_INDEX) {
                // fix moved element
                let movedContactSim = disabledSet.contactSims.get(localIndex);
                let movedContact = world.contacts.get(movedContactSim.contactId);
                assert(movedContact.localIndex == movedLocalIndex);
                movedContact.localIndex = localIndex;
            }
        }
    }

    // transfer touching contacts from sleeping set to contact graph
    {
        let contactCount = set.contactSims.count;
        for (let i = 0; i < contactCount; ++i) {
            let contactSim = set.contactSims.get(i);
            let contact = world.contacts.get(contactSim.contactId);
            assert((contact.flags & b2_contactTouchingFlag) != 0);
            assert((contactSim.simFlags & b2_simTouchingFlag) != 0);
            assert(contactSim.manifold.pointCount > 0);
            assert(contact.setIndex == setIndex);
            b2AddContactToGraph(world, contactSim, contact);
            contact.setIndex = b2_awakeSet;
        }
    }

    // transfer joints from sleeping set to awake set
    {
        let jointCount = set.jointSims.count;
        for (let i = 0; i < jointCount; ++i) {
            let jointSim = set.jointSims.get(i);
            let joint = world.joints.get(jointSim.jointId);
            assert(joint.setIndex == setIndex);
            b2AddJointToGraph(world, jointSim, joint);
            joint.setIndex = b2_awakeSet;
        }
    }

    // transfer island from sleeping set to awake set
    // Usually a sleeping set has only one island, but it is possible
    // that joints are created between sleeping islands and they
    // are moved to the same sleeping set.
    {
        let islandCount = set.islandSims.count;
        for (let i = 0; i < islandCount; ++i) {
            let islandSrc = set.islandSims.get(i);
            let island = world.islands.get(islandSrc.islandId);
            island.setIndex = b2_awakeSet;
            island.localIndex = awakeSet.islandSims.count;
            awakeSet.islandSims.push(islandSrc);
        }
    }

    // destroy the sleeping set
    b2DestroySolverSet(world, setIndex);
}

export function b2TrySleepIsland(world: b2World, islandId: i32): void {
    let island = world.islands.get(islandId);
    assert(island.setIndex == b2_awakeSet);

    // cannot put an island to sleep while it has a pending split
    if (island.constraintRemoveCount > 0) {
        return;
    }

    // island is sleeping
    // - create new sleeping solver set
    // - move island to sleeping solver set
    // - identify non-touching contacts that should move to sleeping solver set or disabled set
    // - remove old island
    // - fix island
    let sleepSetId = b2AllocId(world.solverSetIdPool);
    if (sleepSetId == world.solverSets.count) {
        let set = world.solverSets.add();
        set.setIndex = B2_NULL_INDEX;
    }

    let sleepSet = world.solverSets.get(sleepSetId);

    // grab awake set after creating the sleep set because the solver set array may have been resized
    let awakeSet = world.solverSets.get(b2_awakeSet);
    assert(0 <= island.localIndex && island.localIndex < awakeSet.islandSims.count);

    sleepSet.setIndex = sleepSetId;
    sleepSet.bodySims = new b2Array<b2BodySim>(island.bodyCount);
    sleepSet.bodyStates = new b2Array<b2BodyState>();
    sleepSet.contactSims = new b2Array<b2ContactSim>(island.contactCount);
    sleepSet.jointSims = new b2Array<b2JointSim>(island.jointCount);
    sleepSet.islandSims = new b2Array<b2IslandSim>();

    // move awake bodies to sleeping set
    // this shuffles around bodies in the awake set
    {
        let disabledSet = world.solverSets.get(b2_disabledSet);
        let bodyId = island.headBody;
        while (bodyId != B2_NULL_INDEX) {
            let body = world.bodies.get(bodyId);
            assert(body.setIndex == b2_awakeSet);
            assert(body.islandId == islandId);

            // Update the body move event to indicate this body fell asleep
            // It could happen the body is forced asleep before it ever moves.
            if (body.bodyMoveIndex != B2_NULL_INDEX) {
                let moveEvent = world.bodyMoveEvents.get(body.bodyMoveIndex);
                assert(moveEvent.bodyId.index1 - 1 == bodyId);
                assert(moveEvent.bodyId.generation == body.generation);
                moveEvent.fellAsleep = true;
                body.bodyMoveIndex = B2_NULL_INDEX;
            }

            let awakeBodyIndex = body.localIndex;
            let awakeSim = awakeSet.bodySims.get(awakeBodyIndex);

            // move body sim to sleep set
            let sleepBodyIndex = sleepSet.bodySims.count;
            sleepSet.bodySims.push(awakeSim);

            let movedIndex = awakeSet.bodySims.removeSwap(awakeBodyIndex);
            if (movedIndex != B2_NULL_INDEX) {
                // fix local index on moved element
                let movedSim = awakeSet.bodySims.get(awakeBodyIndex);
                let movedId = movedSim.bodyId;
                let movedBody = world.bodies.get(movedId);
                assert(movedBody.localIndex == movedIndex);
                movedBody.localIndex = awakeBodyIndex;
            }

            // destroy state, no need to clone
            awakeSet.bodyStates.removeSwap(awakeBodyIndex);

            body.setIndex = sleepSetId;
            body.localIndex = sleepBodyIndex;

            // Move non-touching contacts to the disabled set.
            // Non-touching contacts may exist between sleeping islands and there is no clear ownership.
            let contactKey = body.headContactKey;
            while (contactKey != B2_NULL_INDEX) {
                let contactId = contactKey >> 1;
                let edgeIndex = contactKey & 1;

                let contact = world.contacts.get(contactId);

                assert(contact.setIndex == b2_awakeSet || contact.setIndex == b2_disabledSet);
                contactKey = contact.edges[edgeIndex].nextKey;

                if (contact.setIndex == b2_disabledSet) {
                    // already moved to disabled set by another body in the island
                    continue;
                }

                if (contact.colorIndex != B2_NULL_INDEX) {
                    // contact is touching and will be moved separately
                    assert((contact.flags & b2_contactTouchingFlag) != 0);
                    continue;
                }

                // the other body may still be awake, it still may go to sleep and then it will be responsible
                // for moving this contact to the disabled set.
                let otherEdgeIndex = edgeIndex ^ 1;
                let otherBodyId = contact.edges[otherEdgeIndex].bodyId;
                let otherBody = world.bodies.get(otherBodyId);
                if (otherBody.setIndex == b2_awakeSet) {
                    continue;
                }

                let localIndex = contact.localIndex;
                let contactSim = awakeSet.contactSims.get(localIndex);

                assert(contactSim.manifold.pointCount == 0);
                assert((contact.flags & b2_contactTouchingFlag) == 0);

                // move the non-touching contact to the disabled set
                contact.setIndex = b2_disabledSet;
                contact.localIndex = disabledSet.contactSims.count;
                disabledSet.contactSims.push(contactSim);

                let movedLocalIndex = awakeSet.contactSims.removeSwap(localIndex);
                if (movedLocalIndex != B2_NULL_INDEX) {
                    // fix moved element
                    let movedContactSim = awakeSet.contactSims.get(localIndex);
                    let movedContact = world.contacts.get(movedContactSim.contactId);
                    assert(movedContact.localIndex == movedLocalIndex);
                    movedContact.localIndex = localIndex;
                }
            }

            bodyId = body.islandNext;
        }
    }

    // move touching contacts
    // this shuffles contacts in the awake set
    {
        let contactId = island.headContact;
        while (contactId != B2_NULL_INDEX) {
            let contact = world.contacts.get(contactId);
            assert(contact.setIndex == b2_awakeSet);
            assert(contact.islandId == islandId);
            let colorIndex = contact.colorIndex;
            assert(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);

            let color = world.constraintGraph.colors[colorIndex];

            // Remove bodies from graph coloring associated with this constraint
            if (colorIndex != B2_OVERFLOW_INDEX) {
                // might clear a bit for a static body, but this has no effect
                b2ClearBit(color.bodySet, contact.edges[0].bodyId);
                b2ClearBit(color.bodySet, contact.edges[1].bodyId);
            }

            let localIndex = contact.localIndex;
            let awakeContactSim = color.contactSims.get(localIndex);

            let sleepContactIndex = sleepSet.contactSims.count;
            sleepSet.contactSims.push(awakeContactSim);

            let movedLocalIndex = color.contactSims.removeSwap(localIndex);
            if (movedLocalIndex != B2_NULL_INDEX) {
                // fix moved element
                let movedContactSim = color.contactSims.get(localIndex);
                let movedContact = world.contacts.get(movedContactSim.contactId);
                assert(movedContact.localIndex == movedLocalIndex);
                movedContact.localIndex = localIndex;
            }

            contact.setIndex = sleepSetId;
            contact.colorIndex = B2_NULL_INDEX;
            contact.localIndex = sleepContactIndex;

            contactId = contact.islandNext;
        }
    }

    // move joints
    // this shuffles joints in the awake set
    {
        let jointId = island.headJoint;
        while (jointId != B2_NULL_INDEX) {
            let joint = world.joints.get(jointId);
            assert(joint.setIndex == b2_awakeSet);
            assert(joint.islandId == islandId);
            let colorIndex = joint.colorIndex;
            let localIndex = joint.localIndex;

            assert(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);

            let color = world.constraintGraph.colors[colorIndex];

            let awakeJointSim = color.jointSims.get(localIndex);

            if (colorIndex != B2_OVERFLOW_INDEX) {
                // might clear a bit for a static body, but this has no effect
                b2ClearBit(color.bodySet, joint.edges[0].bodyId);
                b2ClearBit(color.bodySet, joint.edges[1].bodyId);
            }

            let sleepJointIndex = sleepSet.jointSims.count;
            sleepSet.jointSims.push(awakeJointSim);

            let movedIndex = color.jointSims.removeSwap(localIndex);
            if (movedIndex != B2_NULL_INDEX) {
                // fix moved element
                let movedJointSim = color.jointSims.get(localIndex);
                let movedId = movedJointSim.jointId;
                let movedJoint = world.joints.get(movedId);
                assert(movedJoint.localIndex == movedIndex);
                movedJoint.localIndex = localIndex;
            }

            joint.setIndex = sleepSetId;
            joint.colorIndex = B2_NULL_INDEX;
            joint.localIndex = sleepJointIndex;

            jointId = joint.islandNext;
        }
    }

    // move island struct
    {
        assert(island.setIndex == b2_awakeSet);

        let islandIndex = island.localIndex;
        let sleepIsland = sleepSet.islandSims.add();
        sleepIsland.islandId = islandId;

        let movedIslandIndex = awakeSet.islandSims.removeSwap(islandIndex);
        if (movedIslandIndex != B2_NULL_INDEX) {
            // fix index on moved element
            let movedIslandSim = awakeSet.islandSims.get(islandIndex);
            let movedIslandId = movedIslandSim.islandId;
            let movedIsland = world.islands.get(movedIslandId);
            assert(movedIsland.localIndex == movedIslandIndex);
            movedIsland.localIndex = islandIndex;
        }

        island.setIndex = sleepSetId;
        island.localIndex = 0;
    }

    b2ValidateSolverSets(world);
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
export function b2MergeSolverSets(world: b2World, setId1: i32, setId2: i32): void {
    assert(setId1 >= b2_firstSleepingSet);
    assert(setId2 >= b2_firstSleepingSet);
    let set1 = world.solverSets.get(setId1);
    let set2 = world.solverSets.get(setId2);

    // Move the fewest number of bodies
    if (set1.bodySims.count < set2.bodySims.count) {
        let tempSet = set1;
        set1 = set2;
        set2 = tempSet;

        let tempId = setId1;
        setId1 = setId2;
        setId2 = tempId;
    }

    // transfer bodies
    {
        let bodyCount = set2.bodySims.count;
        for (let i = 0; i < bodyCount; ++i) {
            let simSrc = set2.bodySims.get(i);

            let body = world.bodies.get(simSrc.bodyId);
            assert(body.setIndex == setId2);
            body.setIndex = setId1;
            body.localIndex = set1.bodySims.count;

            set1.bodySims.push(simSrc);
        }
    }

    // transfer contacts
    {
        let contactCount = set2.contactSims.count;
        for (let i = 0; i < contactCount; ++i) {
            let contactSrc = set2.contactSims.get(i);

            let contact = world.contacts.get(contactSrc.contactId);
            assert(contact.setIndex == setId2);
            contact.setIndex = setId1;
            contact.localIndex = set1.contactSims.count;

            set1.contactSims.push(contactSrc);
        }
    }

    // transfer joints
    {
        let jointCount = set2.jointSims.count;
        for (let i = 0; i < jointCount; ++i) {
            let jointSrc = set2.jointSims.get(i);

            let joint = world.joints.get(jointSrc.jointId);
            assert(joint.setIndex == setId2);
            joint.setIndex = setId1;
            joint.localIndex = set1.jointSims.count;

            set1.jointSims.push(jointSrc);
        }
    }

    // transfer islands
    {
        let islandCount = set2.islandSims.count;
        for (let i = 0; i < islandCount; ++i) {
            let islandSrc = set2.islandSims.get(i);
            let islandId = islandSrc.islandId;

            let island = world.islands.get(islandId);
            island.setIndex = setId1;
            island.localIndex = set1.islandSims.count;

            set1.islandSims.push(islandSrc);
        }
    }

    // destroy the merged set
    b2DestroySolverSet(world, setId2);

    b2ValidateSolverSets(world);
}

export function b2TransferBody(world: b2World, targetSet: b2SolverSet, sourceSet: b2SolverSet, body: b2Body): void {
    if (targetSet == sourceSet) {
        return;
    }

    let sourceIndex = body.localIndex;
    let sourceSim = sourceSet.bodySims.get(sourceIndex);

    let targetIndex = targetSet.bodySims.count;
    targetSet.bodySims.push(sourceSim);

    // Clear transient body flags
    sourceSim.flags &= ~(b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact);

    // Remove body sim from solver set that owns it
    let movedIndex = sourceSet.bodySims.removeSwap(sourceIndex);
    if (movedIndex != B2_NULL_INDEX) {
        // Fix moved body index
        let movedSim = sourceSet.bodySims.get(sourceIndex);
        let movedId = movedSim.bodyId;
        let movedBody = world.bodies.get(movedId);
        assert(movedBody.localIndex == movedIndex);
        movedBody.localIndex = sourceIndex;
    }

    if (sourceSet.setIndex == b2_awakeSet) {
        sourceSet.bodyStates.removeSwap(sourceIndex);
    } else if (targetSet.setIndex == b2_awakeSet) {
        let state = targetSet.bodyStates.add();
        state.flags = body.flags;
    }

    body.setIndex = targetSet.setIndex;
    body.localIndex = targetIndex;
}

export function b2TransferJoint(world: b2World, targetSet: b2SolverSet, sourceSet: b2SolverSet, joint: b2Joint): void {
    if (targetSet == sourceSet) {
        return;
    }

    let localIndex = joint.localIndex;
    let colorIndex = joint.colorIndex;

    // Retrieve source.
    let sourceSim: b2JointSim;
    if (sourceSet.setIndex == b2_awakeSet) {
        assert(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
        let color = world.constraintGraph.colors[colorIndex];

        sourceSim = color.jointSims.get(localIndex);
    } else {
        assert(colorIndex == B2_NULL_INDEX);
        sourceSim = sourceSet.jointSims.get(localIndex);
    }

    // Create target and copy. Fix joint.
    if (targetSet.setIndex == b2_awakeSet) {
        b2AddJointToGraph(world, sourceSim, joint);
        joint.setIndex = b2_awakeSet;
    } else {
        joint.setIndex = targetSet.setIndex;
        joint.localIndex = targetSet.jointSims.count;
        joint.colorIndex = B2_NULL_INDEX;

        targetSet.jointSims.push(sourceSim);
    }

    // Destroy source.
    if (sourceSet.setIndex == b2_awakeSet) {
        b2RemoveJointFromGraph(world, joint.edges[0].bodyId, joint.edges[1].bodyId, colorIndex, localIndex);
    } else {
        let movedIndex = sourceSet.jointSims.removeSwap(localIndex);
        if (movedIndex != B2_NULL_INDEX) {
            // fix swapped element
            let movedJointSim = sourceSet.jointSims.get(localIndex);
            let movedId = movedJointSim.jointId;
            let movedJoint = world.joints.get(movedId);
            movedJoint.localIndex = localIndex;
        }
    }
}
