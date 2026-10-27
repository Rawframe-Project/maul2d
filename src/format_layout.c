// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The journal and snapshot formats, held to one layout on every target. A
// tape is its records' bytes and a snapshot walks its state arrays as they
// lie in memory, so each record, each public def a record embeds and each
// struct a snapshot walks is pinned here to its layout on x86-64: every
// size and every member's offset. A target that would lay one out
// otherwise (32-bit x86 aligns a double or a 64-bit integer in a struct to
// 4, which M2_ALIGN8 undoes) fails to build instead of writing another
// format. A change to one of these types is a format change: update its
// numbers here and bump the format's version.

#include "dynamic_tree.h"
#include "geometry.h"
#include "journal.h"

#include <stddef.h>

// The formats are little-endian; a big-endian target needs byte swaps in
// the journal and the snapshot walk first.
#if defined(__BYTE_ORDER__)
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "maul2d: the journal and snapshot formats are little-endian"
#endif
#elif !defined(_WIN32)
#error "maul2d: cannot tell this target's byte order"
#endif

#define M2_PIN_SIZE(type, bytes)                                                                   \
    _Static_assert(sizeof(type) == (bytes), #type " left the format's layout")
#define M2_PIN_AT(type, member, at)                                                                \
    _Static_assert(offsetof(type, member) == (at), #type "." #member " left the format's layout")

// The value types and ids that records and state arrays hold.
M2_PIN_SIZE(m2Vec2, 8);
M2_PIN_AT(m2Vec2, x, 0);
M2_PIN_AT(m2Vec2, y, 4);

M2_PIN_SIZE(m2Pos2, 16);
M2_PIN_AT(m2Pos2, x, 0);
M2_PIN_AT(m2Pos2, y, 8);

M2_PIN_SIZE(m2Rot, 8);
M2_PIN_AT(m2Rot, c, 0);
M2_PIN_AT(m2Rot, s, 4);

M2_PIN_SIZE(m2Transform, 24);
M2_PIN_AT(m2Transform, p, 0);
M2_PIN_AT(m2Transform, q, 16);

M2_PIN_SIZE(m2BodyId, 8);
M2_PIN_AT(m2BodyId, index1, 0);
M2_PIN_AT(m2BodyId, generation, 4);

M2_PIN_SIZE(m2ShapeId, 8);
M2_PIN_AT(m2ShapeId, index1, 0);
M2_PIN_AT(m2ShapeId, generation, 4);

M2_PIN_SIZE(m2JointId, 8);
M2_PIN_AT(m2JointId, index1, 0);
M2_PIN_AT(m2JointId, generation, 4);

M2_PIN_SIZE(m2ChainId, 8);
M2_PIN_AT(m2ChainId, index1, 0);
M2_PIN_AT(m2ChainId, generation, 4);

M2_PIN_SIZE(m2ParticleId, 8);
M2_PIN_AT(m2ParticleId, index1, 0);
M2_PIN_AT(m2ParticleId, generation, 4);

M2_PIN_SIZE(m2FluidVolumeId, 8);
M2_PIN_AT(m2FluidVolumeId, index1, 0);
M2_PIN_AT(m2FluidVolumeId, generation, 4);

M2_PIN_SIZE(m2Circle, 12);
M2_PIN_AT(m2Circle, center, 0);
M2_PIN_AT(m2Circle, radius, 8);

M2_PIN_SIZE(m2Capsule, 20);
M2_PIN_AT(m2Capsule, point1, 0);
M2_PIN_AT(m2Capsule, point2, 8);
M2_PIN_AT(m2Capsule, radius, 16);

M2_PIN_SIZE(m2Polygon, 136);
M2_PIN_AT(m2Polygon, vertices, 0);
M2_PIN_AT(m2Polygon, normals, 64);
M2_PIN_AT(m2Polygon, count, 128);
M2_PIN_AT(m2Polygon, radius, 132);

M2_PIN_SIZE(m2Segment, 16);
M2_PIN_AT(m2Segment, point1, 0);
M2_PIN_AT(m2Segment, point2, 8);

M2_PIN_SIZE(m2ChainSegment, 32);
M2_PIN_AT(m2ChainSegment, segment, 0);
M2_PIN_AT(m2ChainSegment, ghost1, 16);
M2_PIN_AT(m2ChainSegment, ghost2, 24);

// The structs the snapshot walks (src/world_state.c): array elements, and
// the broadphase trees, held inline in the world.
M2_PIN_SIZE(m2ShapeGeometry, 140);
M2_PIN_AT(m2ShapeGeometry, type, 0);
M2_PIN_AT(m2ShapeGeometry, circle, 4);
M2_PIN_AT(m2ShapeGeometry, capsule, 4);
M2_PIN_AT(m2ShapeGeometry, polygon, 4);
M2_PIN_AT(m2ShapeGeometry, segment, 4);
M2_PIN_AT(m2ShapeGeometry, chainSegment, 4);

M2_PIN_SIZE(m2ManifoldPoint, 32);
M2_PIN_AT(m2ManifoldPoint, anchorA, 0);
M2_PIN_AT(m2ManifoldPoint, anchorB, 8);
M2_PIN_AT(m2ManifoldPoint, separation, 16);
M2_PIN_AT(m2ManifoldPoint, normalImpulse, 20);
M2_PIN_AT(m2ManifoldPoint, tangentImpulse, 24);
M2_PIN_AT(m2ManifoldPoint, id, 28);
M2_PIN_AT(m2ManifoldPoint, flags, 30);

M2_PIN_SIZE(m2Manifold, 80);
M2_PIN_AT(m2Manifold, points, 0);
M2_PIN_AT(m2Manifold, normal, 64);
M2_PIN_AT(m2Manifold, pointCount, 72);
M2_PIN_AT(m2Manifold, padding, 76);

M2_PIN_SIZE(m2Aabb, 32);
M2_PIN_AT(m2Aabb, lowerBound, 0);
M2_PIN_AT(m2Aabb, upperBound, 16);

M2_PIN_SIZE(m2TreeNode, 56);
M2_PIN_AT(m2TreeNode, aabb, 0);
M2_PIN_AT(m2TreeNode, child1, 32);
M2_PIN_AT(m2TreeNode, child2, 36);
M2_PIN_AT(m2TreeNode, parent, 40);
M2_PIN_AT(m2TreeNode, height, 44);
M2_PIN_AT(m2TreeNode, userData, 48);
M2_PIN_AT(m2TreeNode, pad, 52);

M2_PIN_SIZE(m2DynamicTree, 16);
M2_PIN_AT(m2DynamicTree, root, 0);
M2_PIN_AT(m2DynamicTree, nodeCount, 4);
M2_PIN_AT(m2DynamicTree, nodeCapacity, 8);
M2_PIN_AT(m2DynamicTree, freeList, 12);

// The public defs a record embeds as given.
M2_PIN_SIZE(m2BodyDef, 80);
M2_PIN_AT(m2BodyDef, type, 0);
M2_PIN_AT(m2BodyDef, position, 8);
M2_PIN_AT(m2BodyDef, rotation, 24);
M2_PIN_AT(m2BodyDef, linearVelocity, 32);
M2_PIN_AT(m2BodyDef, angularVelocity, 40);
M2_PIN_AT(m2BodyDef, gravityScale, 44);
M2_PIN_AT(m2BodyDef, linearDamping, 48);
M2_PIN_AT(m2BodyDef, angularDamping, 52);
M2_PIN_AT(m2BodyDef, motionLocks, 56);
M2_PIN_AT(m2BodyDef, enableSleep, 59);
M2_PIN_AT(m2BodyDef, isEnabled, 60);
M2_PIN_AT(m2BodyDef, dominance, 61);
M2_PIN_AT(m2BodyDef, isBullet, 62);
M2_PIN_AT(m2BodyDef, userData, 64);
M2_PIN_AT(m2BodyDef, internalValue, 72);

M2_PIN_SIZE(m2MassData, 16);
M2_PIN_AT(m2MassData, mass, 0);
M2_PIN_AT(m2MassData, center, 4);
M2_PIN_AT(m2MassData, centralInertia, 12);

M2_PIN_SIZE(m2ShapeDef, 56);
M2_PIN_AT(m2ShapeDef, density, 0);
M2_PIN_AT(m2ShapeDef, friction, 4);
M2_PIN_AT(m2ShapeDef, tangentSpeed, 8);
M2_PIN_AT(m2ShapeDef, restitution, 12);
M2_PIN_AT(m2ShapeDef, categoryBits, 16);
M2_PIN_AT(m2ShapeDef, maskBits, 24);
M2_PIN_AT(m2ShapeDef, groupIndex, 32);
M2_PIN_AT(m2ShapeDef, isSensor, 36);
M2_PIN_AT(m2ShapeDef, userData, 40);
M2_PIN_AT(m2ShapeDef, internalValue, 48);

M2_PIN_SIZE(m2FluidVolumeDef, 80);
M2_PIN_AT(m2FluidVolumeDef, regionLower, 0);
M2_PIN_AT(m2FluidVolumeDef, regionUpper, 16);
M2_PIN_AT(m2FluidVolumeDef, surface, 32);
M2_PIN_AT(m2FluidVolumeDef, density, 40);
M2_PIN_AT(m2FluidVolumeDef, linearDrag, 44);
M2_PIN_AT(m2FluidVolumeDef, angularDrag, 48);
M2_PIN_AT(m2FluidVolumeDef, flow, 52);
M2_PIN_AT(m2FluidVolumeDef, userData, 64);
M2_PIN_AT(m2FluidVolumeDef, internalValue, 72);

M2_PIN_SIZE(m2DistanceJointDef, 72);
M2_PIN_AT(m2DistanceJointDef, bodyIdA, 0);
M2_PIN_AT(m2DistanceJointDef, bodyIdB, 8);
M2_PIN_AT(m2DistanceJointDef, localAnchorA, 16);
M2_PIN_AT(m2DistanceJointDef, localAnchorB, 24);
M2_PIN_AT(m2DistanceJointDef, length, 32);
M2_PIN_AT(m2DistanceJointDef, minLength, 36);
M2_PIN_AT(m2DistanceJointDef, maxLength, 40);
M2_PIN_AT(m2DistanceJointDef, springHertz, 44);
M2_PIN_AT(m2DistanceJointDef, springDampingRatio, 48);
M2_PIN_AT(m2DistanceJointDef, enableSpring, 52);
M2_PIN_AT(m2DistanceJointDef, userData, 56);
M2_PIN_AT(m2DistanceJointDef, enableConnectedCollision, 64);
M2_PIN_AT(m2DistanceJointDef, internalValue, 68);

M2_PIN_SIZE(m2RevoluteJointDef, 88);
M2_PIN_AT(m2RevoluteJointDef, bodyIdA, 0);
M2_PIN_AT(m2RevoluteJointDef, bodyIdB, 8);
M2_PIN_AT(m2RevoluteJointDef, localAnchorA, 16);
M2_PIN_AT(m2RevoluteJointDef, localAnchorB, 24);
M2_PIN_AT(m2RevoluteJointDef, constraintHertz, 32);
M2_PIN_AT(m2RevoluteJointDef, constraintDampingRatio, 36);
M2_PIN_AT(m2RevoluteJointDef, springHertz, 40);
M2_PIN_AT(m2RevoluteJointDef, springDampingRatio, 44);
M2_PIN_AT(m2RevoluteJointDef, enableSpring, 48);
M2_PIN_AT(m2RevoluteJointDef, enableMotor, 49);
M2_PIN_AT(m2RevoluteJointDef, motorSpeed, 52);
M2_PIN_AT(m2RevoluteJointDef, maxMotorTorque, 56);
M2_PIN_AT(m2RevoluteJointDef, enableLimit, 60);
M2_PIN_AT(m2RevoluteJointDef, lowerAngle, 64);
M2_PIN_AT(m2RevoluteJointDef, upperAngle, 68);
M2_PIN_AT(m2RevoluteJointDef, userData, 72);
M2_PIN_AT(m2RevoluteJointDef, enableConnectedCollision, 80);
M2_PIN_AT(m2RevoluteJointDef, internalValue, 84);

M2_PIN_SIZE(m2PrismaticJointDef, 88);
M2_PIN_AT(m2PrismaticJointDef, bodyIdA, 0);
M2_PIN_AT(m2PrismaticJointDef, bodyIdB, 8);
M2_PIN_AT(m2PrismaticJointDef, localAnchorA, 16);
M2_PIN_AT(m2PrismaticJointDef, localAnchorB, 24);
M2_PIN_AT(m2PrismaticJointDef, localAxisA, 32);
M2_PIN_AT(m2PrismaticJointDef, constraintHertz, 40);
M2_PIN_AT(m2PrismaticJointDef, constraintDampingRatio, 44);
M2_PIN_AT(m2PrismaticJointDef, enableMotor, 48);
M2_PIN_AT(m2PrismaticJointDef, motorSpeed, 52);
M2_PIN_AT(m2PrismaticJointDef, maxMotorForce, 56);
M2_PIN_AT(m2PrismaticJointDef, enableLimit, 60);
M2_PIN_AT(m2PrismaticJointDef, lowerTranslation, 64);
M2_PIN_AT(m2PrismaticJointDef, upperTranslation, 68);
M2_PIN_AT(m2PrismaticJointDef, userData, 72);
M2_PIN_AT(m2PrismaticJointDef, enableConnectedCollision, 80);
M2_PIN_AT(m2PrismaticJointDef, internalValue, 84);

M2_PIN_SIZE(m2WeldJointDef, 64);
M2_PIN_AT(m2WeldJointDef, bodyIdA, 0);
M2_PIN_AT(m2WeldJointDef, bodyIdB, 8);
M2_PIN_AT(m2WeldJointDef, localAnchorA, 16);
M2_PIN_AT(m2WeldJointDef, localAnchorB, 24);
M2_PIN_AT(m2WeldJointDef, linearSpringHertz, 32);
M2_PIN_AT(m2WeldJointDef, linearSpringDampingRatio, 36);
M2_PIN_AT(m2WeldJointDef, angularSpringHertz, 40);
M2_PIN_AT(m2WeldJointDef, angularSpringDampingRatio, 44);
M2_PIN_AT(m2WeldJointDef, userData, 48);
M2_PIN_AT(m2WeldJointDef, enableConnectedCollision, 56);
M2_PIN_AT(m2WeldJointDef, internalValue, 60);

M2_PIN_SIZE(m2WheelJointDef, 96);
M2_PIN_AT(m2WheelJointDef, bodyIdA, 0);
M2_PIN_AT(m2WheelJointDef, bodyIdB, 8);
M2_PIN_AT(m2WheelJointDef, localAnchorA, 16);
M2_PIN_AT(m2WheelJointDef, localAnchorB, 24);
M2_PIN_AT(m2WheelJointDef, localAxisA, 32);
M2_PIN_AT(m2WheelJointDef, enableSpring, 40);
M2_PIN_AT(m2WheelJointDef, springHertz, 44);
M2_PIN_AT(m2WheelJointDef, springDampingRatio, 48);
M2_PIN_AT(m2WheelJointDef, enableMotor, 52);
M2_PIN_AT(m2WheelJointDef, motorSpeed, 56);
M2_PIN_AT(m2WheelJointDef, maxMotorTorque, 60);
M2_PIN_AT(m2WheelJointDef, enableLimit, 64);
M2_PIN_AT(m2WheelJointDef, lowerTranslation, 68);
M2_PIN_AT(m2WheelJointDef, upperTranslation, 72);
M2_PIN_AT(m2WheelJointDef, userData, 80);
M2_PIN_AT(m2WheelJointDef, enableConnectedCollision, 88);
M2_PIN_AT(m2WheelJointDef, internalValue, 92);

M2_PIN_SIZE(m2FilterJointDef, 32);
M2_PIN_AT(m2FilterJointDef, bodyIdA, 0);
M2_PIN_AT(m2FilterJointDef, bodyIdB, 8);
M2_PIN_AT(m2FilterJointDef, userData, 16);
M2_PIN_AT(m2FilterJointDef, internalValue, 24);

M2_PIN_SIZE(m2MotorJointDef, 64);
M2_PIN_AT(m2MotorJointDef, bodyIdA, 0);
M2_PIN_AT(m2MotorJointDef, bodyIdB, 8);
M2_PIN_AT(m2MotorJointDef, linearOffset, 16);
M2_PIN_AT(m2MotorJointDef, angularOffset, 24);
M2_PIN_AT(m2MotorJointDef, maxForce, 28);
M2_PIN_AT(m2MotorJointDef, maxTorque, 32);
M2_PIN_AT(m2MotorJointDef, correctionFactor, 36);
M2_PIN_AT(m2MotorJointDef, springHertz, 40);
M2_PIN_AT(m2MotorJointDef, springDampingRatio, 44);
M2_PIN_AT(m2MotorJointDef, userData, 48);
M2_PIN_AT(m2MotorJointDef, enableConnectedCollision, 56);
M2_PIN_AT(m2MotorJointDef, internalValue, 60);

M2_PIN_SIZE(m2MouseJointDef, 64);
M2_PIN_AT(m2MouseJointDef, bodyIdA, 0);
M2_PIN_AT(m2MouseJointDef, bodyIdB, 8);
M2_PIN_AT(m2MouseJointDef, target, 16);
M2_PIN_AT(m2MouseJointDef, springHertz, 32);
M2_PIN_AT(m2MouseJointDef, springDampingRatio, 36);
M2_PIN_AT(m2MouseJointDef, maxForce, 40);
M2_PIN_AT(m2MouseJointDef, userData, 48);
M2_PIN_AT(m2MouseJointDef, enableConnectedCollision, 56);
M2_PIN_AT(m2MouseJointDef, internalValue, 60);

M2_PIN_SIZE(m2GearJointDef, 40);
M2_PIN_AT(m2GearJointDef, bodyIdA, 0);
M2_PIN_AT(m2GearJointDef, bodyIdB, 8);
M2_PIN_AT(m2GearJointDef, ratio, 16);
M2_PIN_AT(m2GearJointDef, userData, 24);
M2_PIN_AT(m2GearJointDef, enableConnectedCollision, 32);
M2_PIN_AT(m2GearJointDef, internalValue, 36);

M2_PIN_SIZE(m2PulleyJointDef, 88);
M2_PIN_AT(m2PulleyJointDef, bodyIdA, 0);
M2_PIN_AT(m2PulleyJointDef, bodyIdB, 8);
M2_PIN_AT(m2PulleyJointDef, groundAnchorA, 16);
M2_PIN_AT(m2PulleyJointDef, groundAnchorB, 32);
M2_PIN_AT(m2PulleyJointDef, localAnchorA, 48);
M2_PIN_AT(m2PulleyJointDef, localAnchorB, 56);
M2_PIN_AT(m2PulleyJointDef, ratio, 64);
M2_PIN_AT(m2PulleyJointDef, userData, 72);
M2_PIN_AT(m2PulleyJointDef, enableConnectedCollision, 80);
M2_PIN_AT(m2PulleyJointDef, internalValue, 84);

M2_PIN_SIZE(m2RatchetJointDef, 40);
M2_PIN_AT(m2RatchetJointDef, bodyIdA, 0);
M2_PIN_AT(m2RatchetJointDef, bodyIdB, 8);
M2_PIN_AT(m2RatchetJointDef, ratchet, 16);
M2_PIN_AT(m2RatchetJointDef, phase, 20);
M2_PIN_AT(m2RatchetJointDef, userData, 24);
M2_PIN_AT(m2RatchetJointDef, enableConnectedCollision, 32);
M2_PIN_AT(m2RatchetJointDef, internalValue, 36);

// The journal's records, in journal.h's order.
M2_PIN_SIZE(m2OpStep, 8);
M2_PIN_AT(m2OpStep, dt, 0);
M2_PIN_AT(m2OpStep, substepCount, 4);

M2_PIN_SIZE(m2OpCreateBody, 88);
M2_PIN_AT(m2OpCreateBody, def, 0);
M2_PIN_AT(m2OpCreateBody, expected, 80);

M2_PIN_SIZE(m2OpBodyVec, 16);
M2_PIN_AT(m2OpBodyVec, body, 0);
M2_PIN_AT(m2OpBodyVec, value, 8);

M2_PIN_SIZE(m2OpBodyFloat, 12);
M2_PIN_AT(m2OpBodyFloat, body, 0);
M2_PIN_AT(m2OpBodyFloat, value, 8);

M2_PIN_SIZE(m2OpBodyPoint, 32);
M2_PIN_AT(m2OpBodyPoint, body, 0);
M2_PIN_AT(m2OpBodyPoint, value, 8);
M2_PIN_AT(m2OpBodyPoint, point, 16);

M2_PIN_SIZE(m2OpBodyParam, 16);
M2_PIN_AT(m2OpBodyParam, body, 0);
M2_PIN_AT(m2OpBodyParam, value, 8);
M2_PIN_AT(m2OpBodyParam, param, 12);

M2_PIN_SIZE(m2OpBodyByte, 12);
M2_PIN_AT(m2OpBodyByte, body, 0);
M2_PIN_AT(m2OpBodyByte, value, 8);

M2_PIN_SIZE(m2OpBodyUserData, 16);
M2_PIN_AT(m2OpBodyUserData, body, 0);
M2_PIN_AT(m2OpBodyUserData, userData, 8);

M2_PIN_SIZE(m2OpSetTransform, 32);
M2_PIN_AT(m2OpSetTransform, body, 0);
M2_PIN_AT(m2OpSetTransform, position, 8);
M2_PIN_AT(m2OpSetTransform, rotation, 24);

M2_PIN_SIZE(m2OpSetMassData, 24);
M2_PIN_AT(m2OpSetMassData, body, 0);
M2_PIN_AT(m2OpSetMassData, data, 8);

M2_PIN_SIZE(m2OpCreateShape, 216);
M2_PIN_AT(m2OpCreateShape, body, 0);
M2_PIN_AT(m2OpCreateShape, def, 8);
M2_PIN_AT(m2OpCreateShape, geometry, 64);
M2_PIN_AT(m2OpCreateShape, expected, 204);

M2_PIN_SIZE(m2OpShapeParam, 16);
M2_PIN_AT(m2OpShapeParam, shape, 0);
M2_PIN_AT(m2OpShapeParam, value, 8);
M2_PIN_AT(m2OpShapeParam, param, 12);

M2_PIN_SIZE(m2OpShapeFloat, 12);
M2_PIN_AT(m2OpShapeFloat, shape, 0);
M2_PIN_AT(m2OpShapeFloat, value, 8);

M2_PIN_SIZE(m2OpSetFilter, 32);
M2_PIN_AT(m2OpSetFilter, shape, 0);
M2_PIN_AT(m2OpSetFilter, categoryBits, 8);
M2_PIN_AT(m2OpSetFilter, maskBits, 16);
M2_PIN_AT(m2OpSetFilter, groupIndex, 24);

M2_PIN_SIZE(m2OpSetGeometry, 148);
M2_PIN_AT(m2OpSetGeometry, shape, 0);
M2_PIN_AT(m2OpSetGeometry, geometry, 8);

M2_PIN_SIZE(m2OpShapeUserData, 16);
M2_PIN_AT(m2OpShapeUserData, shape, 0);
M2_PIN_AT(m2OpShapeUserData, userData, 8);

M2_PIN_SIZE(m2OpChainHeader, 64);
M2_PIN_AT(m2OpChainHeader, body, 0);
M2_PIN_AT(m2OpChainHeader, count, 8);
M2_PIN_AT(m2OpChainHeader, createdCount, 12);
M2_PIN_AT(m2OpChainHeader, friction, 16);
M2_PIN_AT(m2OpChainHeader, restitution, 20);
M2_PIN_AT(m2OpChainHeader, categoryBits, 24);
M2_PIN_AT(m2OpChainHeader, maskBits, 32);
M2_PIN_AT(m2OpChainHeader, groupIndex, 40);
M2_PIN_AT(m2OpChainHeader, userData, 48);
M2_PIN_AT(m2OpChainHeader, isLoop, 56);

M2_PIN_SIZE(m2OpChainFloat, 12);
M2_PIN_AT(m2OpChainFloat, chain, 0);
M2_PIN_AT(m2OpChainFloat, value, 8);

M2_PIN_SIZE(m2OpJointParam, 16);
M2_PIN_AT(m2OpJointParam, joint, 0);
M2_PIN_AT(m2OpJointParam, value, 8);
M2_PIN_AT(m2OpJointParam, param, 12);

M2_PIN_SIZE(m2OpJointUserData, 16);
M2_PIN_AT(m2OpJointUserData, joint, 0);
M2_PIN_AT(m2OpJointUserData, userData, 8);

M2_PIN_SIZE(m2OpMotorOffsets, 20);
M2_PIN_AT(m2OpMotorOffsets, joint, 0);
M2_PIN_AT(m2OpMotorOffsets, linear, 8);
M2_PIN_AT(m2OpMotorOffsets, angular, 16);

M2_PIN_SIZE(m2OpMouseTarget, 24);
M2_PIN_AT(m2OpMouseTarget, joint, 0);
M2_PIN_AT(m2OpMouseTarget, target, 8);

M2_PIN_SIZE(m2OpCreateParticle, 40);
M2_PIN_AT(m2OpCreateParticle, position, 0);
M2_PIN_AT(m2OpCreateParticle, velocity, 16);
M2_PIN_AT(m2OpCreateParticle, flags, 24);
M2_PIN_AT(m2OpCreateParticle, expected, 28);

M2_PIN_SIZE(m2OpFillParticles, 168);
M2_PIN_AT(m2OpFillParticles, polygon, 0);
M2_PIN_AT(m2OpFillParticles, position, 136);
M2_PIN_AT(m2OpFillParticles, velocity, 152);
M2_PIN_AT(m2OpFillParticles, flags, 160);
M2_PIN_AT(m2OpFillParticles, expected, 164);

M2_PIN_SIZE(m2OpParticleVec, 16);
M2_PIN_AT(m2OpParticleVec, id, 0);
M2_PIN_AT(m2OpParticleVec, value, 8);

M2_PIN_SIZE(m2OpParticleFloat, 12);
M2_PIN_AT(m2OpParticleFloat, id, 0);
M2_PIN_AT(m2OpParticleFloat, value, 8);

M2_PIN_SIZE(m2OpParticleUserData, 16);
M2_PIN_AT(m2OpParticleUserData, id, 0);
M2_PIN_AT(m2OpParticleUserData, userData, 8);

M2_PIN_SIZE(m2OpCreateFluidVolume, 88);
M2_PIN_AT(m2OpCreateFluidVolume, def, 0);
M2_PIN_AT(m2OpCreateFluidVolume, expected, 80);

M2_PIN_SIZE(m2OpFluidSurface, 16);
M2_PIN_AT(m2OpFluidSurface, id, 0);
M2_PIN_AT(m2OpFluidSurface, surface, 8);

M2_PIN_SIZE(m2OpShatterHeader, 16);
M2_PIN_AT(m2OpShatterHeader, body, 0);
M2_PIN_AT(m2OpShatterHeader, pieceCount, 8);
M2_PIN_AT(m2OpShatterHeader, expectedFirst, 12);

M2_PIN_SIZE(m2OpFlag, 1);
M2_PIN_AT(m2OpFlag, flag, 0);

M2_PIN_SIZE(m2OpVec, 8);
M2_PIN_AT(m2OpVec, value, 0);

M2_PIN_SIZE(m2OpSetWind, 12);
M2_PIN_AT(m2OpSetWind, velocity, 0);
M2_PIN_AT(m2OpSetWind, linearDrag, 8);

M2_PIN_SIZE(m2OpCreateDistanceJoint, 80);
M2_PIN_AT(m2OpCreateDistanceJoint, def, 0);
M2_PIN_AT(m2OpCreateDistanceJoint, expected, 72);

M2_PIN_SIZE(m2OpCreateRevoluteJoint, 96);
M2_PIN_AT(m2OpCreateRevoluteJoint, def, 0);
M2_PIN_AT(m2OpCreateRevoluteJoint, expected, 88);

M2_PIN_SIZE(m2OpCreatePrismaticJoint, 96);
M2_PIN_AT(m2OpCreatePrismaticJoint, def, 0);
M2_PIN_AT(m2OpCreatePrismaticJoint, expected, 88);

M2_PIN_SIZE(m2OpCreateWeldJoint, 72);
M2_PIN_AT(m2OpCreateWeldJoint, def, 0);
M2_PIN_AT(m2OpCreateWeldJoint, expected, 64);

M2_PIN_SIZE(m2OpCreateWheelJoint, 104);
M2_PIN_AT(m2OpCreateWheelJoint, def, 0);
M2_PIN_AT(m2OpCreateWheelJoint, expected, 96);

M2_PIN_SIZE(m2OpCreateFilterJoint, 40);
M2_PIN_AT(m2OpCreateFilterJoint, def, 0);
M2_PIN_AT(m2OpCreateFilterJoint, expected, 32);

M2_PIN_SIZE(m2OpCreateMotorJoint, 72);
M2_PIN_AT(m2OpCreateMotorJoint, def, 0);
M2_PIN_AT(m2OpCreateMotorJoint, expected, 64);

M2_PIN_SIZE(m2OpCreateMouseJoint, 72);
M2_PIN_AT(m2OpCreateMouseJoint, def, 0);
M2_PIN_AT(m2OpCreateMouseJoint, expected, 64);

M2_PIN_SIZE(m2OpCreateGearJoint, 48);
M2_PIN_AT(m2OpCreateGearJoint, def, 0);
M2_PIN_AT(m2OpCreateGearJoint, expected, 40);

M2_PIN_SIZE(m2OpCreatePulleyJoint, 96);
M2_PIN_AT(m2OpCreatePulleyJoint, def, 0);
M2_PIN_AT(m2OpCreatePulleyJoint, expected, 88);

M2_PIN_SIZE(m2OpCreateRatchetJoint, 48);
M2_PIN_AT(m2OpCreateRatchetJoint, def, 0);
M2_PIN_AT(m2OpCreateRatchetJoint, expected, 40);
