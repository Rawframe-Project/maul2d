// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Event streams: contact begin and end, sensor overlaps and joint
// breaks, collected during a step and read after it.

#ifndef MAUL2D_EVENTS_H
#define MAUL2D_EVENTS_H

#include "maul2d/joint.h"
#include "maul2d/shape.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// A contact that began touching during a step. Begin and end events
    /// pair up: every path that ends a touching contact (separation, a lost
    /// pair, a destroyed shape or body, a filter or type change) emits its
    /// end. A begin carries what a game reacts to (where, which way, how
    /// hard), never the internal manifold, whose layout stays free to
    /// change.
    typedef struct m2ContactBeginEvent
    {
        m2ShapeId shapeIdA; // the lower shape index of the pair
        m2ShapeId shapeIdB;
        M2_ALIGN8 uint64_t step; // the step the event belongs to
        m2Vec2 normal;           // world frame, from shape A toward shape B
        float approachSpeed;     // closing speed along the normal, never negative
        int32_t pointCount;
        m2Pos2 points[2]; // world contact positions
    } m2ContactBeginEvent;

    /// A contact that stopped touching during a step.
    typedef struct m2ContactEndEvent
    {
        m2ShapeId shapeIdA; // the ids as they were when the touch ended
        m2ShapeId shapeIdB;
        M2_ALIGN8 uint64_t step;
    } m2ContactEndEvent;

    /// The contact events of the last step, in canonical order.
    typedef struct m2ContactEvents
    {
        const m2ContactBeginEvent* beginEvents;
        const m2ContactEndEvent* endEvents;
        int32_t beginCount;
        int32_t endCount;
    } m2ContactEvents;

    /// Returns the contact events of the last step. Events are polled,
    /// never called back. After a rollback, the steps simulated again emit
    /// their events again; a host that must see each once keys them by
    /// step and shape ids.
    ///
    /// @param world  The world.
    /// @return Arrays the world owns, in canonical order, valid until the
    ///         next m2World_Step or m2World_Restore of this world; a restore
    ///         clears them. Empty for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2ContactEvents m2World_GetContactEvents(const m2World* world);

    /// The sensor events of the last step: the contact records, in their
    /// own arrays, with the same pairing. A begin carries the overlap's
    /// point, normal and closing speed; an end, the shapes and the step.
    typedef struct m2SensorEvents
    {
        const m2ContactBeginEvent* beginEvents;
        const m2ContactEndEvent* endEvents;
        int32_t beginCount;
        int32_t endCount;
    } m2SensorEvents;

    /// Returns the sensor events of the last step. A destroyed or filtered
    /// overlap always emits its end.
    ///
    /// @param world  The world.
    /// @return Arrays the world owns, in canonical order, valid until the
    ///         next m2World_Step or m2World_Restore of this world. Empty for
    ///         a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2SensorEvents m2World_GetSensorEvents(const m2World* world);

    /// A joint that exceeded its break limits during a step. Its id is
    /// already stale when the event is read. Two worlds given the same
    /// calls break the same joints on the same step.
    typedef struct m2JointBreakEvent
    {
        m2JointId jointId; // the id the joint had
        M2_ALIGN8 uint64_t step;
        float force;  // the reaction force that broke it
        float torque; // the reaction torque that broke it
    } m2JointBreakEvent;

    /// The joint events of the last step.
    typedef struct m2JointEvents
    {
        const m2JointBreakEvent* breakEvents;
        int32_t breakCount;
    } m2JointEvents;

    /// Returns the joints that broke during the last step.
    ///
    /// @param world  The world.
    /// @return An array the world owns, in ascending joint order, valid
    ///         until the next m2World_Step or m2World_Restore of this world.
    ///         Empty for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2JointEvents m2World_GetJointEvents(const m2World* world);

    /// Lists the shapes inside a sensor now, in canonical pair order.
    ///
    /// @param world          The world.
    /// @param sensorShapeId  The sensor. A stale id counts one misuse; a
    ///                       live shape that is not a sensor lists nothing.
    /// @param overlaps       Receives up to capacity shape ids; may be null
    ///                       to measure.
    /// @param capacity       The length of overlaps.
    /// @return The number of shapes inside, which may exceed capacity.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2Shape_GetSensorOverlaps(const m2World* world, m2ShapeId sensorShapeId,
                                             m2ShapeId* overlaps, int32_t capacity);

    /// One contact touching now, as the solver last saw it.
    typedef struct m2ContactData
    {
        m2ShapeId shapeIdA;
        m2ShapeId shapeIdB;
        m2Vec2 normal; // world frame, from A to B
        int32_t pointCount;
        float separations[2];
        float normalImpulses[2];
        float tangentImpulses[2];
    } m2ContactData;

    /// Lists the solid contacts touching now, in canonical pair order.
    /// Sensor overlaps are not contacts and are left out.
    ///
    /// @param world     The world.
    /// @param data      Receives up to capacity contacts; may be null to
    ///                  measure.
    /// @param capacity  The length of data.
    /// @return The number of touching contacts, which may exceed capacity.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetContactData(const m2World* world, m2ContactData* data,
                                          int32_t capacity);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_EVENTS_H
