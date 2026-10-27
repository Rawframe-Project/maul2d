# Events and queries

Nothing calls back into a program in the middle of a step. After a step, the program reads what happened (events) and asks about the world (queries). Both are readers: they change nothing, and their results come in a canonical order, the same on every platform.

## Events

- `m2World_GetContactEvents`: contacts that began and ended touching. A begin event carries what a game reacts to: the world normal, the contact points and the approach speed. An end event always follows a begin, even when the contact dies because a shape was destroyed or a filter or type changed.
- `m2World_GetSensorEvents`: shapes that began and stopped overlapping a sensor, paired the same way. Each event names the sensor (`sensorShapeId`), the shape inside it (`visitorShapeId`) and the step; two sensors never detect each other.
- `m2World_GetJointEvents`: joints that broke, with the force and torque that broke them.

The arrays belong to the world and stay valid until the next step or restore. A restore clears them, and the steps simulated again after a rollback emit their events again; a program that must see each event once keys it by step and shape ids.

```c
m2World_Step(world, 1.0f / 60.0f, 4);
m2ContactEvents contacts = m2World_GetContactEvents(world);
for (int32_t i = 0; i < contacts.beginCount; ++i)
{
    const m2ContactBeginEvent* begin = &contacts.beginEvents[i];
    if (begin->approachSpeed > 2.0f)
    {
        hardHits += 1; // a sound at begin->points[0], louder for faster hits
    }
}
```

`m2World_GetContactData` lists the solid contacts touching now with their impulses, for tools.

## Rays, sweeps and overlaps

A ray (`m2World_CastRayClosest`) or a swept circle, capsule or polygon (`m2World_CastCircleClosest` and its siblings) returns the closest hit: the fraction along the path, the facing normal, the point. A cast that starts inside a shape reports fraction 0 and a zero normal. On a tie the lower shape index wins. The all-hits variants (`m2World_CastRayAll` and its siblings) list every hit in ascending fraction and keep the closest when there are more than fit.

The overlap queries (`m2World_OverlapAabb`, `m2World_OverlapCircle` and its siblings) list every live shape touching a box (an `m2WorldAabb`, the type `m2Body_ComputeAabb` and `m2Shape_GetAabb` return) or a posed shape, in ascending order; touching within the engine's slop skin counts. Every query takes an `m2QueryFilter` with the same category and mask rule as contacts, and chain segments stay one-sided: a cast that starts on a segment's back side passes through.

A query that fills a caller array returns the true total, which may exceed the array's capacity; a null array with capacity 0 measures.

## The character mover

A character controller is usually not a body: it is a capsule the game moves itself, using three calls.

```c
m2Capsule capsule = {{0.0f, -0.5f}, {0.0f, 0.5f}, 0.3f};
m2MoverPlane planes[M2_MOVER_PLANES];
int32_t planeCount = m2World_CollideMover(world, &capsule, pose, planes, M2_MOVER_PLANES,
                                          m2DefaultQueryFilter());
if (planeCount > M2_MOVER_PLANES)
{
    planeCount = M2_MOVER_PLANES;
}
m2MoverMove move = m2SolveMover(wish, planes, planeCount);
pose.p.x += (double)move.translation.x;
pose.p.y += (double)move.translation.y;
velocity = m2ClipMoverVelocity(velocity, planes, planeCount, move.pressed);
```

`m2World_CollideMover` gathers the planes near the capsule, `m2SolveMover` turns the wished translation into the closest one no plane blocks, and `m2ClipMoverVelocity` removes the velocity that pushes into the planes the character rests on. `m2World_CastCapsuleClosest` is the sweep for fast moves. All of it only reads the world, so a character built on it replays exactly. The testbed's platformer is this loop end to end.

## Walking the world

An editor can enumerate everything from the world: `m2World_GetBodies`, `m2World_GetJoints` and `m2World_GetChains` fill caller arrays in ascending slot order, and `m2Body_GetShapes` and `m2Body_GetJoints` do the same for one body. Every type, filter, flag, geometry and joint parameter reads back, so a world can be rebuilt from the getters alone; a test does exactly that and checks the copy's hash for 90 steps.
