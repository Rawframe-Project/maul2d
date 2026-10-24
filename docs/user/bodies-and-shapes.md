# Bodies and shapes

A body has a pose and a velocity; its shapes give it collision and mass. Bodies are static (never move), kinematic (move at the velocity they are given, pushed by nothing) or dynamic (moved by forces, contacts and joints). Positions are 64-bit (`m2Pos2`) and everything else is 32-bit, so a game three hundred kilometers from the origin keeps exact contacts, casts and rendering.

## Moving bodies

Impulses change the velocity at once (`m2Body_ApplyLinearImpulse`, `m2Body_ApplyLinearImpulseAtPoint`, `m2Body_ApplyAngularImpulse`). Forces (`m2Body_ApplyForce`, `m2Body_ApplyForceAtPoint`, `m2Body_ApplyTorque`) add up across calls, act during the next step and clear when it ends. Forces and impulses apply to dynamic bodies only, and velocities to kinematic and dynamic ones: a static body never moves, so a call that would move it changes nothing and counts one misuse, and a static body def with a velocity is refused.

Linear and angular damping decay velocity as v / (1 + h d) per substep. Gravity scales per body. `m2Body_SetMotionLocks` holds a body still along world x, world y or in rotation, whatever pushes it; the rotation lock is the same as `m2Body_SetFixedRotation`, which also makes the inertia infinite.

`m2Body_SetTransform` teleports a body: it snaps, keeps its velocity, and wakes whatever it touched. A teleport is not a sweep, so nothing stops it from passing through a wall. To move a kinematic body smoothly, `m2Body_SetTargetTransform` sets the velocities that reach a pose in one step. `m2Body_SetType` changes a body's type in place, ending the contacts that no longer apply.

The frame helpers (`m2Body_GetWorldPoint`, `m2Body_GetLocalPoint`, the vector and point-velocity variants) do the pose math in f64.

## Sleep

Bodies that come to rest sleep in islands: everything connected by contacts or joints sleeps and wakes together. When every dynamic body sleeps and nothing kinematic moves, a step costs almost nothing and changes nothing. Everything that should wake a body does: impulses, forces, teleports, type, filter and mass changes, a gravity change, a destroyed neighbor, a broken joint, water, a conveyor that changes speed.

`m2Body_EnableSleep(world, body, false)` keeps one body awake for good, and `m2World_EnableSleeping(world, false)` wakes everyone and keeps them awake.

## Dominance

A body with a higher dominance (`m2Body_SetDominance`) cannot be pushed by a lower one: in a contact, the higher side acts as if it were static, and static bodies outrank everything. Enemies stop shoving the player, and the boss stands firm in an avalanche of crates. Dominance changes contacts only; joints stay symmetric.

## Dormant bodies and mass

`m2Body_Disable` takes a body out of the simulation without destroying it: its contacts end, whatever rested on it wakes, queries stop seeing it, and its joints wait. `m2Body_Enable` puts it back where it stood.

A dynamic body's mass comes from its shapes' densities. `m2Body_SetMassData` overrides it until a shape change, a fixed-rotation change or `m2Body_ApplyMassFromShapes` computes it from the shapes again.

## Shapes and materials

Circles, capsules, rounded polygons (convex, up to 8 vertices) and segments attach to a body in its own frame. A shape's friction pairs by geometric mean and its restitution by maximum. `m2Shape_SetTangentSpeed` makes a conveyor: the surface slides along the contact, and friction drags riders toward that speed (positive drives a rider toward +x on an upward-facing floor; a pair adds both speeds).

Shapes can change at run time: materials, filters, density, and the geometry itself (`m2Shape_SetCircle` and its siblings, a change of type included). The mass recomputes and touching partners wake.

## Filters and sensors

Two shapes collide when each one's category bits intersect the other's mask bits. A nonzero group index shared by both overrides that: positive always collides, negative never does. Two bodies joined by a joint do not collide unless the joint's `collideConnected` is set; the filter joint does nothing else.

A sensor detects overlaps and never pushes back, and never affects sleep. Its overlaps arrive as sensor events, and `m2Shape_GetSensorOverlaps` lists who is inside now.

## Chains

A chain is terrain bodies cannot snag on: a run of one-sided segments whose seams are smoothed by ghost points. Wind it so the solid side is on the right walking from one point to the next: for a floor, list the points right to left. An open chain uses its first and last points as ghosts and needs at least four. `m2CreateChain` returns one id for the whole run: `m2DestroyChain` removes every segment at once, and `m2Chain_SetFriction` and `m2Chain_SetRestitution` retune them all. A chain is the way to build one-way platforms.

## Geometry from points

`m2ComputeHull` turns a loose point cloud into a convex polygon, welding near points and refusing degenerate input. `m2DecomposeOutline` goes further: a simple counter-clockwise outline of up to 64 points, a sprite's silhouette, becomes convex pieces of at most 8 vertices that keep its area. A clockwise or self-intersecting outline is refused.

`m2World_ShatterBody` breaks a dynamic body into one new body per piece. Each piece carries the parent's velocity at its own center of mass and the material of the parent's first shape, and the whole break is one journaled operation, so it replays and rolls back like anything else. When the pools cannot seat every piece, it returns `m2_errorCapacity` and nothing changes.

```c
m2Vec2 outline[6] = {{-2.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f},
                     {0.5f, 1.0f},  {0.5f, 2.0f}, {-2.0f, 2.0f}};
m2Polygon pieces[8];
int32_t pieceCount = 0;
bool whole =
    m2DecomposeOutline(outline, 6, pieces, 8, &pieceCount) == m2_success && pieceCount <= 8;
for (int32_t i = 0; whole && i < pieceCount; ++i)
{
    m2ShapeId piece = m2_nullShapeId;
    whole = m2CreatePolygonShape(world, wall, &shapeDef, &pieces[i], &piece) == m2_success;
}

// Later, when the wall breaks:
m2BodyId fragments[8];
int32_t fragmentCount = 0;
m2Result broke =
    m2World_ShatterBody(world, wall, pieces, pieceCount, fragments, 8, &fragmentCount);
```

## Explosions

`m2World_Explode` is one blast: every dynamic shape within reach gets an outward impulse at its closest point, at full strength inside the radius and fading to zero across the falloff band, filtered by category. It is one journal operation.
