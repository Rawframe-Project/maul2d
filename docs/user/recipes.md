# Recipes

Common builds, each a few calls on top of the other pages.

**A top-down world.** Set the world def's gravity to zero, give bodies linear and angular damping in their defs for the drag a top-down game wants, and drive characters with the mover kit or with velocities set each step.

**A platformer character.** The character is a capsule the game moves, not a body: each frame, `m2World_CollideMover` gathers the planes, `m2SolveMover` resolves the wished move, and `m2ClipMoverVelocity` spends the velocity ([Events and queries](events-and-queries.md#the-character-mover)). Jump by setting the vertical velocity, and let one-sided chains be the one-way platforms. The testbed's platformer is this recipe end to end.

**A vehicle.** A chassis body and two wheel bodies, each joined with a wheel joint, whose suspension spring is built in. Enable the wheels' motors and drive the motor speed from input. `sample_car` is the two-wheel version.

**Destructible geometry.** Run a sprite's outline through `m2DecomposeOutline` once to get convex pieces, and build the body from them. When it should break, call `m2Body_Shatter` with the same pieces: the body becomes one fragment per piece, each moving with the parent. An `m2World_Explode` at the impact point gives the shove ([Bodies and shapes](bodies-and-shapes.md#geometry-from-points)).

**Water without particles.** Place a fluid volume over the region and set its surface: light bodies float and dense ones sink. Give it a flow for a current, and move the surface for a tide. Reach for a particle pool only when the splash matters ([Fluids](fluids.md)).

**Lockstep multiplayer.** Every peer runs the same world from the same def and applies the same inputs at the same step. Exchange inputs, not state; compare `m2World_Hash` now and then, and on a mismatch compare `m2World_GetHashParts` to find the subsystem ([Rollback and replay](rollback-and-replay.md)).
