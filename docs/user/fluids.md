# Fluids

Maul2D has two kinds of water. A particle pool simulates liquid, sand and jelly particle by particle; a water volume is a region of buoyant water with no particles at all. Both are world state: snapshots, the journal and the hash cover them like everything else.

## Particle pools

A world has particles only if its def asks for them: `particleCapacity` above 0 builds the pool for the world's whole life, with its physics settings (radius, density, gravity scale, and the pressure, damping, viscous, powder, spring, elastic, cohesion and near-pressure strengths) fixed in the def. A fixed pool keeps every snapshot the same size, so a rollback can land anywhere in history.

A particle is created from a def, like every other object: `m2CreateParticle` creates one from an `m2ParticleDef` (position, velocity and flags, from `m2DefaultParticleDef`). A full pool, a pool of 0 included, refuses with `m2_errorCapacity`, counting no misuse, and counts in `m2Counters.particleCreatesRefused`; pace emitters off `m2World_GetParticleCount`. `m2CreateParticlesInPolygon` pours a whole convex region in one call, row by row on the rest stride, with every particle taking the def's velocity and flags and the polygon placed at its position. A full pool ends the fill early: the particles that fit stay, and the fill returns `m2_errorCapacity` with their count, which may be 0, counting no misuse. A fill never comes up short without saying so. `m2World_GetParticles` and `m2World_OverlapParticlesAabb` read particles back in slot order with true totals; a circular region is one distance check away.

```c
m2ParticleDef dropDef = m2DefaultParticleDef();
dropDef.position = (m2Pos2){0.0, 2.0};
dropDef.velocity = (m2Vec2){0.0f, -1.0f};
dropDef.flags = m2_tensileParticle;
m2ParticleId drop = m2_nullParticleId;
m2Result created = m2CreateParticle(world, &dropDef, &drop);
if (created == m2_success)
{
    m2Particle_SetLifetime(world, drop, 0.5f); // gone half a second from now
}
bool stepped = true;
for (int32_t i = 0; stepped && i < 60; ++i)
{
    stepped = m2World_Step(world, 1.0f / 60.0f, 4) == m2_success;
}
bool stillThere = m2Particle_IsValid(world, drop); // false: it expired
```

## Behavior

Flags set at creation choose the behavior. Plain water has none. Tensile particles pull their neighbors together below rest density and hold them apart with a near pressure, so spray beads into droplets. Viscous particles drag their neighbors, like honey. Powder grains repel when packed tighter than the rest stride and never cohere, like sand.

The spring and elastic flags turn a fill into a body. Springs remember their lengths at the fill and elastic triads their shapes; the whole batch is one journal operation, rides every snapshot, and its nets die with their particles. A pool's particle pass runs once per step, so a small firm blob wants higher strengths and a large soft mass wants the defaults.

## Particles die on their own

`m2Particle_SetLifetime` gives a particle a countdown; it is destroyed at the end of the step it reaches zero. No event reports it, so a program may hold the id of a particle that is gone, and that is normal: every particle call on a dead id answers zero, changes nothing and counts no misuse, and `m2DestroyParticle` returns `m2_errorStale`. `m2Particle_IsValid` tells which ids are still alive.

## How water meets the world

Sensors are invisible to particles, and one-way chains hold them only on their solid side. Bodies push particles and particles push back: light bodies float, dense ones sink, and anything the water touches wakes up. A world with live particles never sleeps as a whole.

Two speed facts are part of the contract. No particle moves more than one diameter per step, which keeps neighbors findable. The neighbor and body-contact builds run on the task hooks when the pool is large enough to gain from it (a few thousand particles), and any worker count gives the same bits; the force passes stay serial, because each reads the velocities the one before it left. A pool of fifteen hundred particles steps in well under a millisecond either way.

## Water volumes

Where a pool would be more than a scene needs, a water volume is the cheap answer: an axis-aligned region and a horizontal surface (`m2CreateWaterVolume`). The region needs area: a box flat or inverted on either axis could never hold a body and is refused. Volumes have their own pool, sized by the world def's `waterVolumeCapacity`; when it is full, or sized 0, the call refuses with `m2_errorCapacity`, counting no misuse. Every awake dynamic body whose origin lies in the region feels buoyancy on the part of each shape below the surface (exact for circles and polygons, a bounding-box fraction for capsules and segments), linear and angular drag, and an optional flow current. Light bodies float, dense ones sink, and a flow carries what drifts in. `m2WaterVolume_SetSurface` moves the waterline for a rising tide or a draining tank. The forces go into the ordinary accumulators, so a body that settles at the surface simply sleeps there.

## Wind

`m2World_SetWind` sets an air velocity and a drag coefficient for the whole world: every awake dynamic body is pulled toward the wind's speed in proportion to its area. Unlike gravity, wind wakes no sleeper, because wind is meant to gust. Regional wind is a water volume of density 0 with a flow.
