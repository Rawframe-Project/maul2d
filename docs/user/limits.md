# Limits

Every number the step multiplies has a range, in meters, kilograms and seconds. Each leaves realistic game values far inside it, and each is chosen against the products the solver forms in 32-bit floats. A value outside its range is refused, whether it arrives in a def, a setter or a journal tape. A def or setter refuses it with `m2_errorInvalid` (or, for a call that returns nothing, by changing nothing), and it counts one misuse wherever a bad argument counts one. The constants are in `maul2d/base.h`.

## The ranges

| Constant | Value | What it bounds | Realistic values |
|---|---|---|---|
| `M2_MAX_SPEED` | 1e5 m/s, rad/s | a body's velocity per axis and its spin, a conveyor's speed, a motor's speed, the wind, a fluid volume's flow, a particle's velocity | a rifle bullet is about 1e3 m/s; the solver caps motion at 400 m/s anyway |
| `M2_MAX_ACCELERATION` | 1e6 m/s^2 | gravity per axis | Earth is 9.8; a game's gravity rarely passes a few thousand |
| `M2_MAX_GRAVITY_SCALE` | 1e3 | a body's or the particles' gravity scale, either sign | 0 to a few |
| `M2_MAX_RATE` | 1e6 1/s | linear and angular damping, wind and fluid drag, a spring's hertz | damping 0 to 10, springs up to a few hundred hertz |
| `M2_MAX_COEFFICIENT` | 1e3 | friction, a damping ratio, a particle strength | friction 0 to 2, damping ratios 0 to 10 |
| restitution | 0 to 1 | a shape's or a chain's bounce | 0 to 1 |
| `M2_MAX_DENSITY` | 1e6 kg/m^2 | a shape's, a fluid volume's or the particles' density | foam about 30, water 1e3, lead about 1.1e4 |
| `M2_MIN_RATIO`, `M2_MAX_RATIO` | 1e-3 to 1e3 | a gear's or pulley's ratio, by magnitude | 0.01 to 100 |
| `M2_MIN_MASS`, `M2_MAX_MASS` | 1e-9 to 1e12 kg | a dynamic body's mass; a particle's mass | a 1 g bullet to a 1e8 kg ship |
| `M2_MIN_INERTIA`, `M2_MAX_INERTIA` | 1e-12 to 1e24 kg m^2, or 0 | a dynamic body's rotational inertia about its center of mass; 0 does not rotate | a 5 mm disc at density 1 is about 1e-9 |
| `M2_MAX_FORCE` | 1e20 N, N m | a force or torque per axis, a body's accumulated load, a motor's or mouse's budget | a rocket engine is about 1e7 N |
| `M2_IMPULSE_LIMIT` | 1e15 N s | an impulse per axis that one call applies, and every impulse a joint row or contact point accumulates | a 1e8 kg ship stopping from 100 m/s takes 1e10 |
| `M2_MIN_TIME_STEP`, `M2_MAX_TIME_STEP` | 1e-6 to 10 s | `m2World_Step`'s and `m2Body_SetTargetTransform`'s time step | 1/240 to 1/20 |
| `M2_MAX_ANGLE` | 1e5 rad | a joint's angle limits, angular offset, a ratchet's tooth and phase | within a few turns |
| `M2_MIN_RATCHET_TOOTH` | 1e-3 rad | a ratchet's tooth, by magnitude | 2 pi / 12 to 2 pi / 360 |
| `M2_JOINT_REACH` | 1e9 m | a joint's length, translations and limits; the farthest a joint takes its bodies apart | a rope of a few hundred meters |
| `M2_HUGE` | 1e5 m | a body-local length: shape coordinates and radii, joint anchors, an explosion's radius and falloff | a level a few kilometers wide |

World positions are 64-bit and not bounded. A joint takes bodies farther apart than `M2_JOINT_REACH` along an axis as that far apart, and so does an impulse or force applied at a far point. At the solver's 400 m/s cap, closing 1e9 m takes a month.

## Clamped and saturated values

Three kinds of value are held to their ranges instead of refused:

- **Mass from shapes.** A dynamic body's mass and inertia summed from its shapes are clamped into the mass and inertia ranges, the same way its center is clamped to `M2_HUGE`. `m2Body_GetMassData` reads back what the body kept. Within the ranges the clamp changes no bit. The everyday shapes that come nearest are specks: a 5 mm disc (the smallest circle) at density 1e-3, or a 1 cm square at density 1e-3, sits at about 1e-12 kg m^2, the inertia floor. A 100 km disc at the largest density weighs 3e16 kg and is clamped down to `M2_MAX_MASS`.
- **Accumulated motion.** A body's velocity saturates at `M2_MAX_SPEED` per axis after an impulse, an explosion or a shatter, and its accumulated force and torque saturate at `M2_MAX_FORCE` however many calls add up. A single call past its range is still refused.
- **The solver's own values.** Inside the step, every velocity saturates at `M2_MAX_SPEED` after every push (each joint row, point pair and contact row, the integration, and every particle pass), so does every particle's velocity, and every impulse a joint row or contact point accumulates saturates at `M2_IMPULSE_LIMIT`, alongside the speed bound the rows already had. Water and wind loads saturate at `M2_MAX_FORCE`. A coupled pair of rows whose effective-mass term passes 1e19 is solved one row at a time, so the 2x2 solve never multiplies two such terms. A limit's gap is held within four reaches before it becomes a speed, which keeps the distance joint's open-ended maximum finite. None of these is reached by a realistic scene: within the ranges they change no bit.

## Why these values

The bounds are chosen against `FLT_MAX`, about 3.4e38. The tightest product among them is the push a joint row gives a body's spin: inverse inertia times impulse times lever. At the smallest inertia, twice the impulse limit (the change between two impulses within it) and the longest joint lever, that is 1e12 x 2e15 x 1e9 = 2e36, two orders of magnitude below `FLT_MAX`. Making `M2_MIN_INERTIA` smaller or `M2_JOINT_REACH` larger would have to make the impulse limit smaller to match. The other bounds leave more room. With the saturation above, these are the largest values each kind of product can reach, at the worst corner of the ranges:

| Product | Worst case | Bound |
|---|---|---|
| a row's speed J v | speed times the longest joint lever, both bodies | 2 x 1e5 x 1e9 = 2e14 |
| a row's effective-mass term k | inverse inertia times the longest lever squared | 2 x 1e12 x (1e9)^2 = 2e30 |
| a 2x2 solve's k11 k22 | both terms at the fallback threshold | 1e19 x 1e19 = 1e38 |
| a line row's impulse before its clamp | effective mass 1e12 times a limit gap of 4e9 m over the shortest substep (6.4e7 1/s) | 1e12 x 2.6e17 = 2.6e29 |
| a turn row's impulse before its clamp | effective mass 1e24 times an angle gap of 1e5 rad over the shortest substep | 1e24 x 6.4e12 = 6.4e36 |
| a gear row against a body that cannot turn | effective mass 1e24 / r^2 times its speed, r = 1e-3 | 1e24 x 1e5 x 1e6 = 1e35 |
| a push to a velocity, along a line | inverse mass times twice the impulse limit times the largest ratio | 1e9 x 2e15 x 1e3 = 2e27 |
| a push to a spin | inverse inertia times twice the impulse limit times the longest lever | 1e12 x 2e15 x 1e9 = 2e36 |
| a friction bound | friction times the impulse limit | 1e3 x 1e15 = 1e18 |
| a contact's push target | the largest separation over the shortest substep, times its mass | 2.9e5 x 6.4e7 x 1e12 = 1.9e25 |
| a force over a substep | longest substep, lightest body, largest force | 10 x 1e9 x 1e20 = 1e30 |
| a torque over a substep | longest substep, smallest inertia, largest torque | 10 x 1e12 x 1e20 = 1e33 |
| water's spin drag | squared radius of gyration (held to 4e10) times area, spin and drag | 4e10 x 4e10 x 1e5 x 1e6 = 1.6e32 |

`test/test_extremes.c` builds scenes at these corners (the lightest and heaviest bodies, the longest levers, the stiffest springs, ratios at both ends, water and wind at their strongest, particles at every strength) and steps them with overflow, invalid operations and division by zero trapping, so a single product past the float range would stop it.
