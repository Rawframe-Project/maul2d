# Limits

Every number the step multiplies has a range, in meters, kilograms and seconds. The ranges keep every product the solver forms finite in 32-bit floats, and each leaves realistic game values far inside it. A value outside its range is refused, whether it arrives in a def, a setter, a journal tape or a snapshot. A def or setter refuses it with `m2_errorInvalid` (or, for a call that returns nothing, by changing nothing), and it counts one misuse wherever a bad argument counts one. The constants are in `maul2d/base.h`.

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
| `M2_IMPULSE_LIMIT` | 1e15 N s | an impulse per axis | a 1e8 kg ship stopping from 100 m/s takes 1e10 |
| `M2_MIN_TIME_STEP`, `M2_MAX_TIME_STEP` | 1e-6 to 10 s | `m2World_Step`'s and `m2Body_SetTargetTransform`'s time step | 1/240 to 1/20 |
| `M2_MAX_ANGLE` | 1e5 rad | a joint's angle limits, angular offset, a ratchet's tooth and phase | within a few turns |
| `M2_MIN_RATCHET_TOOTH` | 1e-3 rad | a ratchet's tooth, by magnitude | 2 pi / 12 to 2 pi / 360 |
| `M2_JOINT_REACH` | 1e9 m | a joint's length, translations and limits; the farthest a joint takes its bodies apart | a rope of a few hundred meters |
| `M2_HUGE` | 1e5 m | a body-local length: shape coordinates and radii, joint anchors, an explosion's radius and falloff | a level a few kilometers wide |

World positions are 64-bit and not bounded. A joint takes bodies farther apart than `M2_JOINT_REACH` along an axis as that far apart, and so does an impulse or force applied at a far point. At the solver's 400 m/s cap, closing 1e9 m takes a month.

## Clamped and saturated values

Two kinds of value are held to their ranges instead of refused:

- **Mass from shapes.** A dynamic body's mass and inertia summed from its shapes are clamped into the mass and inertia ranges, the same way its center is clamped to `M2_HUGE`. `m2Body_GetMassData` reads back what the body kept. Within the ranges the clamp changes no bit. The everyday shapes that come nearest are specks: a 5 mm disc (the smallest circle) at density 1e-3, or a 1 cm square at density 1e-3, sits at about 1e-12 kg m^2, the inertia floor. A 100 km disc at the largest density weighs 3e16 kg and is clamped down to `M2_MAX_MASS`.
- **Accumulated motion.** A body's velocity saturates at `M2_MAX_SPEED` per axis after an impulse, an explosion or a shatter, and its accumulated force and torque saturate at `M2_MAX_FORCE` however many calls add up. A single call past its range is still refused.

## Why these values

The bounds are budgeted against `FLT_MAX`, about 3.4e38. The tightest product is the push a joint row gives a body's spin: inverse inertia times impulse times lever. At the smallest inertia, the largest impulse and the longest joint lever that is at most 1e12 x 2e15 x 1e9 = 2e36, two orders of magnitude below `FLT_MAX`. Making `M2_MIN_INERTIA` smaller or `M2_JOINT_REACH` larger would have to make the impulse limit smaller to match. The other bounds leave more room. A row's effective mass is at most `M2_MAX_MASS` along a line and `M2_MAX_INERTIA` about a turn, and its speed at most `M2_MAX_SPEED` times its lever, so a row's impulse before its clamp stays below about 1e36. Gravity, forces and drag over the longest substep add at most 1e30 to a velocity before the speed saturates.
