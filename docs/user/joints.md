# Joints

A joint connects two bodies. It joins their sleep island, wakes both when it is created or tuned, and goes when either body is destroyed. Its two bodies do not collide with each other unless the def's `enableConnectedCollision` is set, which is what ragdolls expect.

## The core joints

- **Distance** keeps two anchors at a length: a rigid rod, or with `enableSpring` a spring of `springHertz`. A hard range (`minLength`, `maxLength`) clamps whatever the spring does, so a soft rope still stops hard; with `enableSpring` and a spring hertz of 0 the joint is a slack rope (free until taut) or, with a minimum, a strut.
- **Revolute** pins two anchors together: a hinge, with an optional motor, angle limits, and, with `enableSpring`, an angular spring of `springHertz` toward its creation angle.
- **Prismatic** lets body B slide along an axis of body A, with a motor and travel limits; the relative rotation stays fixed.
- **Weld** holds two bodies as one piece. Its linear and angular rows soften separately into springs (`linearSpringHertz`, `angularSpringHertz`); 0 hertz is rigid. A rigid weld solves its point and its angle together, so it holds a long arm against a small inertia as firmly as a revolute joint does.
- **Wheel** is a suspension: body B slides on a sprung axis and spins freely, and the motor drives the spin.

Motors and limits belong to the revolute, prismatic and wheel joints. `m2Joint_SetMotorSpeed`, `m2Joint_SetMaxMotor`, `m2Joint_EnableMotor`, `m2Joint_EnableLimit` and `m2Joint_SetLimits` tune them at run time; on another type they change nothing and count one misuse. Angles are measured from the pose at creation.

## The utility joints

- **Motor** drives body B's pose toward an offset from body A within a force and a torque budget. A moving platform calls `m2MotorJoint_SetOffsets` every frame and the physics stays correct; `m2Joint_SetMaxMotor` sets its torque budget.
- **Mouse** is a soft spring pulling a point of body B toward a world target (`m2MouseJoint_SetTarget`): dragging, deterministic and journaled. The spring has B's own mass, so it pulls equally hard in every direction, whichever point of B it holds.
- **Filter** has no constraint: it switches collision off between its two bodies while it lives. Destroying it turns collision back on, with begin events where the bodies overlap.
- **Gear** couples two spins at a ratio, positive for meshed cogs and negative for belts. Its phase is exact across any number of turns. Pin each body with its own joint; the gear only owns rotation.
- **Pulley** runs a rope over two fixed world points. Length A plus ratio times length B stays at the total measured at creation, so there is no length to set wrong, and the B side feels ratio times the A-side tension (a ratio-2 hoist lifts twice the mass). Changing the ratio measures the total again from the current pose, so the machine does not snap. A side that unspools fully goes limp near its anchor instead of pushing.
- **Ratchet** lets relative rotation run free in one direction, clicking tooth by tooth, and never gives back more than the last engaged tooth: socket wrenches, winches, turnstiles. It owns rotation only and stays exact across any number of turns.

## Tuning and reading back

Every parameter of a def can be read back and most can be set at run time; each change is journaled. A joint's stiffness comes in two words. A spring (`springHertz`, `springDampingRatio`) pulls toward a rest pose: the distance spring, the wheel suspension, the revolute spring, the motor joint's drive and the mouse joint's drag, tuned by `m2Joint_SetSpringHertz` and `m2Joint_SetSpringDampingRatio`, with a weld's two springs under `m2WeldJoint_`. A constraint softness (`constraintHertz`, `constraintDampingRatio`) loosens the rows that hold a revolute, prismatic, gear, pulley or ratchet joint together, through `m2Joint_SetConstraintHertz` and `m2Joint_SetConstraintDampingRatio`; 0 hertz is the stiff default. `m2Joint_EnableSpring` turns the distance, wheel or revolute spring on and off, and `m2Joint_EnableConnectedCollision` lets a joint's bodies collide or keeps them apart. A call made for one type, such as `m2DistanceJoint_SetLength` on a weld, changes nothing and counts one misuse.

`m2Joint_GetLinearSeparation` and `m2Joint_GetAngularSeparation` say how far a joint is from what it holds right now, which shows a rig being pulled apart. A joint takes bodies more than `M2_REACH_LIMIT` (1e9 m) apart along an axis as that far apart: world positions are 64-bit and unbounded, but the solver works in 32-bit floats, and the clamp keeps it finite when a teleport sends one body to the far end of the world.

## Breaking

`m2Joint_SetBreakLimits` sets the force and torque past which a joint breaks during a step. A broken joint is destroyed and reported in `m2World_GetJointEvents` with the load that broke it; two worlds given the same calls break the same joints on the same step.

`m2Joint_GetReactionForce` and `m2Joint_GetReactionTorque` report the load a joint carried on the last step. They are the numbers the break check compares, bit for bit, so a threshold tuned against a reading behaves exactly as read.
