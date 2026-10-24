# Joints

A joint connects two bodies. It joins their sleep island, wakes both when it is created or tuned, and goes when either body is destroyed. Its two bodies do not collide with each other unless the def's `collideConnected` is set, which is what ragdolls expect.

## The core joints

- **Distance** keeps two anchors at a length. With `hertz` above 0 it is a spring. A hard range (`minLength`, `maxLength`) clamps whatever the spring does, so a soft rope still stops hard; with `enableSpring` and hertz 0 the joint is a slack rope (free until taut) or, with a minimum, a strut.
- **Revolute** pins two anchors together: a hinge, with an optional motor, angle limits, and an angular spring toward its creation angle (`springHertz`; 0 is off).
- **Prismatic** lets body B slide along an axis of body A, with a motor and travel limits; the relative rotation stays fixed.
- **Weld** holds two bodies as one piece. Its linear and angular rows soften separately; 0 hertz is rigid.
- **Wheel** is a suspension: body B slides on a sprung axis and spins freely, and the motor drives the spin.

Motors and limits belong to the revolute, prismatic and wheel joints. `m2Joint_SetMotorSpeed`, `m2Joint_SetMaxMotor`, `m2Joint_EnableMotor`, `m2Joint_EnableLimit` and `m2Joint_SetLimits` tune them at run time; on another type they change nothing and count one misuse. Angles are measured from the pose at creation.

## The utility joints

- **Motor** drives body B's pose toward an offset from body A within a force and a torque budget. A moving platform calls `m2MotorJoint_SetOffsets` every frame and the physics stays correct; `m2Joint_SetMaxMotor` sets its torque budget.
- **Mouse** is a soft spring pulling a point of body B toward a world target (`m2MouseJoint_SetTarget`): dragging, deterministic and journaled.
- **Filter** has no constraint: it switches collision off between its two bodies while it lives. Destroying it turns collision back on, with begin events where the bodies overlap.
- **Gear** couples two spins at a ratio, positive for meshed cogs and negative for belts. Its phase is exact across any number of turns. Pin each body with its own joint; the gear only owns rotation.
- **Pulley** runs a rope over two fixed world points. Length A plus ratio times length B stays at the total measured at creation, so there is no length to set wrong, and the B side feels ratio times the A-side tension (a ratio-2 hoist lifts twice the mass). Changing the ratio measures the total again from the current pose, so the machine does not snap. A side that unspools fully goes limp near its anchor instead of pushing.
- **Ratchet** lets relative rotation run free in one direction, clicking tooth by tooth, and never gives back more than the last engaged tooth: socket wrenches, winches, turnstiles. It owns rotation only and stays exact across any number of turns.

## Tuning and reading back

Every parameter of a def can be read back and most can be set at run time; each change is journaled. `m2Joint_SetSpringHertz` and `m2Joint_SetSpringDampingRatio` soften a joint's main rows (the distance spring, the wheel suspension, the weld's linear row, the mouse spring); the angular variants tune the revolute spring and the weld's angular row. A call made for one type, such as `m2DistanceJoint_SetLength` on a weld, changes nothing and counts one misuse.

`m2Joint_GetLinearSeparation` and `m2Joint_GetAngularSeparation` say how far a joint is from what it holds right now, which shows a rig being pulled apart. A joint takes bodies more than `M2_JOINT_REACH` (1e9 m) apart along an axis as that far apart: world positions are 64-bit and unbounded, but the solver works in 32-bit floats, and the clamp keeps it finite when a teleport sends one body to the far end of the world.

## Breaking

`m2Joint_SetBreakLimits` sets the force and torque past which a joint breaks during a step. A broken joint is destroyed and reported in `m2World_GetJointEvents` with the load that broke it; two worlds given the same calls break the same joints on the same step.

`m2Joint_GetReactionForce` and `m2Joint_GetReactionTorque` report the load a joint carried on the last step. They are the numbers the break check compares, bit for bit, so a threshold tuned against a reading behaves exactly as read.
