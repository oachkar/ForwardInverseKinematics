# Two-Link Kinematics — React recreation

This project recreates the core interaction model of the Wolfram Demonstration
"Forward and Inverse Kinematics for Two-Link Arm" in React + TypeScript + SVG.

## Run it

```bash
npm install
npm run dev
```

Then open the local Vite URL.

## Implemented

- Two-link planar revolute-revolute arm
- `r1 + r2 = 2` length constraint
- Forward kinematics from `theta1`, `theta2`
- Inverse kinematics from draggable target
- Elbow-up / elbow-down solutions
- Reachable annular workspace
- Jacobian
- Forward velocity kinematics: `v = J qdot`
- Inverse velocity kinematics
- Moore–Penrose-style damped pseudoinverse fallback at singularity
- Unit joint-velocity disc
- Manipulability ellipse
- Interactive SVG dragging

## Mathematics

Position:

x = r1 cos(theta1) + r2 cos(theta1 + theta2)

y = r1 sin(theta1) + r2 sin(theta1 + theta2)

Jacobian:

[ -r1 sin(theta1)-r2 sin(theta1+theta2), -r2 sin(theta1+theta2) ]
[  r1 cos(theta1)+r2 cos(theta1+theta2),  r2 cos(theta1+theta2) ]

Inverse kinematics:

D = (x²+y²-r1²-r2²)/(2 r1 r2)

theta2 = atan2(±sqrt(1-D²), D)

theta1 = atan2(y,x) - atan2(r2 sin(theta2), r1+r2 cos(theta2))

The original Wolfram demonstration describes its velocity mapping as
`xi = J(q) q'` and says inverse velocity uses the Moore–Penrose pseudoinverse.


## Iteration 2 fixes from evaluation feedback

The second version addresses the faults found during testing:

1. Inverse-mode angle controls now mirror the actual IK solution shown by the robot and are disabled while IK owns the angles.
2. An unreachable target displays `No solution` instead of fabricated 0°/0° inverse angles.
3. Joint/end-effector labels use adaptive offsets and are hidden when a link is too short for the label to remain useful.
4. Forward and inverse velocity inputs are now separate: forward mode edits q̇, while inverse mode edits desired Cartesian velocity vᵈ. The blue graph arrow is the actual selected input, and the pink arrow is the resulting Cartesian velocity v = Jq̇.
5. Robot links no longer use SVG drop-shadow filters, eliminating the clipping/disappearing artifact near horizontal and vertical poses.
6. The actual end-effector has a dedicated transparent hit area, so it can be dragged directly rather than only through an obscured target marker.
7. In inverse velocity mode the UI reports `‖v − vᵈ‖`, making it straightforward to verify whether the pseudoinverse result actually realizes the requested Cartesian velocity.
