import {
  type Point,
  fk,
  ik,
  jacobian,
  jointSpeed,
  manipulabilityEllipse,
} from "./kinematics";
import { useMemo, useRef, useState, type PointerEvent as ReactPointerEvent } from "react";

type Props = {
  l1: number;
  l2: number;
  theta1: number;
  theta2: number;
  elbowUp: boolean;
  target: Point;
  velocityMode: "forward" | "inverse";
  qdot: Point;
  desiredEndVelocity: Point | null;
  actualEndVelocity: Point;
  onTargetChange: (p: Point) => void;
};

const VIEW = 520;
const WORLD = 2.35;

function worldToSvg(p: Point) {
  return {
    x: VIEW / 2 + (p.x / WORLD) * (VIEW / 2),
    y: VIEW / 2 - (p.y / WORLD) * (VIEW / 2),
  };
}

function svgToWorld(x: number, y: number): Point {
  return {
    x: ((x - VIEW / 2) / (VIEW / 2)) * WORLD,
    y: -((y - VIEW / 2) / (VIEW / 2)) * WORLD,
  };
}

function pointToString(p: Point) {
  const s = worldToSvg(p);
  return `${s.x},${s.y}`;
}

function labelOffset(from: Point, toward: Point, distance: number) {
  const dx = toward.x - from.x;
  const dy = toward.y - from.y;
  const length = Math.hypot(dx, dy) || 1;
  return {
    x: (-dy / length) * distance,
    y: (dx / length) * distance,
  };
}

export default function ArmPlot({
  l1,
  l2,
  theta1,
  theta2,
  elbowUp,
  target,
  velocityMode,
  qdot,
  desiredEndVelocity,
  actualEndVelocity,
  onTargetChange,
}: Props) {
  const svgRef = useRef<SVGSVGElement | null>(null);
  const [dragging, setDragging] = useState(false);

  const pose = useMemo(
    () => fk(theta1, theta2, l1, l2),
    [theta1, theta2, l1, l2]
  );

  const ikResult = useMemo(
    () => ik(target, l1, l2, elbowUp),
    [target, l1, l2, elbowUp]
  );

  const ellipse = useMemo(
    () => manipulabilityEllipse(theta1, theta2, l1, l2),
    [theta1, theta2, l1, l2]
  );

  const maxReach = l1 + l2;
  const minReach = Math.abs(l1 - l2);
  const rho = Math.hypot(target.x, target.y);
  const targetInside =
    rho >= minReach - 1e-8 && rho <= maxReach + 1e-8;

  function updateTargetFromPointer(e: ReactPointerEvent<SVGSVGElement>) {
    if (!dragging || !svgRef.current) return;

    const rect = svgRef.current.getBoundingClientRect();
    const sx = ((e.clientX - rect.left) / rect.width) * VIEW;
    const sy = ((e.clientY - rect.top) / rect.height) * VIEW;
    const p = svgToWorld(sx, sy);

    onTargetChange({
      x: Math.max(-WORLD, Math.min(WORLD, p.x)),
      y: Math.max(-WORLD, Math.min(WORLD, p.y)),
    });
  }

  function startTargetDrag(e: ReactPointerEvent<SVGElement>) {
    e.stopPropagation();
    e.currentTarget.setPointerCapture(e.pointerId);
    setDragging(true);
  }

  function stopTargetDrag() {
    setDragging(false);
  }

  const base = worldToSvg({ x: 0, y: 0 });
  const elbow = worldToSvg(pose.elbow);
  const end = worldToSvg(pose.end);
  const targetSvg = worldToSvg(target);

  // In forward mode the blue vector represents qdot, drawn from the base.
  // In inverse mode it represents the desired Cartesian velocity instead.
  const inputVector = velocityMode === "inverse" ? desiredEndVelocity : qdot;
  const inputScale = velocityMode === "inverse" ? 52 : 32;
  const inputTip = {
    x: base.x + (inputVector?.x ?? 0) * inputScale,
    y: base.y - (inputVector?.y ?? 0) * inputScale,
  };

  const actualTip = {
    x: end.x + actualEndVelocity.x * 45,
    y: end.y - actualEndVelocity.y * 45,
  };

  const ellipsePoints = ellipse.map(pointToString).join(" ");

  const jointLabelOffset = labelOffset(base, pose.elbow, 26);
  const endLabelOffset = labelOffset(pose.elbow, pose.end, 26);

  const showJointLabel = l1 >= 0.35;
  const showEndLabel = l2 >= 0.35;

  return (
    <div className="plot-shell">
      <div className="plot-title">
        <span>Workspace / Velocity</span>
        <span className={targetInside ? "status-pill ok" : "status-pill bad"}>
          {targetInside ? "target reachable" : "target unreachable"}
        </span>
      </div>

      <svg
        ref={svgRef}
        className="arm-svg"
        viewBox={`0 0 ${VIEW} ${VIEW}`}
        onPointerMove={updateTargetFromPointer}
        onPointerUp={stopTargetDrag}
        onPointerCancel={stopTargetDrag}
      >
        <defs>
          <radialGradient id="workspaceFill">
            <stop offset="0%" stopColor="#dcecff" />
            <stop offset="100%" stopColor="#bcd7ff" />
          </radialGradient>
          <marker
            id="arrowGreen"
            viewBox="0 0 10 10"
            refX="8"
            refY="5"
            markerWidth="6"
            markerHeight="6"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill="#11a64a" />
          </marker>
          <marker
            id="arrowPink"
            viewBox="0 0 10 10"
            refX="8"
            refY="5"
            markerWidth="6"
            markerHeight="6"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill="#d34f8a" />
          </marker>
          <marker
            id="arrowBlue"
            viewBox="0 0 10 10"
            refX="8"
            refY="5"
            markerWidth="6"
            markerHeight="6"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill="#405fdb" />
          </marker>
        </defs>

        <rect width={VIEW} height={VIEW} fill="#fafafa" />

        {/* Workspace annulus */}
        <circle
          cx={base.x}
          cy={base.y}
          r={(maxReach / WORLD) * (VIEW / 2)}
          fill="url(#workspaceFill)"
          stroke="#8fb2de"
          strokeWidth="1.5"
        />
        {minReach > 0 && (
          <circle
            cx={base.x}
            cy={base.y}
            r={(minReach / WORLD) * (VIEW / 2)}
            fill="#fafafa"
            stroke="#d2d9e2"
            strokeWidth="1.5"
          />
        )}

        {/* Axes */}
        <line x1="0" x2={VIEW} y1={base.y} y2={base.y} className="axis" />
        <line x1={base.x} x2={base.x} y1="0" y2={VIEW} className="axis" />
        <text x={VIEW - 16} y={base.y - 8} className="axis-label">
          x
        </text>
        <text x={base.x + 8} y={18} className="axis-label">
          y
        </text>

        <text x={base.x + 8} y={base.y - 10} className="workspace-label">
          Workspace
        </text>

        {/* Manipulability ellipse */}
        <polyline points={ellipsePoints} className="ellipse" />

        {/* Target: large invisible hit area means the end joint remains easy to grab. */}
        <circle
          cx={targetSvg.x}
          cy={targetSvg.y}
          r="18"
          className="target-hit-area"
          onPointerDown={startTargetDrag}
        />
        <circle
          cx={targetSvg.x}
          cy={targetSvg.y}
          r="11"
          className="target-hit"
          fill="#11a64a"
          pointerEvents="none"
        />
        <line
          x1={targetSvg.x - 9}
          x2={targetSvg.x + 9}
          y1={targetSvg.y}
          y2={targetSvg.y}
          className="target-crosshair"
          pointerEvents="none"
        />
        <line
          x1={targetSvg.x}
          x2={targetSvg.x}
          y1={targetSvg.y - 9}
          y2={targetSvg.y + 9}
          className="target-crosshair"
          pointerEvents="none"
        />

        {/* Robot links: no SVG filter, so links are not clipped at axes/extremes. */}
        <line
          x1={base.x}
          y1={base.y}
          x2={elbow.x}
          y2={elbow.y}
          className="link link-outline"
        />
        <line
          x1={base.x}
          y1={base.y}
          x2={elbow.x}
          y2={elbow.y}
          className="link link-1"
        />
        <line
          x1={elbow.x}
          y1={elbow.y}
          x2={end.x}
          y2={end.y}
          className="link link-outline"
        />
        <line
          x1={elbow.x}
          y1={elbow.y}
          x2={end.x}
          y2={end.y}
          className="link link-2"
        />

        <circle cx={base.x} cy={base.y} r="15" className="joint-base" />
        <circle cx={elbow.x} cy={elbow.y} r="13" className="joint" />

        {/* End-effector itself is the primary target drag affordance. */}
        <circle
          cx={end.x}
          cy={end.y}
          r="15"
          className="end-effector-hit"
          onPointerDown={startTargetDrag}
        />
        <circle cx={end.x} cy={end.y} r="10" className="end-effector" pointerEvents="none" />

        {/* End-effector orientation */}
        <line
          x1={end.x}
          y1={end.y}
          x2={end.x + 38 * Math.cos(theta1 + theta2)}
          y2={end.y - 38 * Math.sin(theta1 + theta2)}
          className="orientation-arrow"
          markerEnd="url(#arrowGreen)"
        />

        {/* Blue input vector */}
        <line
          x1={base.x}
          y1={base.y}
          x2={inputTip.x}
          y2={inputTip.y}
          className="velocity-input-arrow"
          markerEnd="url(#arrowBlue)"
        />

        {/* Pink actual Cartesian velocity at the end effector */}
        <line
          x1={end.x}
          y1={end.y}
          x2={actualTip.x}
          y2={actualTip.y}
          className="velocity-arrow"
          markerEnd="url(#arrowPink)"
        />

        <g transform="translate(18, 18)">
          <rect width="205" height="90" rx="8" className="legend-bg" />
          <text x="12" y="21" className="legend-title">
            Velocity mapping
          </text>
          <text x="12" y="40" className="legend-text">
            {velocityMode === "inverse" ? "blue = desired v" : "blue = q̇ input"}
          </text>
          <text x="12" y="58" className="legend-text">
            pink = actual v = J q̇
          </text>
          <text x="12" y="77" className="legend-text">
            |q̇| = {jointSpeed(qdot).toFixed(2)}
          </text>
        </g>

        {showJointLabel && (
          <g className="plot-labels">
            <text x={elbow.x + jointLabelOffset.x} y={elbow.y + jointLabelOffset.y}>
              joint 2
            </text>
          </g>
        )}

        {showEndLabel && (
          <g className="plot-labels">
            <text x={end.x + endLabelOffset.x} y={end.y + endLabelOffset.y}>
              end effector
            </text>
          </g>
        )}

        <g className="plot-labels">
          <text x={base.x + 16} y={base.y + 26}>
            base
          </text>
        </g>
      </svg>

      {!targetInside && (
        <div className="plot-warning">
          The target is outside the annular workspace. Drag the green locator
          or the end effector into the blue region.
        </div>
      )}

      {velocityMode === "inverse" && ikResult.reachable && (
        <div className="plot-note">
          Inverse velocity computes q̇ from the desired Cartesian velocity using
          the Jacobian pseudoinverse.
        </div>
      )}
    </div>
  );
}
