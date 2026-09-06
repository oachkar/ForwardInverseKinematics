import {
  type Point,
  fk,
  ik,
  jacobian,
  jointSpeed,
  manipulabilityEllipse,
  pseudoinverseVelocity,
} from "./kinematics";
import { useMemo, useRef, useState } from "react";

type Props = {
  l1: number;
  l2: number;
  theta1: number;
  theta2: number;
  elbowUp: boolean;
  target: Point;
  velocityMode: "forward" | "inverse";
  requestedVelocity: Point;
  onTargetChange: (p: Point) => void;
  onVelocityChange: (v: Point) => void;
  onAnglesChange: (theta1: number, theta2: number) => void;
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

export default function ArmPlot(props: Props) {
  const {
    l1,
    l2,
    theta1,
    theta2,
    elbowUp,
    target,
    velocityMode,
    requestedVelocity,
    onTargetChange,
    onVelocityChange,
    onAnglesChange,
  } = props;

  const svgRef = useRef<SVGSVGElement>(null);
  const [dragging, setDragging] = useState<"target" | "velocity" | null>(null);

  const pose = useMemo(
    () => fk(theta1, theta2, l1, l2),
    [theta1, theta2, l1, l2]
  );

  const ikResult = useMemo(
    () => ik(target, l1, l2, elbowUp),
    [target, l1, l2, elbowUp]
  );

  const J = useMemo(
    () => jacobian(theta1, theta2, l1, l2),
    [theta1, theta2, l1, l2]
  );

  const velocity = useMemo(() => {
    return velocityMode === "forward"
      ? requestedVelocity
      : pseudoinverseVelocity(J, requestedVelocity).qdot;
  }, [J, requestedVelocity, velocityMode]);

  const actualEndVelocity = useMemo(() => {
    return {
      x: J[0][0] * velocity.x + J[0][1] * velocity.y,
      y: J[1][0] * velocity.x + J[1][1] * velocity.y,
    };
  }, [J, velocity]);

  const ellipse = useMemo(
    () => manipulabilityEllipse(theta1, theta2, l1, l2),
    [theta1, theta2, l1, l2]
  );

  const maxReach = l1 + l2;
  const minReach = Math.abs(l1 - l2);

  function handlePointerMove(e: React.PointerEvent<SVGSVGElement>) {
    if (!dragging || !svgRef.current) return;

    const rect = svgRef.current.getBoundingClientRect();
    const sx = ((e.clientX - rect.left) / rect.width) * VIEW;
    const sy = ((e.clientY - rect.top) / rect.height) * VIEW;
    const p = svgToWorld(sx, sy);

    if (dragging === "target") {
      // Don't let the target leave the drawing bounds.
      onTargetChange({
        x: Math.max(-WORLD, Math.min(WORLD, p.x)),
        y: Math.max(-WORLD, Math.min(WORLD, p.y)),
      });
    } else {
      onVelocityChange({
        x: Math.max(-1.25, Math.min(1.25, p.x)),
        y: Math.max(-1.25, Math.min(1.25, p.y)),
      });
    }
  }

  function stopDrag() {
    setDragging(null);
  }

  const base = worldToSvg({ x: 0, y: 0 });
  const elbow = worldToSvg(pose.elbow);
  const end = worldToSvg(pose.end);
  const targetSvg = worldToSvg(target);
  const velocityOrigin = base;
  const velocitySvg = worldToSvg({
    x: velocity.x * 0.35,
    y: velocity.y * 0.35,
  });

  const targetInside =
    target.x * target.x + target.y * target.y <= maxReach * maxReach &&
    target.x * target.x + target.y * target.y >= minReach * minReach;

  const ellipsePoints = ellipse.map(pointToString).join(" ");

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
        onPointerMove={handlePointerMove}
        onPointerUp={stopDrag}
        onPointerCancel={stopDrag}
        onPointerLeave={() => dragging && stopDrag()}
      >
        <defs>
          <radialGradient id="workspaceFill">
            <stop offset="0%" stopColor="#dcecff" />
            <stop offset="100%" stopColor="#bcd7ff" />
          </radialGradient>
          <filter id="shadow">
            <feDropShadow dx="0" dy="2" stdDeviation="2" floodOpacity="0.2" />
          </filter>
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

        {/* Desired target for inverse kinematics */}
        <circle
          cx={targetSvg.x}
          cy={targetSvg.y}
          r="11"
          className="target-hit"
          fill="#11a64a"
          onPointerDown={(e) => {
            e.currentTarget.setPointerCapture(e.pointerId);
            setDragging("target");
          }}
        />
        <line
          x1={targetSvg.x - 9}
          x2={targetSvg.x + 9}
          y1={targetSvg.y}
          y2={targetSvg.y}
          className="target-crosshair"
        />
        <line
          x1={targetSvg.x}
          x2={targetSvg.x}
          y1={targetSvg.y - 9}
          y2={targetSvg.y + 9}
          className="target-crosshair"
        />

        {/* Robot */}
        <line
          x1={base.x}
          y1={base.y}
          x2={elbow.x}
          y2={elbow.y}
          className="link link-1"
          filter="url(#shadow)"
        />
        <line
          x1={elbow.x}
          y1={elbow.y}
          x2={end.x}
          y2={end.y}
          className="link link-2"
          filter="url(#shadow)"
        />
        <circle cx={base.x} cy={base.y} r="15" className="joint-base" />
        <circle cx={elbow.x} cy={elbow.y} r="13" className="joint" />
        <circle cx={end.x} cy={end.y} r="11" className="end-effector" />

        {/* Orientation arrows */}
        <line
          x1={end.x}
          y1={end.y}
          x2={end.x + 42 * Math.cos(theta1 + theta2)}
          y2={end.y - 42 * Math.sin(theta1 + theta2)}
          className="orientation-arrow"
          markerEnd="url(#arrowGreen)"
        />

        {/* End-effector velocity vector */}
        <line
          x1={end.x}
          y1={end.y}
          x2={end.x + actualEndVelocity.x * 70}
          y2={end.y - actualEndVelocity.y * 70}
          className="velocity-arrow"
          markerEnd="url(#arrowPink)"
        />

        {/* Velocity input point */}
        <circle
          cx={velocitySvg.x}
          cy={velocitySvg.y}
          r="9"
          className="velocity-point"
          onPointerDown={(e) => {
            e.currentTarget.setPointerCapture(e.pointerId);
            setDragging("velocity");
          }}
        />
        <line
          x1={velocityOrigin.x}
          y1={velocityOrigin.y}
          x2={velocitySvg.x}
          y2={velocitySvg.y}
          className="velocity-input-arrow"
        />

        <defs>
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
        </defs>

        <g transform="translate(18, 18)">
          <rect width="177" height="70" rx="8" className="legend-bg" />
          <text x="12" y="22" className="legend-title">
            Jacobian
          </text>
          <text x="12" y="42" className="legend-text">
            v = J(q) q̇
          </text>
          <text x="12" y="59" className="legend-text">
            |q̇| = {jointSpeed(velocity).toFixed(2)}
          </text>
        </g>

        <g className="plot-labels">
          <text x={base.x + 14} y={base.y + 27}>
            base
          </text>
          <text x={elbow.x + 12} y={elbow.y - 12}>
            joint 2
          </text>
          <text x={end.x + 13} y={end.y - 12}>
            end effector
          </text>
        </g>
      </svg>

      {!targetInside && (
        <div className="plot-warning">
          The target is outside the annular workspace. Drag the green locator
          into the blue region.
        </div>
      )}

      {velocityMode === "inverse" && ikResult.reachable && (
        <div className="plot-note">
          Inverse velocity uses the Moore–Penrose pseudoinverse of the
          Jacobian.
        </div>
      )}
    </div>
  );
}