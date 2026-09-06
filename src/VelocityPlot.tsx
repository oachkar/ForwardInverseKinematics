import type { Point } from "./kinematics";

type Props = {
  qdot: Point;
  onChange?: (v: Point) => void;
  readOnly?: boolean;
};

const SIZE = 260;
const CENTER = SIZE / 2;
const R = 91;

function toSvg(v: Point) {
  return {
    x: CENTER + v.x * R,
    y: CENTER - v.y * R,
  };
}

function toWorld(x: number, y: number): Point {
  return {
    x: (x - CENTER) / R,
    y: -(y - CENTER) / R,
  };
}

export default function VelocityPlot({
  qdot,
  onChange = () => {},
  readOnly = false,
}: Props) {
  const p = toSvg({
    x: Math.max(-1, Math.min(1, qdot.x)),
    y: Math.max(-1, Math.min(1, qdot.y)),
  });

  function handlePointerMove(e: React.PointerEvent<SVGSVGElement>) {
    if (readOnly || e.buttons !== 1) return;

    const rect = e.currentTarget.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * SIZE;
    const y = ((e.clientY - rect.top) / rect.height) * SIZE;
    const next = toWorld(x, y);

    // Constrain qdot to the unit disc, matching the original demo.
    const mag = Math.hypot(next.x, next.y);
    if (mag > 1) {
      next.x /= mag;
      next.y /= mag;
    }

    onChange(next);
  }

  return (
    <div className="velocity-panel">
      <div className="panel-title-row">
        <span>Joint velocity</span>
        <span className="mono">
          q̇ = [{qdot.x.toFixed(3)}, {qdot.y.toFixed(3)}]
        </span>
      </div>

      <svg
        className={`velocity-svg ${readOnly ? "read-only" : ""}`}
        viewBox={`0 0 ${SIZE} ${SIZE}`}
        onPointerMove={handlePointerMove}
      >
        <circle cx={CENTER} cy={CENTER} r={R} className="unit-disc" />
        <line x1="14" x2={SIZE - 14} y1={CENTER} y2={CENTER} className="axis" />
        <line x1={CENTER} x2={CENTER} y1="14" y2={SIZE - 14} className="axis" />

        <line
          x1={CENTER}
          y1={CENTER}
          x2={p.x}
          y2={p.y}
          className="qdot-arrow"
        />
        <circle
          cx={p.x}
          cy={p.y}
          r="7"
          className="qdot-point"
          style={{ cursor: readOnly ? "default" : "grab" }}
        />

        <text x={SIZE - 18} y={CENTER - 7} className="axis-label">
          q̇₁
        </text>
        <text x={CENTER + 8} y={20} className="axis-label">
          q̇₂
        </text>
        <text x={CENTER + 8} y={CENTER - 9} className="small-label">
          0
        </text>
        <text x={CENTER + R - 7} y={CENTER + 18} className="small-label">
          1
        </text>
        <text x={CENTER - R - 13} y={CENTER + 18} className="small-label">
          −1
        </text>
        <text x={CENTER + 7} y={CENTER - R + 7} className="small-label">
          1
        </text>
        <text x={CENTER + 7} y={CENTER + R} className="small-label">
          −1
        </text>
      </svg>

      <div className="velocity-help">
        {readOnly
          ? "Inverse velocity mode: the joint velocity shown here is computed from the desired end-effector velocity."
          : "Drag the point inside the unit disc. The demo limits joint-velocity magnitude to ≤ 1."}
      </div>
    </div>
  );
}