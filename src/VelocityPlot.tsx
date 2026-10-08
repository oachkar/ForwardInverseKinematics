import { type PointerEvent as ReactPointerEvent } from "react";
import type { Point } from "./kinematics";

type Props = {
  value: Point;
  onChange: (v: Point) => void;
  title: string;
  symbol: string;
  maxMagnitude: number;
};

const SIZE = 260;
const CENTER = SIZE / 2;
const R = 91;

function toSvg(v: Point, maxMagnitude: number) {
  return {
    x: CENTER + (v.x / maxMagnitude) * R,
    y: CENTER - (v.y / maxMagnitude) * R,
  };
}

function toWorld(x: number, y: number, maxMagnitude: number): Point {
  return {
    x: ((x - CENTER) / R) * maxMagnitude,
    y: -((y - CENTER) / R) * maxMagnitude,
  };
}

export default function VelocityPlot({
  value,
  onChange,
  title,
  symbol,
  maxMagnitude,
}: Props) {
  const p = toSvg(value, maxMagnitude);

  function setFromPointer(e: ReactPointerEvent<SVGSVGElement>) {
    const rect = e.currentTarget.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * SIZE;
    const y = ((e.clientY - rect.top) / rect.height) * SIZE;
    const next = toWorld(x, y, maxMagnitude);

    const mag = Math.hypot(next.x, next.y);
    if (mag > maxMagnitude) {
      next.x *= maxMagnitude / mag;
      next.y *= maxMagnitude / mag;
    }

    onChange(next);
  }

  return (
    <div className="velocity-panel">
      <div className="panel-title-row">
        <span>{title}</span>
        <span className="mono">
          {symbol} = [{value.x.toFixed(3)}, {value.y.toFixed(3)}]
        </span>
      </div>

      <svg
        className="velocity-svg"
        viewBox={`0 0 ${SIZE} ${SIZE}`}
        onPointerDown={(e: ReactPointerEvent<SVGSVGElement>) => {
          e.currentTarget.setPointerCapture(e.pointerId);
          setFromPointer(e);
        }}
        onPointerMove={(e: ReactPointerEvent<SVGSVGElement>) => {
          if (e.buttons === 1) setFromPointer(e);
        }}
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
        />

        <text x={SIZE - 22} y={CENTER - 7} className="axis-label">
          {symbol}₁
        </text>
        <text x={CENTER + 8} y={20} className="axis-label">
          {symbol}₂
        </text>
        <text x={CENTER + 8} y={CENTER - 9} className="small-label">
          0
        </text>
        <text x={CENTER + R - 7} y={CENTER + 18} className="small-label">
          +{maxMagnitude === 1 ? "1" : maxMagnitude.toFixed(1)}
        </text>
        <text x={CENTER - R - 16} y={CENTER + 18} className="small-label">
          −{maxMagnitude === 1 ? "1" : maxMagnitude.toFixed(1)}
        </text>
        <text x={CENTER + 7} y={CENTER - R + 7} className="small-label">
          +{maxMagnitude === 1 ? "1" : maxMagnitude.toFixed(1)}
        </text>
        <text x={CENTER + 7} y={CENTER + R} className="small-label">
          −{maxMagnitude === 1 ? "1" : maxMagnitude.toFixed(1)}
        </text>
      </svg>

      <div className="velocity-help">
        Drag the vector. Maximum magnitude: {maxMagnitude.toFixed(1)}.
      </div>
    </div>
  );
}
