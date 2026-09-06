export type Point = { x: number; y: number };
export type Matrix2 = [[number, number], [number, number]];

export type IKResult = {
  reachable: boolean;
  theta1: number;
  theta2: number;
  innerRadius: number;
  outerRadius: number;
};

const EPS = 1e-9;

export function clamp(v: number, min: number, max: number) {
  return Math.max(min, Math.min(max, v));
}

export function radToDeg(rad: number) {
  return (rad * 180) / Math.PI;
}

export function degToRad(deg: number) {
  return (deg * Math.PI) / 180;
}

export function fk(theta1: number, theta2: number, l1: number, l2: number) {
  const a = theta1;
  const b = theta1 + theta2;

  const elbow = {
    x: l1 * Math.cos(a),
    y: l1 * Math.sin(a),
  };

  const end = {
    x: elbow.x + l2 * Math.cos(b),
    y: elbow.y + l2 * Math.sin(b),
  };

  return { elbow, end };
}

/**
 * Standard planar 2R inverse kinematics.
 *
 * D = (x² + y² - l1² - l2²)/(2 l1 l2)
 * theta2 = atan2(±sqrt(1-D²), D)
 * theta1 = atan2(y,x) - atan2(l2 sin(theta2), l1 + l2 cos(theta2))
 *
 * The sign chooses elbow-up vs elbow-down.
 */
export function ik(
  target: Point,
  l1: number,
  l2: number,
  elbowUp: boolean
): IKResult {
  const rho2 = target.x * target.x + target.y * target.y;
  const rho = Math.sqrt(rho2);

  const innerRadius = Math.abs(l1 - l2);
  const outerRadius = l1 + l2;

  if (
    rho < innerRadius - 1e-8 ||
    rho > outerRadius + 1e-8 ||
    l1 <= EPS ||
    l2 <= EPS
  ) {
    return {
      reachable: false,
      theta1: 0,
      theta2: 0,
      innerRadius,
      outerRadius,
    };
  }

  let d = (rho2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  d = clamp(d, -1, 1);

  const sin2 = Math.sqrt(Math.max(0, 1 - d * d)) * (elbowUp ? 1 : -1);
  const theta2 = Math.atan2(sin2, d);

  const theta1 =
    Math.atan2(target.y, target.x) -
    Math.atan2(l2 * Math.sin(theta2), l1 + l2 * Math.cos(theta2));

  return {
    reachable: true,
    theta1,
    theta2,
    innerRadius,
    outerRadius,
  };
}

export function jacobian(
  theta1: number,
  theta2: number,
  l1: number,
  l2: number
): Matrix2 {
  const s1 = Math.sin(theta1);
  const c1 = Math.cos(theta1);
  const s12 = Math.sin(theta1 + theta2);
  const c12 = Math.cos(theta1 + theta2);

  return [
    [-l1 * s1 - l2 * s12, -l2 * s12],
    [l1 * c1 + l2 * c12, l2 * c12],
  ];
}

function determinant(J: Matrix2) {
  return J[0][0] * J[1][1] - J[0][1] * J[1][0];
}

export function inverse2(J: Matrix2): Matrix2 | null {
  const det = determinant(J);
  if (Math.abs(det) < 1e-10) return null;

  const invDet = 1 / det;

  return [
    [J[1][1] * invDet, -J[0][1] * invDet],
    [-J[1][0] * invDet, J[0][0] * invDet],
  ];
}

function multiply2x2Vec(J: Matrix2, v: Point): Point {
  return {
    x: J[0][0] * v.x + J[0][1] * v.y,
    y: J[1][0] * v.x + J[1][1] * v.y,
  };
}

/**
 * Moore–Penrose pseudoinverse for a 2x2 J.
 *
 * For a full-rank square matrix this is just J^-1.
 * At a singular configuration, use J^T (J J^T)^-1 when that
 * matrix is invertible; otherwise fall back to a tiny Tikhonov
 * regularization.
 */
export function pseudoinverseVelocity(
  J: Matrix2,
  velocity: Point
): { qdot: Point; usedPseudoInverse: boolean } {
  const inv = inverse2(J);

  if (inv) {
    return {
      qdot: multiply2x2Vec(inv, velocity),
      usedPseudoInverse: false,
    };
  }

  // Damped least squares: J^T (J J^T + λ²I)^-1 v
  const jt: Matrix2 = [
    [J[0][0], J[1][0]],
    [J[0][1], J[1][1]],
  ];

  const a00 = J[0][0] * J[0][0] + J[0][1] * J[0][1] + 1e-8;
  const a01 = J[0][0] * J[1][0] + J[0][1] * J[1][1];
  const a10 = a01;
  const a11 = J[1][0] * J[1][0] + J[1][1] * J[1][1] + 1e-8;

  const A: Matrix2 = [
    [a00, a01],
    [a10, a11],
  ];

  const Ainv = inverse2(A);
  if (!Ainv) {
    return { qdot: { x: 0, y: 0 }, usedPseudoInverse: true };
  }

  const tmp = multiply2x2Vec(Ainv, velocity);
  return {
    qdot: multiply2x2Vec(jt, tmp),
    usedPseudoInverse: true,
  };
}

/**
 * Map a unit joint-velocity circle into task space:
 * xdot = J qdot.
 */
export function manipulabilityEllipse(
  theta1: number,
  theta2: number,
  l1: number,
  l2: number,
  samples = 96
): Point[] {
  const J = jacobian(theta1, theta2, l1, l2);
  const points: Point[] = [];

  for (let i = 0; i <= samples; i++) {
    const t = (i / samples) * Math.PI * 2;
    points.push(
      multiply2x2Vec(J, {
        x: Math.cos(t),
        y: Math.sin(t),
      })
    );
  }

  return points;
}

export function jointSpeed(qdot: Point) {
  return Math.hypot(qdot.x, qdot.y);
}