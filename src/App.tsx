import { useEffect, useMemo, useState, type ChangeEvent } from "react";
import ArmPlot from "./ArmPlot";
import VelocityPlot from "./VelocityPlot";
import {
  degToRad,
  fk,
  ik,
  jointSpeed,
  radToDeg,
  type Point,
  pseudoinverseVelocity,
  jacobian,
} from "./kinematics";

function format(n: number, digits = 3) {
  return n.toFixed(digits);
}

function Slider({
  label,
  value,
  displayValue,
  min,
  max,
  step,
  onChange,
  suffix = "",
  disabled = false,
}: {
  label: string;
  value: number;
  displayValue?: string;
  min: number;
  max: number;
  step: number;
  onChange: (value: number) => void;
  suffix?: string;
  disabled?: boolean;
}) {
  return (
    <label className={`control ${disabled ? "disabled" : ""}`}>
      <div className="control-row">
        <span>{label}</span>
        <span className="control-value">
          {displayValue ?? `${value.toFixed(step < 0.1 ? 2 : 1)}${suffix}`}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        disabled={disabled}
        onChange={(e: ChangeEvent<HTMLInputElement>) => onChange(Number(e.target.value))}
      />
    </label>
  );
}

export default function App() {
  // The Wolfram demo constrains r1 + r2 = 2.
  const [l1, setL1] = useState(1.59);
  const l2 = 2 - l1;

  // These are the stored forward-kinematics angles. In inverse mode the
  // displayed values are instead taken directly from the current IK solution.
  const [theta1Deg, setTheta1Deg] = useState(-58);
  const [theta2Deg, setTheta2Deg] = useState(-38);
  const theta1 = degToRad(theta1Deg);
  const theta2 = degToRad(theta2Deg);

  const [elbowUp, setElbowUp] = useState(false);
  const [inverseMode, setInverseMode] = useState(false);
  const [velocityMode, setVelocityMode] = useState<"forward" | "inverse">(
    "forward"
  );

  const initialPose = fk(theta1, theta2, l1, l2);
  const [target, setTarget] = useState<Point>(initialPose.end);

  // Forward velocity input: joint velocity qdot.
  const [jointVelocity, setJointVelocity] = useState<Point>({ x: 0.6, y: 0.2 });
  // Inverse velocity input: desired end-effector velocity v_d.
  const [desiredEndVelocity, setDesiredEndVelocity] = useState<Point>({
    x: 0.6,
    y: 0.2,
  });

  const ikSolution = useMemo(
    () => ik(target, l1, l2, elbowUp),
    [target, l1, l2, elbowUp]
  );

  const activeTheta1 =
    inverseMode && ikSolution.reachable ? ikSolution.theta1 : theta1;
  const activeTheta2 =
    inverseMode && ikSolution.reachable ? ikSolution.theta2 : theta2;

  // Keep the angle controls synchronized with whatever the robot is actually
  // showing in inverse mode. If the target is unreachable, leave the stored
  // forward angles alone but show "No solution" rather than fake 0° values.
  useEffect(() => {
    if (!inverseMode || !ikSolution.reachable) return;

    const nextTheta1 = radToDeg(ikSolution.theta1);
    const nextTheta2 = radToDeg(ikSolution.theta2);

    setTheta1Deg((current) =>
      Math.abs(current - nextTheta1) > 1e-9 ? nextTheta1 : current
    );
    setTheta2Deg((current) =>
      Math.abs(current - nextTheta2) > 1e-9 ? nextTheta2 : current
    );
  }, [inverseMode, ikSolution]);

  const pose = fk(activeTheta1, activeTheta2, l1, l2);
  const J = jacobian(activeTheta1, activeTheta2, l1, l2);

  const inverseVelocity = useMemo(
    () => pseudoinverseVelocity(J, desiredEndVelocity),
    [J, desiredEndVelocity]
  );

  const qdot = velocityMode === "forward" ? jointVelocity : inverseVelocity.qdot;
  const actualEndVelocity = {
    x: J[0][0] * qdot.x + J[0][1] * qdot.y,
    y: J[1][0] * qdot.x + J[1][1] * qdot.y,
  };

  const desiredVelocity =
    velocityMode === "forward" ? null : desiredEndVelocity;
  const velocityResidual = velocityMode === "inverse" && ikSolution.reachable
    ? Math.hypot(
        actualEndVelocity.x - desiredEndVelocity.x,
        actualEndVelocity.y - desiredEndVelocity.y
      )
    : null;

  function setForwardTheta1(v: number) {
    setInverseMode(false);
    setTheta1Deg(v);
  }

  function setForwardTheta2(v: number) {
    setInverseMode(false);
    setTheta2Deg(v);
  }

  function handleTargetChange(p: Point) {
    setTarget(p);
    setInverseMode(true);
  }

  function reset() {
    const nextL1 = 1.59;
    const nextL2 = 0.41;
    const nextT1 = degToRad(-58);
    const nextT2 = degToRad(-38);
    const p = fk(nextT1, nextT2, nextL1, nextL2);

    setL1(nextL1);
    setTheta1Deg(-58);
    setTheta2Deg(-38);
    setElbowUp(false);
    setInverseMode(false);
    setVelocityMode("forward");
    setJointVelocity({ x: 0.6, y: 0.2 });
    setDesiredEndVelocity({ x: 0.6, y: 0.2 });
    setTarget(p.end);
  }

  const angle1Display = inverseMode
    ? ikSolution.reachable
      ? `${radToDeg(ikSolution.theta1).toFixed(2)}°`
      : "No solution"
    : `${theta1Deg.toFixed(1)}°`;

  const angle2Display = inverseMode
    ? ikSolution.reachable
      ? `${radToDeg(ikSolution.theta2).toFixed(2)}°`
      : "No solution"
    : `${theta2Deg.toFixed(1)}°`;

  return (
    <div className="app">
      <header className="header">
        <div>
          <div className="eyebrow">ROBOTICS DEMONSTRATION</div>
          <h1>Forward &amp; Inverse Kinematics for a Two-Link Arm</h1>
          <p>
            A React recreation of the Wolfram Demonstration, including
            forward kinematics, inverse kinematics, Jacobian velocity mapping,
            and the manipulability ellipse.
          </p>
        </div>
        <button className="reset-button" onClick={reset}>
          Reset
        </button>
      </header>

      <main className="layout">
        <aside className="sidebar">
          <section className="card">
            <div className="section-title">Arm configuration</div>

            <Slider
              label="r₁"
              value={l1}
              min={0.15}
              max={1.85}
              step={0.01}
              onChange={setL1}
            />

            <div className="readout">
              <span>r₂</span>
              <strong>{format(l2, 2)}</strong>
              <small>r₁ + r₂ = 2</small>
            </div>

            <Slider
              label="θ₁"
              value={theta1Deg}
              displayValue={angle1Display}
              min={-180}
              max={180}
              step={1}
              suffix="°"
              disabled={inverseMode}
              onChange={setForwardTheta1}
            />

            <Slider
              label="θ₂"
              value={theta2Deg}
              displayValue={angle2Display}
              min={-180}
              max={180}
              step={1}
              suffix="°"
              disabled={inverseMode}
              onChange={setForwardTheta2}
            />

            <label className="checkbox-row">
              <input
                type="checkbox"
                checked={elbowUp}
                onChange={(e: ChangeEvent<HTMLInputElement>) => setElbowUp(e.target.checked)}
              />
              <span>elbow up?</span>
            </label>

            <div className="mode-tabs">
              <button
                className={!inverseMode ? "active" : ""}
                onClick={() => setInverseMode(false)}
              >
                Forward
              </button>
              <button
                className={inverseMode ? "active" : ""}
                onClick={() => setInverseMode(true)}
              >
                Inverse
              </button>
            </div>

            <div className="subtle">
              {inverseMode
                ? "The target determines θ₁/θ₂. Angle sliders are locked to the displayed IK solution."
                : "Forward mode uses θ₁/θ₂ as inputs. Drag the end effector or green target to switch to inverse mode."}
            </div>
          </section>

          <VelocityPlot
            value={velocityMode === "forward" ? jointVelocity : desiredEndVelocity}
            onChange={
              velocityMode === "forward"
                ? setJointVelocity
                : setDesiredEndVelocity
            }
            title={
              velocityMode === "forward"
                ? "Joint velocity"
                : "Desired end-effector velocity"
            }
            symbol={velocityMode === "forward" ? "q̇" : "v"}
            maxMagnitude={velocityMode === "forward" ? 1 : 1.5}
          />

          <section className="card velocity-card">
            <div className="section-title">Velocity kinematics</div>
            <div className="mode-tabs">
              <button
                className={velocityMode === "forward" ? "active" : ""}
                onClick={() => setVelocityMode("forward")}
              >
                q̇ → v
              </button>
              <button
                className={velocityMode === "inverse" ? "active" : ""}
                onClick={() => setVelocityMode("inverse")}
              >
                v → q̇
              </button>
            </div>
            <div className="velocity-values">
              <div>
                <span>q̇₁</span>
                <strong>{format(qdot.x)}</strong>
              </div>
              <div>
                <span>q̇₂</span>
                <strong>{format(qdot.y)}</strong>
              </div>
              <div>
                <span>|q̇|</span>
                <strong>{format(jointSpeed(qdot))}</strong>
              </div>
            </div>
            {velocityResidual !== null && (
              <div className="residual-row">
                <span>‖v − vᵈ‖</span>
                <strong>{velocityResidual.toExponential(2)}</strong>
              </div>
            )}
          </section>
        </aside>

        <section className="main-column">
          <ArmPlot
            l1={l1}
            l2={l2}
            theta1={activeTheta1}
            theta2={activeTheta2}
            elbowUp={elbowUp}
            target={target}
            velocityMode={velocityMode}
            qdot={qdot}
            desiredEndVelocity={desiredVelocity}
            actualEndVelocity={actualEndVelocity}
            onTargetChange={handleTargetChange}
          />

          <section className="math-card">
            <div className="math-header">
              <div>
                <div className="section-title">Current kinematic state</div>
                <div className="subtle">
                  These values are calculated directly from the current robot
                  configuration rather than being hard-coded display values.
                </div>
              </div>
              <span
                className={`status-pill ${inverseMode && !ikSolution.reachable ? "bad" : "neutral"}`}
              >
                {inverseMode
                  ? ikSolution.reachable
                    ? "IK active"
                    : "IK: no solution"
                  : "FK active"}
              </span>
            </div>

            <div className="math-grid">
              <div className="formula">
                <div className="formula-title">Forward position</div>
                <code>x = r₁ cos θ₁ + r₂ cos(θ₁ + θ₂)</code>
                <code>y = r₁ sin θ₁ + r₂ sin(θ₁ + θ₂)</code>
                <div className="result">
                  <span>x</span>
                  <strong>{format(pose.end.x)}</strong>
                  <span>y</span>
                  <strong>{format(pose.end.y)}</strong>
                </div>
              </div>

              <div className="formula">
                <div className="formula-title">Inverse position</div>
                <code>D = (x² + y² − r₁² − r₂²) / (2 r₁ r₂)</code>
                <code>θ₂ = atan2(±√(1 − D²), D)</code>
                <code>
                  θ₁ = atan2(y,x) − atan2(r₂ sin θ₂, r₁+r₂ cos θ₂)
                </code>
                {ikSolution.reachable ? (
                  <div className="result">
                    <span>θ₁</span>
                    <strong>{format(radToDeg(ikSolution.theta1), 2)}°</strong>
                    <span>θ₂</span>
                    <strong>{format(radToDeg(ikSolution.theta2), 2)}°</strong>
                  </div>
                ) : (
                  <div className="no-solution">No valid IK solution for the current target.</div>
                )}
              </div>

              <div className="formula wide">
                <div className="formula-title">Jacobian</div>
                <code>
                  J = ∂(x,y)/∂(θ₁,θ₂) = [ −r₁sinθ₁−r₂sin(θ₁+θ₂), −r₂sin(θ₁+θ₂); r₁cosθ₁+r₂cos(θ₁+θ₂), r₂cos(θ₁+θ₂) ]
                </code>
                <div className="matrix-result">
                  <span>[</span>
                  <div>
                    <span>{format(J[0][0])}</span>
                    <span>{format(J[0][1])}</span>
                    <span>{format(J[1][0])}</span>
                    <span>{format(J[1][1])}</span>
                  </div>
                  <span>]</span>
                </div>
              </div>

              <div className="formula wide velocity-mapping">
                <div className="formula-title">Velocity mapping</div>
                <code>v = J q̇</code>
                <div className="velocity-summary">
                  <div>
                    <span>q̇</span>
                    <strong>[{format(qdot.x)}, {format(qdot.y)}]</strong>
                  </div>
                  <div>
                    <span>actual v</span>
                    <strong>[{format(actualEndVelocity.x)}, {format(actualEndVelocity.y)}]</strong>
                  </div>
                  {velocityMode === "inverse" && desiredEndVelocity && (
                    <div>
                      <span>desired v</span>
                      <strong>[{format(desiredEndVelocity.x)}, {format(desiredEndVelocity.y)}]</strong>
                    </div>
                  )}
                </div>
              </div>
            </div>
          </section>

          <footer className="footer">
            Source concept: Aaron T. Becker &amp; Lillian Lin, Wolfram
            Demonstrations Project (2016). This implementation reproduces the
            mathematical/interactive behavior independently in JavaScript.
          </footer>
        </section>
      </main>
    </div>
  );
}
