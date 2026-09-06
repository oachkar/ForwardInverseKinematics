import { useMemo, useState } from "react";
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
  min,
  max,
  step,
  onChange,
  suffix = "",
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (value: number) => void;
  suffix?: string;
}) {
  return (
    <label className="control">
      <div className="control-row">
        <span>{label}</span>
        <span className="control-value">
          {value.toFixed(step < 0.1 ? 2 : 1)}
          {suffix}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
      />
    </label>
  );
}

export default function App() {
  // The Wolfram demo constrains r1 + r2 = 2.
  const [l1, setL1] = useState(1.59);
  const l2 = 2 - l1;

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
  const [velocity, setVelocity] = useState<Point>({ x: 0.6, y: 0.2 });

  const ikSolution = useMemo(
    () => ik(target, l1, l2, elbowUp),
    [target, l1, l2, elbowUp]
  );

  const activeTheta1 =
    inverseMode && ikSolution.reachable ? ikSolution.theta1 : theta1;
  const activeTheta2 =
    inverseMode && ikSolution.reachable ? ikSolution.theta2 : theta2;

  const pose = fk(activeTheta1, activeTheta2, l1, l2);
  const J = jacobian(activeTheta1, activeTheta2, l1, l2);
  const inverseVelocity = pseudoinverseVelocity(J, velocity).qdot;

  // In forward velocity mode, the editable unit-disc locator is qdot.
  // In inverse velocity mode, velocity is the editable end-effector vector
  // and the app computes qdot through the pseudoinverse.
  const qdot = velocityMode === "forward" ? velocity : inverseVelocity;

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
    setVelocity({ x: 0.6, y: 0.2 });
    setTarget(p.end);
  }

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
              min={-180}
              max={180}
              step={1}
              suffix="°"
              onChange={setForwardTheta1}
            />

            <Slider
              label="θ₂"
              value={theta2Deg}
              min={-180}
              max={180}
              step={1}
              suffix="°"
              onChange={setForwardTheta2}
            />

            <label className="checkbox-row">
              <input
                type="checkbox"
                checked={elbowUp}
                onChange={(e) => setElbowUp(e.target.checked)}
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
              Forward mode uses θ₁/θ₂ as inputs. Inverse mode uses the draggable
              green target.
            </div>
          </section>

          <VelocityPlot
            qdot={qdot}
            onChange={setVelocity}
            readOnly={velocityMode === "inverse"}
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
            requestedVelocity={velocity}
            onTargetChange={handleTargetChange}
            onVelocityChange={setVelocity}
            onAnglesChange={(a, b) => {
              setTheta1Deg(radToDeg(a));
              setTheta2Deg(radToDeg(b));
              setInverseMode(false);
            }}
          />

          <section className="math-card">
            <div className="math-header">
              <div>
                <div className="section-title">Current kinematic state</div>
                <div className="subtle">
                  Useful for demonstrating that the web app is computing the
                  same mathematics rather than only animating an image.
                </div>
              </div>
              <span className="status-pill neutral">
                {inverseMode ? "IK active" : "FK active"}
              </span>
            </div>

            <div className="math-grid">
              <div className="formula">
                <div className="formula-title">Forward position</div>
                <code>
                  x = r₁ cos θ₁ + r₂ cos(θ₁ + θ₂)
                </code>
                <code>
                  y = r₁ sin θ₁ + r₂ sin(θ₁ + θ₂)
                </code>
                <div className="result">
                  <span>x</span>
                  <strong>{format(pose.end.x)}</strong>
                  <span>y</span>
                  <strong>{format(pose.end.y)}</strong>
                </div>
              </div>

              <div className="formula">
                <div className="formula-title">Inverse position</div>
                <code>
                  D = (x² + y² − r₁² − r₂²) / (2 r₁ r₂)
                </code>
                <code>θ₂ = atan2(±√(1 − D²), D)</code>
                <code>
                  θ₁ = atan2(y,x) − atan2(r₂ sin θ₂, r₁+r₂ cos θ₂)
                </code>
                <div className="result">
                  <span>θ₁</span>
                  <strong>{format(radToDeg(ikSolution.theta1), 2)}°</strong>
                  <span>θ₂</span>
                  <strong>{format(radToDeg(ikSolution.theta2), 2)}°</strong>
                </div>
              </div>

              <div className="formula wide">
                <div className="formula-title">Jacobian</div>
                <code>
                  J = ∂(x,y)/∂(θ₁,θ₂) = [ −r₁sinθ₁−r₂sin(θ₁+θ₂), −r₂sin(θ₁+θ₂);
                  r₁cosθ₁+r₂cos(θ₁+θ₂), r₂cos(θ₁+θ₂) ]
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