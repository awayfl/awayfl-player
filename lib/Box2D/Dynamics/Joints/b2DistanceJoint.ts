import { b2Vec2, b2Mat22, b2Math } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2Settings } from '../../Common/b2Settings';
import { b2TimeStep } from '../b2TimeStep';
import { b2Joint, b2DistanceJointDef } from '../Joints';

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/**
* A distance joint constrains two points on two bodies
* to remain at a fixed distance from each other. You can view
* this as a massless, rigid rod.
* @see b2DistanceJointDef
*/
export class b2DistanceJoint extends b2Joint {
	/** @inheritDoc */
	public GetAnchorA(): b2Vec2 {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}

	/** @inheritDoc */
	public GetAnchorB(): b2Vec2 {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}

	/** @inheritDoc */
	public GetReactionForce(inv_dt: number): b2Vec2 {
		//b2Vec2 F = (m_inv_dt * m_impulse) * m_u;
		//return F;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
	}

	/** @inheritDoc */
	public GetReactionTorque(inv_dt: number): number {
		//B2_NOT_USED(inv_dt);
		return 0.0;
	}

	/// Set the natural length
	public GetLength(): number {
		return this.m_length;
	}

	/// Get the natural length
	public SetLength(length: number): void {
		this.m_length = length;
	}

	/// Get the frequency in Hz
	public GetFrequency(): number {
		return this.m_frequencyHz;
	}

	/// Set the frequency in Hz
	public SetFrequency(hz: number): void {
		this.m_frequencyHz = hz;
	}

	/// Get damping ratio
	public GetDampingRatio(): number {
		return this.m_dampingRatio;
	}

	/// Set damping ratio
	public SetDampingRatio(ratio: number): void {
		this.m_dampingRatio = ratio;
	}

	//--------------- Internals Below -------------------

	/** @private */
	constructor(def: b2DistanceJointDef) {
		super(def);

		let tMat: b2Mat22;
		let tX: number;
		let tY: number;
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);

		this.m_length = def.length;
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		this.m_impulse = 0.0;
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
	}

	public InitVelocityConstraints(step: b2TimeStep): void {

		let tMat: b2Mat22;
		let tX: number;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		// Compute the effective mass matrix.
		//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//m_u = bB->m_sweep.c + r2 - bA->m_sweep.c - r1;
		this.m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		this.m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;

		// Handle singularity.
		//float32 length = m_u.Length();
		const length: number = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
		if (length > b2Settings.b2_linearSlop) {
			//m_u *= 1.0 / length;
			this.m_u.Multiply(1.0 / length);
		} else {
			this.m_u.SetZero();
		}

		//float32 cr1u = b2Cross(r1, m_u);
		const cr1u: number = (r1X * this.m_u.y - r1Y * this.m_u.x);
		//float32 cr2u = b2Cross(r2, m_u);
		const cr2u: number = (r2X * this.m_u.y - r2Y * this.m_u.x);
		//m_mass = bA->m_invMass + bA->m_invI * cr1u * cr1u + bB->m_invMass + bB->m_invI * cr2u * cr2u;
		const invMass: number = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u;
		this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

		if (this.m_frequencyHz > 0.0) {
			const C: number = length - this.m_length;

			// Frequency
			const omega: number = 2.0 * Math.PI * this.m_frequencyHz;

			// Damping coefficient
			const d: number = 2.0 * this.m_mass * this.m_dampingRatio * omega;

			// Spring stiffness
			const k: number = this.m_mass * omega * omega;

			// magic formulas
			this.m_gamma = step.dt * (d + step.dt * k);
			this.m_gamma = this.m_gamma != 0.0 ? 1 / this.m_gamma : 0.0;
			this.m_bias = C * step.dt * k * this.m_gamma;

			this.m_mass = invMass + this.m_gamma;
			this.m_mass = this.m_mass != 0.0 ? 1.0 / this.m_mass : 0.0;
		}

		if (step.warmStarting) {
			// Scale the impulse to support a variable time step
			this.m_impulse *= step.dtRatio;

			//b2Vec2 P = this.m_impulse * this.m_u;
			const PX: number = this.m_impulse * this.m_u.x;
			const PY: number = this.m_impulse * this.m_u.y;
			//bA->m_linearVelocity -= bA->m_invMass * P;
			bA.m_linearVelocity.x -= bA.m_invMass * PX;
			bA.m_linearVelocity.y -= bA.m_invMass * PY;
			//bA->m_angularVelocity -= bA->m_invI * b2Cross(r1, P);
			bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
			//bB->m_linearVelocity += bB->m_invMass * P;
			bB.m_linearVelocity.x += bB.m_invMass * PX;
			bB.m_linearVelocity.y += bB.m_invMass * PY;
			//bB->m_angularVelocity += bB->m_invI * b2Cross(r2, P);
			bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
		} else {
			this.m_impulse = 0.0;
		}
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {

		let tMat: b2Mat22;

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		//b2Vec2 r1 = b2Mul(bA->m_xf.R, this.m_localAnchor1 - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(bB->m_xf.R, this.m_localAnchor2 - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		// Cdot = dot(u, v + cross(w, r))
		//b2Vec2 v1 = bA->m_linearVelocity + b2Cross(bA->m_angularVelocity, r1);
		const v1X: number = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
		const v1Y: number = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
		//b2Vec2 v2 = bB->m_linearVelocity + b2Cross(bB->m_angularVelocity, r2);
		const v2X: number = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
		const v2Y: number = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
		//float32 Cdot = b2Dot(this.m_u, v2 - v1);
		const Cdot: number = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y));

		const impulse: number = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
		this.m_impulse += impulse;

		//b2Vec2 P = impulse * this.m_u;
		const PX: number = impulse * this.m_u.x;
		const PY: number = impulse * this.m_u.y;
		//bA->m_linearVelocity -= bA->m_invMass * P;
		bA.m_linearVelocity.x -= bA.m_invMass * PX;
		bA.m_linearVelocity.y -= bA.m_invMass * PY;
		//bA->m_angularVelocity -= bA->m_invI * b2Cross(r1, P);
		bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
		//bB->m_linearVelocity += bB->m_invMass * P;
		bB.m_linearVelocity.x += bB.m_invMass * PX;
		bB.m_linearVelocity.y += bB.m_invMass * PY;
		//bB->m_angularVelocity += bB->m_invI * b2Cross(r2, P);
		bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
	}

	public SolvePositionConstraints(baumgarte: number): boolean {
		//B2_NOT_USED(baumgarte);

		let tMat: b2Mat22;

		if (this.m_frequencyHz > 0.0) {
			// There is no position correction for soft distance constraints
			return true;
		}

		const bA: b2Body = this.m_bodyA;
		const bB: b2Body = this.m_bodyB;

		//b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
		tMat = bA.m_xf.R;
		let r1X: number = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		let r1Y: number = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		let tX: number =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		//b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
		tMat = bB.m_xf.R;
		let r2X: number = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		let r2Y: number = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		//b2Vec2 d = bB->m_sweep.c + r2 - bA->m_sweep.c - r1;
		let dX: number = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		let dY: number = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;

		//float32 length = d.Normalize();
		const length: number = Math.sqrt(dX * dX + dY * dY);
		dX /= length;
		dY /= length;
		//float32 C = length - this.m_length;
		let C: number = length - this.m_length;
		C = b2Math.Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);

		const impulse: number = -this.m_mass * C;
		//this.m_u = d;
		this.m_u.Set(dX, dY);
		//b2Vec2 P = impulse * this.m_u;
		const PX: number = impulse * this.m_u.x;
		const PY: number = impulse * this.m_u.y;

		//bA->this.m_sweep.c -= bA->m_invMass * P;
		bA.m_sweep.c.x -= bA.m_invMass * PX;
		bA.m_sweep.c.y -= bA.m_invMass * PY;
		//bA->m_sweep.a -= bA->m_invI * b2Cross(r1, P);
		bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX);
		//bB->m_sweep.c += bB->m_invMass * P;
		bB.m_sweep.c.x += bB.m_invMass * PX;
		bB.m_sweep.c.y += bB.m_invMass * PY;
		//bB->m_sweep.a -= bB->m_invI * b2Cross(r2, P);
		bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX);

		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		return b2Math.Abs(C) < b2Settings.b2_linearSlop;

	}

	private m_localAnchor1: b2Vec2 = new b2Vec2();
	private m_localAnchor2: b2Vec2 = new b2Vec2();
	private m_u: b2Vec2 = new b2Vec2();
	private m_frequencyHz: number;
	private m_dampingRatio: number;
	private m_gamma: number;
	private m_bias: number;
	private m_impulse: number;
	private m_mass: number;	// effective mass for the constraint.
	private m_length: number;
}