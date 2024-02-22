import { b2Vec2, b2Mat22 } from '../../Common/Math';
import { b2Body } from '../b2Body';
import { b2TimeStep } from '../b2TimeStep';
import { b2Joint, b2MouseJointDef } from '../Joints';

/**
* A mouse joint is used to make a point on a body track a
* specified world point. This a soft constraint with a maximum
* force. This allows the constraint to stretch and without
* applying huge forces.
* Note: this joint is not fully documented as it is intended primarily
* for the testbed. See that for more instructions.
* @see b2MouseJointDef
*/
export class b2MouseJoint extends b2Joint {
	/** @inheritDoc */
	public GetAnchorA(): b2Vec2 {
		return this.m_target;
	}

	/** @inheritDoc */
	public GetAnchorB(): b2Vec2 {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
	}

	/** @inheritDoc */
	public GetReactionForce(inv_dt: number): b2Vec2 {
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}

	/** @inheritDoc */
	public GetReactionTorque(inv_dt: number): number {
		return 0.0;
	}

	public GetTarget(): b2Vec2 {
		return this.m_target;
	}

	/**
	 * Use this to update the target point.
	 */
	public SetTarget(target: b2Vec2): void {
		if (this.m_bodyB.IsAwake() == false) {
			this.m_bodyB.SetAwake(true);
		}
		this.m_target = target;
	}

	/// Get the maximum force in Newtons.
	public GetMaxForce(): number {
		return this.m_maxForce;
	}

	/// Set the maximum force in Newtons.
	public SetMaxForce(maxForce: number): void {
		this.m_maxForce = maxForce;
	}

	/// Get frequency in Hz
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
	constructor(def: b2MouseJointDef) {
		super(def);

		//b2Settings.b2Assert(def.target.IsValid());
		//b2Settings.b2Assert(b2Math.b2IsValid(def.maxForce) && def.maxForce > 0.0);
		//b2Settings.b2Assert(b2Math.b2IsValid(def.frequencyHz) && def.frequencyHz > 0.0);
		//b2Settings.b2Assert(b2Math.b2IsValid(def.dampingRatio) && def.dampingRatio > 0.0);

		this.m_target.SetV(def.target);
		//this.m_localAnchor = b2MulT(this.m_bodyB.this.m_xf, this.m_target);
		const tX: number = this.m_target.x - this.m_bodyB.m_xf.position.x;
		const tY: number = this.m_target.y - this.m_bodyB.m_xf.position.y;
		const tMat: b2Mat22 = this.m_bodyB.m_xf.R;
		this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
		this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);

		this.m_maxForce = def.maxForce;
		this.m_impulse.SetZero();

		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;

		this.m_beta = 0.0;
		this.m_gamma = 0.0;
	}

	// Presolve vars
	private K: b2Mat22 = new b2Mat22();
	private K1: b2Mat22 = new b2Mat22();
	private K2: b2Mat22 = new b2Mat22();
	public InitVelocityConstraints(step: b2TimeStep): void {
		const b: b2Body = this.m_bodyB;

		const mass: number = b.GetMass();

		// Frequency
		const omega: number = 2.0 * Math.PI * this.m_frequencyHz;

		// Damping co-efficient
		const d: number = 2.0 * mass * this.m_dampingRatio * omega;

		// Spring stiffness
		const k: number = mass * omega * omega;

		// magic formulas
		// gamma has units of inverse mass
		// beta hs units of inverse time
		//b2Settings.b2Assert(d + step.dt * k > Number.MIN_VALUE)
		this.m_gamma = step.dt * (d + step.dt * k);
		this.m_gamma = this.m_gamma != 0 ? 1 / this.m_gamma : 0.0;
		this.m_beta = step.dt * k * this.m_gamma;

		let tMat: b2Mat22;

		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
		tMat = b.m_xf.R;
		let rX: number = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		let rY: number = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		const tX: number = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		const invMass: number = b.m_invMass;
		const invI: number = b.m_invI;

		//b2Mat22 K1;
		this.K1.col1.x = invMass;	this.K1.col2.x = 0.0;
		this.K1.col1.y = 0.0;		this.K1.col2.y = invMass;

		//b2Mat22 K2;
		this.K2.col1.x =  invI * rY * rY;	this.K2.col2.x = -invI * rX * rY;
		this.K2.col1.y = -invI * rX * rY;	this.K2.col2.y =  invI * rX * rX;

		//b2Mat22 K = K1 + K2;
		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.col1.x += this.m_gamma;
		this.K.col2.y += this.m_gamma;

		//this.m_ptpMass = K.GetInverse();
		this.K.GetInverse(this.m_mass);

		//m_C = b.m_position + r - m_target;
		this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
		this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;

		// Cheat with some damping
		b.m_angularVelocity *= 0.98;

		// Warm starting.
		this.m_impulse.x *= step.dtRatio;
		this.m_impulse.y *= step.dtRatio;
		//b.m_linearVelocity += invMass * this.m_impulse;
		b.m_linearVelocity.x += invMass * this.m_impulse.x;
		b.m_linearVelocity.y += invMass * this.m_impulse.y;
		//b.m_angularVelocity += invI * b2Cross(r, this.m_impulse);
		b.m_angularVelocity += invI * (rX * this.m_impulse.y - rY * this.m_impulse.x);
	}

	public SolveVelocityConstraints(step: b2TimeStep): void {
		const b: b2Body = this.m_bodyB;

		let tMat: b2Mat22;
		let tX: number;
		let tY: number;

		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
		tMat = b.m_xf.R;
		let rX: number = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		let rY: number = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;

		// Cdot = v + cross(w, r)
		//b2Vec2 Cdot = b->m_linearVelocity + b2Cross(b->m_angularVelocity, r);
		const CdotX: number = b.m_linearVelocity.x + (-b.m_angularVelocity * rY);
		const CdotY: number = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
		//b2Vec2 impulse = - b2Mul(this.m_mass, Cdot + this.m_beta * this.m_C + this.m_gamma * this.m_impulse);
		tMat = this.m_mass;
		tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
		tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
		let impulseX: number = -(tMat.col1.x * tX + tMat.col2.x * tY);
		let impulseY: number = -(tMat.col1.y * tX + tMat.col2.y * tY);

		const oldImpulseX: number = this.m_impulse.x;
		const oldImpulseY: number = this.m_impulse.y;
		//this.m_impulse += impulse;
		this.m_impulse.x += impulseX;
		this.m_impulse.y += impulseY;
		const maxImpulse: number = step.dt * this.m_maxForce;
		if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
			//this.m_impulse *= this.m_maxImpulse / this.m_impulse.Length();
			this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
		}
		//impulse = this.m_impulse - oldImpulse;
		impulseX = this.m_impulse.x - oldImpulseX;
		impulseY = this.m_impulse.y - oldImpulseY;

		//b->this.m_linearVelocity += b->m_invMass * impulse;
		b.m_linearVelocity.x += b.m_invMass * impulseX;
		b.m_linearVelocity.y += b.m_invMass * impulseY;
		//b->m_angularVelocity += b->m_invI * b2Cross(r, P);
		b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
	}

	public SolvePositionConstraints(baumgarte: number): boolean {
		//B2_NOT_USED(baumgarte);
		return true;
	}

	private m_localAnchor: b2Vec2 = new b2Vec2();
	private m_target: b2Vec2 = new b2Vec2();
	private m_impulse: b2Vec2 = new b2Vec2();

	private m_mass: b2Mat22 = new b2Mat22();	// effective mass for point-to-point constraint.
	private m_C: b2Vec2 = new b2Vec2();			// position error
	private m_maxForce: number;
	private m_frequencyHz: number;
	private m_dampingRatio: number;
	private m_beta: number;						// bias factor
	private m_gamma: number;						// softness
}