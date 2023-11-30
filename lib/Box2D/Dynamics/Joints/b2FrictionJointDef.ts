import { b2Body } from '../b2Body';
import { b2Vec2 } from '../../Common/Math';
import { b2JointDef, b2Joint } from '../Joints';

/**
 * Friction joint defintion
 * @see b2FrictionJoint
 */
export class b2FrictionJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_frictionJoint;
		this.maxForce = 0.0;
		this.maxTorque = 0.0;
	}

	/**
	 * Initialize the bodies, anchors, axis, and reference angle using the world
	 * anchor and world axis.
	 */
	public Initialize(bA: b2Body, bB: b2Body,
		anchor: b2Vec2): void {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
	}

	/**
	* The local anchor point relative to bodyA's origin.
	*/
	public localAnchorA: b2Vec2 = new b2Vec2();

	/**
	* The local anchor point relative to bodyB's origin.
	*/
	public localAnchorB: b2Vec2 = new b2Vec2();

	/**
	 * The maximun force in N.
	 */
	public maxForce: number;

	/**
	 * The maximun friction torque in N-m
	 */
	public maxTorque: number;
}