import { b2JointDef, b2Joint } from '../Joints';
import { b2Vec2 } from '../../Common/Math';
import { b2Body } from '../b2Body';

/**
 * Weld joint definition. You need to specify local anchor points
 * where they are attached and the relative body angle. The position
 * of the anchor points is important for computing the reaction torque.
 * @see b2WeldJoint
 */
export class b2WeldJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_weldJoint;
		this.referenceAngle = 0.0;
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
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
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
	 * The body2 angle minus body1 angle in the reference state (radians).
	 */
	public referenceAngle: number;
}