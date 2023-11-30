import { b2Body } from '../b2Body';
import { b2Vec2 } from '../../Common/Math';
import { b2JointDef, b2Joint } from '../Joints';

/**
* Distance joint definition. This requires defining an
* anchor point on both bodies and the non-zero length of the
* distance joint. The definition uses local anchor points
* so that the initial configuration can violate the constraint
* slightly. This helps when saving and loading a game.
* @warning Do not use a zero or short length.
* @see b2DistanceJoint
*/
export class b2DistanceJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_distanceJoint;
		//localAnchor1.Set(0.0, 0.0);
		//localAnchor2.Set(0.0, 0.0);
		this.length = 1.0;
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}

	/**
	* Initialize the bodies, anchors, and length using the world
	* anchors.
	*/
	public Initialize(bA: b2Body, bB: b2Body,
		anchorA: b2Vec2, anchorB: b2Vec2): void {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
		const dX: number = anchorB.x - anchorA.x;
		const dY: number = anchorB.y - anchorA.y;
		length = Math.sqrt(dX * dX + dY * dY);
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}

	/**
	* The local anchor point relative to body1's origin.
	*/
	public localAnchorA: b2Vec2 = new b2Vec2();

	/**
	* The local anchor point relative to body2's origin.
	*/
	public localAnchorB: b2Vec2 = new b2Vec2();

	/**
	* The natural length between the anchor points.
	*/
	public length: number;

	/**
	* The mass-spring-damper frequency in Hertz.
	*/
	public frequencyHz: number;

	/**
	* The damping ratio. 0 = no damping, 1 = critical damping.
	*/
	public dampingRatio: number;
}