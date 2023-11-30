import { b2Body } from '../b2Body';
import { b2Vec2 } from '../../Common/Math';
import { b2JointDef, b2PulleyJoint, b2Joint } from '../Joints';

/**
* Pulley joint definition. This requires two ground anchors,
* two dynamic body anchor points, max lengths for each side,
* and a pulley ratio.
* @see b2PulleyJoint
*/
export class b2PulleyJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_pulleyJoint;
		this.groundAnchorA.Set(-1.0, 1.0);
		this.groundAnchorB.Set(1.0, 1.0);
		this.localAnchorA.Set(-1.0, 0.0);
		this.localAnchorB.Set(1.0, 0.0);
		this.lengthA = 0.0;
		this.maxLengthA = 0.0;
		this.lengthB = 0.0;
		this.maxLengthB = 0.0;
		this.ratio = 1.0;
		this.collideConnected = true;
	}

	public Initialize(bA: b2Body, bB: b2Body,
		gaA: b2Vec2, gaB: b2Vec2,
		anchorA: b2Vec2, anchorB: b2Vec2,
		r: number): void {
		this.bodyA = bA;
		this.bodyB = bB;
		this.groundAnchorA.SetV(gaA);
		this.groundAnchorB.SetV(gaB);
		this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
		//b2Vec2 d1 = anchorA - gaA;
		const d1X: number = anchorA.x - gaA.x;
		const d1Y: number = anchorA.y - gaA.y;
		//length1 = d1.Length();
		this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);

		//b2Vec2 d2 = anchor2 - ga2;
		const d2X: number = anchorB.x - gaB.x;
		const d2Y: number = anchorB.y - gaB.y;
		//length2 = d2.Length();
		this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);

		this.ratio = r;
		//b2Settings.b2Assert(ratio > Number.MIN_VALUE);
		const C: number = this.lengthA + this.ratio * this.lengthB;
		this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
		this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
	}

	/**
	* The first ground anchor in world coordinates. This point never moves.
	*/
	public groundAnchorA: b2Vec2 = new b2Vec2();

	/**
	* The second ground anchor in world coordinates. This point never moves.
	*/
	public groundAnchorB: b2Vec2 = new b2Vec2();

	/**
	* The local anchor point relative to bodyA's origin.
	*/
	public localAnchorA: b2Vec2 = new b2Vec2();

	/**
	* The local anchor point relative to bodyB's origin.
	*/
	public localAnchorB: b2Vec2 = new b2Vec2();

	/**
	* The a reference length for the segment attached to bodyA.
	*/
	public lengthA: number;

	/**
	* The maximum length of the segment attached to bodyA.
	*/
	public maxLengthA: number;

	/**
	* The a reference length for the segment attached to bodyB.
	*/
	public lengthB: number;

	/**
	* The maximum length of the segment attached to bodyB.
	*/
	public maxLengthB: number;

	/**
	* The pulley ratio, used to simulate a block-and-tackle.
	*/
	public ratio: number;

}