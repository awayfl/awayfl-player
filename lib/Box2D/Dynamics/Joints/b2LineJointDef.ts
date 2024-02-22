import { b2JointDef, b2Joint } from '../Joints';
import { b2Body } from '../b2Body';
import { b2Vec2 } from '../../Common/Math';

/**
 * Line joint definition. This requires defining a line of
 * motion using an axis and an anchor point. The definition uses local
 * anchor points and a local axis so that the initial configuration
 * can violate the constraint slightly. The joint translation is zero
 * when the local anchor points coincide in world space. Using local
 * anchors and a local axis helps when saving and loading a game.
 * @see b2LineJoint
 */
export class b2LineJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_lineJoint;
		//this.localAnchor1.SetZero();
		//this.localAnchor2.SetZero();
		this.localAxisA.Set(1.0, 0.0);
		this.enableLimit = false;
		this.lowerTranslation = 0.0;
		this.upperTranslation = 0.0;
		this.enableMotor = false;
		this.maxMotorForce = 0.0;
		this.motorSpeed = 0.0;
	}

	public Initialize(bA: b2Body, bB: b2Body, anchor: b2Vec2, axis: b2Vec2): void {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
		this.localAxisA = this.bodyA.GetLocalVector(axis);
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
	* The local translation axis in bodyA.
	*/
	public localAxisA: b2Vec2 = new b2Vec2();

	/**
	* Enable/disable the joint limit.
	*/
	public enableLimit: boolean;

	/**
	* The lower translation limit, usually in meters.
	*/
	public lowerTranslation: number;

	/**
	* The upper translation limit, usually in meters.
	*/
	public upperTranslation: number;

	/**
	* Enable/disable the joint motor.
	*/
	public enableMotor: boolean;

	/**
	* The maximum motor torque, usually in N-m.
	*/
	public maxMotorForce: number;

	/**
	* The desired motor speed in radians per second.
	*/
	public motorSpeed: number;

}