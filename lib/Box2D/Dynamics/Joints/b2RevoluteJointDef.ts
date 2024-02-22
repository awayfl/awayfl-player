import { b2JointDef, b2Joint } from '../Joints';
import { b2Vec2 } from '../../Common/Math';
import { b2Body } from '../b2Body';

/**
* Revolute joint definition. This requires defining an
* anchor point where the bodies are joined. The definition
* uses local anchor points so that the initial configuration
* can violate the constraint slightly. You also need to
* specify the initial relative angle for joint limits. This
* helps when saving and loading a game.
* The local anchor points are measured from the body's origin
* rather than the center of mass because:
* 1. you might not know where the center of mass will be.
* 2. if you add/remove shapes from a body and recompute the mass,
* the joints will be broken.
* @see b2RevoluteJoint
*/
export class b2RevoluteJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_revoluteJoint;
		this.localAnchorA.Set(0.0, 0.0);
		this.localAnchorB.Set(0.0, 0.0);
		this.referenceAngle = 0.0;
		this.lowerAngle = 0.0;
		this.upperAngle = 0.0;
		this.maxMotorTorque = 0.0;
		this.motorSpeed = 0.0;
		this.enableLimit = false;
		this.enableMotor = false;
	}

	/**
	* Initialize the bodies, anchors, and reference angle using the world
	* anchor.
	*/
	public Initialize(bA: b2Body, bB: b2Body, anchor: b2Vec2): void {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
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
	* The bodyB angle minus bodyA angle in the reference state (radians).
	*/
	public referenceAngle: number;

	/**
	* A flag to enable joint limits.
	*/
	public enableLimit: boolean;

	/**
	* The lower angle for the joint limit (radians).
	*/
	public lowerAngle: number;

	/**
	* The upper angle for the joint limit (radians).
	*/
	public upperAngle: number;

	/**
	* A flag to enable the joint motor.
	*/
	public enableMotor: boolean;

	/**
	* The desired motor speed. Usually in radians per second.
	*/
	public motorSpeed: number;

	/**
	* The maximum motor torque used to achieve the desired motor speed.
	* Usually in N-m.
	*/
	public maxMotorTorque: number;

}