import { b2JointDef, b2Joint } from '../Joints';
import { b2Vec2 } from '../../Common/Math';

/**
* Mouse joint definition. This requires a world target point,
* tuning parameters, and the time step.
* @see b2MouseJoint
*/
export class b2MouseJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_mouseJoint;
		this.maxForce = 0.0;
		this.frequencyHz = 5.0;
		this.dampingRatio = 0.7;
	}

	/**
	* The initial world target point. This is assumed
	* to coincide with the body anchor initially.
	*/
	public target: b2Vec2 = new b2Vec2();
	/**
	* The maximum constraint force that can be exerted
	* to move the candidate body. Usually you will express
	* as some multiple of the weight (multiplier * mass * gravity).
	*/
	public maxForce: number;
	/**
	* The response speed.
	*/
	public frequencyHz: number;
	/**
	* The damping ratio. 0 = no damping, 1 = critical damping.
	*/
	public dampingRatio: number;
}