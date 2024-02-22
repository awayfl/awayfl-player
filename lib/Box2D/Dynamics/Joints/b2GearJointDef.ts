import { b2JointDef, b2Joint } from '../Joints';

/**
* Gear joint definition. This definition requires two existing
* revolute or prismatic joints (any combination will work).
* The provided joints must attach a dynamic body to a static body.
* @see b2GearJoint
*/
export class b2GearJointDef extends b2JointDef {
	constructor() {
		super();

		this.type = b2Joint.e_gearJoint;
		this.joint1 = null;
		this.joint2 = null;
		this.ratio = 1.0;
	}

	/**
	* The first revolute/prismatic joint attached to the gear joint.
	*/
	public joint1: b2Joint;
	/**
	* The second revolute/prismatic joint attached to the gear joint.
	*/
	public joint2: b2Joint;
	/**
	* The gear ratio.
	* @see b2GearJoint for explanation.
	*/
	public ratio: number;
}