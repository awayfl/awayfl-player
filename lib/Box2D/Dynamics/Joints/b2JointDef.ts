import { b2Body } from '../b2Body';
import { b2Joint } from '../Joints';

/**
* Joint definitions are used to construct joints.
* @see b2Joint
*/
export class b2JointDef {

	constructor() {
		this.type = b2Joint.e_unknownJoint;
		this.userData = null;
		this.bodyA = null;
		this.bodyB = null;
		this.collideConnected = false;
	}

	/**
	* The joint type is set automatically for concrete joint types.
	*/
	public type: number /** int */;
	/**
	* Use this to attach application specific data to your joints.
	*/
	public userData: any;
	/**
	* The first attached body.
	*/
	public bodyA: b2Body;
	/**
	* The second attached body.
	*/
	public bodyB: b2Body;
	/**
	* Set this flag to true if the attached bodies should collide.
	*/
	public collideConnected: boolean;

}