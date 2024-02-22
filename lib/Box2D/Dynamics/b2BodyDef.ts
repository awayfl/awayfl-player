import { b2Vec2 } from '../Common/Math';
import { b2Body } from './b2Body';

/**
* A body definition holds all the data needed to construct a rigid body.
* You can safely re-use body definitions.
*/
export class b2BodyDef {
	__fast__ = true;

	/**
    * This constructor sets the body definition default values.
    */
	constructor() {
		this.userData = null;
		this.position.Set(0.0, 0.0);
		this.angle = 0.0;
		this.linearVelocity.Set(0, 0);
		this. angularVelocity = 0.0;
		this.linearDamping = 0.0;
		this.angularDamping = 0.0;
		this.allowSleep = true;
		this.awake = true;
		this.fixedRotation = false;
		this.bullet = false;
		this.type = b2Body.b2_staticBody;
		this.active = true;
		this.inertiaScale = 1.0;
	}

	/** The body type: static, kinematic, or dynamic. A member of the b2BodyType class
     * Note: if a dynamic body would have zero mass, the mass is set to one.
     * @see b2Body#b2_staticBody
     * @see b2Body#b2_dynamicBody
     * @see b2Body#b2_kinematicBody
     */
	public type: number /** uint */;

	/**
     * The world position of the body. Avoid creating bodies at the origin
     * since this can lead to many overlapping shapes.
     */
	public position: b2Vec2 = new b2Vec2();

	/**
     * The world angle of the body in radians.
     */
	public angle: number;

	/**
     * The linear velocity of the body's origin in world co-ordinates.
     */
	public linearVelocity: b2Vec2 = new b2Vec2();

	/**
     * The angular velocity of the body.
     */
	public angularVelocity: number;

	/**
     * Linear damping is use to reduce the linear velocity. The damping parameter
     * can be larger than 1.0f but the damping effect becomes sensitive to the
     * time step when the damping parameter is large.
     */
	public linearDamping: number;

	/**
     * Angular damping is use to reduce the angular velocity. The damping parameter
     * can be larger than 1.0f but the damping effect becomes sensitive to the
     * time step when the damping parameter is large.
     */
	public angularDamping: number;

	/**
     * Set this flag to false if this body should never fall asleep. Note that
     * this increases CPU usage.
     */
	public allowSleep: boolean ;

	/**
     * Is this body initially awake or sleeping?
     */
	public awake: boolean ;

	/**
     * Should this body be prevented from rotating? Useful for characters.
     */
	public fixedRotation: boolean ;

	/**
     * Is this a fast moving body that should be prevented from tunneling through
     * other moving bodies? Note that all bodies are prevented from tunneling through
     * static bodies.
     * @warning You should use this flag sparingly since it increases processing time.
     */
	public bullet: boolean ;

	/**
     * Does this body start out active?
     */
	public active: boolean ;

	/**
     * Use this to store application specific body data.
     */
	public userData: any;

	/**
     * Scales the inertia tensor.
     * @warning Experimental
     */
	public inertiaScale: number;
}