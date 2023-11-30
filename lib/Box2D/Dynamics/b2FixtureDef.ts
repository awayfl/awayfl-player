import { b2Shape } from '../Collision/Shapes/b2Shape';
import { b2FilterData } from './b2FilterData';

/**
 * A fixture definition is used to create a fixture. This class defines an
 * abstract fixture definition. You can reuse fixture definitions safely.
 */
export class b2FixtureDef {
	__fast__: boolean = true;

	/**
	 * The constructor sets the default fixture definition values.
	 */
	constructor() {
		this.shape = null;
		this.userData = null;
		this.friction = 0.2;
		this.restitution = 0.0;
		this.density = 0.0;
		this.filter.categoryBits = 0x0001;
		this.filter.maskBits = 0xFFFF;
		this.filter.groupIndex = 0;
		this.isSensor = false;
	}

	/**
	 * The shape, this must be set. The shape will be cloned, so you
	 * can create the shape on the stack.
	 */
	public shape: b2Shape;

	/**
	 * Use this to store application specific fixture data.
	 */
	public userData: any;

	/**
	 * The friction coefficient, usually in the range [0,1].
	 */
	public friction: number;

	/**
	 * The restitution (elasticity) usually in the range [0,1].
	 */
	public restitution: number;

	/**
	 * The density, usually in kg/m^2.
	 */
	public density: number;

	/**
	 * A sensor shape collects contact information but never generates a collision
	 * response.
	 */
	public isSensor: boolean;

	/**
	 * Contact filtering data.
	 */
	public filter: b2FilterData = new b2FilterData();
}
