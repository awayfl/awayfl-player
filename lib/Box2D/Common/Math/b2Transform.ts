import { b2Vec2, b2Mat22 } from '../Math';

/**
* A transform contains translation and rotation. It is used to represent
* the position and orientation of rigid frames.
*/
export class b2Transform {
	readonly __fast__ = true;
	/**
    * The default constructor does nothing (for performance).
    */
	constructor(pos: b2Vec2 = null, r: b2Mat22 = null) {
		if (pos) {
			this.position.SetV(pos);
			this.R.SetM(r);

		}
	}

	/**
    * Initialize using a position vector and a rotation matrix.
    */
	public Initialize(pos: b2Vec2, r: b2Mat22): void {
		this.position.SetV(pos);
		this.R.SetM(r);
	}

	/**
    * Set this to the identity transform.
    */
	public SetIdentity(): void {
		this.position.SetZero();
		this.R.SetIdentity();
	}

	public Set(x: b2Transform): void {

		this.position.SetV(x.position);

		this.R.SetM(x.R);

	}

	/**
     * Calculate the angle that the rotation matrix represents.
     */
	public GetAngle(): number {
		return Math.atan2(this.R.col1.y, this.R.col1.x);
	}

	public position: b2Vec2 = new b2Vec2;
	public R: b2Mat22 = new b2Mat22();
}