import { b2Transform, b2Mat22, b2Vec2 } from '../Math';

/**
* This describes the motion of a body/shape for TOI computation.
* Shapes are defined with respect to the body origin, which may
* no coincide with the center of mass. However, to support dynamics
* we must interpolate the center of mass position.
*/
export class b2Sweep {
	public Set(other: b2Sweep): void {
		this.localCenter.SetV(other.localCenter);
		this.c0.SetV(other.c0);
		this.c.SetV(other.c);
		this.a0 = other.a0;
		this.a = other.a;
		this.t0 = other.t0;
	}

	public Copy(): b2Sweep {
		const copy: b2Sweep = new b2Sweep();
		copy.localCenter.SetV(this.localCenter);
		copy.c0.SetV(this.c0);
		copy.c.SetV(this.c);
		copy.a0 = this.a0;
		copy.a = this.a;
		copy.t0 = this.t0;
		return copy;
	}

	/**
	* Get the interpolated transform at a specific time.
	* @param alpha is a factor in [0,1], where 0 indicates t0.
	*/
	public GetTransform(xf: b2Transform, alpha: number): void {
		xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
		xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
		const angle: number = (1.0 - alpha) * this.a0 + alpha * this.a;
		xf.R.Set(angle);

		// Shift to origin
		//xf->position -= b2Mul(xf->R, localCenter);
		const tMat: b2Mat22 = xf.R;
		xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
		xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);
	}

	/**
	* Advance the sweep forward, yielding a new initial state.
	* @param t the new initial time.
	*/
	public Advance(t: number): void {
		if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
			const alpha: number = (t - this.t0) / (1.0 - this.t0);
			//this.c0 = (1.0f - alpha) * c0 + alpha * c;
			this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
			this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
			this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
			this.t0 = t;
		}
	}

	/** Local center of mass position */
	public localCenter: b2Vec2 = new b2Vec2();
	/** Center world position */
	public c0: b2Vec2 = new b2Vec2;
	/** Center world position */
	public c: b2Vec2 = new b2Vec2();
	/** World angle */
	public a0: number;
	/** World angle */
	public a: number;
	/** Time interval = [t0,1], where t0 is in [0,1] */
	public t0: number;
}