import { b2Vec2 } from '../Common/Math';

/**
 * Specifies a segment for use with RayCast functions.
 */
export class b2RayCastInput {
	__fast__: boolean = true;

	constructor(p1: b2Vec2 = null, p2: b2Vec2 = null, maxFraction: number = 1) {
		if (p1)
			this.p1.SetV(p1);
		if (p2)
			this.p2.SetV(p2);
		this.maxFraction = maxFraction;
	}

	/**
	 * The start point of the ray
	 */
	public p1: b2Vec2 = new b2Vec2();
	/**
	 * The end point of the ray
	 */
	public p2: b2Vec2 = new b2Vec2();
	/**
	 * Truncate the ray to reach up to this fraction from p1 to p2
	 */
	public maxFraction: number;
}