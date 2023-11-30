import { b2Vec2 } from '../Common/Math';

/**
 * Output for b2Distance.
 */
export class b2DistanceOutput {
	__fast__: boolean = true;

	/**
	 * Closest point on shapea
	 */
	public pointA: b2Vec2 = new b2Vec2();

	/**
	 * Closest point on shapeb
	 */
	public pointB: b2Vec2 = new b2Vec2();

	public distance: number;

	/**
	 * Number of gjk iterations used
	 */
	public iterations: number /** int */;
}