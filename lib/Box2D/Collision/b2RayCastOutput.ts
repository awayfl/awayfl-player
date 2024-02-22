import { b2Vec2 } from '../Common/Math';

/**
 * Returns data on the collision between a ray and a shape.
 */
export class b2RayCastOutput {
	__fast__: boolean = true;

	/**
	 * The normal at the point of collision
	 */
	public normal: b2Vec2 = new b2Vec2();
	/**
	 * The fraction between p1 and p2 that the collision occurs at
	 */
	public fraction: number;
}