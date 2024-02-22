import { b2Shape } from './Shapes/b2Shape';
import { b2Vec2 } from '../Common/Math';
import { b2ContactID } from './b2ContactID';

/**
* This structure is used to report contact points.
*/
export class b2ContactPoint {
	readonly __fast__ = true;

	/** The first shape */
	public shape1: b2Shape;
	/** The second shape */
	public shape2: b2Shape;
	/** Position in world coordinates */
	public position: b2Vec2 = new b2Vec2();
	/** Velocity of point on body2 relative to point on body1 (pre-solver) */
	public velocity: b2Vec2 = new b2Vec2();
	/** Points from shape1 to shape2 */
	public normal: b2Vec2 = new b2Vec2();
	/** The separation is negative when shapes are touching */
	public separation: number;
	/** The combined friction coefficient */
	public friction: number;
	/** The combined restitution coefficient */
	public restitution: number;
	/** The contact id identifies the features in contact */
	public id: b2ContactID = new b2ContactID();
}