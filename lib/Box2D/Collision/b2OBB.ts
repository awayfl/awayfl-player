import { b2Mat22, b2Vec2 } from '../Common/Math';

/**
* An oriented bounding box.
*/
export class b2OBB {
	__fast__: boolean = true;

	/** The rotation matrix */
	public R: b2Mat22 = new b2Mat22();
	/** The local centroid */
	public center: b2Vec2 = new b2Vec2();
	/** The half-widths */
	public extents: b2Vec2 = new b2Vec2();
}