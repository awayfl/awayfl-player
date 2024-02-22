import { b2Manifold } from '../../Collision/b2Manifold';
import { b2Body } from '../b2Body';
import { b2Mat22, b2Vec2 } from '../../Common/Math';
import { b2ContactConstraintPoint } from '../Contacts';
import { b2Settings } from '../../Common/b2Settings';

/**
* @private
*/
export class b2ContactConstraint {
	constructor() {
		this.points = new Array<b2ContactConstraintPoint>(b2Settings.b2_maxManifoldPoints);
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.points[i] = new b2ContactConstraintPoint();
		}

	}

	public points: Array<b2ContactConstraintPoint>;
	public localPlaneNormal: b2Vec2 = new b2Vec2();
	public localPoint: b2Vec2 = new b2Vec2();
	public normal: b2Vec2 = new b2Vec2();
	public normalMass: b2Mat22 = new b2Mat22();
	public K: b2Mat22 = new b2Mat22();
	public bodyA: b2Body;
	public bodyB: b2Body;
	public type: number /** int */;//b2Manifold::Type
	public radius: number;
	public friction: number;
	public restitution: number;
	public pointCount: number /** int */;
	public manifold: b2Manifold;
}