import { b2Vec2 } from '../Common/Math';
import { b2ManifoldPoint } from './b2ManifoldPoint';
import { b2Settings } from '../Common/b2Settings';

/**
 * A manifold for two touching convex shapes.
 * Box2D supports multiple types of contact:
 * - clip point versus plane with radius
 * - point versus point with radius (circles)
 * The local point usage depends on the manifold type:
 * -e_circles: the local center of circleA
 * -e_faceA: the center of faceA
 * -e_faceB: the center of faceB
 * Similarly the local normal usage:
 * -e_circles: not used
 * -e_faceA: the normal on polygonA
 * -e_faceB: the normal on polygonB
 * We store contacts in this way so that position correction can
 * account for movement, which is critical for continuous physics.
 * All contact scenarios must be expressed in one of these types.
 * This structure is stored across time steps, so we keep it small.
 */
export class b2Manifold {
	readonly __fast__ = true;

	constructor() {
		this.m_points = new Array<b2ManifoldPoint>(b2Settings.b2_maxManifoldPoints);
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.m_points[i] = new b2ManifoldPoint();
		}
		this.m_localPlaneNormal = new b2Vec2();
		this.m_localPoint = new b2Vec2();
	}

	public Reset(): void {
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			(this.m_points[i] as b2ManifoldPoint).Reset();
		}
		this.m_localPlaneNormal.SetZero();
		this.m_localPoint.SetZero();
		this.m_type = 0;
		this.m_pointCount = 0;
	}

	public Set(m: b2Manifold): void {
		this.m_pointCount = m.m_pointCount;
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			(this.m_points[i] as b2ManifoldPoint).Set(m.m_points[i]);
		}
		this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
		this.m_localPoint.SetV(m.m_localPoint);
		this.m_type = m.m_type;
	}

	public Copy(): b2Manifold {
		const copy: b2Manifold = new b2Manifold();
		copy.Set(this);
		return copy;
	}

	/** The points of contact */
	public m_points: Array<b2ManifoldPoint>;
	/** Not used for Type e_points*/
	public m_localPlaneNormal: b2Vec2;
	/** Usage depends on manifold type */
	public m_localPoint: b2Vec2;
	public m_type: number /** int */;
	/** The number of manifold points */
	public m_pointCount: number /** int */ = 0;

	//enum Type
	public static readonly e_circles: number /** int */ = 0x0001;
	public static readonly e_faceA: number /** int */ = 0x0002;
	public static readonly e_faceB: number /** int */ = 0x0004;
}