import { b2Vec2 } from '../Common/Math';
import { b2ContactID } from './b2ContactID';

/**
 * A manifold point is a contact point belonging to a contact
 * manifold. It holds details related to the geometry and dynamics
 * of the contact points.
 * The local point usage depends on the manifold type:
 * -e_circles: the local center of circleB
 * -e_faceA: the local center of cirlceB or the clip point of polygonB
 * -e_faceB: the clip point of polygonA
 * This structure is stored across time steps, so we keep it small.
 * Note: the impulses are used for internal caching and may not
 * provide reliable contact forces, especially for high speed collisions.
 */
export class b2ManifoldPoint {
	readonly __fast__ = true;

	constructor() {
		this.Reset();
	}

	public Reset(): void {
		this.m_localPoint.SetZero();
		this.m_normalImpulse = 0.0;
		this.m_tangentImpulse = 0.0;
		this.m_id.key = 0;
	}

	public Set(m: b2ManifoldPoint): void {
		this.m_localPoint.SetV(m.m_localPoint);
		this.m_normalImpulse = m.m_normalImpulse;
		this.m_tangentImpulse = m.m_tangentImpulse;
		this.m_id.Set(m.m_id);
	}

	public m_localPoint: b2Vec2 = new b2Vec2();
	public m_normalImpulse: number;
	public m_tangentImpulse: number;
	public m_id: b2ContactID = new b2ContactID();
}