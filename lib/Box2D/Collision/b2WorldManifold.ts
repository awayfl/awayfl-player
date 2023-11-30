import { b2Manifold } from './b2Manifold';
import { b2Transform, b2Mat22, b2Vec2 } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';

/**
 * This is used to compute the current state of a contact manifold.
 */
export class b2WorldManifold {
	public constructor() {
		this.m_points = new Array<b2Vec2>(b2Settings.b2_maxManifoldPoints);
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.m_points[i] = new b2Vec2();
		}
	}

	/**
	 * Evaluate the manifold with supplied transforms. This assumes
	 * modest motion from the original state. This does not change the
	 * point count, impulses, etc. The radii must come from the shapes
	 * that generated the manifold.
	 */
	public Initialize(manifold: b2Manifold,
		xfA: b2Transform, radiusA: number,
		xfB: b2Transform, radiusB: number): void {
		if (manifold.m_pointCount == 0) {
			return;
		}

		let i: number /** int */;
		let tVec: b2Vec2;
		let tMat: b2Mat22;
		let normalX: number;
		let normalY: number;
		let planePointX: number;
		let planePointY: number;
		let clipPointX: number;
		let clipPointY: number;

		switch (manifold.m_type) {
			case b2Manifold.e_circles:
				{
				//var pointA:b2Vec2 = b2Math.b2MulX(xfA, manifold.m_localPoint);
					tMat = xfA.R;
					tVec = manifold.m_localPoint;
					const pointAX: number = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					const pointAY: number = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
					//var pointB:b2Vec2 = b2Math.b2MulX(xfB, manifold.m_points[0].m_localPoint);
					tMat = xfB.R;
					tVec = manifold.m_points[0].m_localPoint;
					const pointBX: number = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					const pointBY: number = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

					const dX: number = pointBX - pointAX;
					const dY: number = pointBY - pointAY;
					const d2: number = dX * dX + dY * dY;
					if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
						const d: number = Math.sqrt(d2);
						this.m_normal.x = dX / d;
						this.m_normal.y = dY / d;
					} else {
						this.m_normal.x = 1;
						this.m_normal.y = 0;
					}

					//b2Vec2 cA = pointA + radiusA * m_normal;
					const cAX: number = pointAX + radiusA * this.m_normal.x;
					const cAY: number = pointAY + radiusA * this.m_normal.y;
					//b2Vec2 cB = pointB - radiusB * m_normal;
					const cBX: number = pointBX - radiusB * this.m_normal.x;
					const cBY: number = pointBY - radiusB * this.m_normal.y;
					this.m_points[0].x = 0.5 * (cAX + cBX);
					this.m_points[0].y = 0.5 * (cAY + cBY);
				}
				break;
			case b2Manifold.e_faceA:
				{
				//normal = b2Math.b2MulMV(xfA.R, manifold.m_localPlaneNormal);
					tMat = xfA.R;
					tVec = manifold.m_localPlaneNormal;
					normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

					//planePoint = b2Math.b2MulX(xfA, manifold.m_localPoint);
					tMat = xfA.R;
					tVec = manifold.m_localPoint;
					planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

					// Ensure normal points from A to B
					this.m_normal.x = normalX;
					this.m_normal.y = normalY;
					for (i = 0; i < manifold.m_pointCount; i++) {
					//clipPoint = b2Math.b2MulX(xfB, manifold.m_points[i].m_localPoint);
						tMat = xfB.R;
						tVec = manifold.m_points[i].m_localPoint;
						clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
						clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

						//b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
						//b2Vec2 cB = clipPoint - radiusB * normal;
						//m_points[i] = 0.5f * (cA + cB);
						this.m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
						this.m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY;

					}
				}
				break;
			case b2Manifold.e_faceB:
				{
				//normal = b2Math.b2MulMV(xfB.R, manifold.m_localPlaneNormal);
					tMat = xfB.R;
					tVec = manifold.m_localPlaneNormal;
					normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

					//planePoint = b2Math.b2MulX(xfB, manifold.m_localPoint);
					tMat = xfB.R;
					tVec = manifold.m_localPoint;
					planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

					// Ensure normal points from A to B
					this.m_normal.x = -normalX;
					this.m_normal.y = -normalY;
					for (i = 0; i < manifold.m_pointCount; i++) {
					//clipPoint = b2Math.b2MulX(xfA, manifold.m_points[i].m_localPoint);
						tMat = xfA.R;
						tVec = manifold.m_points[i].m_localPoint;
						clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
						clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

						//b2Vec2 cA = clipPoint - radiusA * normal;
						//b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
						//m_points[i] = 0.5f * (cA + cB);
						this.m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
						this.m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY;

					}
				}
				break;
		}
	}

	/**
	 * world vector pointing from A to B
	 */
	public m_normal: b2Vec2 = new b2Vec2();
	/**
	 * world contact point (point of intersection)
	 */
	public m_points: Array<b2Vec2>;

}