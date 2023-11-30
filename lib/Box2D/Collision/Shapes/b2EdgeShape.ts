import { b2Shape } from './b2Shape';
import { b2MassData } from './b2MassData';
import { b2Settings } from '../../Common/b2Settings';
import { b2Transform, b2Vec2, b2Mat22, b2Math } from '../../Common/Math';
import { b2RayCastOutput } from '../b2RayCastOutput';
import { b2RayCastInput } from '../b2RayCastInput';
import { b2AABB } from '../b2AABB';

/**
 * An edge shape.
 * @private
 * @see b2EdgeChainDef
 */
export class b2EdgeShape extends b2Shape {
	__fast__: boolean = true;

	/**
	* Returns false. Edges cannot contain points.
	*/
	public TestPoint(transform: b2Transform, p: b2Vec2): boolean {
		return false;
	}

	/**
	* @inheritDoc
	*/
	public RayCast(output: b2RayCastOutput, input: b2RayCastInput, transform: b2Transform): boolean {
		let tMat: b2Mat22;
		const rX: number = input.p2.x - input.p1.x;
		const rY: number = input.p2.y - input.p1.y;

		//b2Vec2 v1 = b2Mul(transform, this.m_v1);
		tMat = transform.R;
		const v1X: number = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
		const v1Y: number = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);

		//b2Vec2 n = b2Cross(d, 1.0);
		const nX: number = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y) - v1Y;
		const nY: number = -(transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y) - v1X);

		const k_slop: number = 100.0 * Number.MIN_VALUE;
		const denom: number = -(rX * nX + rY * nY);

		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop) {
			// Does the segment intersect the infinite line associated with this segment?
			const bX: number = input.p1.x - v1X;
			const bY: number = input.p1.y - v1Y;
			let a: number = (bX * nX + bY * nY);

			if (0.0 <= a && a <= input.maxFraction * denom) {
				const mu2: number = -rX * bY + rY * bX;

				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
					a /= denom;
					output.fraction = a;
					const nLen: number = Math.sqrt(nX * nX + nY * nY);
					output.normal.x = nX / nLen;
					output.normal.y = nY / nLen;
					return true;
				}
			}
		}

		return false;
	}

	/**
	* @inheritDoc
	*/
	public ComputeAABB(aabb: b2AABB, transform: b2Transform): void {
		const tMat: b2Mat22 = transform.R;
		//b2Vec2 v1 = b2Mul(transform, this.m_v1);
		const v1X: number = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
		const v1Y: number = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
		//b2Vec2 v2 = b2Mul(transform, this.m_v2);
		const v2X: number = transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y);
		const v2Y: number = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y);
		if (v1X < v2X) {
			aabb.lowerBound.x = v1X;
			aabb.upperBound.x = v2X;
		} else {
			aabb.lowerBound.x = v2X;
			aabb.upperBound.x = v1X;
		}
		if (v1Y < v2Y) {
			aabb.lowerBound.y = v1Y;
			aabb.upperBound.y = v2Y;
		} else {
			aabb.lowerBound.y = v2Y;
			aabb.upperBound.y = v1Y;
		}
	}

	/**
	* @inheritDoc
	*/
	public ComputeMass(massData: b2MassData, density: number): void {
		massData.mass = 0;
		massData.center.SetV(this.m_v1);
		massData.I = 0;
	}

	/**
	* @inheritDoc
	*/
	public ComputeSubmergedArea(
		normal: b2Vec2,
		offset: number,
		xf: b2Transform,
		c: b2Vec2): number {
		// Note that v0 is independant of any details of the specific edge
		// We are relying on v0 being consistent between multiple edges of the same body
		//b2Vec2 v0 = offset * normal;
		const v0: b2Vec2 = new b2Vec2(normal.x * offset, normal.y * offset);

		const v1: b2Vec2 = b2Math.MulX(xf, this.m_v1);
		const v2: b2Vec2 = b2Math.MulX(xf, this.m_v2);

		const d1: number = b2Math.Dot(normal, v1) - offset;
		const d2: number = b2Math.Dot(normal, v2) - offset;
		if (d1 > 0) {
			if (d2 > 0) {
				return 0;
			} else {
				//v1 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				v1.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v1.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
		} else {
			if (d2 > 0) {
				//v2 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				v2.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v2.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			} else {
				// Nothing
			}
		}
		// v0,v1,v2 represents a fully submerged triangle
		// Area weighted centroid
		c.x = (v0.x + v1.x + v2.x) / 3;
		c.y = (v0.y + v1.y + v2.y) / 3;

		//b2Vec2 e1 = v1 - v0;
		//b2Vec2 e2 = v2 - v0;
		//return 0.5f * b2Cross(e1, e2);
		return 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x));
	}

	/**
	* Get the distance from vertex1 to vertex2.
	*/
	public GetLength(): number {
		return this.m_length;
	}

	/**
	* Get the local position of vertex1 in parent body.
	*/
	public GetVertex1(): b2Vec2 {
		return this.m_v1;
	}

	/**
	* Get the local position of vertex2 in parent body.
	*/
	public GetVertex2(): b2Vec2 {
		return this.m_v2;
	}

	/**
	* Get a core vertex in local coordinates. These vertices
	* represent a smaller edge that is used for time of impact
	* computations.
	*/
	public GetCoreVertex1(): b2Vec2 {
		return this.m_coreV1;
	}

	/**
	* Get a core vertex in local coordinates. These vertices
	* represent a smaller edge that is used for time of impact
	* computations.
	*/
	public GetCoreVertex2(): b2Vec2 {
		return this.m_coreV2;
	}

	/**
	* Get a perpendicular unit vector, pointing
	* from the solid side to the empty side.
	*/
	public GetNormalVector(): b2Vec2 {
		return this.m_normal;
	}

	/**
	* Get a parallel unit vector, pointing
	* from vertex1 to vertex2.
	*/
	public GetDirectionVector(): b2Vec2 {
		return this.m_direction;
	}

	/**
	* Returns a unit vector halfway between
	* m_direction and m_prevEdge.m_direction.
	*/
	public GetCorner1Vector(): b2Vec2 {
		return this.m_cornerDir1;
	}

	/**
	* Returns a unit vector halfway between
	* m_direction and m_nextEdge.m_direction.
	*/
	public GetCorner2Vector(): b2Vec2 {
		return this.m_cornerDir2;
	}

	/**
	* Returns true if the first corner of this edge
	* bends towards the solid side.
	*/
	public Corner1IsConvex(): boolean {
		return this.m_cornerConvex1;
	}

	/**
	* Returns true if the second corner of this edge
	* bends towards the solid side.
	*/
	public Corner2IsConvex(): boolean {
		return this.m_cornerConvex2;
	}

	/**
	* Get the first vertex and apply the supplied transform.
	*/
	public GetFirstVertex(xf: b2Transform): b2Vec2 {
		//return b2Mul(xf, m_coreV1);
		const tMat: b2Mat22 = xf.R;
		return new b2Vec2(xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y),
		                  xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y));
	}

	/**
	* Get the next edge in the chain.
	*/
	public GetNextEdge(): b2EdgeShape {
		return this.m_nextEdge;
	}

	/**
	* Get the previous edge in the chain.
	*/
	public GetPrevEdge(): b2EdgeShape {
		return this.m_prevEdge;
	}

	private s_supportVec: b2Vec2 = new b2Vec2();
	/**
	* Get the support point in the given world direction.
	* Use the supplied transform.
	*/
	public Support(xf: b2Transform, dX: number, dY: number): b2Vec2 {
		const tMat: b2Mat22 = xf.R;
		//b2Vec2 v1 = b2Mul(xf, m_coreV1);
		const v1X: number = xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y);
		const v1Y: number = xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y);

		//b2Vec2 v2 = b2Mul(xf, m_coreV2);
		const v2X: number = xf.position.x + (tMat.col1.x * this.m_coreV2.x + tMat.col2.x * this.m_coreV2.y);
		const v2Y: number = xf.position.y + (tMat.col1.y * this.m_coreV2.x + tMat.col2.y * this.m_coreV2.y);

		if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
			this.s_supportVec.x = v1X;
			this.s_supportVec.y = v1Y;
		} else {
			this.s_supportVec.x = v2X;
			this.s_supportVec.y = v2Y;
		}
		return this.s_supportVec;
	}

	//--------------- Internals Below -------------------

	/**
	* @private
	*/
	constructor(v1: b2Vec2, v2: b2Vec2) {
		super();
		this.m_type = b2Shape.e_edgeShape;

		this.m_prevEdge = null;
		this.m_nextEdge = null;

		this.m_v1 = v1;
		this.m_v2 = v2;

		this.m_direction.Set(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);
		this.m_length = this.m_direction.Normalize();
		this.m_normal.Set(this.m_direction.y, -this.m_direction.x);

		this.m_coreV1.Set(-b2Settings.b2_toiSlop * (this.m_normal.x - this.m_direction.x) + this.m_v1.x,
		             -b2Settings.b2_toiSlop * (this.m_normal.y - this.m_direction.y) + this.m_v1.y);
					 this.m_coreV2.Set(-b2Settings.b2_toiSlop * (this.m_normal.x + this.m_direction.x) + this.m_v2.x,
		             -b2Settings.b2_toiSlop * (this.m_normal.y + this.m_direction.y) + this.m_v2.y);

					 this.m_cornerDir1 = this.m_normal;
					 this.m_cornerDir2.Set(-this.m_normal.x, -this.m_normal.y);
	}

	/**
	* @private
	*/
	public SetPrevEdge(edge: b2EdgeShape, core: b2Vec2, cornerDir: b2Vec2, convex: boolean): void {
		this.m_prevEdge = edge;
		this.m_coreV1 = core;
		this.m_cornerDir1 = cornerDir;
		this.m_cornerConvex1 = convex;
	}

	/**
	* @private
	*/
	public SetNextEdge(edge: b2EdgeShape, core: b2Vec2, cornerDir: b2Vec2, convex: boolean): void {
		this.m_nextEdge = edge;
		this.m_coreV2 = core;
		this.m_cornerDir2 = cornerDir;
		this.m_cornerConvex2 = convex;
	}

	public m_v1: b2Vec2 = new b2Vec2();
	public m_v2: b2Vec2 = new b2Vec2();

	public m_coreV1: b2Vec2 = new b2Vec2();
	public m_coreV2: b2Vec2 = new b2Vec2();

	public m_length: number;

	public m_normal: b2Vec2 = new b2Vec2();

	public m_direction: b2Vec2 = new b2Vec2();

	public m_cornerDir1: b2Vec2 = new b2Vec2();

	public m_cornerDir2: b2Vec2 = new b2Vec2();

	public m_cornerConvex1: boolean;
	public m_cornerConvex2: boolean;

	public m_nextEdge: b2EdgeShape;
	public m_prevEdge: b2EdgeShape;

}