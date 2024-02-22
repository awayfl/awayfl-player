/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import { b2Shape } from './b2Shape';
import { b2XForm, b2Vec2, b2Mat22, b2Math } from '../../Common/Math';
import { b2Settings } from '../../Common/b2Settings';
import { b2AABB } from '../b2AABB';
import { b2OBB } from '../b2OBB';
import { b2ShapeDef } from './b2ShapeDef';
import { b2PolygonDef } from './b2PolygonDef';
import { b2Segment } from '../b2Segment';
import { b2MassData } from './b2MassData';

/// Convex polygon. The vertices must be in CCW order for a right-handed
/// coordinate system with the z-axis coming out of the screen.

export class b2PolygonShape extends b2Shape {
	/// @see b2Shape::TestPoint
	public TestPoint(xf: b2XForm, p: b2Vec2): boolean {
		let tVec: b2Vec2;

		//b2Vec2 pLocal = b2MulT(xf.R, p - xf.position);
		const tMat: b2Mat22 = xf.R;
		let tX: number = p.x - xf.position.x;
		let tY: number = p.y - xf.position.y;
		const pLocalX: number = (tX * tMat.col1.x + tY * tMat.col1.y);
		const pLocalY: number = (tX * tMat.col2.x + tY * tMat.col2.y);

		for (let i: number /** int */ = 0; i < this.m_vertexCount; ++i) {
			//float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			tVec = this.m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			tVec = this.m_normals[i];
			const dot: number = (tVec.x * tX + tVec.y * tY);
			if (dot > 0.0) {
				return false;
			}
		}

		return true;
	}

	/// @see b2Shape::TestSegment
	public TestSegment(xf: b2XForm,
		lambda: number[], // float ptr
		normal: b2Vec2, // ptr
		segment: b2Segment,
		maxLambda: number): boolean {
		let lower: number = 0.0;
		let upper: number = maxLambda;

		let tX: number;
		let tY: number;
		let tMat: b2Mat22;
		let tVec: b2Vec2;

		//b2Vec2 p1 = b2MulT(xf.R, segment.p1 - xf.position);
		tX = segment.p1.x - xf.position.x;
		tY = segment.p1.y - xf.position.y;
		tMat = xf.R;
		const p1X: number = (tX * tMat.col1.x + tY * tMat.col1.y);
		const p1Y: number = (tX * tMat.col2.x + tY * tMat.col2.y);
		//b2Vec2 p2 = b2MulT(xf.R, segment.p2 - xf.position);
		tX = segment.p2.x - xf.position.x;
		tY = segment.p2.y - xf.position.y;
		tMat = xf.R;
		const p2X: number = (tX * tMat.col1.x + tY * tMat.col1.y);
		const p2Y: number = (tX * tMat.col2.x + tY * tMat.col2.y);
		//b2Vec2 d = p2 - p1;
		const dX: number = p2X - p1X;
		const dY: number = p2Y - p1Y;
		let index: number /** int */ = -1;

		for (let i: number /** int */ = 0; i < this.m_vertexCount; ++i) {
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0

			//float32 numerator = b2Dot(this.m_normals[i], this.m_vertices[i] - p1);
			tVec = this.m_vertices[i];
			tX = tVec.x - p1X;
			tY = tVec.y - p1Y;
			tVec = this.m_normals[i];
			const numerator: number = (tVec.x * tX + tVec.y * tY);
			//float32 denominator = b2Dot(this.m_normals[i], d);
			const denominator: number = (tVec.x * dX + tVec.y * dY);

			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.

			if (denominator < 0.0 && numerator < lower * denominator) {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			} else if (denominator > 0.0 && numerator < upper * denominator) {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}

			if (upper < lower) {
				return false;
			}
		}

		//b2Settings.b2Assert(0.0 <= lower && lower <= maxLambda);

		if (index >= 0) {
			//*lambda = lower;
			lambda[0] = lower;
			//*normal = b2Mul(xf.R, this.m_normals[index]);
			tMat = xf.R;
			tVec = this.m_normals[index];
			normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			return true;
		}

		return false;
	}

	/// @see b2Shape::ComputeAABB
	//
	private static s_computeMat: b2Mat22 = new b2Mat22();
	//
	public ComputeAABB(aabb: b2AABB, xf: b2XForm): void {
		let tMat: b2Mat22;
		let tVec: b2Vec2;

		const R: b2Mat22 = b2PolygonShape.s_computeMat;
		//b2Mat22 R = b2Mul(xf.R, this.m_obb.R);
		tMat = xf.R;
		tVec = this.m_obb.R.col1;
		//R.col1 = b2MulMV(A, B.col1)
		R.col1.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		R.col1.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//
		tVec = this.m_obb.R.col2;
		//R.col1 = b2MulMV(A, B.col2)
		R.col2.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		R.col2.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		//b2Mat22 absR = b2Abs(R);
		R.Abs();
		const absR: b2Mat22 = R;
		//b2Vec2 h = b2Mul(absR, this.m_obb.extents);
		tVec = this.m_obb.extents;
		const hX: number = (absR.col1.x * tVec.x + absR.col2.x * tVec.y);
		const hY: number = (absR.col1.y * tVec.x + absR.col2.y * tVec.y);
		//b2Vec2 position = xf.position + b2Mul(xf.R, this.m_obb.center);
		tMat = xf.R;
		tVec = this.m_obb.center;
		const positionX: number = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const positionY: number = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//aabb->lowerBound = position - h;
		aabb.lowerBound.Set(positionX - hX, positionY - hY);
		//aabb->upperBound = position + h;
		aabb.upperBound.Set(positionX + hX, positionY + hY);
	}

	/// @see b2Shape::ComputeSweptAABB
	//
	private static s_sweptAABB1: b2AABB = new b2AABB();
	private static s_sweptAABB2: b2AABB = new b2AABB();
	//
	public ComputeSweptAABB(aabb: b2AABB,
		transform1: b2XForm,
		transform2: b2XForm): void {
		//b2AABB aabb1, aabb2;
		const aabb1: b2AABB = b2PolygonShape.s_sweptAABB1;
		const aabb2: b2AABB = b2PolygonShape.s_sweptAABB2;
		this.ComputeAABB(aabb1, transform1);
		this.ComputeAABB(aabb2, transform2);
		//aabb.lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
		aabb.lowerBound.Set((aabb1.lowerBound.x < aabb2.lowerBound.x ? aabb1.lowerBound.x : aabb2.lowerBound.x),
			(aabb1.lowerBound.y < aabb2.lowerBound.y ? aabb1.lowerBound.y : aabb2.lowerBound.y));
		//aabb.upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
		aabb.upperBound.Set((aabb1.upperBound.x > aabb2.upperBound.x ? aabb1.upperBound.x : aabb2.upperBound.x),
			(aabb1.upperBound.y > aabb2.upperBound.y ? aabb1.upperBound.y : aabb2.upperBound.y));
	}

	/// @see b2Shape::ComputeMass
	//

	//
	public ComputeMass(massData: b2MassData): void {
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		//b2Settings.b2Assert(this.m_vertexCount >= 3);

		//b2Vec2 center; center.Set(0.0f, 0.0f);
		let centerX: number = 0.0;
		let centerY: number = 0.0;
		let area: number = 0.0;
		let I: number = 0.0;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		//b2Vec2 pRef(0.0f, 0.0f);
		const p1X: number = 0.0;
		const p1Y: number = 0.0;
		/*#if 0
		// This code would put the reference point inside the polygon.
		for (int32 i = 0; i < this.m_vertexCount; ++i)
		{
			pRef += this.m_vertices[i];
		}
		pRef *= 1.0f / count;
		#endif*/

		const k_inv3: number = 1.0 / 3.0;

		for (let i: number /** int */ = 0; i < this.m_vertexCount; ++i) {
			// Triangle vertices.
			//b2Vec2 p1 = pRef;
			//
			//b2Vec2 p2 = this.m_vertices[i];
			const p2: b2Vec2 = this.m_vertices[i];
			//b2Vec2 p3 = i + 1 < this.m_vertexCount ? this.m_vertices[i+1] : this.m_vertices[0];
			const p3: b2Vec2 = i + 1 < this.m_vertexCount ? this.m_vertices[i + 1] : this.m_vertices[0];

			//b2Vec2 e1 = p2 - p1;
			const e1X: number = p2.x - p1X;
			const e1Y: number = p2.y - p1Y;
			//b2Vec2 e2 = p3 - p1;
			const e2X: number = p3.x - p1X;
			const e2Y: number = p3.y - p1Y;

			//float32 D = b2Cross(e1, e2);
			const D: number = e1X * e2Y - e1Y * e2X;

			//float32 triangleArea = 0.5f * D;
			const triangleArea: number = 0.5 * D;
			area += triangleArea;

			// Area weighted centroid
			//center += triangleArea * k_inv3 * (p1 + p2 + p3);
			centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
			centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);

			//float32 px = p1.x, py = p1.y;
			const px: number = p1X;
			const py: number = p1Y;
			//float32 ex1 = e1.x, ey1 = e1.y;
			const ex1: number = e1X;
			const ey1: number = e1Y;
			//float32 ex2 = e2.x, ey2 = e2.y;
			const ex2: number = e2X;
			const ey2: number = e2Y;

			//float32 intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
			const intx2: number = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px;
			//float32 inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;
			const inty2: number = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py;

			I += D * (intx2 + inty2);
		}

		// Total mass
		massData.mass = this.m_density * area;

		// Center of mass
		//b2Settings.b2Assert(area > Number.MIN_VALUE);
		//center *= 1.0f / area;
		centerX *= 1.0 / area;
		centerY *= 1.0 / area;
		//massData->center = center;
		massData.center.Set(centerX, centerY);

		// Inertia tensor relative to the local origin.
		massData.I = this.m_density * I;
	}

	/// Get the oriented bounding box relative to the parent body.
	public GetOBB(): b2OBB {
		return this.m_obb;
	}

	/// Get local centroid relative to the parent body.
	public GetCentroid(): b2Vec2 {
		return this.m_centroid;
	}

	/// Get the vertex count.
	public GetVertexCount(): number /** int */{
		return this.m_vertexCount;
	}

	/// Get the vertices in local coordinates.
	public GetVertices(): b2Vec2[] {
		return this.m_vertices;
	}

	/// Get the core vertices in local coordinates. These vertices
	/// represent a smaller polygon that is used for time of impact
	/// computations.
	public GetCoreVertices(): b2Vec2[] {
		return this.m_coreVertices;
	}

	/// Get the edge normal vectors. There is one for each vertex.
	public GetNormals(): b2Vec2[] {
		return this.m_normals;
	}

	/// Get the first vertex and apply the supplied transform.
	public GetFirstVertex(xf: b2XForm): b2Vec2 {
		return b2Math.b2MulX(xf, this.m_coreVertices[0]);
	}

	/// Get the centroid and apply the supplied transform.
	public Centroid(xf: b2XForm): b2Vec2 {
		return b2Math.b2MulX(xf, this.m_centroid);
	}

	/// Get the support point in the given world direction.
	/// Use the supplied transform.
	private s_supportVec: b2Vec2 = new b2Vec2();
	public Support(xf: b2XForm, dX: number, dY: number): b2Vec2 {
		let tVec: b2Vec2;

		let tMat: b2Mat22;
		//b2Vec2 dLocal = b2MulT(xf.R, d);
		tMat = xf.R;
		const dLocalX: number = (dX * tMat.col1.x + dY * tMat.col1.y);
		const dLocalY: number = (dX * tMat.col2.x + dY * tMat.col2.y);

		let bestIndex: number /** int */ = 0;
		//var bestValue:number = b2Dot(this.m_coreVertices[0], dLocal);
		tVec = this.m_coreVertices[0];
		let bestValue: number = (tVec.x * dLocalX + tVec.y * dLocalY);
		for (let i: number /** int */ = 1; i < this.m_vertexCount; ++i) {
			//var value:number = b2Dot(this.m_coreVertices[i], dLocal);
			tVec = this.m_coreVertices[i];
			const value: number = (tVec.x * dLocalX + tVec.y * dLocalY);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}

		//return b2Math.b2MulX(xf, this.m_coreVertices[bestIndex]);
		tMat = xf.R;
		tVec = this.m_coreVertices[bestIndex];
		this.s_supportVec.x = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		this.s_supportVec.y = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		return this.s_supportVec;

	}

	//--------------- Internals Below -------------------

	constructor(def: b2ShapeDef) {
		super(def);

		//b2Settings.b2Assert(def.type == e_polygonShape);
		this.m_type = b2PolygonShape.e_polygonShape;
		const poly: b2PolygonDef = def as b2PolygonDef;

		// Get the vertices transformed into the body frame.
		this.m_vertexCount = poly.vertexCount;
		//b2Settings.b2Assert(3 <= this.m_vertexCount && this.m_vertexCount <= b2_maxPolygonVertices);

		let i: number /** int */ = 0;
		let i1: number /** int */ = i;
		let i2: number /** int */ = i;

		// AWAY fix, beacuse it can be ASArray
		let v_arr: Array<b2Vec2> = poly.vertices;

		if (!v_arr) {
			console.error('[B2D] Try create polygon shape from def', def);
			return this;
		}

		if (typeof v_arr['traits'] !== 'undefined') {
			v_arr  = <any>v_arr['value'] as Array<b2Vec2>;
		}

		// Copy vertices.
		for (i = 0; i < this.m_vertexCount; ++i) {
			this.m_vertices[i] = v_arr[i].Copy();
		}

		// Compute normals. Ensure the edges have non-zero length.
		for (i = 0; i < this.m_vertexCount; ++i) {
			i1 = i;
			i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
			//b2Vec2 edge = this.m_vertices[i2] - this.m_vertices[i1];
			const edgeX: number = this.m_vertices[i2].x - this.m_vertices[i1].x;
			const edgeY: number = this.m_vertices[i2].y - this.m_vertices[i1].y;
			//b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE * Number.MIN_VALUE);
			//this.m_normals[i] = b2Cross(edge, 1.0f); ^^
			const len: number = Math.sqrt(edgeX * edgeX + edgeY * edgeY);
			//this.m_normals[i].Normalize();
			this.m_normals[i] = new b2Vec2(edgeY / len, -edgeX / len);
		}

		/*#ifdef _DEBUG
		// Ensure the polygon is convex.
		for (int32 i = 0; i < this.m_vertexCount; ++i)
		{
			for (int32 j = 0; j < this.m_vertexCount; ++j)
			{
				// Don't check vertices on the current edge.
				if (j == i || j == (i + 1) % this.m_vertexCount)
				{
					continue;
				}

				// Your polygon is non-convex (it has an indentation).
				// Or your polygon is too skinny.
				float32 s = b2Dot(this.m_normals[i], this.m_vertices[j] - this.m_vertices[i]);
				b2Assert(s < -b2_linearSlop);
			}
		}

		// Ensure the polygon is counter-clockwise.
		for (i = 1; i < this.m_vertexCount; ++i)
		{
			var cross:number = b2Math.b2CrossVV(this.m_normals[int(i-1)], this.m_normals[i]);

			// Keep asinf happy.
			cross = b2Math.b2Clamp(cross, -1.0, 1.0);

			// You have consecutive edges that are almost parallel on your polygon.
			var angle:number = Math.asin(cross);
			//b2Assert(angle > b2_angularSlop);
			trace(angle > b2Settings.b2_angularSlop);
		}
		#endif*/

		// Compute the polygon centroid.
		this.m_centroid = b2PolygonShape.ComputeCentroid(v_arr, poly.vertexCount);

		// Compute the oriented bounding box.
		b2PolygonShape.ComputeOBB(this.m_obb, this.m_vertices, this.m_vertexCount);

		// Create core polygon shape by shifting edges inward.
		// Also compute the min/max radius for CCD.
		for (i = 0; i < this.m_vertexCount; ++i) {
			i1 = i - 1 >= 0 ? i - 1 : this.m_vertexCount - 1;
			i2 = i;

			//b2Vec2 n1 = this.m_normals[i1];
			const n1X: number = this.m_normals[i1].x;
			const n1Y: number = this.m_normals[i1].y;
			//b2Vec2 n2 = this.m_normals[i2];
			const n2X: number = this.m_normals[i2].x;
			const n2Y: number = this.m_normals[i2].y;
			//b2Vec2 v = this.m_vertices[i] - this.m_centroid;
			const vX: number = this.m_vertices[i].x - this.m_centroid.x;
			const vY: number = this.m_vertices[i].y - this.m_centroid.y;

			//b2Vec2 d;
			const dX: number = (n1X * vX + n1Y * vY) - b2Settings.b2_toiSlop;
			const dY: number = (n2X * vX + n2Y * vY) - b2Settings.b2_toiSlop;

			// Shifting the edge inward by b2_toiSlop should
			// not cause the plane to pass the centroid.

			// Your shape has a radius/extent less than b2_toiSlop.
			//b2Settings.b2Assert(d.x >= 0.0);
			//b2Settings.b2Assert(d.y >= 0.0);
			//var A:b2Mat22;
			//A.col1.x = n1.x; A.col2.x = n1.y;
			//A.col1.y = n2.x; A.col2.y = n2.y;
			//this.m_coreVertices[i] = A.Solve(d) + this.m_centroid;
			//float32 det = a11 * a22 - a12 * a21;
			const det: number = 1.0 / (n1X * n2Y - n1Y * n2X);
			//det = 1.0 / det;
			this.m_coreVertices[i] = new b2Vec2(det * (n2Y * dX - n1Y * dY) + this.m_centroid.x,
				det * (n1X * dY - n2X * dX) + this.m_centroid.y);
		}
	}

	public UpdateSweepRadius(center: b2Vec2): void {
		let tVec: b2Vec2;

		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		this.m_sweepRadius = 0.0;
		for (let i: number /** int */ = 0; i < this.m_vertexCount; ++i) {
			//b2Vec2 d = this.m_coreVertices[i] - center;
			tVec = this.m_coreVertices[i];
			let dX: number = tVec.x - center.x;
			const dY: number = tVec.y - center.y;
			dX = Math.sqrt(dX * dX + dY * dY);
			//this.m_sweepRadius = b2Max(this.m_sweepRadius, d.Length());
			if (dX > this.m_sweepRadius) this.m_sweepRadius = dX;
		}
	}

	// Local position of the polygon centroid.
	public m_centroid: b2Vec2;

	public m_obb: b2OBB = new b2OBB();

	public m_vertices: b2Vec2[] = new Array(b2Settings.b2_maxPolygonVertices);
	public m_normals: b2Vec2[] = new Array(b2Settings.b2_maxPolygonVertices);
	public m_coreVertices: b2Vec2[] = new Array(b2Settings.b2_maxPolygonVertices);

	public m_vertexCount: number /** int */;

	public static ComputeCentroid(vs: b2Vec2[], count: number /** int */): b2Vec2 {
		//b2Settings.b2Assert(count >= 3);

		//b2Vec2 c; c.Set(0.0f, 0.0f);
		const c: b2Vec2 = new b2Vec2();
		let area: number = 0.0;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		//b2Vec2 pRef(0.0f, 0.0f);
		const p1X: number = 0.0;
		const p1Y: number = 0.0;
		/*#if 0
		// This code would put the reference point inside the polygon.
		for (int32 i = 0; i < count; ++i)
		{
			pRef += vs[i];
		}
		pRef *= 1.0f / count;
	#endif*/

		const inv3: number = 1.0 / 3.0;

		for (let i: number /** int */ = 0; i < count; ++i) {
			// Triangle vertices.
			//b2Vec2 p1 = pRef;
			// 0.0, 0.0
			//b2Vec2 p2 = vs[i];
			const p2: b2Vec2 = vs[i];
			//b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];
			const p3: b2Vec2 = i + 1 < count ? vs[i + 1] : vs[0];

			//b2Vec2 e1 = p2 - p1;
			const e1X: number = p2.x - p1X;
			const e1Y: number = p2.y - p1Y;
			//b2Vec2 e2 = p3 - p1;
			const e2X: number = p3.x - p1X;
			const e2Y: number = p3.y - p1Y;

			//float32 D = b2Cross(e1, e2);
			const D: number = (e1X * e2Y - e1Y * e2X);

			//float32 triangleArea = 0.5f * D;
			const triangleArea: number = 0.5 * D;
			area += triangleArea;

			// Area weighted centroid
			//c += triangleArea * inv3 * (p1 + p2 + p3);
			c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
			c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
		}

		// Centroid
		//beSettings.b2Assert(area > Number.MIN_VALUE);
		//c *= 1.0 / area;
		c.x *= 1.0 / area;
		c.y *= 1.0 / area;
		return c;
	}

	// http://www.geometrictools.com/Documentation/MinimumAreaRectangle.pdf
	public static ComputeOBB(obb: b2OBB, vs: b2Vec2[], count: number /** int */): void {
		let i: number /** int */;
		//b2Settings.b2Assert(count <= b2Settings.b2_maxPolygonVertices);
		const p: b2Vec2[] = new Array(b2Settings.b2_maxPolygonVertices + 1);
		for (i = 0; i < count; ++i) {
			p[i] = vs[i];
		}
		p[count] = p[0];

		let minArea: number = Number.MAX_VALUE;

		for (i = 1; i <= count; ++i) {
			const root: b2Vec2 = p[i - 1];
			//b2Vec2 ux = p[i] - root;
			let uxX: number = p[i].x - root.x;
			let uxY: number = p[i].y - root.y;
			//var length:number = ux.Normalize();
			const length: number = Math.sqrt(uxX * uxX + uxY * uxY);
			uxX /= length;
			uxY /= length;
			//b2Settings.b2Assert(length > Number.MIN_VALUE);
			//b2Vec2 uy(-ux.y, ux.x);
			const uyX: number = -uxY;
			const uyY: number = uxX;
			//b2Vec2 lower(FLT_MAX, FLT_MAX);
			let lowerX: number = Number.MAX_VALUE;
			let lowerY: number = Number.MAX_VALUE;
			//b2Vec2 upper(-FLT_MAX, -FLT_MAX);
			let upperX: number = -Number.MAX_VALUE;
			let upperY: number = -Number.MAX_VALUE;

			for (let j: number /** int */ = 0; j < count; ++j) {
				//b2Vec2 d = p[j] - root;
				const dX: number = p[j].x - root.x;
				const dY: number = p[j].y - root.y;
				//b2Vec2 r;
				//var rX:number = b2Dot(ux, d);
				const rX: number = (uxX * dX + uxY * dY);
				//var rY:number = b2Dot(uy, d);
				const rY: number = (uyX * dX + uyY * dY);
				//lower = b2Min(lower, r);
				if (rX < lowerX) lowerX = rX;
				if (rY < lowerY) lowerY = rY;
				//upper = b2Max(upper, r);
				if (rX > upperX) upperX = rX;
				if (rY > upperY) upperY = rY;
			}

			const area: number = (upperX - lowerX) * (upperY - lowerY);
			if (area < 0.95 * minArea) {
				minArea = area;
				//obb->R.col1 = ux;
				obb.R.col1.x = uxX;
				obb.R.col1.y = uxY;
				//obb->R.col2 = uy;
				obb.R.col2.x = uyX;
				obb.R.col2.y = uyY;
				//b2Vec2 center = 0.5f * (lower + upper);
				const centerX: number = 0.5 * (lowerX + upperX);
				const centerY: number = 0.5 * (lowerY + upperY);
				//obb->center = root + b2Mul(obb->R, center);
				const tMat: b2Mat22 = obb.R;
				obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
				obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
				//obb->extents = 0.5f * (upper - lower);
				obb.extents.x = 0.5 * (upperX - lowerX);
				obb.extents.y = 0.5 * (upperY - lowerY);
			}
		}

		//b2Settings.b2Assert(minArea < Number.MAX_VALUE);
	}

}