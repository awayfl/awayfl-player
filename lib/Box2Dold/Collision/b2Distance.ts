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

import { b2Vec2, b2XForm, b2Math, b2Mat22 } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2Shape } from './Shapes/b2Shape';
import { b2Point } from './b2Point';
import { b2PolygonShape } from './Shapes/b2PolygonShape';
import { b2CircleShape } from './Shapes/b2CircleShape';
import { b2DistanceRegister } from './b2DistanceRegister';
import { b2ManifoldPoint } from './b2ManifoldPoint';
import { b2ConcaveArcShape } from './Shapes/b2ConcaveArcShape';
import { b2StaticEdgeShape } from './Shapes/b2StaticEdgeShape';

export class b2Distance {

	public static AddType(fcn:Function, type1:number /** int */, type2:number /** int */) : void
	{
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		this.s_registers[type1 + type2 * b2Shape.e_shapeTypeCount] = new b2DistanceRegister(fcn, true);
		
		if (type1 != type2)
		{
			this.s_registers[type2 + type1 * b2Shape.e_shapeTypeCount] = new b2DistanceRegister(fcn, false);
		}
	}
	public static InitializeRegisters() : void {
		if (this.s_initialized == true) {
			return;
		}
		this.s_initialized = true;
		
		this.s_registers = new Array(b2Shape.e_shapeTypeCount * b2Shape.e_shapeTypeCount);
		
		//Flash only: Function closures
		this.AddType(this.DistanceCC,  b2Shape.e_circleShape, b2Shape.e_circleShape);
		this.AddType(this.DistancePC, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		this.AddType(this.DistanceGeneric,  b2Shape.e_polygonShape, b2Shape.e_polygonShape);
		
		this.AddType(this.DistanceCcaC, b2Shape.e_concaveArcShape, b2Shape.e_circleShape);
		this.AddType(this.DistancePCca, b2Shape.e_polygonShape, b2Shape.e_concaveArcShape);
		
		this.AddType(this.DistanceSeC, b2Shape.e_staticEdgeShape, b2Shape.e_circleShape);
		this.AddType(this.DistanceGeneric,  b2Shape.e_polygonShape, b2Shape.e_staticEdgeShape);
	}

	// GJK using Voronoi regions (Christer Ericson) and region selection
	// optimizations (Casey Muratori).

	// The origin is either in the region of points[1] or in the edge region. The origin is
	// not in region of points[0] because that is the old point.
	public static ProcessTwo(x1: b2Vec2, x2: b2Vec2, p1s: b2Vec2[], p2s: b2Vec2[], points: b2Vec2[]): number /** int */
	{
		const points_0: b2Vec2 = points[0];
		const points_1: b2Vec2 = points[1];
		const p1s_0: b2Vec2 = p1s[0];
		const p1s_1: b2Vec2 = p1s[1];
		const p2s_0: b2Vec2 = p2s[0];
		const p2s_1: b2Vec2 = p2s[1];

		// If in point[1] region
		//b2Vec2 r = -points[1];
		const rX: number = -points_1.x;
		const rY: number = -points_1.y;
		//b2Vec2 d = points[1] - points[0];
		let dX: number = points_0.x - points_1.x;
		let dY: number = points_0.y - points_1.y;
		//float32 length = d.Normalize();
		const length: number = Math.sqrt(dX * dX + dY * dY);
		dX /= length;
		dY /= length;

		//float32 lambda = b2Dot(r, d);
		let lambda: number = rX * dX + rY * dY;
		if (lambda <= 0.0 || length < Number.MIN_VALUE) {
			// The simplex is reduced to a point.
			//*p1Out = p1s[1];
			x1.SetV(p1s_1);
			//*p2Out = p2s[1];
			x2.SetV(p2s_1);
			//p1s[0] = p1s[1];
			p1s_0.SetV(p1s_1);
			//p2s[0] = p2s[1];
			p2s_0.SetV(p2s_1);
			points_0.SetV(points_1);
			return 1;
		}

		// Else in edge region
		lambda /= length;
		//*x1 = p1s[1] + lambda * (p1s[0] - p1s[1]);
		x1.x = p1s_1.x + lambda * (p1s_0.x - p1s_1.x);
		x1.y = p1s_1.y + lambda * (p1s_0.y - p1s_1.y);
		//*x2 = p2s[1] + lambda * (p2s[0] - p2s[1]);
		x2.x = p2s_1.x + lambda * (p2s_0.x - p2s_1.x);
		x2.y = p2s_1.y + lambda * (p2s_0.y - p2s_1.y);
		return 2;
	}

	// Possible regions:
	// - points[2]
	// - edge points[0]-points[2]
	// - edge points[1]-points[2]
	// - inside the triangle
	public static ProcessThree(x1: b2Vec2, x2: b2Vec2, p1s: b2Vec2[], p2s: b2Vec2[], points: b2Vec2[]): number /** int */
	{
		const points_0: b2Vec2 = points[0];
		const points_1: b2Vec2 = points[1];
		const points_2: b2Vec2 = points[2];
		const p1s_0: b2Vec2 = p1s[0];
		const p1s_1: b2Vec2 = p1s[1];
		const p1s_2: b2Vec2 = p1s[2];
		const p2s_0: b2Vec2 = p2s[0];
		const p2s_1: b2Vec2 = p2s[1];
		const p2s_2: b2Vec2 = p2s[2];

		//b2Vec2 a = points[0];
		const aX: number = points_0.x;
		const aY: number = points_0.y;
		//b2Vec2 b = points[1];
		const bX: number = points_1.x;
		const bY: number = points_1.y;
		//b2Vec2 c = points[2];
		const cX: number = points_2.x;
		const cY: number = points_2.y;

		//b2Vec2 ab = b - a;
		const abX: number = bX - aX;
		const abY: number = bY - aY;
		//b2Vec2 ac = c - a;
		const acX: number = cX - aX;
		const acY: number = cY - aY;
		//b2Vec2 bc = c - b;
		const bcX: number = cX - bX;
		const bcY: number = cY - bY;

		//float32 sn = -b2Dot(a, ab), sd = b2Dot(b, ab);
		const sn: number = -(aX * abX + aY * abY);
		const sd: number = (bX * abX + bY * abY);
		//float32 tn = -b2Dot(a, ac), td = b2Dot(c, ac);
		const tn: number = -(aX * acX + aY * acY);
		const td: number = (cX * acX + cY * acY);
		//float32 un = -b2Dot(b, bc), ud = b2Dot(c, bc);
		const un: number = -(bX * bcX + bY * bcY);
		const ud: number = (cX * bcX + cY * bcY);

		// In vertex c region?
		if (td <= 0.0 && ud <= 0.0) {
			// Single point
			//*x1 = p1s[2];
			x1.SetV(p1s_2);
			//*x2 = p2s[2];
			x2.SetV(p2s_2);
			//p1s[0] = p1s[2];
			p1s_0.SetV(p1s_2);
			//p2s[0] = p2s[2];
			p2s_0.SetV(p2s_2);
			points_0.SetV(points_2);
			return 1;
		}

		// Should not be in vertex a or b region.
		//b2Settings.b2Assert(sn > 0.0 || tn > 0.0);
		//b2Settings.b2Assert(sd > 0.0 || un > 0.0);

		//float32 n = b2Cross(ab, ac);
		const n: number = abX * acY - abY * acX;

		// Should not be in edge ab region.
		//float32 vc = n * b2Cross(a, b);
		const vc: number = n * (aX * bY - aY * bX);
		//b2Settings.b2Assert(vc > 0.0 || sn > 0.0 || sd > 0.0);
		let lambda: number;

		// In edge bc region?
		//float32 va = n * b2Cross(b, c);
		const va: number = n * (bX * cY - bY * cX);
		if (va <= 0.0 && un >= 0.0 && ud >= 0.0 && (un + ud) > 0.0) {
			//b2Settings.b2Assert(un + ud > 0.0);

			//float32 lambda = un / (un + ud);
			lambda = un / (un + ud);
			//*x1 = p1s[1] + lambda * (p1s[2] - p1s[1]);
			x1.x = p1s_1.x + lambda * (p1s_2.x - p1s_1.x);
			x1.y = p1s_1.y + lambda * (p1s_2.y - p1s_1.y);
			//*x2 = p2s[1] + lambda * (p2s[2] - p2s[1]);
			x2.x = p2s_1.x + lambda * (p2s_2.x - p2s_1.x);
			x2.y = p2s_1.y + lambda * (p2s_2.y - p2s_1.y);
			//p1s[0] = p1s[2];
			p1s_0.SetV(p1s_2);
			//p2s[0] = p2s[2];
			p2s_0.SetV(p2s_2);
			//points[0] = points[2];
			points_0.SetV(points_2);
			return 2;
		}

		// In edge ac region?
		//float32 vb = n * b2Cross(c, a);
		const vb: number = n * (cX * aY - cY * aX);
		if (vb <= 0.0 && tn >= 0.0 && td >= 0.0 && (tn + td) > 0.0) {
			//b2Settings.b2Assert(tn + td > 0.0);

			//float32 lambda = tn / (tn + td);
			lambda = tn / (tn + td);
			//*x1 = p1s[0] + lambda * (p1s[2] - p1s[0]);
			x1.x = p1s_0.x + lambda * (p1s_2.x - p1s_0.x);
			x1.y = p1s_0.y + lambda * (p1s_2.y - p1s_0.y);
			//*x2 = p2s[0] + lambda * (p2s[2] - p2s[0]);
			x2.x = p2s_0.x + lambda * (p2s_2.x - p2s_0.x);
			x2.y = p2s_0.y + lambda * (p2s_2.y - p2s_0.y);
			//p1s[1] = p1s[2];
			p1s_1.SetV(p1s_2);
			//p2s[1] = p2s[2];
			p2s_1.SetV(p2s_2);
			//points[1] = points[2];
			points_1.SetV(points_2);
			return 2;
		}

		// Inside the triangle, compute barycentric coordinates
		//float32 denom = va + vb + vc;
		let denom: number = va + vb + vc;
		//b2Settings.b2Assert(denom > 0.0);
		denom = 1.0 / denom;
		//float32 u = va * denom;
		const u: number = va * denom;
		//float32 v = vb * denom;
		const v: number = vb * denom;
		//float32 w = 1.0f - u - v;
		const w: number = 1.0 - u - v;
		//*x1 = u * p1s[0] + v * p1s[1] + w * p1s[2];
		x1.x = u * p1s_0.x + v * p1s_1.x + w * p1s_2.x;
		x1.y = u * p1s_0.y + v * p1s_1.y + w * p1s_2.y;
		//*x2 = u * p2s[0] + v * p2s[1] + w * p2s[2];
		x2.x = u * p2s_0.x + v * p2s_1.x + w * p2s_2.x;
		x2.y = u * p2s_0.y + v * p2s_1.y + w * p2s_2.y;
		return 3;
	}

	public static InPoints(w: b2Vec2, points: b2Vec2[], pointCount: number /** int */): boolean {
		const k_tolerance: number = 100.0 * Number.MIN_VALUE;
		for (let i: number /** int */ = 0; i < pointCount; ++i) {
			const points_i: b2Vec2 = points[i];
			//b2Vec2 d = b2Abs(w - points[i]);
			const dX: number = Math.abs(w.x - points_i.x);
			const dY: number = Math.abs(w.y - points_i.y);
			//b2Vec2 m = b2Max(b2Abs(w), b2Abs(points[i]));
			const mX: number = Math.max(Math.abs(w.x), Math.abs(points_i.x));
			const mY: number = Math.max(Math.abs(w.y), Math.abs(points_i.y));

			if (dX < k_tolerance * (mX + 1.0) &&
				dY < k_tolerance * (mY + 1.0)) {
				return true;
			}
		}

		return false;
	}

	//
	private static s_p1s: b2Vec2[] = [new b2Vec2(), new b2Vec2(), new b2Vec2()];
	private static s_p2s: b2Vec2[] = [new b2Vec2(), new b2Vec2(), new b2Vec2()];
	private static s_points: b2Vec2[] = [new b2Vec2(), new b2Vec2(), new b2Vec2()];
	//

	public static DistanceGeneric(x1: b2Vec2, x2: b2Vec2,
		shape1: any, xf1: b2XForm,
		shape2: any, xf2: b2XForm): number {
		let tVec: b2Vec2;

		//b2Vec2 p1s[3], p2s[3];
		const p1s: b2Vec2[] = b2Distance.s_p1s;
		const p2s: b2Vec2[] = b2Distance.s_p2s;
		//b2Vec2 points[3];
		const points: b2Vec2[] = b2Distance.s_points;
		//int32 pointCount = 0;
		let pointCount: number /** int */ = 0;

		//*x1 = shape1->GetFirstVertex(xf1);
		x1.SetV(shape1.GetFirstVertex(xf1));
		//*x2 = shape2->GetFirstVertex(xf2);
		x2.SetV(shape2.GetFirstVertex(xf2));

		let vSqr: number = 0.0;
		const maxIterations: number /** int */ = 20;
		for (let iter: number /** int */ = 0; iter < maxIterations; ++iter) {
			//b2Vec2 v = *x2 - *x1;
			let vX: number = x2.x - x1.x;
			let vY: number = x2.y - x1.y;
			//b2Vec2 w1 = shape1->Support(xf1, v);
			const w1: b2Vec2 = shape1.Support(xf1, vX, vY);
			//b2Vec2 w2 = shape2->Support(xf2, -v);
			const w2: b2Vec2 = shape2.Support(xf2, -vX, -vY);
			//float32 vSqr = b2Dot(v, v);
			vSqr = (vX * vX + vY * vY);
			//b2Vec2 w = w2 - w1;
			const wX: number = w2.x - w1.x;
			const wY: number = w2.y - w1.y;
			//float32 vw = b2Dot(v, w);
			const vw: number = (vX * wX + vY * wY);
			//if (vSqr - b2Dot(v, w) <= 0.01f * vSqr) // or w in points
			if (vSqr - (vX * wX + vY * wY) <= 0.01 * vSqr) // or w in points
			{
				if (pointCount == 0) {
					//*x1 = w1;
					x1.SetV(w1);
					//*x2 = w2;
					x2.SetV(w2);
				}
				b2Distance.g_GJK_Iterations = iter;
				return Math.sqrt(vSqr);
			}

			switch (pointCount) {
				case 0:
				//p1s[0] = w1;
					tVec = p1s[0];
					tVec.SetV(w1);
					//p2s[0] = w2;
					tVec = p2s[0];
					tVec.SetV(w2);
					//points[0] = w;
					tVec = points[0];
					tVec.x = wX;
					tVec.y = wY;
					//*x1 = p1s[0];
					x1.SetV(p1s[0]);
					//*x2 = p2s[0];
					x2.SetV(p2s[0]);
					++pointCount;
					break;

				case 1:
				//p1s[1] = w1;
					tVec = p1s[1];
					tVec.SetV(w1);
					//p2s[1] = w2;
					tVec = p2s[1];
					tVec.SetV(w2);
					//points[1] = w;
					tVec = points[1];
					tVec.x = wX;
					tVec.y = wY;
					pointCount = b2Distance.ProcessTwo(x1, x2, p1s, p2s, points);
					break;

				case 2:
				//p1s[2] = w1;
					tVec = p1s[2];
					tVec.SetV(w1);
					//p2s[2] = w2;
					tVec = p2s[2];
					tVec.SetV(w2);
					//points[2] = w;
					tVec = points[2];
					tVec.x = wX;
					tVec.y = wY;
					pointCount = b2Distance.ProcessThree(x1, x2, p1s, p2s, points);
					break;
			}

			// If we have three points, then the origin is in the corresponding triangle.
			if (pointCount == 3) {
				b2Distance.g_GJK_Iterations = iter;
				return 0.0;
			}

			//float32 maxSqr = -FLT_MAX;
			let maxSqr: number = -Number.MAX_VALUE;
			for (let i: number /** int */ = 0; i < pointCount; ++i) {
				//maxSqr = b2Math.b2Max(maxSqr, b2Dot(points[i], points[i]));
				tVec = points[i];
				maxSqr = b2Math.b2Max(maxSqr, (tVec.x * tVec.x + tVec.y * tVec.y));
			}

			if (pointCount == 3 || vSqr <= 100.0 * Number.MIN_VALUE * maxSqr) {
				b2Distance.g_GJK_Iterations = iter;
				//v = *x2 - *x1;
				vX = x2.x - x1.x;
				vY = x2.y - x1.y;
				//vSqr = b2Dot(v, v);
				vSqr = (vX * vX + vY * vY);
				return Math.sqrt(vSqr);
			}
		}

		b2Distance.g_GJK_Iterations = maxIterations;
		return Math.sqrt(vSqr);
	}

	public static DistanceCC(
		x1: b2Vec2, x2: b2Vec2,
		circle1: b2CircleShape, xf1: b2XForm,
		circle2: b2CircleShape, xf2: b2XForm): number {
		let tMat: b2Mat22;
		let tVec: b2Vec2;
		//b2Vec2 p1 = b2Mul(xf1, circle1->m_localPosition);
		tMat = xf1.R;
		tVec = circle1.m_localPosition;
		const p1X: number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const p1Y: number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 p2 = b2Mul(xf2, circle2->m_localPosition);
		tMat = xf2.R;
		tVec = circle2.m_localPosition;
		const p2X: number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const p2Y: number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		//b2Vec2 d = p2 - p1;
		let dX: number = p2X - p1X;
		let dY: number = p2Y - p1Y;
		const dSqr: number = (dX * dX + dY * dY);
		const r1: number = circle1.m_radius - b2Settings.b2_toiSlop;
		const r2: number = circle2.m_radius - b2Settings.b2_toiSlop;
		const r: number = r1 + r2;
		if (dSqr > r * r) {
			//var dLen:number = d.Normalize();
			var dLen: number = Math.sqrt(dX * dX + dY * dY);
			dX /= dLen;
			dY /= dLen;
			const distance: number = dLen - r;
			//*x1 = p1 + r1 * d;
			x1.x = p1X + r1 * dX;
			x1.y = p1Y + r1 * dY;
			//*x2 = p2 - r2 * d;
			x2.x = p2X - r2 * dX;
			x2.y = p2Y - r2 * dY;
			return distance;
		} else if (dSqr > Number.MIN_VALUE * Number.MIN_VALUE) {
			//d.Normalize();
			dLen = Math.sqrt(dX * dX + dY * dY);
			dX /= dLen;
			dY /= dLen;
			//*x1 = p1 + r1 * d;
			x1.x = p1X + r1 * dX;
			x1.y = p1Y + r1 * dY;
			//*x2 = *x1;
			x2.x = x1.x;
			x2.y = x1.y;
			return 0.0;
		}

		//*x1 = p1;
		x1.x = p1X;
		x1.y = p1Y;
		//*x2 = *x1;
		x2.x = x1.x;
		x2.y = x1.y;
		return 0.0;
	}

	public static DistanceSeC(
		x1: b2Vec2, x2:b2Vec2,
		edge:b2StaticEdgeShape, xf1:b2XForm,
		circle:b2CircleShape, xf2:b2XForm) : number
	{
		var dX:number;
		var dY:number;
		var dSqr:number;
		var dLen: number;
		var r:number = circle.m_radius - b2Settings.b2_toiSlop;
		var tPoint:b2ManifoldPoint;
		
		var tMat:b2Mat22 = xf2.R; 
		var tVec:b2Vec2 = circle.m_localPosition;
		var circleX:number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var circleY:number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tVec = edge.m_direction;
		var dirDist: number = (circleX - edge.m_coreV1.x) * tVec.x + (circleY - edge.m_coreV1.y) * tVec.y;
		if (dirDist <= 0) {
			x1.SetV(edge.m_coreV1);
			dX = circleX - edge.m_coreV1.x;
			dY = circleY - edge.m_coreV1.y;
			dSqr = dX*dX + dY*dY;
			if (dSqr > r*r) {
				dLen = Math.sqrt(dSqr);
				dX /= dLen;
				dY /= dLen;
				x2.x = circleX - dX * r;
				x2.y = circleY - dY * r;
				return dLen - r;
			} else {
				x2.SetV(edge.m_coreV1);
				return 0.0;
			}
		} else if (dirDist >= edge.m_length) {
			x1.SetV(edge.m_coreV2);
			dX = circleX - edge.m_coreV2.x;
			dY = circleY - edge.m_coreV2.y;
			dSqr = dX*dX + dY*dY;
			if (dSqr > r*r) {
				dLen = Math.sqrt(dSqr);
				dX /= dLen;
				dY /= dLen;
				x2.x = circleX - dX * r;
				x2.y = circleY - dY * r;
				return dLen - r;
			} else {
				x2.SetV(edge.m_coreV2);
				return 0.0;
			}
		} else {
			x1.x = edge.m_coreV1.x + tVec.x * dirDist;
			x1.y = edge.m_coreV1.y + tVec.y * dirDist;
			tVec = edge.m_normal;
			dLen = (circleX - edge.m_coreV1.x) * tVec.x + 
				   (circleY - edge.m_coreV1.y) * tVec.y;
			if (dLen < 0.0) {
				if (dLen < -r) {
					x2.x = circleX + r * tVec.x;
					x2.y = circleY + r * tVec.y;
					return -dLen - r;
				} else {
					x2.SetV(x1);
					return 0.0;
				}
			} else {
				if (dLen > r) {
					x2.x = circleX - r * tVec.x;
					x2.y = circleY - r * tVec.y;
					return dLen - r;
				} else {
					x2.SetV(x1);
					return 0.0;
				}
			}
		}
	}

	// GJK is more robust with polygon-vs-point than polygon-vs-circle.
	// So we convert polygon-vs-circle to polygon-vs-point.
	private static gPoint: b2Point = new b2Point();
	///
	public static DistancePC(
		x1: b2Vec2, x2: b2Vec2,
		polygon: b2PolygonShape, xf1: b2XForm,
		circle: b2CircleShape, xf2: b2XForm): number {

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		const point: b2Point = b2Distance.gPoint;
		//point.p = b2Mul(xf2, circle->m_localPosition);
		tVec = circle.m_localPosition;
		tMat = xf2.R;
		point.p.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		point.p.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		// Create variation of function to replace template
		let distance: number = b2Distance.DistanceGeneric(x1, x2, polygon, xf1, point, b2Math.b2XForm_identity);

		const r: number = circle.m_radius - b2Settings.b2_toiSlop;

		if (distance > r) {
			distance -= r;
			//b2Vec2 d = *x2 - *x1;
			let dX: number = x2.x - x1.x;
			let dY: number = x2.y - x1.y;
			//d.Normalize();
			const dLen: number = Math.sqrt(dX * dX + dY * dY);
			dX /= dLen;
			dY /= dLen;
			//*x2 -= r * d;
			x2.x -= r * dX;
			x2.y -= r * dY;
		} else {
			distance = 0.0;
			//*x2 = *x1;
			x2.x = x1.x;
			x2.y = x1.y;
		}

		return distance;
	}

	public static DistanceCcaC(
		x1:b2Vec2, x2:b2Vec2,
		polygon:b2ConcaveArcShape, xf1:b2XForm,
		circle:b2CircleShape, xf2:b2XForm) : number
	{
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		var point:b2Point = b2Distance.gPoint;
		//point.p = b2Mul(xf2, circle->m_localPosition);
		tVec = circle.m_localPosition;
		tMat = xf2.R;
		point.p.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		point.p.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	
		// Create variation of function to replace template
		var distance:number = b2Distance.DistanceGeneric(x1, x2, polygon, xf1, point, b2Math.b2XForm_identity);
		
		var r:number = circle.m_radius - b2Settings.b2_toiSlop;
		
		//Check if x1 is along the first edge of the arc, in which case DistanceGeneric will be wrong
		/*
		//Alternate calculation of n
		var x1l:b2Vec2 = b2Math.b2MulXT(xf1,x1);
		x1l.x -= polygon.m_coreVertices[0].x;
		x1l.y -= polygon.m_coreVertices[0].y;
		tVec = polygon.m_normals[0];
		var n:number = x1l.x * tVec.x + x1l.y * tVec.y;*/
		
		var vx0:b2Vec2 = b2Math.b2MulX(xf1,polygon.m_coreVertices[0]);
		var normal:b2Vec2 = b2Math.b2MulMV(xf1.R,polygon.m_normals[0]);
		var n:number = (x1.x-vx0.x) * normal.x + (x1.y-vx0.y) * normal.y;
		
		if(n >= 0){
			//The center is closest to the arc of all the edges, so do appropriate distance calcs
			
			var arcCenter:b2Vec2 = b2Math.b2MulX(xf1,polygon.m_arcCenter);
			
			//See similar code in b2ConcaveArcAndCircleContact
			var c2X: number = point.p.x - arcCenter.x;
			var c2Y: number = point.p.y - arcCenter.y;
			var norm:number = -normal.y*c2X+normal.x*c2Y;
			if (c2X*normal.x+c2Y*normal.y>0)
			{
				if(norm < 0){
					//Vertex 0
					tVec = vx0;
				}else{
					//Vertex 1
					tVec = b2Math.b2MulX(xf1,polygon.m_coreVertices[1]);
				}
			}else{
				if (norm <= -polygon.m_norm){
					//Vertex 0
					tVec = vx0;
				}else if (norm >= polygon.m_norm){
					//Vertex 1
					tVec = b2Math.b2MulX(xf1,polygon.m_coreVertices[1]);
				}else{
					//Nearest point on arc
					var c: number = Math.sqrt(c2X*c2X+c2Y*c2Y);
					//trace([c2X,c2Y]);
					//trace([polygon.m_radius+b2Settings.b2_toiSlop,r,c]);
					distance = (polygon.m_radius+b2Settings.b2_toiSlop*2)-r-c;
					c2X /= c;
					c2Y /= c;
					if(distance<0) distance = 0;
					x1.x = arcCenter.x + c2X * (polygon.m_radius+b2Settings.b2_toiSlop);
					x1.y = arcCenter.y + c2Y * (polygon.m_radius+b2Settings.b2_toiSlop);
					x2.x += c2X * r;
					x2.y += c2Y * r;
					//trace(distance);
					return distance;
				}
			}
			x1.SetV(tVec);
			tVec.x -= point.p.x;
			tVec.y -= point.p.y;
			distance = tVec.Normalize() - r;
			if(distance > 0){
				x2.x += r * tVec.x;
				x2.y += r * tVec.y;
				return distance;
			}else{
				x2.SetV(x1);
				return 0;
			}
		}
	
		if (distance > r)
		{
			distance -= r;
			//b2Vec2 d = *x2 - *x1;
			var dX:number = x2.x - x1.x;
			var dY:number = x2.y - x1.y;
			//d.Normalize();
			var dLen:number = Math.sqrt(dX*dX + dY*dY);
			dX /= dLen;
			dY /= dLen;
			//*x2 -= r * d
			x2.x -= r * dX;
			x2.y -= r * dY;
		}
		else
		{
			distance = 0.0;
			//*x2 = *x1;
			x2.x = x1.x;
			x2.y = x1.y;
		}
		return distance;
	}
	
	
	public static t:number=-1;
	
	public static DistancePCca(
		x1:b2Vec2, x2:b2Vec2,
		polygon:b2PolygonShape, xf1:b2XForm,
		arc:b2ConcaveArcShape, xf2:b2XForm) : number
	{
		let t=-1;
		var gd:number = b2Distance.DistanceGeneric(x1,x2,polygon,xf1,arc,xf2);
		
		var vx0:b2Vec2 = b2Math.b2MulX(xf2,arc.m_coreVertices[0]);
		var normal:b2Vec2 = b2Math.b2MulMV(xf2.R,arc.m_normals[0]);
		var n:number = (x2.x-vx0.x) * normal.x + (x2.y-vx0.y) * normal.y;
		
		//n should be compared to zero, but apprently the rounding errors are pretty bad
		if(n < -b2Settings.b2_linearSlop/4 && gd>0){
			//The closest point on the bounding poly of the arc shape is not on the first edge
			//So it is correct
			t=0;
			return gd;
		}
		
		var vx1:b2Vec2 = b2Math.b2MulX(xf2,arc.m_coreVertices[1]);
		
		//Check if x2==vx0 or x2==vx1. This would be a lot easier if there was some results caching going on
		var tVec:b2Vec2 = new b2Vec2();
		var tolerance: number = b2Settings.b2_linearSlop * b2Settings.b2_linearSlop;
		tVec.x = vx0.x - x2.x;
		tVec.y = vx0.y - x2.y;
		if(tVec.x*tVec.x+tVec.y*tVec.y< tolerance){
			t=1;
			return gd;
		}
		tVec.x = vx1.x - x2.x;
		tVec.y = vx1.y - x2.y;
		if(tVec.x*tVec.x+tVec.y*tVec.y< tolerance){
			t=2;
			return gd;
		}
		
		//Otherwise, calculate the nearest point on the arc to the poly
		//AFAIK, there is little you can do more than brute force
		//Perhaps start with a sensible vertex, so that others are more likely to get discarded earlier
		
		var localCenter:b2Vec2 = b2Math.b2MulXT(xf1, b2Math.b2MulX(xf2, arc.m_arcCenter));
		var localNorm:b2Vec2 = b2Math.b2MulTMV(xf1.R, normal)
		var maxDist2:number = -1;//Square of the distance of the furthest (valid) vertex (so far) from localCenter; or equivalent
		var bestVx:number = -1;
		var separation: number = Number.MAX_VALUE;
		var dist2:number;
		var dist:number;
		
		
		for(var i:number = 0;i<polygon.m_vertexCount;i++){
			tVec.x = polygon.m_coreVertices[i].x - localCenter.x;
			tVec.y = polygon.m_coreVertices[i].y - localCenter.y;
			var norm: number = tVec.x*localNorm.y-tVec.y*localNorm.x;
			dist2 = tVec.x*tVec.x+tVec.y*tVec.y;
			if(		norm*norm<arc.m_norm*arc.m_norm*dist2
				&&	tVec.x*localNorm.x+tVec.y*localNorm.y<0 ){
				//Near the curve of the arc
				if(dist2>maxDist2){
					maxDist2 = dist2;
					bestVx = i;
					dist = Math.sqrt(dist2);
					separation = arc.m_radius+b2Settings.b2_toiSlop-dist;
					if(separation<0) separation=0;
					x1.SetV(b2Math.b2MulX(xf1,polygon.m_coreVertices[i]))
					tVec.x *= (arc.m_radius+b2Settings.b2_toiSlop)/dist;
					tVec.y *= (arc.m_radius+b2Settings.b2_toiSlop)/dist;
					tVec.x += localCenter.x;
					tVec.y += localCenter.y;
					x2.SetV(b2Math.b2MulX(xf1,tVec));
					t=3;
				}
			}
		}
		
		var sx1:b2Vec2 = new b2Vec2();
		var sx2:b2Vec2 = new b2Vec2();
		var point:b2Point = b2Distance.gPoint;
		point.p.SetV(vx0);
		var anotherDistance:number = b2Distance.DistanceGeneric(sx1, sx2, polygon, xf1, point, b2Math.b2XForm_identity);
		if(anotherDistance<separation){
			t=4;
			separation=anotherDistance;
			x1.SetV(sx1);
			x2.SetV(sx2);
		}
		point.p.SetV(vx1);
		anotherDistance = b2Distance.DistanceGeneric(sx1, sx2, polygon, xf1, point, b2Math.b2XForm_identity);
		if(anotherDistance<separation){
			t=5;
			separation=anotherDistance;
			x1.SetV(sx1);
			x2.SetV(sx2);
		}
		
		//As we have shrank the arc from it's bounding poly,
		//It should never be closer to the polygon than it's bounding poly
		//b2Settings.b2Assert(separation>gd);
		return separation;
	}

	public static Distance(x1: b2Vec2, x2: b2Vec2,
		shape1: b2Shape, xf1: b2XForm,
		shape2: b2Shape, xf2: b2XForm): number {
		//b2ShapeType type1 = shape1->GetType();
		const type1: number /** int */ = shape1.m_type;
		//b2ShapeType type2 = shape2->GetType();
		const type2: number /** int */ = shape2.m_type;

		var register: b2DistanceRegister = b2Distance.s_registers[type1 + type2 * b2Shape.e_shapeTypeCount];
		if (register != null) {
			if (register.primary) {
				return register.fcn(x1, x2, shape1, xf1, shape2, xf2);
			} else {
				return register.fcn(x2, x1, shape2, xf2, shape1, xf1);
			}
		}

		return 0.0;
	}

	public static g_GJK_Iterations: number /** int */ = 0;
	public static s_registers: any[];
	public static s_initialized: boolean = false;	

}