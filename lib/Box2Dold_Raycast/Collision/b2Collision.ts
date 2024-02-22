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

import { b2Vec2 } from '../Common/Math';
import { b2Math } from '../Common/Math';
import { ClipVertex } from './ClipVertex';
import { b2PolygonShape } from './Shapes/b2PolygonShape';
import { b2XForm } from '../Common/Math';
import { b2Mat22 } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2ManifoldPoint } from './b2ManifoldPoint';
import { b2Manifold } from './b2Manifold';
import { b2AABB } from './b2AABB';
import { b2CircleShape } from './Shapes/b2CircleShape';

export class b2Collision {

	// Null feature
	public static readonly b2_nullFeature: number /** uint */ = 0x000000ff;//UCHAR_MAX;

	public static ClipSegmentToLine(vOut: ClipVertex[], vIn: ClipVertex[], normal: b2Vec2, offset: number): number /** int */
	{
		let cv: ClipVertex;

		// Start with no output points
		let numOut: number /** int */ = 0;

		cv = vIn[0];
		const vIn0: b2Vec2 = cv.v;
		cv = vIn[1];
		const vIn1: b2Vec2 = cv.v;

		// Calculate the distance of end points to the line
		const distance0: number = b2Math.b2Dot(normal, vIn0) - offset;
		const distance1: number = b2Math.b2Dot(normal, vIn1) - offset;

		// If the points are behind the plane
		if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
		if (distance1 <= 0.0) vOut[numOut++] = vIn[1];

		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0) {
			// Find intersection point of edge and plane
			const interp: number = distance0 / (distance0 - distance1);
			// expanded for performance
			// vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			cv = vOut[numOut];
			const tVec: b2Vec2 = cv.v;
			tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
			tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
			cv = vOut[numOut];
			let cv2: ClipVertex;
			if (distance0 > 0.0) {
				cv2 = vIn[0];
				cv.id = cv2.id;
			} else {
				cv2 = vIn[1];
				cv.id = cv2.id;
			}
			++numOut;
		}

		return numOut;
	}

	// Find the separation between poly1 and poly2 for a give edge normal on poly1.
	public static EdgeSeparation(poly1: b2PolygonShape, xf1: b2XForm, edge1: number /** int */,
		poly2: b2PolygonShape, xf2: b2XForm): number {
		const count1: number /** int */ = poly1.m_vertexCount;
		const vertices1: b2Vec2[] = poly1.m_vertices;
		const normals1: b2Vec2[] = poly1.m_normals;

		const count2: number /** int */ = poly2.m_vertexCount;
		const vertices2: b2Vec2[] = poly2.m_vertices;

		//b2Assert(0 <= edge1 && edge1 < count1);

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		// Convert normal from poly1's frame into poly2's frame.
		//b2Vec2 normal1World = b2Mul(xf1.R, normals1[edge1]);
		tMat = xf1.R;
		tVec = normals1[edge1];
		const normal1WorldX: number = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const normal1WorldY: number = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 normal1 = b2MulT(xf2.R, normal1World);
		tMat = xf2.R;
		const normal1X: number = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
		const normal1Y: number = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);

		// Find support vertex on poly2 for -normal.
		let index: number /** int */ = 0;
		let minDot: number = Number.MAX_VALUE;
		for (let i: number /** int */ = 0; i < count2; ++i) {
			//float32 dot = b2Dot(poly2->m_vertices[i], normal1);
			tVec = vertices2[i];
			const dot: number = tVec.x * normal1X + tVec.y * normal1Y;
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}

		//b2Vec2 v1 = b2Mul(xf1, vertices1[edge1]);
		tVec = vertices1[edge1];
		tMat = xf1.R;
		const v1X: number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const v1Y: number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 v2 = b2Mul(xf2, vertices2[index]);
		tVec = vertices2[index];
		tMat = xf2.R;
		let v2X: number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		let v2Y: number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		//var separation:number = b2Math.b2Dot( b2Math.SubtractVV( v2, v1 ) , normal);
		v2X -= v1X;
		v2Y -= v1Y;
		//float32 separation = b2Dot(v2 - v1, normal1World);
		const separation: number = v2X * normal1WorldX + v2Y * normal1WorldY;
		return separation;
	}

	// Find the max separation between poly1 and poly2 using edge normals
	// from poly1.
	public static FindMaxSeparation(edgeIndex: number[] /*int ptr*/,
		poly1: b2PolygonShape, xf1: b2XForm,
		poly2: b2PolygonShape, xf2: b2XForm): number {
		const count1: number /** int */ = poly1.m_vertexCount;
		const normals1: b2Vec2[] = poly1.m_normals;

		let tVec: b2Vec2;
		let tMat: b2Mat22;

		// Vector pointing from the centroid of poly1 to the centroid of poly2.
		//b2Vec2 d = b2Mul(xf2, poly2->m_centroid) - b2Mul(xf1, poly1->m_centroid);
		tMat = xf2.R;
		tVec = poly2.m_centroid;
		let dX: number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		let dY: number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf1.R;
		tVec = poly1.m_centroid;
		dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		//b2Vec2 dLocal1 = b2MulT(xf1.R, d);
		const dLocal1X: number = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
		const dLocal1Y: number = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);

		// Get support vertex as a hint for our search
		let edge: number /** int */ = 0;
		let maxDot: number = -Number.MAX_VALUE;
		for (let i: number /** int */ = 0; i < count1; ++i) {
			//var dot:number = b2Math.b2Dot(normals1[i], dLocal1);
			tVec = normals1[i];
			const dot: number = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
			if (dot > maxDot) {
				maxDot = dot;
				edge = i;
			}
		}

		// Get the separation for the edge normal.
		let s: number = this.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0) {
			return s;
		}

		// Check the separation for the previous edge normal.
		const prevEdge: number /** int */ = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		const sPrev: number = this.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
		if (sPrev > 0.0) {
			return sPrev;
		}

		// Check the separation for the next edge normal.
		const nextEdge: number /** int */ = edge + 1 < count1 ? edge + 1 : 0;
		const sNext: number = this.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
		if (sNext > 0.0) {
			return sNext;
		}

		// Find the best edge and the search direction.
		let bestEdge: number /** int */;
		let bestSeparation: number;
		let increment: number /** int */;
		if (sPrev > s && sPrev > sNext) {
			increment = -1;
			bestEdge = prevEdge;
			bestSeparation = sPrev;
		} else if (sNext > s) {
			increment = 1;
			bestEdge = nextEdge;
			bestSeparation = sNext;
		} else {
			// pointer out
			edgeIndex[0] = edge;
			return s;
		}

		// Perform a local search for the best edge normal.
		while (true) {

			if (increment == -1)
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			else
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

			s = this.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
			if (s > 0.0) {
				return s;
			}

			if (s > bestSeparation) {
				bestEdge = edge;
				bestSeparation = s;
			} else {
				break;
			}
		}

		// pointer out
		edgeIndex[0] = bestEdge;
		return bestSeparation;
	}

	public static FindIncidentEdge(c: ClipVertex[],
		poly1: b2PolygonShape, xf1: b2XForm, edge1: number /** int */,
		poly2: b2PolygonShape, xf2: b2XForm): void {
		const count1: number /** int */ = poly1.m_vertexCount;
		const normals1: b2Vec2[] = poly1.m_normals;

		const count2: number /** int */ = poly2.m_vertexCount;
		const vertices2: b2Vec2[] = poly2.m_vertices;
		const normals2: b2Vec2[] = poly2.m_normals;

		//b2Assert(0 <= edge1 && edge1 < count1);

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		// Get the normal of the reference edge in poly2's frame.
		//b2Vec2 normal1 = b2MulT(xf2.R, b2Mul(xf1.R, normals1[edge1]));
		tMat = xf1.R;
		tVec = normals1[edge1];
		let normal1X: number = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		let normal1Y: number = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf2.R;
		const tX: number = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
		normal1Y = 		(tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
		normal1X = tX;

		// Find the incident edge on poly2.
		let index: number /** int */ = 0;
		let minDot: number = Number.MAX_VALUE;
		for (let i: number /** int */ = 0; i < count2; ++i) {
			//var dot:number = b2Dot(normal1, normals2[i]);
			tVec = normals2[i];
			const dot: number = (normal1X * tVec.x + normal1Y * tVec.y);
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}

		let tClip: ClipVertex;
		// Build the clip vertices for the incident edge.
		const i1: number /** int */ = index;
		const i2: number /** int */ = i1 + 1 < count2 ? i1 + 1 : 0;

		tClip = c[0];
		//c[0].v = b2Mul(xf2, vertices2[i1]);
		tVec = vertices2[i1];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i1;
		tClip.id.features.incidentVertex = 0;

		tClip = c[1];
		//c[1].v = b2Mul(xf2, vertices2[i2]);
		tVec = vertices2[i2];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i2;
		tClip.id.features.incidentVertex = 1;
	}

	// Find edge normal of max separation on A - return if separating axis is found
	// Find edge normal of max separation on B - return if separation axis is found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip
	private static b2CollidePolyTempVec: b2Vec2 = new b2Vec2();
	// The normal points from 1 to 2
	public static b2CollidePolygons(manifold: b2Manifold,
		polyA: b2PolygonShape, xfA: b2XForm,
		polyB: b2PolygonShape, xfB: b2XForm): void {
		let cv: ClipVertex;

		manifold.pointCount = 0;

		let edgeA: number /** int */ = 0;
		const edgeAO: number[] = [edgeA];
		const separationA: number = this.FindMaxSeparation(edgeAO, polyA, xfA, polyB, xfB);
		edgeA = edgeAO[0];
		if (separationA > 0.0)
			return;

		let edgeB: number /** int */ = 0;
		const edgeBO: number[] = [edgeB];
		const separationB: number = this.FindMaxSeparation(edgeBO, polyB, xfB, polyA, xfA);
		edgeB = edgeBO[0];
		if (separationB > 0.0)
			return;

		let poly1: b2PolygonShape;	// reference poly
		let poly2: b2PolygonShape;	// incident poly
		const xf1: b2XForm = new b2XForm();
		const xf2: b2XForm = new b2XForm();
		let edge1: number /** int */;		// reference edge
		let flip: number /** uint */;
		const k_relativeTol: number = 0.98;
		const k_absoluteTol: number = 0.001;

		// TODO_ERIN use "radius" of poly for absolute tolerance.
		if (separationB > k_relativeTol * separationA + k_absoluteTol) {
			poly1 = polyB;
			poly2 = polyA;
			xf1.Set(xfB);
			xf2.Set(xfA);
			edge1 = edgeB;
			flip = 1;
		} else {
			poly1 = polyA;
			poly2 = polyB;
			xf1.Set(xfA);
			xf2.Set(xfB);
			edge1 = edgeA;
			flip = 0;
		}

		const incidentEdge: ClipVertex[] = [new ClipVertex(), new ClipVertex()];
		this.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		const count1: number /** int */ = poly1.m_vertexCount;
		const vertices1: b2Vec2[] = poly1.m_vertices;

		let tVec: b2Vec2 = vertices1[edge1];
		let v11: b2Vec2 = tVec.Copy();
		if (edge1 + 1 < count1) {
			tVec = vertices1[edge1 + 1];
			var v12: b2Vec2 = tVec.Copy();
		} else {
			tVec = vertices1[0];
			v12 = tVec.Copy();
		}

		const dv: b2Vec2 = b2Math.SubtractVV(v12 , v11);
		const sideNormal: b2Vec2 = b2Math.b2MulMV(xf1.R, b2Math.SubtractVV(v12 , v11));
		sideNormal.Normalize();
		const frontNormal: b2Vec2 = b2Math.b2CrossVF(sideNormal, 1.0);

		v11 = b2Math.b2MulX(xf1, v11);
		v12 = b2Math.b2MulX(xf1, v12);

		const frontOffset: number = b2Math.b2Dot(frontNormal, v11);
		const sideOffset1: number = -b2Math.b2Dot(sideNormal, v11);
		const sideOffset2: number = b2Math.b2Dot(sideNormal, v12);

		// Clip incident edge against extruded edge1 side edges.
		const clipPoints1: ClipVertex[] = [new ClipVertex(), new ClipVertex()];
		const clipPoints2: ClipVertex[] = [new ClipVertex(), new ClipVertex()];
		let np: number /** int */;

		// Clip to box side 1
		//np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);
		np = this.ClipSegmentToLine(clipPoints1, incidentEdge, sideNormal.Negative(), sideOffset1);

		if (np < 2)
			return;

		// Clip to negative box side 1
		np = this.ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, sideOffset2);

		if (np < 2)
			return;

		// Now clipPoints2 contains the clipped points.
		manifold.normal = flip ? frontNormal.Negative() : frontNormal.Copy();

		let pointCount: number /** int */ = 0;
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints; ++i) {
			cv = clipPoints2[i];
			const separation: number = b2Math.b2Dot(frontNormal, cv.v) - frontOffset;

			if (separation <= 0.0) {
				const cp: b2ManifoldPoint = manifold.points[ pointCount ];
				cp.separation = separation;
				cp.localPoint1 = b2Math.b2MulXT(xfA, cv.v);
				cp.localPoint2 = b2Math.b2MulXT(xfB, cv.v);
				cp.id.key = cv.id._key;
				cp.id.features.flip = flip;
				++pointCount;
			}
		}

		manifold.pointCount = pointCount;
	}

	public static b2CollideCircles(
		manifold: b2Manifold,
		circle1: b2CircleShape, xf1: b2XForm,
		circle2: b2CircleShape, xf2: b2XForm): void {
		manifold.pointCount = 0;

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		//b2Vec2 p1 = b2Mul(xf1, circle1->m_localPosition);
		tMat = xf1.R; tVec = circle1.m_localPosition;
		let p1X: number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		let p1Y: number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 p2 = b2Mul(xf2, circle2->m_localPosition);
		tMat = xf2.R; tVec = circle2.m_localPosition;
		let p2X: number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		let p2Y: number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 d = p2 - p1;
		const dX: number = p2X - p1X;
		const dY: number = p2Y - p1Y;
		//var distSqr:number = b2Math.b2Dot(d, d);
		const distSqr: number = dX * dX + dY * dY;
		const r1: number = circle1.m_radius;
		const r2: number = circle2.m_radius;
		const radiusSum: number = r1 + r2;
		if (distSqr > radiusSum * radiusSum) {
			return;
		}

		let separation: number;
		if (distSqr < Number.MIN_VALUE) {
			separation = -radiusSum;
			manifold.normal.Set(0.0, 1.0);
		} else {
			const dist: number = Math.sqrt(distSqr);
			separation = dist - radiusSum;
			const a: number = 1.0 / dist;
			manifold.normal.x = a * dX;
			manifold.normal.y = a * dY;
		}

		manifold.pointCount = 1;
		const tPoint: b2ManifoldPoint = manifold.points[0];
		tPoint.id.key = 0;
		tPoint.separation = separation;

		p1X += r1 * manifold.normal.x;
		p1Y += r1 * manifold.normal.y;
		p2X -= r2 * manifold.normal.x;
		p2Y -= r2 * manifold.normal.y;

		//b2Vec2 p = 0.5f * (p1 + p2);
		const pX: number = 0.5 * (p1X + p2X);
		const pY: number = 0.5 * (p1Y + p2Y);

		//tPoint.localPoint1 = b2MulT(xf1, p);
		let tX: number = pX - xf1.position.x;
		let tY: number = pY - xf1.position.y;
		tPoint.localPoint1.x = (tX * xf1.R.col1.x + tY * xf1.R.col1.y);
		tPoint.localPoint1.y = (tX * xf1.R.col2.x + tY * xf1.R.col2.y);
		//tPoint.localPoint2 = b2MulT(xf2, p);
		tX = pX - xf2.position.x;
		tY = pY - xf2.position.y;
		tPoint.localPoint2.x = (tX * xf2.R.col1.x + tY * xf2.R.col1.y);
		tPoint.localPoint2.y = (tX * xf2.R.col2.x + tY * xf2.R.col2.y);
	}

	public static b2CollidePolygonAndCircle(
		manifold: b2Manifold,
		polygon: b2PolygonShape, xf1: b2XForm,
		circle: b2CircleShape, xf2: b2XForm): void {
		manifold.pointCount = 0;
		let tPoint: b2ManifoldPoint;

		let dX: number;
		let dY: number;
		let positionX: number;
		let positionY: number;

		let tVec: b2Vec2;
		let tMat: b2Mat22;

		// Compute circle position in the frame of the polygon.
		//b2Vec2 c = b2Mul(xf2, circle->m_localPosition);
		tMat = xf2.R;
		tVec = circle.m_localPosition;
		const cX: number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const cY: number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		//b2Vec2 cLocal = b2MulT(xf1, c);
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;
		tMat = xf1.R;
		const cLocalX: number = (dX * tMat.col1.x + dY * tMat.col1.y);
		const cLocalY: number = (dX * tMat.col2.x + dY * tMat.col2.y);

		let dist: number;

		// Find the min separating edge.
		let normalIndex: number /** int */ = 0;
		let separation: number = -Number.MAX_VALUE;
		const radius: number = circle.m_radius;
		const vertexCount: number /** int */ = polygon.m_vertexCount;
		const vertices: b2Vec2[] = polygon.m_vertices;
		const normals: b2Vec2[] = polygon.m_normals;

		for (let i: number /** int */ = 0; i < vertexCount; ++i) {
			//float32 s = b2Dot(normals[i], cLocal - vertices[i]);
			tVec = vertices[i];
			dX = cLocalX - tVec.x;
			dY = cLocalY - tVec.y;
			tVec = normals[i];
			const s: number = tVec.x * dX + tVec.y * dY;

			if (s > radius) {
				// Early out.
				return;
			}

			if (s > separation) {
				separation = s;
				normalIndex = i;
			}
		}

		// If the center is inside the polygon ...
		if (separation < Number.MIN_VALUE) {
			manifold.pointCount = 1;
			//manifold->normal = b2Mul(xf1.R, normals[normalIndex]);
			tVec = normals[normalIndex];
			tMat = xf1.R;
			manifold.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			manifold.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			//b2Vec2 position = c - radius * manifold->normal;
			positionX = cX - radius * manifold.normal.x;
			positionY = cY - radius * manifold.normal.y;
			//manifold->points[0].localPoint1 = b2MulT(xf1, position);
			dX = positionX - xf1.position.x;
			dY = positionY - xf1.position.y;
			tMat = xf1.R;
			tPoint.localPoint1.x = (dX * tMat.col1.x + dY * tMat.col1.y);
			tPoint.localPoint1.y = (dX * tMat.col2.x + dY * tMat.col2.y);
			//manifold->points[0].localPoint2 = b2MulT(xf2, position);
			dX = positionX - xf2.position.x;
			dY = positionY - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (dX * tMat.col1.x + dY * tMat.col1.y);
			tPoint.localPoint2.y = (dX * tMat.col2.x + dY * tMat.col2.y);

			tPoint.separation = separation - radius;
			return;
		}

		// Project the circle center onto the edge segment.
		const vertIndex1: number /** int */ = normalIndex;
		const vertIndex2: number /** int */ = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		tVec = vertices[vertIndex1];
		const tVec2: b2Vec2 = vertices[vertIndex2];
		//var e:b2Vec2 = b2Math.SubtractVV(vertices[vertIndex2] , polygon.vertices[vertIndex1]);
		let eX: number = tVec2.x - tVec.x;
		let eY: number = tVec2.y - tVec.y;

		//var length:number = e.Normalize();
		const length: number = Math.sqrt(eX * eX + eY * eY);
		eX /= length;
		eY /= length;
		//b2Assert(length > B2_FLT_EPSILON);

		// Project the center onto the edge.
		//float32 u = b2Dot(cLocal - polygon->m_vertices[vertIndex1], e);
		dX = cLocalX - tVec.x;
		dY = cLocalY - tVec.y;
		const u: number = dX * eX + dY * eY;

		tPoint = manifold.points[0];

		let pX: number, pY: number;
		if (u <= 0.0) {
			pX = tVec.x;
			pY = tVec.y;
			tPoint.id.features.incidentEdge = b2Collision.b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex1;
		} else if (u >= length) {
			pX = tVec2.x;
			pY = tVec2.y;
			tPoint.id.features.incidentEdge = b2Collision.b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex2;
		} else {
			//p = vertices[vertIndex1] + u * e;
			pX = eX * u + tVec.x;
			pY = eY * u + tVec.y;
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = 0;
		}

		//d = b2Math.SubtractVV(xLocal , p);
		dX = cLocalX - pX;
		dY = cLocalY - pY;
		//dist = d.Normalize();
		dist = Math.sqrt(dX * dX + dY * dY);
		dX /= dist;
		dY /= dist;
		if (dist > radius) {
			return;
		}

		manifold.pointCount = 1;
		//manifold->normal = b2Mul(xf1.R, d);
		tMat = xf1.R;
		manifold.normal.x = tMat.col1.x * dX + tMat.col2.x * dY;
		manifold.normal.y = tMat.col1.y * dX + tMat.col2.y * dY;
		//b2Vec2 position = c - radius * manifold->normal;
		positionX = cX - radius * manifold.normal.x;
		positionY = cY - radius * manifold.normal.y;
		//manifold->points[0].localPoint1 = b2MulT(xf1, position);
		dX = positionX - xf1.position.x;
		dY = positionY - xf1.position.y;
		tMat = xf1.R;
		tPoint.localPoint1.x = (dX * tMat.col1.x + dY * tMat.col1.y);
		tPoint.localPoint1.y = (dX * tMat.col2.x + dY * tMat.col2.y);
		//manifold->points[0].localPoint2 = b2MulT(xf2, position);
		dX = positionX - xf2.position.x;
		dY = positionY - xf2.position.y;
		tMat = xf2.R;
		tPoint.localPoint2.x = (dX * tMat.col1.x + dY * tMat.col1.y);
		tPoint.localPoint2.y = (dX * tMat.col2.x + dY * tMat.col2.y);
		tPoint.separation = dist - radius;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
	}

	public static b2TestOverlap(a: b2AABB, b: b2AABB): boolean {
		let t1: b2Vec2 = b.lowerBound;
		let t2: b2Vec2 = a.upperBound;
		//d1 = b2Math.SubtractVV(b.lowerBound, a.upperBound);
		const d1X: number = t1.x - t2.x;
		const d1Y: number = t1.y - t2.y;
		//d2 = b2Math.SubtractVV(a.lowerBound, b.upperBound);
		t1 = a.lowerBound;
		t2 = b.upperBound;
		const d2X: number = t1.x - t2.x;
		const d2Y: number = t1.y - t2.y;

		if (d1X > 0.0 || d1Y > 0.0)
			return false;

		if (d2X > 0.0 || d2Y > 0.0)
			return false;

		return true;
	}
}