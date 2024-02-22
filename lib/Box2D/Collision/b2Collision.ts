import { b2Vec2, b2Mat22, b2Transform } from '../Common/Math';
import { b2PolygonShape } from './Shapes/b2PolygonShape';
import { ClipVertex } from './ClipVertex';
import { b2Manifold } from './b2Manifold';
import { b2Settings } from '../Common/b2Settings';
import { b2ManifoldPoint } from './b2ManifoldPoint';
import { b2CircleShape } from './Shapes/b2CircleShape';
import { b2AABB } from './b2AABB';

/**
* @private
*/
export class b2Collision {

	// Null feature
	public static readonly b2_nullFeature: number /** uint */ = 0x000000ff;//UCHAR_MAX;

	// Sutherland-Hodgman clipping.
	static ClipSegmentToLine(vOut: Array<ClipVertex>, vIn: Array<ClipVertex>, normal: b2Vec2, offset: number): number /** int */
	{
		let cv: ClipVertex;

		// Start with no output points
		let numOut: number /** int */ = 0;

		cv = vIn[0];
		const vIn0: b2Vec2 = cv.v;
		cv = vIn[1];
		const vIn1: b2Vec2 = cv.v;

		// Calculate the distance of end points to the line
		const distance0: number = normal.x * vIn0.x + normal.y * vIn0.y - offset;
		const distance1: number = normal.x * vIn1.x + normal.y * vIn1.y - offset;

		// If the points are behind the plane
		if (distance0 <= 0.0) vOut[numOut++].Set(vIn[0]);
		if (distance1 <= 0.0) vOut[numOut++].Set(vIn[1]);

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
	public static EdgeSeparation(poly1: b2PolygonShape, xf1: b2Transform, edge1: number /** int */,
		poly2: b2PolygonShape, xf2: b2Transform): number {
		const count1: number /** int */ = poly1.m_vertexCount;
		const vertices1: Array<b2Vec2> = poly1.m_vertices;
		const normals1: Array<b2Vec2> = poly1.m_normals;

		const count2: number /** int */ = poly2.m_vertexCount;
		const vertices2: Array<b2Vec2> = poly2.m_vertices;

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
	public static FindMaxSeparation(edgeIndex: Array<number /** int */>,
		poly1: b2PolygonShape, xf1: b2Transform,
		poly2: b2PolygonShape, xf2: b2Transform): number {
		const count1: number /** int */ = poly1.m_vertexCount;
		const normals1: Array<b2Vec2> = poly1.m_normals;

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

		// Check the separation for the previous edge normal.
		const prevEdge: number /** int */ = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		const sPrev: number = this.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

		// Check the separation for the next edge normal.
		const nextEdge: number /** int */ = edge + 1 < count1 ? edge + 1 : 0;
		const sNext: number = this.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

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

	public static FindIncidentEdge(c: Array<ClipVertex>,
		poly1: b2PolygonShape, xf1: b2Transform, edge1: number /** int */,
		poly2: b2PolygonShape, xf2: b2Transform): void {
		const count1: number /** int */ = poly1.m_vertexCount;
		const normals1: Array<b2Vec2> = poly1.m_normals;

		const count2: number /** int */ = poly2.m_vertexCount;
		const vertices2: Array<b2Vec2> = poly2.m_vertices;
		const normals2: Array<b2Vec2> = poly2.m_normals;

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

	private static MakeClipPointVector(): Array<ClipVertex> {
		const r: Array<ClipVertex> = new Array<ClipVertex>(2);
		r[0] = new ClipVertex();
		r[1] = new ClipVertex();
		return r;
	}

	private static s_incidentEdge: Array<ClipVertex> = b2Collision.MakeClipPointVector();
	private static s_clipPoints1: Array<ClipVertex> = b2Collision.MakeClipPointVector();
	private static s_clipPoints2: Array<ClipVertex> = b2Collision.MakeClipPointVector();
	private static s_edgeAO: Array<number /** int */> = new Array<number /** int */>(1);
	private static s_edgeBO: Array<number /** int */> = new Array<number /** int */>(1);
	private static s_localTangent: b2Vec2 = new b2Vec2();
	private static s_localNormal: b2Vec2 = new b2Vec2();
	private static s_planePoint: b2Vec2 = new b2Vec2();
	private static s_normal: b2Vec2 = new b2Vec2();
	private static s_tangent: b2Vec2 = new b2Vec2();
	private static s_tangent2: b2Vec2 = new b2Vec2();
	private static s_v11: b2Vec2 = new b2Vec2();
	private static s_v12: b2Vec2 = new b2Vec2();
	// Find edge normal of max separation on A - return if separating axis is found
	// Find edge normal of max separation on B - return if separation axis is found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip
	private static b2CollidePolyTempVec: b2Vec2 = new b2Vec2();
	// The normal points from 1 to 2
	public static CollidePolygons(manifold: b2Manifold,
		polyA: b2PolygonShape, xfA: b2Transform,
		polyB: b2PolygonShape, xfB: b2Transform): void {
		let cv: ClipVertex;

		manifold.m_pointCount = 0;
		const totalRadius: number = polyA.m_radius + polyB.m_radius;

		let edgeA: number /** int */ = 0;
		b2Collision.s_edgeAO[0] = edgeA;
		const separationA: number = this.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB);
		edgeA = b2Collision.s_edgeAO[0];
		if (separationA > totalRadius)
			return;

		let edgeB: number /** int */ = 0;
		b2Collision.s_edgeBO[0] = edgeB;
		const separationB: number = this.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA);
		edgeB = b2Collision.s_edgeBO[0];
		if (separationB > totalRadius)
			return;

		let poly1: b2PolygonShape;	// reference poly
		let poly2: b2PolygonShape;	// incident poly
		let xf1: b2Transform;
		let xf2: b2Transform;
		let edge1: number /** int */;		// reference edge
		let flip: number /** uint */;
		const k_relativeTol: number = 0.98;
		const k_absoluteTol: number = 0.001;
		let tMat: b2Mat22;

		if (separationB > k_relativeTol * separationA + k_absoluteTol) {
			poly1 = polyB;
			poly2 = polyA;
			xf1 = xfB;
			xf2 = xfA;
			edge1 = edgeB;
			manifold.m_type = b2Manifold.e_faceB;
			flip = 1;
		} else {
			poly1 = polyA;
			poly2 = polyB;
			xf1 = xfA;
			xf2 = xfB;
			edge1 = edgeA;
			manifold.m_type = b2Manifold.e_faceA;
			flip = 0;
		}

		const incidentEdge: Array<ClipVertex> = b2Collision.s_incidentEdge;
		this.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		const count1: number /** int */ = poly1.m_vertexCount;
		const vertices1: Array<b2Vec2> = poly1.m_vertices;

		const local_v11: b2Vec2 = vertices1[edge1];
		let local_v12: b2Vec2;
		if (edge1 + 1 < count1) {
			local_v12 = vertices1[edge1 + 1];
		} else {
			local_v12 = vertices1[0];
		}

		const localTangent: b2Vec2 = b2Collision.s_localTangent;
		localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
		localTangent.Normalize();

		const localNormal: b2Vec2 = b2Collision.s_localNormal;
		localNormal.x = localTangent.y;
		localNormal.y = -localTangent.x;

		const planePoint: b2Vec2 = b2Collision.s_planePoint;
		planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));

		const tangent: b2Vec2 = b2Collision.s_tangent;
		//tangent = b2Math.b2MulMV(xf1.R, localTangent);
		tMat = xf1.R;
		tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y);
		tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y);
		const tangent2: b2Vec2 = b2Collision.s_tangent2;
		tangent2.x = -tangent.x;
		tangent2.y = -tangent.y;
		const normal: b2Vec2 = b2Collision.s_normal;
		normal.x = tangent.y;
		normal.y = -tangent.x;

		//v11 = b2Math.MulX(xf1, local_v11);
		//v12 = b2Math.MulX(xf1, local_v12);
		const v11: b2Vec2 = b2Collision.s_v11;
		const v12: b2Vec2 = b2Collision.s_v12;
		v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
		v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
		v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
		v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);

		// Face offset
		const frontOffset: number = normal.x * v11.x + normal.y * v11.y;
		// Side offsets, extended by polytope skin thickness
		const sideOffset1: number = -tangent.x * v11.x - tangent.y * v11.y + totalRadius;
		const sideOffset2: number = tangent.x * v12.x + tangent.y * v12.y + totalRadius;

		// Clip incident edge against extruded edge1 side edges.
		const clipPoints1: Array<ClipVertex> = b2Collision.s_clipPoints1;
		const clipPoints2: Array<ClipVertex> = b2Collision.s_clipPoints2;
		let np: number /** int */;

		// Clip to box side 1
		//np = ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1);
		np = this.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);

		if (np < 2)
			return;

		// Clip to negative box side 1
		np = this.ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2);

		if (np < 2)
			return;

		// Now clipPoints2 contains the clipped points.
		manifold.m_localPlaneNormal.SetV(localNormal);
		manifold.m_localPoint.SetV(planePoint);

		let pointCount: number /** int */ = 0;
		for (let i: number /** int */ = 0; i < b2Settings.b2_maxManifoldPoints;++i) {
			cv = clipPoints2[i];
			const separation: number = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
			if (separation <= totalRadius) {
				const cp: b2ManifoldPoint = manifold.m_points[ pointCount ];
				//cp.m_localPoint = b2Math.b2MulXT(xf2, cv.v);
				tMat = xf2.R;
				const tX: number = cv.v.x - xf2.position.x;
				const tY: number = cv.v.y - xf2.position.y;
				cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y);
				cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y);
				cp.m_id.Set(cv.id);
				cp.m_id.features.flip = flip;
				++pointCount;
			}
		}

		manifold.m_pointCount = pointCount;
	}

	public static CollideCircles(
		manifold: b2Manifold,
		circle1: b2CircleShape, xf1: b2Transform,
		circle2: b2CircleShape, xf2: b2Transform): void {
		manifold.m_pointCount = 0;

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		//b2Vec2 p1 = b2Mul(xf1, circle1->m_p);
		tMat = xf1.R; tVec = circle1.m_p;
		const p1X: number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const p1Y: number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 p2 = b2Mul(xf2, circle2->m_p);
		tMat = xf2.R; tVec = circle2.m_p;
		const p2X: number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		const p2Y: number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 d = p2 - p1;
		const dX: number = p2X - p1X;
		const dY: number = p2Y - p1Y;
		//var distSqr:number = b2Math.b2Dot(d, d);
		const distSqr: number = dX * dX + dY * dY;
		const radius: number = circle1.m_radius + circle2.m_radius;
		if (distSqr > radius * radius) {
			return;
		}
		manifold.m_type = b2Manifold.e_circles;
		manifold.m_localPoint.SetV(circle1.m_p);
		manifold.m_localPlaneNormal.SetZero();
		manifold.m_pointCount = 1;
		manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
		manifold.m_points[0].m_id.key = 0;
	}

	public static CollidePolygonAndCircle(
		manifold: b2Manifold,
		polygon: b2PolygonShape, xf1: b2Transform,
		circle: b2CircleShape, xf2: b2Transform): void {
		manifold.m_pointCount = 0;
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
		tVec = circle.m_p;
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
		const radius: number = polygon.m_radius + circle.m_radius;
		const vertexCount: number /** int */ = polygon.m_vertexCount;
		const vertices: Array<b2Vec2> = polygon.m_vertices;
		const normals: Array<b2Vec2> = polygon.m_normals;

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
		// Vertices that subtend the incident face
		const vertIndex1: number /** int */ = normalIndex;
		const vertIndex2: number /** int */ = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		const v1: b2Vec2 = vertices[vertIndex1];
		const v2: b2Vec2 = vertices[vertIndex2];

		// If the center is inside the polygon ...
		if (separation < Number.MIN_VALUE) {
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
			manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
			manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
			return;
		}

		// Project the circle center onto the edge segment.
		const u1: number = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
		const u2: number = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
		if (u1 <= 0.0) {
			if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius)
				return;
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v1.x;
			manifold.m_localPlaneNormal.y = cLocalY - v1.y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.SetV(v1);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		} else if (u2 <= 0) {
			if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius)
				return;
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v2.x;
			manifold.m_localPlaneNormal.y = cLocalY - v2.y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.SetV(v2);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		} else {
			const faceCenterX: number = 0.5 * (v1.x + v2.x);
			const faceCenterY: number = 0.5 * (v1.y + v2.y);
			separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
			if (separation > radius)
				return;
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
			manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.Set(faceCenterX,faceCenterY);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
	}

	public static TestOverlap(a: b2AABB, b: b2AABB): boolean {
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
