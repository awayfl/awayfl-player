import { b2DistanceProxy } from './b2DistanceProxy';
import { b2Transform, b2Vec2, b2Math } from '../Common/Math';
import { b2Simplex } from './b2Simplex';
import { b2DistanceOutput } from './b2DistanceOutput';
import { b2SimplexCache } from './b2SimplexCache';
import { b2DistanceInput } from './b2DistanceInput';
import { b2SimplexVertex } from './b2SimplexVertex';
import { b2Settings } from '../Common/b2Settings';

/**
* @private
*/
export class b2Distance {
	__fast__: boolean = true;

	// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.

	private static b2_gjkCalls: number /** int */;
	private static b2_gjkIters: number /** int */;
	private static b2_gjkMaxIters: number /** int */;

	private static s_simplex: b2Simplex = new b2Simplex();
	private static s_saveA: number /** int */[] = new Array<number>(3);
	private static s_saveB: number /** int */[] = new Array<number>(3);
	public static Distance(output: b2DistanceOutput, cache: b2SimplexCache, input: b2DistanceInput): void {
		++this.b2_gjkCalls;

		const proxyA: b2DistanceProxy = input.proxyA;
		const proxyB: b2DistanceProxy = input.proxyB;

		const transformA: b2Transform = input.transformA;
		const transformB: b2Transform = input.transformB;

		// Initialize the simplex
		const simplex: b2Simplex = this.s_simplex;
		simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

		// Get simplex vertices as an vector.
		const vertices: Array<b2SimplexVertex> = simplex.m_vertices;
		const k_maxIters: number /** int */ = 20;

		// These store the vertices of the last simplex so that we
		// can check for duplicates and preven cycling
		const saveA: Array<number /** int */> = this.s_saveA;
		const saveB: Array<number /** int */> = this.s_saveB;
		let saveCount: number /** int */ = 0;

		const closestPoint: b2Vec2 = simplex.GetClosestPoint();
		let distanceSqr1: number = closestPoint.LengthSquared();
		let distanceSqr2: number = distanceSqr1;

		let i: number /** int */;
		let p: b2Vec2;

		// Main iteration loop
		let iter: number /** int */ = 0;
		while (iter < k_maxIters) {
			// Copy the simplex so that we can identify duplicates
			saveCount = simplex.m_count;
			for (i = 0; i < saveCount; i++) {
				saveA[i] = vertices[i].indexA;
				saveB[i] = vertices[i].indexB;
			}

			switch (simplex.m_count) {
				case 1:
					break;
				case 2:
					simplex.Solve2();
					break;
				case 3:
					simplex.Solve3();
					break;
				default:
					b2Settings.b2Assert(false);
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if (simplex.m_count == 3) {
				break;
			}

			// Compute the closest point.
			p = simplex.GetClosestPoint();
			distanceSqr2 = p.LengthSquared();

			// Ensure progress
			if (distanceSqr2 > distanceSqr1) {
				//break;
			}
			distanceSqr1 = distanceSqr2;

			// Get search direction.
			const d: b2Vec2 = simplex.GetSearchDirection();

			// Ensure the search direction is numerically fit.
			if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
				// THe origin is probably contained by a line segment or triangle.
				// Thus the shapes are overlapped.

				// We can't return zero here even though there may be overlap.
				// In case the simplex is a point, segment or triangle it is very difficult
				// to determine if the origin is contained in the CSO or very close to it
				break;
			}

			// Compute a tentative new simplex vertex using support points
			const vertex: b2SimplexVertex = vertices[simplex.m_count];
			vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()));
			vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
			vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
			vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
			vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);

			// Iteration count is equated to the number of support point calls.
			++iter;
			++this.b2_gjkIters;

			// Check for duplicate support points. This is the main termination criteria.
			let duplicate: boolean = false;
			for (i = 0; i < saveCount; i++) {
				if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
					duplicate = true;
					break;
				}
			}

			// If we found a duplicate support point we must exist to avoid cycling
			if (duplicate) {
				break;
			}

			// New vertex is ok and needed.
			++simplex.m_count;
		}

		this.b2_gjkMaxIters = b2Math.Max(this.b2_gjkMaxIters, iter);

		// Prepare output
		simplex.GetWitnessPoints(output.pointA, output.pointB);
		output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
		output.iterations = iter;

		// Cache the simplex
		simplex.WriteCache(cache);

		// Apply radii if requested.
		if (input.useRadii) {
			const rA: number = proxyA.m_radius;
			const rB: number = proxyB.m_radius;

			if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
				// Shapes are still not overlapped.
				// Move the witness points to the outer surface.
				output.distance -= rA + rB;
				const normal: b2Vec2 = b2Math.SubtractVV(output.pointB, output.pointA);
				normal.Normalize();
				output.pointA.x += rA * normal.x;
				output.pointA.y += rA * normal.y;
				output.pointB.x -= rB * normal.x;
				output.pointB.y -= rB * normal.y;
			} else {
				// Shapes are overlapped when radii are considered.
				// Move the witness points to the middle.
				p = new b2Vec2();
				p.x = .5 * (output.pointA.x + output.pointB.x);
				p.y = .5 * (output.pointA.y + output.pointB.y);
				output.pointA.x = output.pointB.x = p.x;
				output.pointA.y = output.pointB.y = p.y;
				output.distance = 0.0;
			}
		}
	}
}