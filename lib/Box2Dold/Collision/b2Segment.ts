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

// A manifold for two touching convex shapes.
export class b2Segment {
	/// Ray cast against this segment with another segment.
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.4.1
	// x = mu1 * p1 + mu2 * p2
	// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
	// mu1 = 1 - mu2;
	// x = (1 - mu2) * p1 + mu2 * p2
	//   = p1 + mu2 * (p2 - p1)
	// x = s + a * r (s := start, r := end - start)
	// s + a * r = p1 + mu2 * d (d := p2 - p1)
	// -a * r + mu2 * d = b (b := s - p1)
	// [-r d] * [a; mu2] = b
	// Cramer's rule:
	// denom = det[-r d]
	// a = det[b d] / denom
	// mu2 = det[-r b] / denom
	public TestSegment(lambda: number[], // float pointer
		normal: b2Vec2, // pointer
		segment: b2Segment,
		maxLambda: number): boolean {
		//b2Vec2 s = segment.p1;
		const s: b2Vec2 = segment.p1;
		//b2Vec2 r = segment.p2 - s;
		const rX: number = segment.p2.x - s.x;
		const rY: number = segment.p2.y - s.y;
		//b2Vec2 d = this.p2 - this.p1;
		const dX: number = this.p2.x - this.p1.x;
		const dY: number = this.p2.y - this.p1.y;
		//b2Vec2 n = b2Cross(d, 1.0f);
		let nX: number = dY;
		let nY: number = -dX;

		const k_slop: number = 100.0 * Number.MIN_VALUE;
		//var denom:number = -b2Dot(r, n);
		const denom: number = -(rX * nX + rY * nY);

		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop) {
			// Does the segment intersect the infinite line associated with this segment?
			//b2Vec2 b = s - p1;
			const bX: number = s.x - this.p1.x;
			const bY: number = s.y - this.p1.y;
			//var a:number = b2Dot(b, n);
			let a: number = (bX * nX + bY * nY);

			if (0.0 <= a && a <= maxLambda * denom) {
				const mu2: number = -rX * bY + rY * bX;

				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
					a /= denom;
					//n.Normalize();
					const nLen: number = Math.sqrt(nX * nX + nY * nY);
					nX /= nLen;
					nY /= nLen;
					//*lambda = a;
					lambda[0] = a;
					//*normal = n;
					normal.Set(nX, nY);
					return true;
				}
			}
		}

		return false;
	}

	public p1: b2Vec2 = new b2Vec2();	///< the starting point
	public p2: b2Vec2 = new b2Vec2();	///< the ending point
}