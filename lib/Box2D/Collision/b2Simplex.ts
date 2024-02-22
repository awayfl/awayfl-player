import { b2DistanceProxy } from './b2DistanceProxy';
import { b2Transform, b2Vec2, b2Math } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2SimplexVertex } from './b2SimplexVertex';
import { b2SimplexCache } from './b2SimplexCache';

export class b2Simplex {
	__fast__: boolean = true;

	constructor() {
		this.m_vertices[0] = this.m_v1;
		this.m_vertices[1] = this.m_v2;
		this.m_vertices[2] = this.m_v3;
	}

	public ReadCache(cache: b2SimplexCache,
		proxyA: b2DistanceProxy, transformA: b2Transform,
		proxyB: b2DistanceProxy, transformB: b2Transform): void {
		b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);

		let wALocal: b2Vec2;
		let wBLocal: b2Vec2;

		// Copy data from cache.
		this.m_count = cache.count;
		const vertices: Array<b2SimplexVertex> = this.m_vertices;
		for (let i: number /** int */ = 0; i < this.m_count; i++) {
			var v: b2SimplexVertex = vertices[i];
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			wALocal = proxyA.GetVertex(v.indexA);
			wBLocal = proxyB.GetVertex(v.indexB);
			v.wA = b2Math.MulX(transformA, wALocal);
			v.wB = b2Math.MulX(transformB, wBLocal);
			v.w = b2Math.SubtractVV(v.wB, v.wA);
			v.a = 0;
		}

		// Compute the new simplex metric, if it substantially different than
		// old metric then flush the simplex
		if (this.m_count > 1) {
			const metric1: number = cache.metric;
			const metric2: number = this.GetMetric();
			if (metric2 < .5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
				// Reset the simplex
				this.m_count = 0;
			}
		}

		// If the cache is empty or invalid
		if (this.m_count == 0) {
			v = vertices[0];
			v.indexA = 0;
			v.indexB = 0;
			wALocal = proxyA.GetVertex(0);
			wBLocal = proxyB.GetVertex(0);
			v.wA = b2Math.MulX(transformA, wALocal);
			v.wB = b2Math.MulX(transformB, wBLocal);
			v.w = b2Math.SubtractVV(v.wB, v.wA);
			this.m_count = 1;
		}
	}

	public WriteCache(cache: b2SimplexCache): void {
		cache.metric = this.GetMetric();
		cache.count = this.m_count >>> 0;
		const vertices: Array<b2SimplexVertex> = this.m_vertices;
		for (let i: number /** int */ = 0; i < this.m_count; i++) {
			cache.indexA[i] = vertices[i].indexA;
			cache.indexB[i] = vertices[i].indexB;
		}
	}

	public GetSearchDirection(): b2Vec2 {
		switch (this.m_count) {
			case 1:
				return this.m_v1.w.GetNegative();

			case 2:
			{
				const e12: b2Vec2 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w);
				const sgn: number = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
				if (sgn > 0.0) {
					// Origin is left of e12.
					return b2Math.CrossFV(1.0, e12);
				} else {
					// Origin is right of e12.
					return b2Math.CrossVF(e12, 1.0);
				}
			}
			default:
				b2Settings.b2Assert(false);
				return new b2Vec2();
		}
	}

	public GetClosestPoint(): b2Vec2 {
		switch (this.m_count) {
			case 0:
				b2Settings.b2Assert(false);
				return new b2Vec2();
			case 1:
				return this.m_v1.w;
			case 2:
				return new b2Vec2(
					this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x,
					this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
			default:
				b2Settings.b2Assert(false);
				return new b2Vec2();
		}
	}

	public GetWitnessPoints(pA: b2Vec2, pB: b2Vec2): void {
		switch (this.m_count) {
			case 0:
				b2Settings.b2Assert(false);
				break;
			case 1:
				pA.SetV(this.m_v1.wA);
				pB.SetV(this.m_v1.wB);
				break;
			case 2:
				pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
				pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
				pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
				pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
				break;
			case 3:
				pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
				pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
				break;
			default:
				b2Settings.b2Assert(false);
				break;
		}
	}

	public GetMetric(): number {
		switch (this.m_count) {
			case 0:
				b2Settings.b2Assert(false);
				return 0.0;

			case 1:
				return 0.0;

			case 2:
				return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w).Length();

			case 3:
				return b2Math.CrossVV(b2Math.SubtractVV(this.m_v2.w, this.m_v1.w),b2Math.SubtractVV(this.m_v3.w, this.m_v1.w));

			default:
				b2Settings.b2Assert(false);
				return 0.0;
		}
	}

	// Solve a line segment using barycentric coordinates.
	//
	// p = a1 * w1 + a2 * w2
	// a1 + a2 = 1
	//
	// The vector from the origin to the closest point on the line is
	// perpendicular to the line.
	// e12 = w2 - w1
	// dot(p, e) = 0
	// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
	//
	// 2-by-2 linear system
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	//
	// Define
	// d12_1 =  dot(w2, e12)
	// d12_2 = -dot(w1, e12)
	// d12 = d12_1 + d12_2
	//
	// Solution
	// a1 = d12_1 / d12
	// a2 = d12_2 / d12
	public Solve2(): void {
		const w1: b2Vec2 = this.m_v1.w;
		const w2: b2Vec2 = this.m_v2.w;
		const e12: b2Vec2 = b2Math.SubtractVV(w2, w1);

		// w1 region
		const d12_2: number = -(w1.x * e12.x + w1.y * e12.y);
		if (d12_2 <= 0.0) {
			// a2 <= 0, so we clamp it to 0
			this.m_v1.a = 1.0;
			this.m_count = 1;
			return;
		}

		// w2 region
		const d12_1: number = (w2.x * e12.x + w2.y * e12.y);
		if (d12_1 <= 0.0) {
			// a1 <= 0, so we clamp it to 0
			this.m_v2.a = 1.0;
			this.m_count = 1;
			this.m_v1.Set(this.m_v2);
			return;
		}

		// Must be in e12 region.
		const inv_d12: number = 1.0 / (d12_1 + d12_2);
		this.m_v1.a = d12_1 * inv_d12;
		this.m_v2.a = d12_2 * inv_d12;
		this.m_count = 2;
	}

	public Solve3(): void {
		const w1: b2Vec2 = this.m_v1.w;
		const w2: b2Vec2 = this.m_v2.w;
		const w3: b2Vec2 = this.m_v3.w;

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		const e12: b2Vec2 = b2Math.SubtractVV(w2, w1);
		const w1e12: number = b2Math.Dot(w1, e12);
		const w2e12: number = b2Math.Dot(w2, e12);
		const d12_1: number = w2e12;
		const d12_2: number = -w1e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		const e13: b2Vec2 = b2Math.SubtractVV(w3, w1);
		const w1e13: number = b2Math.Dot(w1, e13);
		const w3e13: number = b2Math.Dot(w3, e13);
		const d13_1: number = w3e13;
		const d13_2: number = -w1e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		const e23: b2Vec2 = b2Math.SubtractVV(w3, w2);
		const w2e23: number = b2Math.Dot(w2, e23);
		const w3e23: number = b2Math.Dot(w3, e23);
		const d23_1: number = w3e23;
		const d23_2: number = -w2e23;

		// Triangle123
		const n123: number = b2Math.CrossVV(e12, e13);

		const d123_1: number = n123 * b2Math.CrossVV(w2, w3);
		const d123_2: number = n123 * b2Math.CrossVV(w3, w1);
		const d123_3: number = n123 * b2Math.CrossVV(w1, w2);

		// w1 region
		if (d12_2 <= 0.0 && d13_2 <= 0.0) {
			this.m_v1.a = 1.0;
			this.m_count = 1;
			return;
		}

		// e12
		if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
			const inv_d12: number = 1.0 / (d12_1 + d12_2);
			this.m_v1.a = d12_1 * inv_d12;
			this.m_v2.a = d12_2 * inv_d12;
			this.m_count = 2;
			return;
		}

		// e13
		if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
			const inv_d13: number = 1.0 / (d13_1 + d13_2);
			this.m_v1.a = d13_1 * inv_d13;
			this.m_v3.a = d13_2 * inv_d13;
			this.m_count = 2;
			this.m_v2.Set(this.m_v3);
			return;
		}

		// w2 region
		if (d12_1 <= 0.0 && d23_2 <= 0.0) {
			this.m_v2.a = 1.0;
			this.m_count = 1;
			this.m_v1.Set(this.m_v2);
			return;
		}

		// w3 region
		if (d13_1 <= 0.0 && d23_1 <= 0.0) {
			this.m_v3.a = 1.0;
			this.m_count = 1;
			this.m_v1.Set(this.m_v3);
			return;
		}

		// e23
		if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
			const inv_d23: number = 1.0 / (d23_1 + d23_2);
			this.m_v2.a = d23_1 * inv_d23;
			this.m_v3.a = d23_2 * inv_d23;
			this.m_count = 2;
			this.m_v1.Set(this.m_v3);
			return;
		}

		// Must be in triangle123
		const inv_d123: number = 1.0 / (d123_1 + d123_2 + d123_3);
		this.m_v1.a = d123_1 * inv_d123;
		this.m_v2.a = d123_2 * inv_d123;
		this.m_v3.a = d123_3 * inv_d123;
		this.m_count = 3;
	}

	public m_v1: b2SimplexVertex = new b2SimplexVertex();
	public m_v2: b2SimplexVertex = new b2SimplexVertex();
	public m_v3: b2SimplexVertex = new b2SimplexVertex();
	public m_vertices: Array<b2SimplexVertex> = new Array<b2SimplexVertex>(3);
	public m_count: number /** int */;
}