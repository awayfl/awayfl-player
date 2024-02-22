import { b2Vec2 } from '../Common/Math';
import { b2RayCastOutput } from './b2RayCastOutput';
import { b2RayCastInput } from './b2RayCastInput';

/**
* An axis aligned bounding box.
*/
export class b2AABB {
	readonly __fast__ = true;
	/**
    * Verify that the bounds are sorted.
    */
	public IsValid(): boolean {
		//b2Vec2 d = upperBound - lowerBound;;
		const dX: number = this.upperBound.x - this.lowerBound.x;
		const dY: number = this.upperBound.y - this.lowerBound.y;
		let valid: boolean = dX >= 0.0 && dY >= 0.0;
		valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
		return valid;
	}

	/** Get the center of the AABB. */
	public GetCenter(): b2Vec2 {
		return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2,
			(this.lowerBound.y + this.upperBound.y) / 2);
	}

	/** Get the extents of the AABB (half-widths). */
	public GetExtents(): b2Vec2 {
		return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2,
			(this.upperBound.y - this.lowerBound.y) / 2);
	}

	/**
     * Is an AABB contained within this one.
     */
	public Contains(aabb: b2AABB): boolean {
		let result: boolean = true;
		result = result && this.lowerBound.x <= aabb.lowerBound.x;
		result = result && this.lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= this.upperBound.x;
		result = result && aabb.upperBound.y <= this.upperBound.y;
		return result;
	}

	// From Real-time Collision Detection, p179.
	/**
     * Perform a precise raycast against the AABB.
     */
	public RayCast(output: b2RayCastOutput, input: b2RayCastInput): boolean {
		let tmin: number = -Number.MAX_VALUE;
		let tmax: number = Number.MAX_VALUE;

		const pX: number = input.p1.x;
		const pY: number = input.p1.y;
		const dX: number = input.p2.x - input.p1.x;
		const dY: number = input.p2.y - input.p1.y;
		const absDX: number = Math.abs(dX);
		const absDY: number = Math.abs(dY);

		const normal: b2Vec2 = output.normal;

		let inv_d: number;
		let t1: number;
		let t2: number;
		let t3: number;
		let s: number;

		//x
		{
			if (absDX < Number.MIN_VALUE) {
				// Parallel.
				if (pX < this.lowerBound.x || this.upperBound.x < pX)
					return false;
			} else {
				inv_d = 1.0 / dX;
				t1 = (this.lowerBound.x - pX) * inv_d;
				t2 = (this.upperBound.x - pX) * inv_d;

				// Sign of the normal vector
				s = -1.0;

				if (t1 > t2) {
					t3 = t1;
					t1 = t2;
					t2 = t3;
					s = 1.0;
				}

				// Push the min up
				if (t1 > tmin) {
					normal.x = s;
					normal.y = 0;
					tmin = t1;
				}

				// Pull the max down
				tmax = Math.min(tmax, t2);

				if (tmin > tmax)
					return false;
			}
		}
		//y
		{
			if (absDY < Number.MIN_VALUE) {
				// Parallel.
				if (pY < this.lowerBound.y || this.upperBound.y < pY)
					return false;
			} else {
				inv_d = 1.0 / dY;
				t1 = (this.lowerBound.y - pY) * inv_d;
				t2 = (this.upperBound.y - pY) * inv_d;

				// Sign of the normal vector
				s = -1.0;

				if (t1 > t2) {
					t3 = t1;
					t1 = t2;
					t2 = t3;
					s = 1.0;
				}

				// Push the min up
				if (t1 > tmin) {
					normal.y = s;
					normal.x = 0;
					tmin = t1;
				}

				// Pull the max down
				tmax = Math.min(tmax, t2);

				if (tmin > tmax)
					return false;
			}
		}

		output.fraction = tmin;
		return true;
	}

	/**
     * Tests if another AABB overlaps this one.
     */
	public TestOverlap(other: b2AABB): boolean {
		const d1X: number = other.lowerBound.x - this.upperBound.x;
		const d1Y: number = other.lowerBound.y - this.upperBound.y;
		const d2X: number = this.lowerBound.x - other.upperBound.x;
		const d2Y: number = this.lowerBound.y - other.upperBound.y;

		if (d1X > 0.0 || d1Y > 0.0)
			return false;

		if (d2X > 0.0 || d2Y > 0.0)
			return false;

		return true;
	}

	/** Combine two AABBs into one. */
	public static Combine(aabb1: b2AABB, aabb2: b2AABB): b2AABB {
		const aabb: b2AABB = new b2AABB();
		aabb.Combine(aabb1, aabb2);
		return aabb;
	}

	/** Combine two AABBs into one. */
	public Combine(aabb1: b2AABB, aabb2: b2AABB): void {
		this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
		this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
		this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
		this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
	}

	/** The lower vertex */
	public lowerBound: b2Vec2 = new b2Vec2();
	/** The upper vertex */
	public upperBound: b2Vec2 = new b2Vec2();
}