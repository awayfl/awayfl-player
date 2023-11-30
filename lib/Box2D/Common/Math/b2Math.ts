import { b2Vec2, b2Mat22, b2Transform } from '../Math';

/**
* @private
*/
export class b2Math {

	/**
	* This function is used to ensure that a floating point number is
	* not a NaN or infinity.
	*/
	public static IsValid(x: number): boolean {
		return isFinite(x);
	}

	/*public static b2InvSqrt(x:number):number{
		union
		{
			float32 x;
			int32 i;
		} convert;

		convert.x = x;
		float32 xhalf = 0.5f * x;
		convert.i = 0x5f3759df - (convert.i >> 1);
		x = convert.x;
		x = x * (1.5f - xhalf * x * x);
		return x;
	}*/

	public static Dot(a: b2Vec2, b: b2Vec2): number {
		return a.x * b.x + a.y * b.y;
	}

	public static CrossVV(a: b2Vec2, b: b2Vec2): number {
		return a.x * b.y - a.y * b.x;
	}

	public static CrossVF(a: b2Vec2, s: number): b2Vec2 {
		const v: b2Vec2 = new b2Vec2(s * a.y, -s * a.x);
		return v;
	}

	public static CrossFV(s: number, a: b2Vec2): b2Vec2 {
		const v: b2Vec2 = new b2Vec2(-s * a.y, s * a.x);
		return v;
	}

	public static MulMV(A: b2Mat22, v: b2Vec2): b2Vec2 {
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		const u: b2Vec2 = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
		return u;
	}

	public static MulTMV(A: b2Mat22, v: b2Vec2): b2Vec2 {
		// (tVec.x * tMat.col1.x + tVec.y * tMat.col1.y)
		// (tVec.x * tMat.col2.x + tVec.y * tMat.col2.y)
		const u: b2Vec2 = new b2Vec2(this.Dot(v, A.col1), this.Dot(v, A.col2));
		return u;
	}

	public static MulX(T: b2Transform, v: b2Vec2): b2Vec2 {
		const a: b2Vec2 = this.MulMV(T.R, v);
		a.x += T.position.x;
		a.y += T.position.y;
		//return T.position + b2Mul(T.R, v);
		return a;
	}

	public static MulXT(T: b2Transform, v: b2Vec2): b2Vec2 {
		const a: b2Vec2 = this.SubtractVV(v, T.position);
		//return b2MulT(T.R, v - T.position);
		const tX: number = (a.x * T.R.col1.x + a.y * T.R.col1.y);
		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
		a.x = tX;
		return a;
	}

	public static AddVV(a: b2Vec2, b: b2Vec2): b2Vec2 {
		const v: b2Vec2 = new b2Vec2(a.x + b.x, a.y + b.y);
		return v;
	}

	public static SubtractVV(a: b2Vec2, b: b2Vec2): b2Vec2 {
		const v: b2Vec2 = new b2Vec2(a.x - b.x, a.y - b.y);
		return v;
	}

	public static Distance(a: b2Vec2, b: b2Vec2): Number {
		const cX: number = a.x - b.x;
		const cY: number = a.y - b.y;
		return Math.sqrt(cX * cX + cY * cY);
	}

	public static DistanceSquared(a: b2Vec2, b: b2Vec2): Number {
		const cX: number = a.x - b.x;
		const cY: number = a.y - b.y;
		return (cX * cX + cY * cY);
	}

	public static MulFV(s: number, a: b2Vec2): b2Vec2 {
		const v: b2Vec2 = new b2Vec2(s * a.x, s * a.y);
		return v;
	}

	public static AddMM(A: b2Mat22, B: b2Mat22): b2Mat22 {
		const C: b2Mat22 = b2Mat22.FromVV(this.AddVV(A.col1, B.col1), this.AddVV(A.col2, B.col2));
		return C;
	}

	// A * B
	public static MulMM(A: b2Mat22, B: b2Mat22): b2Mat22 {
		const C: b2Mat22 = b2Mat22.FromVV(this.MulMV(A, B.col1), this.MulMV(A, B.col2));
		return C;
	}

	// A^T * B
	public static MulTMM(A: b2Mat22, B: b2Mat22): b2Mat22 {
		const c1: b2Vec2 = new b2Vec2(this.Dot(A.col1, B.col1), this.Dot(A.col2, B.col1));
		const c2: b2Vec2 = new b2Vec2(this.Dot(A.col1, B.col2), this.Dot(A.col2, B.col2));
		const C: b2Mat22 = b2Mat22.FromVV(c1, c2);
		return C;
	}

	public static Abs(a: number): number {
		return a > 0.0 ? a : -a;
	}

	public static AbsV(a: b2Vec2): b2Vec2 {
		const b: b2Vec2 = new b2Vec2(this.Abs(a.x), this.Abs(a.y));
		return b;
	}

	public static AbsM(A: b2Mat22): b2Mat22 {
		const B: b2Mat22 = b2Mat22.FromVV(this.AbsV(A.col1), this.AbsV(A.col2));
		return B;
	}

	public static Min(a: number, b: number): number {
		return a < b ? a : b;
	}

	public static MinV(a: b2Vec2, b: b2Vec2): b2Vec2 {
		const c: b2Vec2 = new b2Vec2(this.Min(a.x, b.x), this.Min(a.y, b.y));
		return c;
	}

	public static Max(a: number, b: number): number {
		return a > b ? a : b;
	}

	public static MaxV(a: b2Vec2, b: b2Vec2): b2Vec2 {
		const c: b2Vec2 = new b2Vec2(this.Max(a.x, b.x), this.Max(a.y, b.y));
		return c;
	}

	public static Clamp(a: number, low: number, high: number): number {
		return a < low ? low : a > high ? high : a;
	}

	public static ClampV(a: b2Vec2, low: b2Vec2, high: b2Vec2): b2Vec2 {
		return this.MaxV(low, this.MinV(a, high));
	}

	public static Swap(a: Array<any>, b: Array<any>): void {
		const tmp: any = a[0];
		a[0] = b[0];
		b[0] = tmp;
	}

	// b2Random number in range [-1,1]
	public static Random(): number {
		return Math.random() * 2 - 1;
	}

	public static RandomRange(lo: number, hi: number): Number {
		let r: number = Math.random();
		r = (hi - lo) * r + lo;
		return r;
	}

	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	public static NextPowerOfTwo(x: number /** uint */): number /** uint */
	{
		x |= (x >> 1) & 0x7FFFFFFF;
		x |= (x >> 2) & 0x3FFFFFFF;
		x |= (x >> 4) & 0x0FFFFFFF;
		x |= (x >> 8) & 0x00FFFFFF;
		x |= (x >> 16) & 0x0000FFFF;
		return x + 1;
	}

	public static IsPowerOfTwo(x: number /** uint */): boolean {
		const result: boolean = x > 0 && (x & (x - 1)) == 0;
		return result;
	}

	// Temp vector functions to reduce calls to 'new'
	/*public static var tempVec:b2Vec2 = new b2Vec2();
	public static var tempVec2:b2Vec2 = new b2Vec2();
	public static var tempVec3:b2Vec2 = new b2Vec2();
	public static var tempVec4:b2Vec2 = new b2Vec2();
	public static var tempVec5:b2Vec2 = new b2Vec2();

	public static var tempMat:b2Mat22 = new b2Mat22();

	public static var tempAABB:b2AABB = new b2AABB();	*/

	public static readonly b2Vec2_zero: b2Vec2 = new b2Vec2(0.0, 0.0);
	public static readonly b2Mat22_identity: b2Mat22 = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
	public static readonly b2Transform_identity: b2Transform = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);

}