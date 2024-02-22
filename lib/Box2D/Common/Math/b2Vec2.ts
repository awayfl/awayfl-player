import { b2Math, b2Mat22 } from '../Math';

/**
* A 2D column vector.
*/
export class b2Vec2 {
	__fast__ = true;

	constructor(x_: number = 0, y_: number = 0) {
		this.x = x_;
		this.y = y_;
	}

	public SetZero(): void {
		this.x = 0.0; this.y = 0.0;
	}

	public Set(x_: number = 0, y_: number = 0): void {
		this.x = x_;
		this.y = y_;
	}

	public SetV(v: b2Vec2): void {
		this.x = v.x;
		this.y = v.y;
	}

	public GetNegative(): b2Vec2 {
		return new b2Vec2(-this.x, -this.y);
	}

	public NegativeSelf(): void {
		this.x = -this.x; this.y = -this.y;
	}

	public static Make(x_: number, y_: number): b2Vec2 {
		return new b2Vec2(x_, y_);
	}

	public Copy(): b2Vec2 {
		return new b2Vec2(this.x,this.y);
	}

	public Add(v: b2Vec2): void {
		this.x += v.x; this.y += v.y;
	}

	public Subtract(v: b2Vec2): void {
		this.x -= v.x; this.y -= v.y;
	}

	public Multiply(a: number): void {
		this.x *= a; this.y *= a;
	}

	public MulM(A: b2Mat22): void {
		const tX: number = this.x;
		this.x = A.col1.x * tX + A.col2.x * this.y;
		this.y = A.col1.y * tX + A.col2.y * this.y;
	}

	public MulTM(A: b2Mat22): void {
		const tX: number = b2Math.Dot(this, A.col1);
		this.y = b2Math.Dot(this, A.col2);
		this.x = tX;
	}

	public CrossVF(s: number): void {
		const tX: number = this.x;
		this.x = s * this.y;
		this.y = -s * tX;
	}

	public CrossFV(s: number): void {
		const tX: number = this.x;
		this.x = -s * this.y;
		this.y = s * tX;
	}

	public MinV(b: b2Vec2): void {
		this.x = this.x < b.x ? this.x : b.x;
		this.y = this.y < b.y ? this.y : b.y;
	}

	public MaxV(b: b2Vec2): void {
		this.x = this.x > b.x ? this.x : b.x;
		this.y = this.y > b.y ? this.y : b.y;
	}

	public Abs(): void {
		if (this.x < 0) this.x = -this.x;
		if (this.y < 0) this.y = -this.y;
	}

	public Length(): number {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}

	public LengthSquared(): number {
		return (this.x * this.x + this.y * this.y);
	}

	public Normalize(): number {
		const length: number = Math.sqrt(this.x * this.x + this.y * this.y);
		if (length < Number.MIN_VALUE) {
			return 0.0;
		}
		const invLength: number = 1.0 / length;
		this.x *= invLength;
		this.y *= invLength;

		return length;
	}

	public IsValid(): boolean {
		return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
	}

	public x: number;
	public y: number;
}