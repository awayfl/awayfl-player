/**
* A 2D column vector with 3 elements.
*/
export class b2Vec3 {
	__fast__ = true;

	/**
     * Construct using co-ordinates
     */
	constructor(x: number = 0, y: number = 0, z: number = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
     * Sets this vector to all zeros
     */
	public SetZero(): void {
		this.x = this.y = this.z = 0.0;
	}

	/**
     * Set this vector to some specified coordinates.
     */
	public Set(x: number, y: number, z: number): void {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public SetV(v: b2Vec3): void {
		this.x = v.x;
		this.y = v.y;
		this.z = v.z;
	}

	/**
     * Negate this vector
     */
	public GetNegative(): b2Vec3 {
		return new b2Vec3(-this.x, -this.y, -this.z);
	}

	public NegativeSelf(): void {
		this.x = -this.x;
		this.y = -this.y;
		this.z = -this.z;
	}

	public Copy(): b2Vec3 {
		return new b2Vec3(this.x,this.y,this.z);
	}

	public Add(v: b2Vec3): void {
		this.x += v.x;
		this.y += v.y;
		this.z += v.z;
	}

	public Subtract(v: b2Vec3): void {
		this.x -= v.x;
		this.y -= v.y;
		this.z -= v.z;
	}

	public Multiply(a: number): void {
		this.x *= a;
		this.y *= a;
		this.z *= a;
	}

	public x: number;
	public y: number;
	public z: number;

}