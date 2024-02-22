import { b2Vec3, b2Vec2 } from '../Math';

/**
* A 3-by-3 matrix. Stored in column-major order.
*/
export class b2Mat33 {
	constructor(c1: b2Vec3 = null, c2: b2Vec3 = null, c3: b2Vec3 = null) {
		if (!c1 && !c2 && !c3) {
			this.col1.SetZero();
			this.col2.SetZero();
			this.col3.SetZero();
		} else {
			this.col1.SetV(c1);
			this.col2.SetV(c2);
			this.col3.SetV(c3);
		}
	}

	public SetVVV(c1: b2Vec3, c2: b2Vec3, c3: b2Vec3): void {
		this.col1.SetV(c1);
		this.col2.SetV(c2);
		this.col3.SetV(c3);
	}

	public Copy(): b2Mat33 {
		return new b2Mat33(this.col1, this.col2, this.col3);
	}

	public SetM(m: b2Mat33): void {
		this.col1.SetV(m.col1);
		this.col2.SetV(m.col2);
		this.col3.SetV(m.col3);
	}

	public AddM(m: b2Mat33): void {
		this.col1.x += m.col1.x;
		this.col1.y += m.col1.y;
		this.col1.z += m.col1.z;
		this.col2.x += m.col2.x;
		this.col2.y += m.col2.y;
		this.col2.z += m.col2.z;
		this.col3.x += m.col3.x;
		this.col3.y += m.col3.y;
		this.col3.z += m.col3.z;
	}

	public SetIdentity(): void {
		this.col1.x = 1.0; this.col2.x = 0.0; this.col3.x = 0.0;
		this.col1.y = 0.0; this.col2.y = 1.0; this.col3.y = 0.0;
		this.col1.z = 0.0; this.col2.z = 0.0; this.col3.z = 1.0;
	}

	public SetZero(): void {
		this.col1.x = 0.0; this.col2.x = 0.0; this.col3.x = 0.0;
		this.col1.y = 0.0; this.col2.y = 0.0; this.col3.y = 0.0;
		this.col1.z = 0.0; this.col2.z = 0.0; this.col3.z = 0.0;
	}

	// Solve A * x = b
	public Solve22(out: b2Vec2, bX: number, bY: number): b2Vec2 {
		//float32 a11 = this.col1.x, a12 = this.col2.x, a21 = this.col1.y, a22 = this.col2.y;
		const a11: number = this.col1.x;
		const a12: number = this.col2.x;
		const a21: number = this.col1.y;
		const a22: number = this.col2.y;
		//float32 det = a11 * a22 - a12 * a21;
		let det: number = a11 * a22 - a12 * a21;
		if (det != 0.0) {
			det = 1.0 / det;
		}
		out.x = det * (a22 * bX - a12 * bY);
		out.y = det * (a11 * bY - a21 * bX);

		return out;
	}

	// Solve A * x = b
	public Solve33(out: b2Vec3, bX: number, bY: number, bZ: number): b2Vec3 {
		const a11: number = this.col1.x;
		const a21: number = this.col1.y;
		const a31: number = this.col1.z;
		const a12: number = this.col2.x;
		const a22: number = this.col2.y;
		const a32: number = this.col2.z;
		const a13: number = this.col3.x;
		const a23: number = this.col3.y;
		const a33: number = this.col3.z;
		//float32 det = b2Dot(col1, b2Cross(col2, col3));
		let det: number = 	a11 * (a22 * a33 - a32 * a23) +
                            a21 * (a32 * a13 - a12 * a33) +
                            a31 * (a12 * a23 - a22 * a13);
		if (det != 0.0) {
			det = 1.0 / det;
		}
		//out.x = det * b2Dot(b, b2Cross(col2, col3));
		out.x = det * (bX * (a22 * a33 - a32 * a23) +
                        bY * (a32 * a13 - a12 * a33) +
                        bZ * (a12 * a23 - a22 * a13));
		//out.y = det * b2Dot(col1, b2Cross(b, col3));
		out.y = det * (a11 * (bY * a33 - bZ * a23) +
                        a21 * (bZ * a13 - bX * a33) +
                        a31 * (bX * a23 - bY * a13));
		//out.z = det * b2Dot(col1, b2Cross(col2, b));
		out.z = det * (a11 * (a22 * bZ - a32 * bY) +
                        a21 * (a32 * bX - a12 * bZ) +
                        a31 * (a12 * bY - a22 * bX));
		return out;
	}

	public col1: b2Vec3 = new b2Vec3();
	public col2: b2Vec3 = new b2Vec3();
	public col3: b2Vec3 = new b2Vec3();
}