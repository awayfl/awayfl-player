import { b2Vec2 } from '../../Common/Math';

/**
* @private
*/
export class b2Jacobian {
	public linearA: b2Vec2 = new b2Vec2();
	public angularA: number;
	public linearB: b2Vec2 = new b2Vec2();
	public angularB: number;

	public SetZero(): void {
		this.linearA.SetZero(); this.angularA = 0.0;
		this.linearB.SetZero(); this.angularB = 0.0;
	}

	public Set(x1: b2Vec2, a1: number, x2: b2Vec2, a2: number): void {
		this.linearA.SetV(x1); this.angularA = a1;
		this.linearB.SetV(x2); this.angularB = a2;
	}

	public Compute(x1: b2Vec2, a1: number, x2: b2Vec2, a2: number): number {

		//return b2Math.b2Dot(this.linearA, x1) + this.angularA * a1 + b2Math.b2Dot(this.linearV, x2) + this.angularV * a2;
		return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
	}
}