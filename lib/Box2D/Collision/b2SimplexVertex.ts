import { b2Vec2 } from '../Common/Math';

export class b2SimplexVertex {
	__fast__: boolean = true;

	public Set(other: b2SimplexVertex): void {
		this.wA.SetV(other.wA);
		this.wB.SetV(other.wB);
		this.w.SetV(other.w);
		this.a = other.a;
		this.indexA = other.indexA;
		this.indexB = other.indexB;
	}

	public wA: b2Vec2;		// support point in proxyA
	public wB: b2Vec2;		// support point in proxyB
	public w: b2Vec2;		// wB - wA
	public a: number;		// barycentric coordinate for closest point
	public indexA: number /** int */;	// wA index
	public indexB: number /** int */;	// wB index
}