import { b2Shape } from './Shapes/b2Shape';
import { b2Vec2 } from '../Common/Math';
import { b2CircleShape } from './Shapes/b2CircleShape';
import { b2PolygonShape } from './Shapes/b2PolygonShape';
import { b2Settings } from '../Common/b2Settings';

/**
 * A distance proxy is used by the GJK algorithm.
 * It encapsulates any shape.
 */
export class b2DistanceProxy {
	__fast__: boolean = true;

	/**
	 * Initialize the proxy using the given shape. The shape
	 * must remain in scope while the proxy is in use.
	 */
	public Set(shape: b2Shape): void {
		switch (shape.GetType()) {
			case b2Shape.e_circleShape:
				{
					const circle: b2CircleShape = <b2CircleShape> shape;
					this.m_vertices = new Array<b2Vec2>(1);
					this.m_vertices[0] = circle.m_p;
					this.m_count = 1;
					this.m_radius = circle.m_radius;
				}
				break;
			case b2Shape.e_polygonShape:
				{
					const polygon: b2PolygonShape =  <b2PolygonShape> shape;
					this.m_vertices = polygon.m_vertices;
					this.m_count = polygon.m_vertexCount;
					this.m_radius = polygon.m_radius;
				}
				break;
			default:
				b2Settings.b2Assert(false);
		}
	}

	/**
	 * Get the supporting vertex index in the given direction.
	 */
	public GetSupport(d: b2Vec2): number {
		let bestIndex: number /** int */ = 0;
		let bestValue: number = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
		for (let i: number /** int */= 1; i < this.m_count; ++i) {
			const value: number = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	}

	/**
	 * Get the supporting vertex in the given direction.
	 */
	public GetSupportVertex(d: b2Vec2): b2Vec2 {
		let bestIndex: number /** int */ = 0;
		let bestValue: number = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
		for (let i: number /** int */= 1; i < this.m_count; ++i) {
			const value: number = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return this.m_vertices[bestIndex];
	}

	/**
	 * Get the vertex count.
	 */
	public GetVertexCount(): number /** int */
	{
		return this.m_count;
	}

	/**
	 * Get a vertex by index. Used by b2Distance.
	 */
	public GetVertex(index: number /** int */): b2Vec2 {
		b2Settings.b2Assert(0 <= index && index < this.m_count);
		return this.m_vertices[index];
	}

	public m_vertices: b2Vec2[];
	public m_count: number /** int */;
	public m_radius: number;
}
