import { b2Vec2, b2Transform } from '../../Common/Math';
import { b2AABB } from '../../Collision/b2AABB';
import { b2RayCastOutput } from '../b2RayCastOutput';
import { b2RayCastInput } from '../b2RayCastInput';
import { b2DistanceInput } from '../b2DistanceInput';
import { b2DistanceProxy } from '../b2DistanceProxy';
import { b2SimplexCache } from '../b2SimplexCache';
import { b2DistanceOutput } from '../b2DistanceOutput';
import { b2Distance } from '../b2Distance';
import { b2Settings } from '../../Common/b2Settings';
import { b2MassData } from './b2MassData';

/**
* A shape is used for collision detection. Shapes are created in b2Body.
* You can use shape for collision detection before they are attached to the world.
* @warning you cannot reuse shapes.
*/
export class b2Shape {
	__fast__: boolean = true;

	/**
	 * Clone the shape
	 */
	public Copy(): b2Shape {
		//var s:b2Shape = new b2Shape();
		//s.Set(this);
		//return s;
		return null; // Abstract type
	}

	/**
	 * Assign the properties of anther shape to this
	 */
	public Set(other: b2Shape): void {
		//Don't copy m_type?
		//m_type = other.m_type;
		this.m_radius = other.m_radius;
	}

	/**
	* Get the type of this shape. You can use this to down cast to the concrete shape.
	* @return the shape type.
	*/
	public GetType(): number /** int */
	{
		return this.m_type;
	}

	/**
	* Test a point for containment in this shape. This only works for convex shapes.
	* @param xf the shape world transform.
	* @param p a point in world coordinates.
	*/
	public TestPoint(xf: b2Transform, p: b2Vec2): boolean {return false;}

	/**
	 * Cast a ray against this shape.
	 * @param output the ray-cast results.
	 * @param input the ray-cast input parameters.
	 * @param transform the transform to be applied to the shape.
	 */
	public RayCast(output: b2RayCastOutput, input: b2RayCastInput, transform: b2Transform): boolean {
		return false;
	}

	/**
	* Given a transform, compute the associated axis aligned bounding box for this shape.
	* @param aabb returns the axis aligned box.
	* @param xf the world transform of the shape.
	*/
	public ComputeAABB(aabb: b2AABB, xf: b2Transform): void {}

	/**
	* Compute the mass properties of this shape using its dimensions and density.
	* The inertia tensor is computed about the local origin, not the centroid.
	* @param massData returns the mass data for this shape.
	*/
	public ComputeMass(massData: b2MassData, density: number): void { }

	/**
	 * Compute the volume and centroid of this shape intersected with a half plane
	 * @param normal the surface normal
	 * @param offset the surface offset along normal
	 * @param xf the shape transform
	 * @param c returns the centroid
	 * @return the total volume less than offset along normal
	 */
	public ComputeSubmergedArea(
		normal: b2Vec2,
		offset: number,
		xf: b2Transform,
		c: b2Vec2): number { return 0; }

	public static TestOverlap(shape1: b2Shape, transform1: b2Transform, shape2: b2Shape, transform2: b2Transform): boolean {
		const input: b2DistanceInput = new b2DistanceInput();
		input.proxyA = new b2DistanceProxy();
		input.proxyA.Set(shape1);
		input.proxyB = new b2DistanceProxy();
		input.proxyB.Set(shape2);
		input.transformA = transform1;
		input.transformB = transform2;
		input.useRadii = true;
		const simplexCache: b2SimplexCache = new b2SimplexCache();
		simplexCache.count = 0;
		const output: b2DistanceOutput = new b2DistanceOutput();
		b2Distance.Distance(output, simplexCache, input);
		return output.distance  < 10.0 * Number.MIN_VALUE;
	}

	//--------------- Internals Below -------------------
	/**
	 * @private
	 */
	constructor() {
		this.m_type = b2Shape.e_unknownShape;
		this.m_radius = b2Settings.b2_linearSlop;
	}

	//virtual ~b2Shape();

	public m_type: number /** int */;
	public m_radius: number;

	/**
	* The various collision shape types supported by Box2D.
	*/
	//enum b2ShapeType
	//{
	public static readonly e_unknownShape: number /** int */ = 	-1;
	public static readonly e_circleShape: number /** int */ = 	0;
	public static readonly e_polygonShape: number /** int */ = 	1;
	public static readonly e_edgeShape: number /** int */ =       2;
	public static readonly e_shapeTypeCount: number /** int */ = 	3;
	//};

	/**
	 * Possible return values for TestSegment
	 */
	/** Return value for TestSegment indicating a hit. */
	public static readonly e_hitCollide: number /** int */ = 1;
	/** Return value for TestSegment indicating a miss. */
	public static readonly e_missCollide: number /** int */ = 0;
	/** Return value for TestSegment indicating that the segment starting point, p1, is already inside the shape. */
	public static readonly e_startsInsideCollide: number /** int */ = -1;
}