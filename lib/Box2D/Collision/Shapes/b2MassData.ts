import { b2Vec2 } from '../../Common/Math';

/**
* This holds the mass data computed for a shape.
*/
export class b2MassData {
	/**
	* The mass of the shape, usually in kilograms.
	*/
	public mass: number = 0.0;
	/**
	* The position of the shape's centroid relative to the shape's origin.
	*/
	public center: b2Vec2 = new b2Vec2(0,0);
	/**
	* The rotational inertia of the shape.
	* This may be about the center or local origin, depending on usage.
	*/
	public I: number = 0.0;
}