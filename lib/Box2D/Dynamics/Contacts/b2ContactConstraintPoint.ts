import { b2Vec2 } from '../../Common/Math';

/**
* @private
*/
export class b2ContactConstraintPoint {
	public localPoint: b2Vec2=new b2Vec2();
	public rA: b2Vec2=new b2Vec2();
	public rB: b2Vec2=new b2Vec2();
	public normalImpulse: number;
	public tangentImpulse: number;
	public normalMass: number;
	public tangentMass: number;
	public equalizedMass: number;
	public velocityBias: number;
}