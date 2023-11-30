import { b2DistanceProxy } from './b2DistanceProxy';
import { b2Transform } from '../Common/Math';

/**
 * Input for b2Distance.
 * You have to option to use the shape radii
 * in the computation. Even
 */
export class b2DistanceInput {
	__fast__: boolean = true;

	public proxyA: b2DistanceProxy;
	public proxyB: b2DistanceProxy;
	public transformA: b2Transform;
	public transformB: b2Transform;
	public useRadii: boolean;
}
