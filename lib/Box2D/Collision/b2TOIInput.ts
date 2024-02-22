import { b2DistanceProxy } from './b2DistanceProxy';
import { b2Sweep } from '../Common/Math';

/**
 * Inpute parameters for b2TimeOfImpact
 */
export class b2TOIInput {
	public proxyA: b2DistanceProxy = new b2DistanceProxy();
	public proxyB: b2DistanceProxy = new b2DistanceProxy();
	public sweepA: b2Sweep = new b2Sweep();
	public sweepB: b2Sweep = new b2Sweep();
	public tolerance: number;
}