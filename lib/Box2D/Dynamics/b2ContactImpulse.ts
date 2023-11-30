import { b2Settings } from '../Common/b2Settings';

/**
 * Contact impulses for reporting. Impulses are used instead of forces because
 * sub-step forces may approach infinity for rigid body collisions. These
 * match up one-to-one with the contact points in b2Manifold.
 */
export class b2ContactImpulse {
	__fast__: boolean = true;

	public normalImpulses: Array<number> = new Array<number>(b2Settings.b2_maxManifoldPoints);
	public tangentImpulses: Array<number> = new Array<number>(b2Settings.b2_maxManifoldPoints);
}