import { b2AABB } from './b2AABB';
import { b2Vec2 } from '../Common/Math';
import { b2RayCastInput } from './b2RayCastInput';

/**
 * Interface for objects tracking overlap of many AABBs.
 */
export interface IBroadPhase
{
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * UpdatePairs is called.
	 */
	CreateProxy(aabb: b2AABB, userData: any): any;

	/**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 */
	DestroyProxy(proxy: any): void;

	/**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 */
	MoveProxy(proxy: any, aabb: b2AABB, displacement: b2Vec2): void;

	TestOverlap(proxyA: any, proxyB: any): boolean;

	/**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
	GetUserData(proxy: any): any;

	/**
	 * Get the fat AABB for a proxy.
	 */
	GetFatAABB(proxy: any): b2AABB;

	/**
	 * Get the number of proxies.
	 */
	GetProxyCount(): number /** int */;

	/**
	 * Update the pairs. This results in pair callbacks. This can only add pairs.
	 */
	UpdatePairs(callback: Function): void;

	/**
	 * Query an AABB for overlapping proxies. The callback class
	 * is called with each proxy that overlaps
	 * the supplied AABB, and return a Boolean indicating if
	 * the broaphase should proceed to the next match.
	 * @param callback This should be a matching signature
	 * <code>Callback(proxy:any):boolean</code>
	 */
	Query(callback: Function, aabb: b2AABB): void;

	/**
	 * Ray-cast  agains the proxies in the tree. This relies on the callback
	 * to perform exact ray-cast in the case where the proxy contains a shape
	 * The callback also performs any collision filtering
	 * @param callback This should be a matching signature
	 * <code>Callback(subInput:b2RayCastInput, proxy:any):number</code>
	 * Where the returned number is the new value for maxFraction
	 */
	RayCast(callback: Function, input: b2RayCastInput): void;

	/**
	 * For debugging, throws in invariants have been broken
	 */
	Validate(): void;

	/**
	 * Give the broadphase a chance for structural optimizations
	 */
	Rebalance(iterations: number /** int */): void;
}