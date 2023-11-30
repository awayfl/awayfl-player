import { IBroadPhase } from './IBroadPhase';
import { b2AABB } from './b2AABB';
import { b2Vec2 } from '../Common/Math';
import { b2DynamicTreeNode } from './b2DynamicTreeNode';
import { b2DynamicTreePair } from './b2DynamicTreePair';
import { b2DynamicTree } from './b2DynamicTree';
import { b2RayCastInput } from './b2RayCastInput';

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 */
export class b2DynamicTreeBroadPhase implements IBroadPhase {
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * UpdatePairs is called.
	 */
	public CreateProxy(aabb: b2AABB, userData: any): any {
		const proxy: b2DynamicTreeNode = this.m_tree.CreateProxy(aabb, userData);
		++this.m_proxyCount;
		this.BufferMove(proxy);
		return proxy;
	}

	/**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 */
	public DestroyProxy(proxy: any): void {
		this.UnBufferMove(proxy);
		--this.m_proxyCount;
		this.m_tree.DestroyProxy(proxy);
	}

	/**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 */
	public MoveProxy(proxy: any, aabb: b2AABB, displacement: b2Vec2): void {
		const buffer: boolean = this.m_tree.MoveProxy(proxy, aabb, displacement);
		if (buffer) {
			this.BufferMove(proxy);
		}
	}

	public TestOverlap(proxyA: any, proxyB: any): boolean {
		const aabbA: b2AABB = this.m_tree.GetFatAABB(proxyA);
		const aabbB: b2AABB = this.m_tree.GetFatAABB(proxyB);
		return aabbA.TestOverlap(aabbB);
	}

	/**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
	public GetUserData(proxy: any): any {
		return this.m_tree.GetUserData(proxy);
	}

	/**
	 * Get the AABB for a proxy.
	 */
	public GetFatAABB(proxy: any): b2AABB {
		return this.m_tree.GetFatAABB(proxy);
	}

	/**
	 * Get the number of proxies.
	 */
	public GetProxyCount(): number /**int */
	{
		return this.m_proxyCount;
	}

	/**
	 * Update the pairs. This results in pair callbacks. This can only add pairs.
	 */
	public UpdatePairs(callback: Function): void {
		this.m_pairCount = 0;
		// Perform tree queries for all moving queries
		for (var queryProxy of this.m_moveBuffer) {
			const QueryCallback: Function = (proxy: b2DynamicTreeNode) => {
				// A proxy cannot form a pair with itself.
				if (proxy == queryProxy)
					return true;

				// Grow the pair buffer as needed
				if (this.m_pairCount == this.m_pairBuffer.length) {
					this.m_pairBuffer[this.m_pairCount] = new b2DynamicTreePair();
				}

				const pair: b2DynamicTreePair = this.m_pairBuffer[this.m_pairCount];
				pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
				pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;
				++this.m_pairCount;

				return true;
			};
			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			const fatAABB: b2AABB = this.m_tree.GetFatAABB(queryProxy);
			this.m_tree.Query(QueryCallback, fatAABB);
		}

		// Reset move buffer
		this.m_moveBuffer.length = 0;

		// Sort the pair buffer to expose duplicates.
		// TODO: Something more sensible
		//m_pairBuffer.sort(ComparePairs);

		// Send the pair buffer
		for (let i: number /**int */ = 0; i < this.m_pairCount;) {
			const primaryPair: b2DynamicTreePair = this.m_pairBuffer[i];
			const userDataA: any = this.m_tree.GetUserData(primaryPair.proxyA);
			const userDataB: any = this.m_tree.GetUserData(primaryPair.proxyB);
			callback(userDataA, userDataB);
			++i;

			// Skip any duplicate pairs
			while (i < this.m_pairCount) {
				const pair: b2DynamicTreePair = this.m_pairBuffer[i];
				if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
					break;
				}
				++i;
			}
		}
	}

	/**
	 * @inheritDoc
	 */
	public Query(callback: Function, aabb: b2AABB): void {
		this.m_tree.Query(callback, aabb);
	}

	/**
	 * @inheritDoc
	 */
	public RayCast(callback: Function, input: b2RayCastInput): void {
		this.m_tree.RayCast(callback, input);
	}

	public Validate(): void {
		//TODO_BORIS
	}

	public Rebalance(iterations: number /**int */): void {
		this.m_tree.Rebalance(iterations);
	}

	// Private ///////////////

	private BufferMove(proxy: b2DynamicTreeNode): void {
		this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
	}

	private UnBufferMove(proxy: b2DynamicTreeNode): void {
		const i: number /**int */ = this.m_moveBuffer.indexOf(proxy);
		this.m_moveBuffer.splice(i, 1);
	}

	private ComparePairs(pair1: b2DynamicTreePair, pair2: b2DynamicTreePair): number /**int */
	{
		//TODO_BORIS:
		// We cannot consistently sort objects easily in AS3
		// The caller of this needs replacing with a different method.
		return 0;
	}

	private m_tree: b2DynamicTree = new b2DynamicTree();
	private m_proxyCount: number /**int */;
	private m_moveBuffer: Array<b2DynamicTreeNode> = new Array<b2DynamicTreeNode>();

	private m_pairBuffer: Array<b2DynamicTreePair> = new Array<b2DynamicTreePair>();
	private m_pairCount: number /**int */ = 0;
}