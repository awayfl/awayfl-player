/**
 * Used to warm start b2Distance.
 * Set count to zero on first call.
 */
export class b2SimplexCache {
	__fast__: boolean = true;

	/** Length or area */
	public metric: number;
	public count: number /** uint */;
	/** Vertices on shape a */
	public indexA: Array<number /** int */> = new Array<number>(3);
	/** Vertices on shape b */
	public indexB: Array<number /** int */> = new Array<number>(3);
}