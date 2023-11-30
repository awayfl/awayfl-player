/**
* @private
*/
export class b2TimeStep {
	__fast__: boolean = true;

	public Set(step: b2TimeStep): void {
		this.dt = step.dt;
		this.inv_dt = step.inv_dt;
		this.positionIterations = step.positionIterations;
		this.velocityIterations = step.velocityIterations;
		this.warmStarting = step.warmStarting;
	}

	public dt: number;			// time step
	public inv_dt: number;		// inverse time step (0 if dt == 0).
	public dtRatio: number;		// dt * inv_dt0
	public velocityIterations: number /** int */;
	public positionIterations: number /** int */;
	public warmStarting: boolean;
}