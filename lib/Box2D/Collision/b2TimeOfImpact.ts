import { b2SimplexCache } from './b2SimplexCache';
import { b2DistanceInput } from './b2DistanceInput';
import { b2Transform, b2Math, b2Sweep } from '../Common/Math';
import { b2DistanceOutput } from './b2DistanceOutput';
import { b2Settings } from '../Common/b2Settings';
import { b2Distance } from './b2Distance';
import { b2DistanceProxy } from './b2DistanceProxy';
import { b2TOIInput } from './b2TOIInput';
import { b2SeparationFunction } from './b2SeparationFunction';

/**
* @private
*/
export class b2TimeOfImpact {

	private static b2_toiCalls: number /** int */ = 0;
	private static b2_toiIters: number /** int */ = 0;
	private static b2_toiMaxIters: number /** int */ = 0;
	private static b2_toiRootIters: number /** int */ = 0;
	private static b2_toiMaxRootIters: number /** int */ = 0;

	private static s_cache: b2SimplexCache = new b2SimplexCache();
	private static s_distanceInput: b2DistanceInput = new b2DistanceInput();
	private static s_xfA: b2Transform = new b2Transform();
	private static s_xfB: b2Transform = new b2Transform();
	private static s_fcn: b2SeparationFunction = new b2SeparationFunction();
	private static s_distanceOutput: b2DistanceOutput = new b2DistanceOutput();
	public static TimeOfImpact(input: b2TOIInput): number {
		++this.b2_toiCalls;

		const proxyA: b2DistanceProxy = input.proxyA;
		const proxyB: b2DistanceProxy = input.proxyB;

		const sweepA: b2Sweep = input.sweepA;
		const sweepB: b2Sweep = input.sweepB;

		b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
		b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);

		const radius: number = proxyA.m_radius + proxyB.m_radius;
		const tolerance: number = input.tolerance;

		let alpha: number = 0.0;

		const k_maxIterations: number /** int */ = 1000; //TODO_ERIN b2Settings
		let iter: number /** int */ = 0;
		let target: number = 0.0;

		// Prepare input for distance query.
		this.s_cache.count = 0;
		this.s_distanceInput.useRadii = false;

		for (;;) {
			sweepA.GetTransform(this.s_xfA, alpha);
			sweepB.GetTransform(this.s_xfB, alpha);

			// Get the distance between shapes
			this.s_distanceInput.proxyA = proxyA;
			this.s_distanceInput.proxyB = proxyB;
			this.s_distanceInput.transformA = this.s_xfA;
			this.s_distanceInput.transformB = this.s_xfB;

			b2Distance.Distance(this.s_distanceOutput, this.s_cache, this.s_distanceInput);

			if (this.s_distanceOutput.distance <= 0.0) {
				alpha = 1.0;
				break;
			}

			this.s_fcn.Initialize(this.s_cache, proxyA, this.s_xfA, proxyB, this.s_xfB);

			const separation: number = this.s_fcn.Evaluate(this.s_xfA, this.s_xfB);
			if (separation <= 0.0) {
				alpha = 1.0;
				break;
			}

			if (iter == 0) {
				// Compute a reasonable target distance to give some breathing room
				// for conservative advancement. We take advantage of the shape radii
				// to create additional clearance
				if (separation > radius) {
					target = b2Math.Max(radius - tolerance, 0.75 * radius);
				} else {
					target = b2Math.Max(separation - tolerance, 0.02 * radius);
				}
			}

			if (separation - target < 0.5 * tolerance) {
				if (iter == 0) {
					alpha = 1.0;
					break;
				}
				break;
			}

			//#if 0
			// Dump the curve seen by the root finder
			//{
			//const N:number /** int */ = 100;
			//var dx:number = 1.0 / N;
			//var xs:Vector.<Number> = new Array(N + 1);
			//var fs:Vector.<Number> = new Array(N + 1);
			//
			//var x:number = 0.0;
			//for (var i:number /** int */ = 0; i <= N; i++)
			//{
			//sweepA.GetTransform(xfA, x);
			//sweepB.GetTransform(xfB, x);
			//var f:number = fcn.Evaluate(xfA, xfB) - target;
			//
			//trace(x, f);
			//xs[i] = x;
			//fx[i] = f'
			//
			//x += dx;
			//}
			//}
			//#endif
			// Compute 1D root of f(x) - target = 0
			let newAlpha: number = alpha;
			{
				let x1: number = alpha;
				let x2: number = 1.0;

				let f1: number = separation;

				sweepA.GetTransform(this.s_xfA, x2);
				sweepB.GetTransform(this.s_xfB, x2);

				let f2: number = this.s_fcn.Evaluate(this.s_xfA, this.s_xfB);

				// If intervals don't overlap at t2, then we are done
				if (f2 >= target) {
					alpha = 1.0;
					break;
				}

				// Determine when intervals intersect
				let rootIterCount: number /** int */ = 0;
				for (;;) {
					// Use a mis of the secand rule and bisection
					var x: number;
					if (rootIterCount & 1) {
						// Secant rule to improve convergence
						x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
					} else {
						// Bisection to guarantee progress
						x = 0.5 * (x1 + x2);
					}

					sweepA.GetTransform(this.s_xfA, x);
					sweepB.GetTransform(this.s_xfB, x);

					const f: number = this.s_fcn.Evaluate(this.s_xfA, this.s_xfB);

					if (b2Math.Abs(f - target) < 0.025 * tolerance) {
						newAlpha = x;
						break;
					}

					// Ensure we continue to bracket the root
					if (f > target) {
						x1 = x;
						f1 = f;
					} else {
						x2 = x;
						f2 = f;
					}

					++rootIterCount;
					++this.b2_toiRootIters;
					if (rootIterCount == 50) {
						break;
					}
				}

				this.b2_toiMaxRootIters = b2Math.Max(this.b2_toiMaxRootIters, rootIterCount);
			}

			// Ensure significant advancement
			if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
				break;
			}

			alpha = newAlpha;

			iter++;
			++this.b2_toiIters;

			if (iter == k_maxIterations) {
				break;
			}
		}

		this.b2_toiMaxIters = b2Math.Max(this.b2_toiMaxIters, iter);

		return alpha;
	}

}