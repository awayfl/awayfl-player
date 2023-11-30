/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import { b2TimeStep } from '../b2TimeStep';
import { b2Contact } from './b2Contact';
import { b2Mat22, b2Vec2, b2Math } from '../../Common/Math';
import { b2Manifold } from '../../Collision/b2Manifold';
import { b2ManifoldPoint } from '../../Collision/b2ManifoldPoint';
import { b2Settings } from '../../Common/b2Settings';
import { b2ContactConstraintPoint } from './b2ContactConstraintPoint';
import { b2ContactConstraint } from './b2ContactConstraint';
import { b2Body } from '../b2Body';

export class b2ContactSolver {
	constructor(step: b2TimeStep, contacts: b2Contact[], contactCount: number /** int */, allocator: any) {
		let contact: b2Contact;

		//m_step = step;
		this.m_step.dt = step.dt;
		this.m_step.inv_dt = step.inv_dt;
		this.m_step.maxIterations = step.maxIterations;

		this.m_allocator = allocator;

		let i: number /** int */;
		let tVec: b2Vec2;
		let tMat: b2Mat22;

		this.m_constraintCount = 0;
		for (i = 0; i < contactCount; ++i) {
			// b2Assert(contacts[i].IsSolid());
			contact = contacts[i];
			this.m_constraintCount += contact.m_manifoldCount;
		}

		// fill array
		for (i = 0; i < this.m_constraintCount; i++) {
			this.m_constraints[i] = new b2ContactConstraint();
		}

		let count: number /** int */ = 0;
		for (i = 0; i < contactCount; ++i) {
			contact = contacts[i];
			const b1: b2Body = contact.m_shape1.m_body;
			const b2: b2Body = contact.m_shape2.m_body;
			const manifoldCount: number /** int */ = contact.m_manifoldCount;
			const manifolds: b2Manifold[] = contact.GetManifolds();
			const friction: number = contact.m_friction;
			const restitution: number = contact.m_restitution;

			//var v1:b2Vec2 = b1.m_linearVelocity.Copy();
			const v1X: number = b1.m_linearVelocity.x;
			const v1Y: number = b1.m_linearVelocity.y;
			//var v2:b2Vec2 = b2.m_linearVelocity.Copy();
			const v2X: number = b2.m_linearVelocity.x;
			const v2Y: number = b2.m_linearVelocity.y;
			const w1: number = b1.m_angularVelocity;
			const w2: number = b2.m_angularVelocity;

			for (let j: number /** int */ = 0; j < manifoldCount; ++j) {
				const manifold: b2Manifold = manifolds[ j ];

				//b2Settings.b2Assert(manifold.pointCount > 0);

				//var normal:b2Vec2 = manifold.normal.Copy();
				const normalX: number = manifold.normal.x;
				const normalY: number = manifold.normal.y;

				//b2Settings.b2Assert(count < m_constraintCount);
				const c: b2ContactConstraint = this.m_constraints[ count ];
				c.body1 = b1; //p
				c.body2 = b2; //p
				c.manifold = manifold; //p
				//c.normal = normal;
				c.normal.x = normalX;
				c.normal.y = normalY;
				c.pointCount = manifold.pointCount;
				c.friction = friction;
				c.restitution = restitution;

				for (let k: number /** uint */ = 0; k < c.pointCount; ++k) {
					const cp: b2ManifoldPoint = manifold.points[ k ];
					const ccp: b2ContactConstraintPoint = c.points[ k ];

					ccp.normalImpulse = cp.normalImpulse;
					ccp.tangentImpulse = cp.tangentImpulse;
					ccp.separation = cp.separation;
					ccp.positionImpulse = 0.0;

					ccp.localAnchor1.SetV(cp.localPoint1);
					ccp.localAnchor2.SetV(cp.localPoint2);

					var tX: number;
					var tY: number;

					//ccp->r1 = b2Mul(b1->GetXForm().R, cp->localPoint1 - b1->GetLocalCenter());
					tMat = b1.m_xf.R;
					let r1X: number = cp.localPoint1.x - b1.m_sweep.localCenter.x;
					let r1Y: number = cp.localPoint1.y - b1.m_sweep.localCenter.y;
					tX  = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
					r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
					r1X = tX;
					ccp.r1.Set(r1X,r1Y);
					//ccp->r2 = b2Mul(b2->GetXForm().R, cp->localPoint2 - b2->GetLocalCenter());
					tMat = b2.m_xf.R;
					let r2X: number = cp.localPoint2.x - b2.m_sweep.localCenter.x;
					let r2Y: number = cp.localPoint2.y - b2.m_sweep.localCenter.y;
					tX  = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
					r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
					r2X = tX;
					ccp.r2.Set(r2X,r2Y);

					const r1Sqr: number = r1X * r1X + r1Y * r1Y;//b2Math.b2Dot(r1, r1);
					const r2Sqr: number = r2X * r2X + r2Y * r2Y;//b2Math.b2Dot(r2, r2);

					//var rn1:number = b2Math.b2Dot(r1, normal);
					const rn1: number = r1X * normalX + r1Y * normalY;
					//var rn2:number = b2Math.b2Dot(r2, normal);
					const rn2: number = r2X * normalX + r2Y * normalY;
					let kNormal: number = b1.m_invMass + b2.m_invMass;
					kNormal += b1.m_invI * (r1Sqr - rn1 * rn1) + b2.m_invI * (r2Sqr - rn2 * rn2);
					//b2Settings.b2Assert(kNormal > Number.MIN_VALUE);
					ccp.normalMass = 1.0 / kNormal;

					let kEqualized: number = b1.m_mass * b1.m_invMass + b2.m_mass * b2.m_invMass;
					kEqualized += b1.m_mass * b1.m_invI * (r1Sqr - rn1 * rn1) + b2.m_mass * b2.m_invI * (r2Sqr - rn2 * rn2);
					//b2Assert(kEqualized > Number.MIN_VALUE);
					ccp.equalizedMass = 1.0 / kEqualized;

					//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
					const tangentX: number = normalY;
					const tangentY: number = -normalX;

					//var rt1:number = b2Math.b2Dot(r1, tangent);
					const rt1: number = r1X * tangentX + r1Y * tangentY;
					//var rt2:number = b2Math.b2Dot(r2, tangent);
					const rt2: number = r2X * tangentX + r2Y * tangentY;
					let kTangent: number = b1.m_invMass + b2.m_invMass;
					kTangent += b1.m_invI * (r1Sqr - rt1 * rt1) + b2.m_invI * (r2Sqr - rt2 * rt2);
					//b2Settings.b2Assert(kTangent > Number.MIN_VALUE);
					ccp.tangentMass = 1.0 /  kTangent;

					// Setup a velocity bias for restitution.
					ccp.velocityBias = 0.0;
					if (ccp.separation > 0.0) {
						ccp.velocityBias = -60.0 * ccp.separation; // TODO_ERIN b2TimeStep
					}
					//b2Dot(c.normal, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
					tX = v2X + (-w2 * r2Y) - v1X - (-w1 * r1Y);
					tY = v2Y + (w2 * r2X) - v1Y - (w1 * r1X);
					//var vRel:number = b2Dot(c.normal, t);
					const vRel: number = c.normal.x * tX + c.normal.y * tY;
					if (vRel < -b2Settings.b2_velocityThreshold) {
						ccp.velocityBias += -c.restitution * vRel;
					}
				}

				++count;
			}
		}

		//b2Settings.b2Assert(count == m_constraintCount);
	}
	//~b2ContactSolver();

	public InitVelocityConstraints(step: b2TimeStep): void {
		let tVec: b2Vec2;
		let tVec2: b2Vec2;
		let tMat: b2Mat22;

		// Warm start.
		for (let i: number /** int */ = 0; i < this.m_constraintCount; ++i) {
			const c: b2ContactConstraint = this.m_constraints[ i ];

			const b1: b2Body = c.body1;
			const b2: b2Body = c.body2;
			const invMass1: number = b1.m_invMass;
			const invI1: number = b1.m_invI;
			const invMass2: number = b2.m_invMass;
			const invI2: number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			const normalX: number = c.normal.x;
			const normalY: number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			const tangentX: number = normalY;
			const tangentY: number = -normalX;

			var tX: number;

			var j: number /** int */;
			var tCount: number /** int */;
			if (step.warmStarting) {
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j) {
					const ccp: b2ContactConstraintPoint = c.points[ j ];
					ccp.normalImpulse *= step.dtRatio;
					ccp.tangentImpulse *= step.dtRatio;
					//b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
					const PX: number = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
					const PY: number = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;

					//b1.m_angularVelocity -= invI1 * b2Math.b2CrossVV(r1, P);
					b1.m_angularVelocity -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);
					//b1.m_linearVelocity.Subtract( b2Math.MulFV(invMass1, P) );
					b1.m_linearVelocity.x -= invMass1 * PX;
					b1.m_linearVelocity.y -= invMass1 * PY;
					//b2.m_angularVelocity += invI2 * b2Math.b2CrossVV(r2, P);
					b2.m_angularVelocity += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);
					//b2.m_linearVelocity.Add( b2Math.MulFV(invMass2, P) );
					b2.m_linearVelocity.x += invMass2 * PX;
					b2.m_linearVelocity.y += invMass2 * PY;
				}
			} else {
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j) {
					const ccp2: b2ContactConstraintPoint = c.points[ j ];
					ccp2.normalImpulse = 0.0;
					ccp2.tangentImpulse = 0.0;
				}
			}
		}
	}

	public SolveVelocityConstraints(): void {
		let j: number /** int */;
		let ccp: b2ContactConstraintPoint;
		let r1X: number;
		let r1Y: number;
		let r2X: number;
		let r2Y: number;
		let dvX: number;
		let dvY: number;
		let vn: number;
		let vt: number;
		let lambda_n: number;
		let lambda_t: number;
		let newImpulse_n: number;
		let newImpulse_t: number;
		let PX: number;
		let PY: number;

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		for (let i: number /** int */ = 0; i < this.m_constraintCount; ++i) {
			const c: b2ContactConstraint = this.m_constraints[ i ];
			const b1: b2Body = c.body1;
			const b2: b2Body = c.body2;
			let w1: number = b1.m_angularVelocity;
			let w2: number = b2.m_angularVelocity;
			const v1: b2Vec2 = b1.m_linearVelocity;
			const v2: b2Vec2 = b2.m_linearVelocity;

			const invMass1: number = b1.m_invMass;
			const invI1: number = b1.m_invI;
			const invMass2: number = b2.m_invMass;
			const invI2: number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			const normalX: number = c.normal.x;
			const normalY: number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			const tangentX: number = normalY;
			const tangentY: number = -normalX;
			const friction: number = c.friction;

			var tX: number;

			const tCount: number /** int */ = c.pointCount;
			for (j = 0; j < tCount; ++j) {
				ccp = c.points[ j ];

				// Relative velocity at contact
				//b2Vec2 dv = v2 + b2Cross(w2, ccp->r2) - v1 - b2Cross(w1, ccp->r1);
				dvX = v2.x + (-w2 * ccp.r2.y) - v1.x - (-w1 * ccp.r1.y);
				dvY = v2.y + (w2 * ccp.r2.x) - v1.y - (w1 * ccp.r1.x);

				// Compute normal impulse
				//var vn:number = b2Math.b2Dot(dv, normal);
				vn = dvX * normalX + dvY * normalY;
				lambda_n = -ccp.normalMass * (vn - ccp.velocityBias);

				// Compute tangent impulse - normal
				vt = dvX * tangentX + dvY * tangentY;//b2Math.b2Dot(dv, tangent);
				lambda_t = ccp.tangentMass * (-vt);

				// b2Clamp the accumulated impulse - tangent
				newImpulse_n = b2Math.b2Max(ccp.normalImpulse + lambda_n, 0.0);
				lambda_n = newImpulse_n - ccp.normalImpulse;

				// b2Clamp the accumulated force
				const maxFriction: number = friction * ccp.normalImpulse;
				newImpulse_t = b2Math.b2Clamp(ccp.tangentImpulse + lambda_t, -maxFriction, maxFriction);
				lambda_t = newImpulse_t - ccp.tangentImpulse;

				// Apply contact impulse
				//b2Vec2 P = lambda * normal;
				PX = lambda_n * normalX + lambda_t * tangentX;
				PY = lambda_n * normalY + lambda_t * tangentY;

				//v1.Subtract( b2Math.MulFV( invMass1, P ) );
				v1.x -= invMass1 * PX;
				v1.y -= invMass1 * PY;
				w1 -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);//invI1 * b2Math.b2CrossVV(ccp.r1, P);

				//v2.Add( b2Math.MulFV( invMass2, P ) );
				v2.x += invMass2 * PX;
				v2.y += invMass2 * PY;
				w2 += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);//invI2 * b2Math.b2CrossVV(ccp.r2, P);

				ccp.normalImpulse = newImpulse_n;
				ccp.tangentImpulse = newImpulse_t;
			}

			// b2Vec2s in AS3 are copied by reference. The originals are
			// references to the same things here and there is no need to
			// copy them back, unlike in C++ land where b2Vec2s are
			// copied by value.
			/*b1->m_linearVelocity = v1;
			b2->m_linearVelocity = v2;*/
			b1.m_angularVelocity = w1;
			b2.m_angularVelocity = w2;
		}
	}

	public FinalizeVelocityConstraints(): void {
		for (let i: number /** int */ = 0; i < this.m_constraintCount; ++i) {
			const c: b2ContactConstraint = this.m_constraints[ i ];
			const m: b2Manifold = c.manifold;

			for (let j: number /** int */ = 0; j < c.pointCount; ++j) {
				const point1: b2ManifoldPoint = m.points[j];
				const point2: b2ContactConstraintPoint = c.points[j];
				point1.normalImpulse = point2.normalImpulse;
				point1.tangentImpulse = point2.tangentImpulse;
			}
		}
	}

	public SolvePositionConstraints(baumgarte: number): boolean {
		let minSeparation: number = 0.0;

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		for (let i: number /** int */ = 0; i < this.m_constraintCount; ++i) {
			const c: b2ContactConstraint = this.m_constraints[ i ];
			const b1: b2Body = c.body1;
			const b2: b2Body = c.body2;
			const b1_sweep_c: b2Vec2 = b1.m_sweep.c;
			let b1_sweep_a: number = b1.m_sweep.a;
			const b2_sweep_c: b2Vec2 = b2.m_sweep.c;
			let b2_sweep_a: number = b2.m_sweep.a;

			const invMass1: number = b1.m_mass * b1.m_invMass;
			const invI1: number = b1.m_mass * b1.m_invI;
			const invMass2: number = b2.m_mass * b2.m_invMass;
			const invI2: number = b2.m_mass * b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			const normalX: number = c.normal.x;
			const normalY: number = c.normal.y;

			// Solver normal constraints
			const tCount: number /** int */ = c.pointCount;
			for (let j: number /** int */ = 0; j < tCount; ++j) {
				const ccp: b2ContactConstraintPoint = c.points[ j ];

				//r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
				tMat = b1.m_xf.R;
				tVec = b1.m_sweep.localCenter;
				let r1X: number = ccp.localAnchor1.x - tVec.x;
				let r1Y: number = ccp.localAnchor1.y - tVec.y;
				tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
				r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
				r1X = tX;

				//r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());
				tMat = b2.m_xf.R;
				tVec = b2.m_sweep.localCenter;
				let r2X: number = ccp.localAnchor2.x - tVec.x;
				let r2Y: number = ccp.localAnchor2.y - tVec.y;
				var tX: number =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
				r2Y = 			 (tMat.col1.y * r2X + tMat.col2.y * r2Y);
				r2X = tX;

				//b2Vec2 p1 = b1->m_sweep.c + r1;
				const p1X: number = b1_sweep_c.x + r1X;
				const p1Y: number = b1_sweep_c.y + r1Y;

				//b2Vec2 p2 = b2->m_sweep.c + r2;
				const p2X: number = b2_sweep_c.x + r2X;
				const p2Y: number = b2_sweep_c.y + r2Y;

				//var dp:b2Vec2 = b2Math.SubtractVV(p2, p1);
				const dpX: number = p2X - p1X;
				const dpY: number = p2Y - p1Y;

				// Approximate the current separation.
				//var separation:number = b2Math.b2Dot(dp, normal) + ccp.separation;
				const separation: number = (dpX * normalX + dpY * normalY) + ccp.separation;

				// Track max constraint error.
				minSeparation = b2Math.b2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				const C: number = baumgarte * b2Math.b2Clamp(separation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);

				// Compute normal impulse
				let dImpulse: number = -ccp.equalizedMass * C;

				// b2Clamp the accumulated impulse
				const impulse0: number = ccp.positionImpulse;
				ccp.positionImpulse = b2Math.b2Max(impulse0 + dImpulse, 0.0);
				dImpulse = ccp.positionImpulse - impulse0;

				//var impulse:b2Vec2 = b2Math.MulFV( dImpulse, normal );
				const impulseX: number = dImpulse * normalX;
				const impulseY: number = dImpulse * normalY;

				//b1.m_position.Subtract( b2Math.MulFV( invMass1, impulse ) );
				b1_sweep_c.x -= invMass1 * impulseX;
				b1_sweep_c.y -= invMass1 * impulseY;
				b1_sweep_a -= invI1 * (r1X * impulseY - r1Y * impulseX);//b2Math.b2CrossVV(r1, impulse);
				b1.m_sweep.a = b1_sweep_a;
				b1.SynchronizeTransform();

				//b2.m_position.Add( b2Math.MulFV( invMass2, impulse ) );
				b2_sweep_c.x += invMass2 * impulseX;
				b2_sweep_c.y += invMass2 * impulseY;
				b2_sweep_a += invI2 * (r2X * impulseY - r2Y * impulseX);//b2Math.b2CrossVV(r2, impulse);
				b2.m_sweep.a = b2_sweep_a;
				b2.SynchronizeTransform();
			}
			// Update body rotations
			//b1.m_sweep.a = b1_sweep_a;
			//b2.m_sweep.a = b2_sweep_a;
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -1.5 * b2Settings.b2_linearSlop;
	}

	public m_step: b2TimeStep = new b2TimeStep();
	public m_allocator: any;
	public m_constraints: b2ContactConstraint[] = [];
	public m_constraintCount: number /** int */;
}