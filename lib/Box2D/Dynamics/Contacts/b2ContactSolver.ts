import { b2Shape } from '../../Collision/Shapes/b2Shape';
import { b2Mat22, b2Vec2, b2Math } from '../../Common/Math';
import { b2Fixture } from '../b2Fixture';
import { b2Body } from '../b2Body';
import { b2Settings } from '../../Common/b2Settings';
import { b2Manifold } from '../../Collision/b2Manifold';
import { b2WorldManifold } from '../../Collision/b2WorldManifold';
import { b2TimeStep } from '../b2TimeStep';
import { b2ManifoldPoint } from '../../Collision/b2ManifoldPoint';
import { b2Contact, b2ContactConstraintPoint, b2ContactConstraint, b2PositionSolverManifold } from '../Contacts';

/**
* @private
*/
export class b2ContactSolver {
	public constructor() {
	}

	private static s_worldManifold: b2WorldManifold = new b2WorldManifold();
	public Initialize(step: b2TimeStep, contacts: Array<b2Contact>, contactCount: number /** int */, allocator: any): void {
		let contact: b2Contact;

		this.m_step.Set(step);

		this.m_allocator = allocator;

		let i: number /** int */;
		let tVec: b2Vec2;
		let tMat: b2Mat22;

		this.m_constraintCount = contactCount;

		// fill vector to hold enough constraints
		while (this.m_constraints.length < this.m_constraintCount) {
			this.m_constraints[this.m_constraints.length] = new b2ContactConstraint();
		}

		for (i = 0; i < contactCount; ++i) {
			contact = contacts[i];
			const fixtureA: b2Fixture = contact.m_fixtureA;
			const fixtureB: b2Fixture  = contact.m_fixtureB;
			const shapeA: b2Shape = fixtureA.m_shape;
			const shapeB: b2Shape = fixtureB.m_shape;
			const radiusA: number = shapeA.m_radius;
			const radiusB: number = shapeB.m_radius;
			const bodyA: b2Body = fixtureA.m_body;
			const bodyB: b2Body = fixtureB.m_body;
			const manifold: b2Manifold = contact.GetManifold();

			const friction: number = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
			const restitution: number = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());

			//var vA:b2Vec2 = bodyA.m_linearVelocity.Copy();
			const vAX: number = bodyA.m_linearVelocity.x;
			const vAY: number = bodyA.m_linearVelocity.y;
			//var vB:b2Vec2 = bodyB.m_linearVelocity.Copy();
			const vBX: number = bodyB.m_linearVelocity.x;
			const vBY: number = bodyB.m_linearVelocity.y;
			const wA: number = bodyA.m_angularVelocity;
			const wB: number = bodyB.m_angularVelocity;

			b2Settings.b2Assert(manifold.m_pointCount > 0);

			b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);

			const normalX: number = b2ContactSolver.s_worldManifold.m_normal.x;
			const normalY: number = b2ContactSolver.s_worldManifold.m_normal.y;

			const cc: b2ContactConstraint = this.m_constraints[ i ];
			cc.bodyA = bodyA; //p
			cc.bodyB = bodyB; //p
			cc.manifold = manifold; //p
			//c.normal = normal;
			cc.normal.x = normalX;
			cc.normal.y = normalY;
			cc.pointCount = manifold.m_pointCount;
			cc.friction = friction;
			cc.restitution = restitution;

			cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
			cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
			cc.localPoint.x = manifold.m_localPoint.x;
			cc.localPoint.y = manifold.m_localPoint.y;
			cc.radius = radiusA + radiusB;
			cc.type = manifold.m_type;

			for (let k: number /** uint */ = 0; k < cc.pointCount; ++k) {
				const cp: b2ManifoldPoint = manifold.m_points[ k ];
				const ccp: b2ContactConstraintPoint = cc.points[ k ];

				ccp.normalImpulse = cp.m_normalImpulse;
				ccp.tangentImpulse = cp.m_tangentImpulse;

				ccp.localPoint.SetV(cp.m_localPoint);

				const rAX: number = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
				const rAY: number = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
				const rBX: number = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
				const rBY: number = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;

				let rnA: number = rAX * normalY - rAY * normalX;//b2Math.b2Cross(r1, normal);
				let rnB: number = rBX * normalY - rBY * normalX;//b2Math.b2Cross(r2, normal);

				rnA *= rnA;
				rnB *= rnB;

				const kNormal: number = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
				//b2Settings.b2Assert(kNormal > Number.MIN_VALUE);
				ccp.normalMass = 1.0 / kNormal;

				let kEqualized: number = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
				kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
				//b2Assert(kEqualized > Number.MIN_VALUE);
				ccp.equalizedMass = 1.0 / kEqualized;

				//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
				const tangentX: number = normalY;
				const tangentY: number = -normalX;

				//var rtA:number = b2Math.b2Cross(rA, tangent);
				let rtA: number = rAX * tangentY - rAY * tangentX;
				//var rtB:number = b2Math.b2Cross(rB, tangent);
				let rtB: number = rBX * tangentY - rBY * tangentX;

				rtA *= rtA;
				rtB *= rtB;

				const kTangent: number = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
				//b2Settings.b2Assert(kTangent > Number.MIN_VALUE);
				ccp.tangentMass = 1.0 /  kTangent;

				// Setup a velocity bias for restitution.
				ccp.velocityBias = 0.0;
				//b2Dot(c.normal, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
				const tX: number = vBX + (-wB * rBY) - vAX - (-wA * rAY);
				const tY: number = vBY + (wB * rBX) - vAY - (wA * rAX);
				//var vRel:number = b2Dot(cc.normal, t);
				const vRel: number = cc.normal.x * tX + cc.normal.y * tY;
				if (vRel < -b2Settings.b2_velocityThreshold) {
					ccp.velocityBias += -cc.restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (cc.pointCount == 2) {
				const ccp1: b2ContactConstraintPoint = cc.points[0];
				const ccp2: b2ContactConstraintPoint = cc.points[1];

				const invMassA: number = bodyA.m_invMass;
				const invIA: number = bodyA.m_invI;
				const invMassB: number = bodyB.m_invMass;
				const invIB: number = bodyB.m_invI;

				//var rn1A:number = b2Cross(ccp1.rA, normal);
				//var rn1B:number = b2Cross(ccp1.rB, normal);
				//var rn2A:number = b2Cross(ccp2.rA, normal);
				//var rn2B:number = b2Cross(ccp2.rB, normal);
				const rn1A: number = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
				const rn1B: number = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
				const rn2A: number = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
				const rn2B: number = ccp2.rB.x * normalY - ccp2.rB.y * normalX;

				const k11: number = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				const k22: number = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				const k12: number = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				const k_maxConditionNumber: number = 100.0;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
					// K is safe to invert.
					cc.K.col1.Set(k11, k12);
					cc.K.col2.Set(k12, k22);
					cc.K.GetInverse(cc.normalMass);
				} else {
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					cc.pointCount = 1;
				}
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

			const bodyA: b2Body = c.bodyA;
			const bodyB: b2Body = c.bodyB;
			const invMassA: number = bodyA.m_invMass;
			const invIA: number = bodyA.m_invI;
			const invMassB: number = bodyB.m_invMass;
			const invIB: number = bodyB.m_invI;
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

					//bodyA.m_angularVelocity -= invIA * b2Math.b2CrossVV(rA, P);
					bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
					//bodyA.m_linearVelocity.Subtract( b2Math.MulFV(invMassA, P) );
					bodyA.m_linearVelocity.x -= invMassA * PX;
					bodyA.m_linearVelocity.y -= invMassA * PY;
					//bodyB.m_angularVelocity += invIB * b2Math.b2CrossVV(rB, P);
					bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
					//bodyB.m_linearVelocity.Add( b2Math.MulFV(invMassB, P) );
					bodyB.m_linearVelocity.x += invMassB * PX;
					bodyB.m_linearVelocity.y += invMassB * PY;
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
		let rAX: number;
		let rAY: number;
		let rBX: number;
		let rBY: number;
		let dvX: number;
		let dvY: number;
		let vn: number;
		let vt: number;
		let lambda: number;
		let maxFriction: number;
		let newImpulse: number;
		let PX: number;
		let PY: number;
		let dX: number;
		let dY: number;
		let P1X: number;
		let P1Y: number;
		let P2X: number;
		let P2Y: number;

		let tMat: b2Mat22;
		let tVec: b2Vec2;

		for (let i: number /** int */ = 0; i < this.m_constraintCount; ++i) {
			const c: b2ContactConstraint = this.m_constraints[ i ];
			const bodyA: b2Body = c.bodyA;
			const bodyB: b2Body = c.bodyB;
			let wA: number = bodyA.m_angularVelocity;
			let wB: number = bodyB.m_angularVelocity;
			const vA: b2Vec2 = bodyA.m_linearVelocity;
			const vB: b2Vec2 = bodyB.m_linearVelocity;

			const invMassA: number = bodyA.m_invMass;
			const invIA: number = bodyA.m_invI;
			const invMassB: number = bodyB.m_invMass;
			const invIB: number = bodyB.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			const normalX: number = c.normal.x;
			const normalY: number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			const tangentX: number = normalY;
			const tangentY: number = -normalX;
			const friction: number = c.friction;

			var tX: number;

			//b2Settings.b2Assert(c.pointCount == 1 || c.pointCount == 2);
			// Solve the tangent constraints
			for (j = 0; j < c.pointCount; j++) {
				ccp = c.points[j];

				// Relative velocity at contact
				//b2Vec2 dv = vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA);
				dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
				dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;

				// Compute tangent force
				vt = dvX * tangentX + dvY * tangentY;
				lambda = ccp.tangentMass * -vt;

				// b2Clamp the accumulated force
				maxFriction = friction * ccp.normalImpulse;
				newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - ccp.tangentImpulse;

				// Apply contact impulse
				PX = lambda * tangentX;
				PY = lambda * tangentY;

				vA.x -= invMassA * PX;
				vA.y -= invMassA * PY;
				wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);

				vB.x += invMassB * PX;
				vB.y += invMassB * PY;
				wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);

				ccp.tangentImpulse = newImpulse;
			}

			// Solve the normal constraints
			const tCount: number /** int */ = c.pointCount;
			if (c.pointCount == 1) {
				ccp = c.points[ 0 ];

				// Relative velocity at contact
				//b2Vec2 dv = vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA);
				dvX = vB.x + (-wB * ccp.rB.y) - vA.x - (-wA * ccp.rA.y);
				dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);

				// Compute normal impulse
				//var vn:number = b2Math.b2Dot(dv, normal);
				vn = dvX * normalX + dvY * normalY;
				lambda = -ccp.normalMass * (vn - ccp.velocityBias);

				// b2Clamp the accumulated impulse
				//newImpulse = b2Math.b2Max(ccp.normalImpulse + lambda, 0.0);
				newImpulse = ccp.normalImpulse + lambda;
				newImpulse = newImpulse > 0 ? newImpulse : 0.0;
				lambda = newImpulse - ccp.normalImpulse;

				// Apply contact impulse
				//b2Vec2 P = lambda * normal;
				PX = lambda * normalX;
				PY = lambda * normalY;

				//vA.Subtract( b2Math.MulFV( invMassA, P ) );
				vA.x -= invMassA * PX;
				vA.y -= invMassA * PY;
				wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);//invIA * b2Math.b2CrossVV(ccp.rA, P);

				//vB.Add( b2Math.MulFV( invMass2, P ) );
				vB.x += invMassB * PX;
				vB.y += invMassB * PY;
				wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);//invIB * b2Math.b2CrossVV(ccp.rB, P);

				ccp.normalImpulse = newImpulse;
			} else {
				// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
				// Build the mini LCP for this contact patch
				//
				// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
				//
				// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
				// b = vn_0 - velocityBias
				//
				// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
				// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
				// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
				// solution that satisfies the problem is chosen.
				//
				// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
				// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
				//
				// Substitute:
				//
				// x = x' - a
				//
				// Plug into above equation:
				//
				// vn = A * x + b
				//    = A * (x' - a) + b
				//    = A * x' + b - A * a
				//    = A * x' + b'
				// b' = b - A * a;

				const cp1: b2ContactConstraintPoint = c.points[ 0 ];
				const cp2: b2ContactConstraintPoint = c.points[ 1 ];

				const aX: number = cp1.normalImpulse;
				const aY: number = cp2.normalImpulse;
				//b2Settings.b2Assert( aX >= 0.0f && aY >= 0.0f );

				// Relative velocity at contact
				//var dv1:b2Vec2 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
				const dv1X: number = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
				const dv1Y: number = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
				//var dv2:b2Vec2 = vB + b2Cross(wB, cpB.r2) - vA - b2Cross(wA, cp2.rA);
				const dv2X: number = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
				const dv2Y: number = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;

				// Compute normal velocity
				//var vn1:number = b2Dot(dv1, normal);
				let vn1: number = dv1X * normalX + dv1Y * normalY;
				//var vn2:number = b2Dot(dv2, normal);
				let vn2: number = dv2X * normalX + dv2Y * normalY;

				let bX: number = vn1 - cp1.velocityBias;
				let bY: number = vn2 - cp2.velocityBias;

				//b -= b2Mul(c.K,a);
				tMat = c.K;
				bX -= tMat.col1.x * aX + tMat.col2.x * aY;
				bY -= tMat.col1.y * aX + tMat.col2.y * aY;

				const k_errorTol: number  = 0.001;
				for (;;) {
					//
					// Case 1: vn = 0
					//
					// 0 = A * x' + b'
					//
					// Solve for x':
					//
					// x' = -inv(A) * b'
					//

					//var x:b2Vec2 = - b2Mul(c->normalMass, b);
					tMat = c.normalMass;
					let xX: number = -(tMat.col1.x * bX + tMat.col2.x * bY);
					let xY: number = -(tMat.col1.y * bX + tMat.col2.y * bY);

					if (xX >= 0.0 && xY >= 0.0) {
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;

						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;

						//vA -= invMass1 * (P1 + P2)
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						//wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

						//vB += invMassB * (P1 + P2)
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						//wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
						wB   += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;

						//#if B2_DEBUG_SOLVER == 1
						//					// Post conditions
						//					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
						//					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
						//					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
						//					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
						//					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
						//					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
						//					// Compute normal velocity
						//					//vn1 = b2Dot(dv1, normal);
						//					vn1 = dv1X * normalX + dv1Y * normalY;
						//					//vn2 = b2Dot(dv2, normal);
						//					vn2 = dv2X * normalX + dv2Y * normalY;
						//
						//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
						//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
						//#endif
						break;
					}

					//
					// Case 2: vn1 = 0  and x2 = 0
					//
					//   0 = a11 * x1' + a12 * 0 + b1'
					// vn2 = a21 * x1' + a22 * 0 + b2'
					//

					xX = -cp1.normalMass * bX;
					xY = 0.0;
					vn1 = 0.0;
					vn2 = c.K.col1.y * xX + bY;

					if (xX >= 0.0 && vn2 >= 0.0) {
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;

						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;

						//vA -= invMassA * (P1 + P2)
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						//wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

						//vB += invMassB * (P1 + P2)
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						//wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
						wB   += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;

						//#if B2_DEBUG_SOLVER == 1
						//					// Post conditions
						//					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
						//					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
						//					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
						//					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
						//					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
						//					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
						//					// Compute normal velocity
						//					//vn1 = b2Dot(dv1, normal);
						//					vn1 = dv1X * normalX + dv1Y * normalY;
						//					//vn2 = b2Dot(dv2, normal);
						//					vn2 = dv2X * normalX + dv2Y * normalY;
						//
						//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
						//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
						//#endif
						break;
					}

					//
					// Case 3: wB = 0 and x1 = 0
					//
					// vn1 = a11 * 0 + a12 * x2' + b1'
					//   0 = a21 * 0 + a22 * x2' + b2'
					//

					xX = 0.0;
					xY = -cp2.normalMass * bY;
					vn1 = c.K.col2.x * xY + bX;
					vn2 = 0.0;
					if (xY >= 0.0 && vn1 >= 0.0) {
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;

						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;

						//vA -= invMassA * (P1 + P2)
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						//wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

						//vB += invMassB * (P1 + P2)
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						//wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
						wB   += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;

						//#if B2_DEBUG_SOLVER == 1
						//					// Post conditions
						//					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
						//					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
						//					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
						//					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
						//					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
						//					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
						//					// Compute normal velocity
						//					//vn1 = b2Dot(dv1, normal);
						//					vn1 = dv1X * normalX + dv1Y * normalY;
						//					//vn2 = b2Dot(dv2, normal);
						//					vn2 = dv2X * normalX + dv2Y * normalY;
						//
						//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
						//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
						//#endif
						break;
					}

					//
					// Case 4: x1 = 0 and x2 = 0
					//
					// vn1 = b1
					// vn2 = b2

					xX = 0.0;
					xY = 0.0;
					vn1 = bX;
					vn2 = bY;

					if (vn1 >= 0.0 && vn2 >= 0.0) {
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;

						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;

						//vA -= invMassA * (P1 + P2)
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						//wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

						//vB += invMassB * (P1 + P2)
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						//wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
						wB   += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;

						//#if B2_DEBUG_SOLVER == 1
						//					// Post conditions
						//					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
						//					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
						//					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
						//					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
						//					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
						//					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
						//					// Compute normal velocity
						//					//vn1 = b2Dot(dv1, normal);
						//					vn1 = dv1X * normalX + dv1Y * normalY;
						//					//vn2 = b2Dot(dv2, normal);
						//					vn2 = dv2X * normalX + dv2Y * normalY;
						//
						//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
						//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
						//#endif
						break;
					}

					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
			}

			// b2Vec2s in AS3 are copied by reference. The originals are
			// references to the same things here and there is no need to
			// copy them back, unlike in C++ land where b2Vec2s are
			// copied by value.
			/*bodyA->m_linearVelocity = vA;
			bodyB->m_linearVelocity = vB;*/
			bodyA.m_angularVelocity = wA;
			bodyB.m_angularVelocity = wB;
		}
	}

	public FinalizeVelocityConstraints(): void {
		for (let i: number /** int */ = 0; i < this.m_constraintCount; ++i) {
			const c: b2ContactConstraint = this.m_constraints[ i ];
			const m: b2Manifold = c.manifold;

			for (let j: number /** int */ = 0; j < c.pointCount; ++j) {
				const point1: b2ManifoldPoint = m.m_points[j];
				const point2: b2ContactConstraintPoint = c.points[j];
				point1.m_normalImpulse = point2.normalImpulse;
				point1.m_tangentImpulse = point2.tangentImpulse;
			}
		}
	}

	//#if 1
	// Sequential solver
	//	public SolvePositionConstraints(baumgarte:number):boolean{
	//		var minSeparation:number = 0.0;
	//
	//		var tMat:b2Mat22;
	//		var tVec:b2Vec2;
	//
	//		for (var i:number /** int */ = 0; i < m_constraintCount; ++i)
	//		{
	//			var c:b2ContactConstraint = m_constraints[ i ];
	//			var bodyA:b2Body = c.bodyA;
	//			var bodyB:b2Body = c.bodyB;
	//			var bA_sweep_c:b2Vec2 = bodyA.m_sweep.c;
	//			var bA_sweep_a:number = bodyA.m_sweep.a;
	//			var bB_sweep_c:b2Vec2 = bodyB.m_sweep.c;
	//			var bB_sweep_a:number = bodyB.m_sweep.a;
	//
	//			var invMassa:number = bodyA.m_mass * bodyA.m_invMass;
	//			var invIa:number = bodyA.m_mass * bodyA.m_invI;
	//			var invMassb:number = bodyB.m_mass * bodyB.m_invMass;
	//			var invIb:number = bodyB.m_mass * bodyB.m_invI;
	//			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
	//			var normalX:number = c.normal.x;
	//			var normalY:number = c.normal.y;
	//
	//			// Solver normal constraints
	//			var tCount:number /** int */ = c.pointCount;
	//			for (var j:number /** int */ = 0; j < tCount; ++j)
	//			{
	//				var ccp:b2ContactConstraintPoint = c.points[ j ];
	//
	//				//r1 = b2Mul(bodyA->m_xf.R, ccp->localAnchor1 - bodyA->GetLocalCenter());
	//				tMat = bodyA.m_xf.R;
	//				tVec = bodyA.m_sweep.localCenter;
	//				var r1X:number = ccp.localAnchor1.x - tVec.x;
	//				var r1Y:number = ccp.localAnchor1.y - tVec.y;
	//				tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
	//				r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
	//				r1X = tX;
	//
	//				//r2 = b2Mul(bodyB->m_xf.R, ccp->localAnchor2 - bodyB->GetLocalCenter());
	//				tMat = bodyB.m_xf.R;
	//				tVec = bodyB.m_sweep.localCenter;
	//				var r2X:number = ccp.localAnchor2.x - tVec.x;
	//				var r2Y:number = ccp.localAnchor2.y - tVec.y;
	//				var tX:number =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
	//				r2Y = 			 (tMat.col1.y * r2X + tMat.col2.y * r2Y);
	//				r2X = tX;
	//
	//				//b2Vec2 p1 = bodyA->m_sweep.c + r1;
	//				var p1X:number = b1_sweep_c.x + r1X;
	//				var p1Y:number = b1_sweep_c.y + r1Y;
	//
	//				//b2Vec2 p2 = bodyB->m_sweep.c + r2;
	//				var p2X:number = b2_sweep_c.x + r2X;
	//				var p2Y:number = b2_sweep_c.y + r2Y;
	//
	//				//var dp:b2Vec2 = b2Math.SubtractVV(p2, p1);
	//				var dpX:number = p2X - p1X;
	//				var dpY:number = p2Y - p1Y;
	//
	//				// Approximate the current separation.
	//				//var separation:number = b2Math.b2Dot(dp, normal) + ccp.separation;
	//				var separation:number = (dpX*normalX + dpY*normalY) + ccp.separation;
	//
	//				// Track max constraint error.
	//				minSeparation = b2Math.b2Min(minSeparation, separation);
	//
	//				// Prevent large corrections and allow slop.
	//				var C:number =  b2Math.b2Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);
	//
	//				// Compute normal impulse
	//				var dImpulse:number = -ccp.equalizedMass * C;
	//
	//				//var P:b2Vec2 = b2Math.MulFV( dImpulse, normal );
	//				var PX:number = dImpulse * normalX;
	//				var PY:number = dImpulse * normalY;
	//
	//				//bodyA.m_position.Subtract( b2Math.MulFV( invMass1, impulse ) );
	//				b1_sweep_c.x -= invMass1 * PX;
	//				b1_sweep_c.y -= invMass1 * PY;
	//				b1_sweep_a -= invI1 * (r1X * PY - r1Y * PX);//b2Math.b2CrossVV(r1, P);
	//				bodyA.m_sweep.a = b1_sweep_a;
	//				bodyA.SynchronizeTransform();
	//
	//				//bodyB.m_position.Add( b2Math.MulFV( invMass2, P ) );
	//				b2_sweep_c.x += invMass2 * PX;
	//				b2_sweep_c.y += invMass2 * PY;
	//				b2_sweep_a += invI2 * (r2X * PY - r2Y * PX);//b2Math.b2CrossVV(r2, P);
	//				bodyB.m_sweep.a = b2_sweep_a;
	//				bodyB.SynchronizeTransform();
	//			}
	//			// Update body rotations
	//			//bodyA.m_sweep.a = b1_sweep_a;
	//			//bodyB.m_sweep.a = b2_sweep_a;
	//		}
	//
	//		// We can't expect minSpeparation >= -b2_linearSlop because we don't
	//		// push the separation above -b2_linearSlop.
	//		return minSeparation >= -1.5 * b2Settings.b2_linearSlop;
	//	}
	//#else
	// Sequential solver.
	private static s_psm: b2PositionSolverManifold = new b2PositionSolverManifold();
	public SolvePositionConstraints(baumgarte: number): boolean {
		let minSeparation: number = 0.0;

		for (let i: number /** int */ = 0; i < this.m_constraintCount; i++) {
			const c: b2ContactConstraint = this.m_constraints[i];
			const bodyA: b2Body = c.bodyA;
			const bodyB: b2Body = c.bodyB;

			const invMassA: number = bodyA.m_mass * bodyA.m_invMass;
			const invIA: number = bodyA.m_mass * bodyA.m_invI;
			const invMassB: number = bodyB.m_mass * bodyB.m_invMass;
			const invIB: number = bodyB.m_mass * bodyB.m_invI;

			b2ContactSolver.s_psm.Initialize(c);
			const normal: b2Vec2 = b2ContactSolver.s_psm.m_normal;

			// Solve normal constraints
			for (let j: number /** int */ = 0; j < c.pointCount; j++) {
				const ccp: b2ContactConstraintPoint = c.points[j];

				const point: b2Vec2 = b2ContactSolver.s_psm.m_points[j];
				const separation: number = b2ContactSolver.s_psm.m_separations[j];

				const rAX: number = point.x - bodyA.m_sweep.c.x;
				const rAY: number = point.y - bodyA.m_sweep.c.y;
				const rBX: number = point.x - bodyB.m_sweep.c.x;
				const rBY: number = point.y - bodyB.m_sweep.c.y;

				// Track max constraint error.
				minSeparation = minSeparation < separation ? minSeparation : separation;

				// Prevent large corrections and allow slop.
				const C: number = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);

				// Compute normal impulse
				const impulse: number = -ccp.equalizedMass * C;

				const PX: number = impulse * normal.x;
				const PY: number = impulse * normal.y;

				//bodyA.m_sweep.c -= invMassA * P;
				bodyA.m_sweep.c.x -= invMassA * PX;
				bodyA.m_sweep.c.y -= invMassA * PY;
				//bodyA.m_sweep.a -= invIA * b2Cross(rA, P);
				bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
				bodyA.SynchronizeTransform();

				//bodyB.m_sweep.c += invMassB * P;
				bodyB.m_sweep.c.x += invMassB * PX;
				bodyB.m_sweep.c.y += invMassB * PY;
				//bodyB.m_sweep.a += invIB * b2Cross(rB, P);
				bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
				bodyB.SynchronizeTransform();
			}
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation > -1.5 * b2Settings.b2_linearSlop;
	}

	//#endif
	private m_step: b2TimeStep = new b2TimeStep();
	private m_allocator: any;
	public m_constraints: Array<b2ContactConstraint> = new Array<b2ContactConstraint> ();
	private m_constraintCount: number /** int */;
}