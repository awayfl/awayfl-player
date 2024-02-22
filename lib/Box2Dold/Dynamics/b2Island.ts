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

import { b2ContactListener } from './b2ContactListener';
import { b2TimeStep } from './b2TimeStep';
import { b2Vec2 } from '../Common/Math';
import { b2Joint } from './Joints';
import { b2Body } from './b2Body';
import { b2Math } from '../Common/Math';
import { b2Settings } from '../Common/b2Settings';
import { b2ContactSolver } from './Contacts/b2ContactSolver';
import { b2ContactResult } from './Contacts/b2ContactResult';
import { b2Mat22 } from '../Common/Math';
import { b2Contact } from './Contacts/b2Contact';
import { b2ContactConstraint } from './Contacts/b2ContactConstraint';
import { b2Manifold } from '../Collision/b2Manifold';
import { b2ManifoldPoint } from '../Collision/b2ManifoldPoint';
import { b2ContactConstraintPoint } from './Contacts/b2ContactConstraintPoint';

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

export class b2Island {
	constructor(
		bodyCapacity: number /** int */,
		contactCapacity: number /** int */,
		jointCapacity: number /** int */,
		allocator: any,
		listener: b2ContactListener) {
		let i: number /** int */;

		this.m_bodyCapacity = bodyCapacity;
		this.m_contactCapacity = contactCapacity;
		this.m_jointCapacity	 = jointCapacity;
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;

		this.m_allocator = allocator;
		this.m_listener = listener;

		//this.m_bodies = (b2Body**)allocator->Allocate(bodyCapacity * sizeof(b2Body*));
		this.m_bodies = new Array(bodyCapacity);
		for (i = 0; i < bodyCapacity; i++)
			this.m_bodies[i] = null;

		//this.m_contacts = (b2Contact**)allocator->Allocate(contactCapacity	 * sizeof(b2Contact*));
		this.m_contacts = new Array(contactCapacity);
		for (i = 0; i < contactCapacity; i++)
			this.m_contacts[i] = null;

		//this.m_joints = (b2Joint**)allocator->Allocate(jointCapacity * sizeof(b2Joint*));
		this.m_joints = new Array(jointCapacity);
		for (i = 0; i < jointCapacity; i++)
			this.m_joints[i] = null;

		this.m_positionIterationCount = 0;

	}
	//~b2Island();

	public Clear(): void {
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
	}

	public Solve(step: b2TimeStep, gravity: b2Vec2, correctPositions: boolean, allowSleep: boolean): void {
		let i: number /** int */;
		let b: b2Body;
		let joint: b2Joint;

		// Integrate velocities and apply damping.
		for (i = 0; i < this.m_bodyCount; ++i) {
			b = this.m_bodies[i];

			if (b.IsStatic())
				continue;

			// Integrate velocities.
			//b.m_linearVelocity += step.dt * (gravity + b.m_invMass * b.m_force);
			b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
			b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;

			// Reset forces.
			b.m_force.SetZero();
			b.m_torque = 0.0;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Taylor expansion:
			// v2 = (1.0f - c * dt) * v1
			b.m_linearVelocity.Multiply(b2Math.b2Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0));
			b.m_angularVelocity *= b2Math.b2Clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0);

			// Check for large velocities.
			//if (b2Dot(b->this.m_linearVelocity, b->this.m_linearVelocity) > b2_maxLinearVelocitySquared)
			if ((b.m_linearVelocity.LengthSquared()) > b2Settings.b2_maxLinearVelocitySquared) {
				b.m_linearVelocity.Normalize();
				b.m_linearVelocity.x *= b2Settings.b2_maxLinearVelocity;
				b.m_linearVelocity.y *= b2Settings.b2_maxLinearVelocity;
			}

			if (b.m_angularVelocity * b.m_angularVelocity > b2Settings.b2_maxAngularVelocitySquared) {
				if (b.m_angularVelocity < 0.0) {
					b.m_angularVelocity = -b2Settings.b2_maxAngularVelocity;
				} else {
					b.m_angularVelocity = b2Settings.b2_maxAngularVelocity;
				}
			}
		}

		const contactSolver: b2ContactSolver = new b2ContactSolver(step, this.m_contacts, this.m_contactCount, this.m_allocator);

		// Initialize velocity constraints.
		contactSolver.InitVelocityConstraints(step);

		for (i = 0; i < this.m_jointCount; ++i) {
			joint = this.m_joints[i];
			joint.InitVelocityConstraints(step);
		}

		// Solve velocity constraints.
		for (i = 0; i < step.maxIterations; ++i) {
			contactSolver.SolveVelocityConstraints();

			for (let j: number /** int */ = 0; j < this.m_jointCount; ++j) {
				joint = this.m_joints[j];
				joint.SolveVelocityConstraints(step);
			}
		}

		// Post-solve (store impulses for warm starting).
		contactSolver.FinalizeVelocityConstraints();

		// Integrate positions.
		for (i = 0; i < this.m_bodyCount; ++i) {
			b = this.m_bodies[i];

			if (b.IsStatic())
				continue;

			// Store positions for continuous collision.
			b.m_sweep.c0.SetV(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;

			// Integrate
			//b.m_sweep.c += step.dt * b.m_linearVelocity;
			b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
			b.m_sweep.a += step.dt * b.m_angularVelocity;

			// Compute new transform
			b.SynchronizeTransform();

			// Note: shapes are synchronized later.
		}

		if (correctPositions) {
			// Initialize position constraints.
			// Contacts don't need initialization.
			for (i = 0; i < this.m_jointCount; ++i) {
				joint = this.m_joints[i];
				joint.InitPositionConstraints();
			}

			// Iterate over constraints.
			for (this.m_positionIterationCount = 0; this.m_positionIterationCount < step.maxIterations; ++this.m_positionIterationCount) {
				const contactsOkay: boolean = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);

				let jointsOkay: boolean = true;
				for (i = 0; i < this.m_jointCount; ++i) {
					joint = this.m_joints[i];
					const jointOkay: boolean = joint.SolvePositionConstraints();
					jointsOkay = jointsOkay && jointOkay;
				}

				if (contactsOkay && jointsOkay) {
					break;
				}
			}
		}

		this.Report(contactSolver.m_constraints);

		if (allowSleep) {

			let minSleepTime: number = Number.MAX_VALUE;

			const linTolSqr: number = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
			const angTolSqr: number = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;

			for (i = 0; i < this.m_bodyCount; ++i) {
				b = this.m_bodies[i];
				if (b.m_invMass == 0.0) {
					continue;
				}

				if ((b.m_flags & b2Body.e_allowSleepFlag) == 0) {
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				}

				if ((b.m_flags & b2Body.e_allowSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					b2Math.b2Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				} else {
					b.m_sleepTime += step.dt;
					minSleepTime = b2Math.b2Min(minSleepTime, b.m_sleepTime);
				}
			}

			if (minSleepTime >= b2Settings.b2_timeToSleep) {
				for (i = 0; i < this.m_bodyCount; ++i) {
					b = this.m_bodies[i];
					b.m_flags |= b2Body.e_sleepFlag;
					b.m_linearVelocity.SetZero();
					b.m_angularVelocity = 0.0;
				}
			}
		}
	}

	public SolveTOI(subStep: b2TimeStep): void {
		let i: number /** int */;
		const contactSolver: b2ContactSolver = new b2ContactSolver(subStep, this.m_contacts, this.m_contactCount, this.m_allocator);

		// No warm starting needed for TOI events.

		// Solve velocity constraints.
		for (i = 0; i < subStep.maxIterations; ++i) {
			contactSolver.SolveVelocityConstraints();
		}

		// Don't store the TOI contact forces for warm starting
		// because they can be quite large.

		// Integrate positions.
		for (i = 0; i < this.m_bodyCount; ++i) {
			const b: b2Body = this.m_bodies[i];

			if (b.IsStatic())
				continue;

			// Store positions for continuous collision.
			b.m_sweep.c0.SetV(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;

			// Integrate
			b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
			b.m_sweep.a += subStep.dt * b.m_angularVelocity;

			// Compute new transform
			b.SynchronizeTransform();

			// Note: shapes are synchronized later.
		}

		// Solve position constraints.
		const k_toiBaumgarte: number = 0.75;
		for (i = 0; i < subStep.maxIterations; ++i) {
			const contactsOkay: boolean = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
			if (contactsOkay) {
				break;
			}
		}

		this.Report(contactSolver.m_constraints);
	}

	private static s_reportCR: b2ContactResult = new b2ContactResult();
	public Report(constraints: b2ContactConstraint[]): void {
		let tMat: b2Mat22;
		let tVec: b2Vec2;
		if (this.m_listener == null) {
			return;
		}

		for (let i: number /** int */ = 0; i < this.m_contactCount; ++i) {
			const c: b2Contact = this.m_contacts[i];
			const cc: b2ContactConstraint = constraints[ i ];
			const cr: b2ContactResult = b2Island.s_reportCR;
			cr.shape1 = c.m_shape1;
			cr.shape2 = c.m_shape2;
			const b1: b2Body = cr.shape1.m_body;
			const manifoldCount: number /** int */ = c.m_manifoldCount;
			const manifolds: b2Manifold[] = c.GetManifolds();
			for (let j: number /** int */ = 0; j < manifoldCount; ++j) {
				const manifold: b2Manifold = manifolds[ j ];
				cr.normal.SetV(manifold.normal);
				for (let k: number /** int */ = 0; k < manifold.pointCount; ++k) {
					const point: b2ManifoldPoint = manifold.points[ k ];
					const ccp: b2ContactConstraintPoint = cc.points[ k ];
					cr.position = b1.GetWorldPoint(point.localPoint1);

					// TOI constraint results are not stored, so get
					// the result from the constraint.
					cr.normalImpulse = ccp.normalImpulse;
					cr.tangentImpulse = ccp.tangentImpulse;
					cr.id.key = point.id.key;

					this.m_listener.Result(cr);
				}
			}
		}
	}

	public AddBody(body: b2Body): void {
		//b2Settings.b2Assert(this.m_bodyCount < this.m_bodyCapacity);
		this.m_bodies[this.m_bodyCount++] = body;
	}

	public AddContact(contact: b2Contact): void {
		//b2Settings.b2Assert(this.m_contactCount < this.m_contactCapacity);
		this.m_contacts[this.m_contactCount++] = contact;
	}

	public AddJoint(joint: b2Joint): void {
		//b2Settings.b2Assert(this.m_jointCount < this.m_jointCapacity);
		this.m_joints[this.m_jointCount++] = joint;
	}

	public m_allocator: any;
	public m_listener: b2ContactListener;

	public m_bodies: b2Body[];
	public m_contacts: b2Contact[];
	public m_joints: b2Joint[];

	public m_bodyCount: number /** int */;
	public m_jointCount: number /** int */;
	public m_contactCount: number /** int */;

	public m_bodyCapacity: number /** int */;
	public m_contactCapacity: number /** int */;
	public m_jointCapacity: number /** int */;

	public m_positionIterationCount: number /** int */;

}