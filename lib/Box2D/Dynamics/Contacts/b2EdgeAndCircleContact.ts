import { b2Contact } from '../Contacts';
import { b2Fixture } from '../b2Fixture';
import { b2Body } from '../b2Body';
import { b2EdgeShape } from '../../Collision/Shapes/b2EdgeShape';
import { b2CircleShape } from '../../Collision/Shapes/b2CircleShape';
import { b2Manifold } from '../../Collision/b2Manifold';
import { b2Transform } from '../../Common/Math';

/**
* @private
*/
export class b2EdgeAndCircleContact extends b2Contact {
	public static Create(allocator: any): b2Contact {
		return new b2EdgeAndCircleContact();
	}

	public static Destroy(contact: b2Contact, allocator: any): void {
		//
	}

	public Reset(fixtureA: b2Fixture, fixtureB: b2Fixture): void {
		super.Reset(fixtureA, fixtureB);
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_circleShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
	}
	//~b2EdgeAndCircleContact() {}

	public Evaluate(): void {
		const bA: b2Body = this.m_fixtureA.GetBody();
		const bB: b2Body = this.m_fixtureB.GetBody();
		this.b2CollideEdgeAndCircle(this.m_manifold,
			this.m_fixtureA.GetShape() as b2EdgeShape, bA.m_xf,
			this.m_fixtureB.GetShape() as b2CircleShape, bB.m_xf);
	}

	private b2CollideEdgeAndCircle(manifold: b2Manifold,
	                                        edge: b2EdgeShape,
	                                        xf1: b2Transform,
	                                        circle: b2CircleShape,
	                                        xf2: b2Transform): void {
		//TODO_BORIS
		/*
		manifold.m_pointCount = 0;
		var tMat: b2Mat22;
		var tVec: b2Vec2;
		var dX: Number;
		var dY: Number;
		var tX: Number;
		var tY: Number;
		var tPoint:b2ManifoldPoint;

		//b2Vec2 c = b2Mul(xf2, circle->GetLocalPosition());
		tMat = xf2.R;
		tVec = circle.m_r;
		var cX: Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY: Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		//b2Vec2 cLocal = b2MulT(xf1, c);
		tMat = xf1.R;
		tX = cX - xf1.position.x;
		tY = cY - xf1.position.y;
		var cLocalX: Number = (tX * tMat.col1.x + tY * tMat.col1.y );
		var cLocalY: Number = (tX * tMat.col2.x + tY * tMat.col2.y );

		var n: b2Vec2 = edge.m_normal;
		var v1: b2Vec2 = edge.m_v1;
		var v2: b2Vec2 = edge.m_v2;
		var radius: Number = circle.m_radius;
		var separation: Number;

		var dirDist: Number = (cLocalX - v1.x) * edge.m_direction.x +
		                      (cLocalY - v1.y) * edge.m_direction.y;

		var normalCalculated: Boolean = false;

		if (dirDist <= 0) {
			dX = cLocalX - v1.x;
			dY = cLocalY - v1.y;
			if (dX * edge.m_cornerDir1.x + dY * edge.m_cornerDir1.y < 0) {
				return;
			}
			dX = cX - (xf1.position.x + (tMat.col1.x * v1.x + tMat.col2.x * v1.y));
			dY = cY - (xf1.position.y + (tMat.col1.y * v1.x + tMat.col2.y * v1.y));
		} else if (dirDist >= edge.m_length) {
			dX = cLocalX - v2.x;
			dY = cLocalY - v2.y;
			if (dX * edge.m_cornerDir2.x + dY * edge.m_cornerDir2.y > 0) {
				return;
			}
			dX = cX - (xf1.position.x + (tMat.col1.x * v2.x + tMat.col2.x * v2.y));
			dY = cY - (xf1.position.y + (tMat.col1.y * v2.x + tMat.col2.y * v2.y));
		} else {
			separation = (cLocalX - v1.x) * n.x + (cLocalY - v1.y) * n.y;
			if (separation > radius || separation < -radius) {
				return;
			}
			separation -= radius;

			//manifold.normal = b2Mul(xf1.R, n);
			tMat = xf1.R;
			manifold.normal.x = (tMat.col1.x * n.x + tMat.col2.x * n.y);
			manifold.normal.y = (tMat.col1.y * n.x + tMat.col2.y * n.y);

			normalCalculated = true;
		}

		if (!normalCalculated) {
			var distSqr: Number = dX * dX + dY * dY;
			if (distSqr > radius * radius)
			{
				return;
			}

			if (distSqr < Number.MIN_VALUE)
			{
				separation = -radius;
				manifold.normal.x = (tMat.col1.x * n.x + tMat.col2.x * n.y);
				manifold.normal.y = (tMat.col1.y * n.x + tMat.col2.y * n.y);
			}
			else
			{
				distSqr = Math.sqrt(distSqr);
				dX /= distSqr;
				dY /= distSqr;
				separation = distSqr - radius;
				manifold.normal.x = dX;
				manifold.normal.y = dY;
			}
		}

		tPoint = manifold.points[0];
		manifold.pointCount = 1;
		tPoint.id.key = 0;
		tPoint.separation = separation;
		cX = cX - radius * manifold.normal.x;
		cY = cY - radius * manifold.normal.y;

		tX = cX - xf1.position.x;
		tY = cY - xf1.position.y;
		tPoint.localPoint1.x = (tX * tMat.col1.x + tY * tMat.col1.y );
		tPoint.localPoint1.y = (tX * tMat.col2.x + tY * tMat.col2.y );

		tMat = xf2.R;
		tX = cX - xf2.position.x;
		tY = cY - xf2.position.y;
		tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y );
		tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y );
		*/
	}
}