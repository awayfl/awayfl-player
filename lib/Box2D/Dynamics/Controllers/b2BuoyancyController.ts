import { b2Vec2 } from "../../Common/Math";
import { b2Color } from "../../Common/b2Color";
import { b2Body } from "../b2Body";
import { b2DebugDraw } from "../b2DebugDraw";
import { b2Fixture } from "../b2Fixture";
import { b2TimeStep } from "../b2TimeStep";
import { b2Controller } from "./b2Controller";
import { b2ControllerEdge } from "./b2ControllerEdge";

/**
 * Calculates buoyancy forces for fluids in the form of a half plane
 */
export class b2BuoyancyController extends b2Controller
{
	/**
	 * The outer surface normal
	 */
	public normal:b2Vec2 = new b2Vec2(0,-1);
	/**
	 * The height of the fluid surface along the normal
	 */
	public offset:number = 0;
	/**
	 * The fluid density
	 */
	public density:number = 0;
	/**
	 * Fluid velocity, for drag calculations
	 */
	public velocity:b2Vec2 = new b2Vec2(0,0);
	/**
	 * Linear drag co-efficient
	 */
	public linearDrag:number = 2;
	/**
	 * Linear drag co-efficient
	 */
	public angularDrag:number = 1;
	/**
	 * If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities
	 */
	public useDensity:Boolean = false; //False by default to prevent a gotcha
	/**
	 * If true, gravity is taken from the world instead of the gravity parameter.
	 */
	public useWorldGravity:Boolean = true;
	/**
	 * Gravity vector, if the world's gravity is not used
	 */
	public gravity:b2Vec2 = null;
	
		
	public Step(step: b2TimeStep): void{
		if(!this.m_bodyList)
			return;
		if(this.useWorldGravity){
			this.gravity = this.GetWorld().GetGravity().Copy();
		}
		for(var i:b2ControllerEdge = this.m_bodyList;i;i=i.nextBody){
			var body:b2Body = i.body;
			if(body.IsAwake() == false) {
				//Buoyancy force is just a function of position,
				//so unlike most forces, it is safe to ignore sleeping bodes
				continue;
			}
			var areac:b2Vec2 = new b2Vec2();
			var massc:b2Vec2 = new b2Vec2();
			var area:number = 0.0;
			var mass:number = 0.0;
			for(var fixture:b2Fixture = body.GetFixtureList();fixture;fixture=fixture.GetNext()){
				var sc:b2Vec2 = new b2Vec2();
				var sarea:number = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
				area += sarea;
				areac.x += sarea * sc.x;
				areac.y += sarea * sc.y;
				var shapeDensity:number;
				if (this.useDensity) {
					//TODO: Figure out what to do now density is gone
					shapeDensity = 1;
				}else{
					shapeDensity = 1;
				}
				mass += sarea*shapeDensity;
				massc.x += sarea * sc.x * shapeDensity;
				massc.y += sarea * sc.y * shapeDensity;
			}
			areac.x/=area;
			areac.y/=area;
			massc.x/=mass;
			massc.y/=mass;
			if(area<Number.MIN_VALUE)
				continue;
			//Buoyancy
			var buoyancyForce:b2Vec2 = this.gravity.GetNegative();
			buoyancyForce.Multiply(this.density*area)
			body.ApplyForce(buoyancyForce,massc);
			//Linear drag
			var dragForce:b2Vec2 = body.GetLinearVelocityFromWorldPoint(areac);
			dragForce.Subtract(this.velocity);
			dragForce.Multiply(-this.linearDrag*area);
			body.ApplyForce(dragForce,areac);
			//Angular drag
			//TODO: Something that makes more physical sense?
			body.ApplyTorque(-body.GetInertia()/body.GetMass()*area*body.GetAngularVelocity()*this.angularDrag)
			
		}
	}
	
	public Draw(debugDraw: b2DebugDraw):void
	{
		var r:number = 1000;
		//Would like to draw a semi-transparent box
		//But debug draw doesn't support that
		var p1:b2Vec2 = new b2Vec2();
		var p2:b2Vec2 = new b2Vec2();
		p1.x = this.normal.x * this.offset + this.normal.y * r;
		p1.y = this.normal.y * this.offset - this.normal.x * r;
		p2.x = this.normal.x * this.offset - this.normal.y * r;
		p2.y = this.normal.y * this.offset + this.normal.x * r;
		var color:b2Color = new b2Color(0,0,1);
		debugDraw.DrawSegment(p1,p2,color);
	}
}