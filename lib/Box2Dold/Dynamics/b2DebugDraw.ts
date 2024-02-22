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

import { b2XForm, b2Vec2 } from '../Common/Math';
import { b2Color } from '../Common/b2Color';

/// Implement and register this class with a b2World to provide debug drawing of physics
/// entities in your game.
export class b2DebugDraw {

	constructor() {
		this.m_drawFlags = 0;
	}

	//virtual ~b2DebugDraw() {}

	//enum
	//{
	public static e_shapeBit: number /** uint */ 			= 0x0001; ///< draw shapes
	public static e_jointBit: number /** uint */			= 0x0002; ///< draw joint connections
	public static e_coreShapeBit: number /** uint */		= 0x0004; ///< draw core (TOI) shapes
	public static e_aabbBit: number /** uint */			= 0x0008; ///< draw axis aligned bounding boxes
	public static e_obbBit: number /** uint */				= 0x0010; ///< draw oriented bounding boxes
	public static e_pairBit: number /** uint */			= 0x0020; ///< draw broad-phase pairs
	public static e_centerOfMassBit: number /** uint */	= 0x0040; ///< draw center of mass frame
	//};

	/// Set the drawing flags.
	public SetFlags(flags: number /** uint */): void {
		this.m_drawFlags = flags;
	}

	/// Get the drawing flags.
	public GetFlags(): number /** uint */{
		return this.m_drawFlags;
	}

	/// Append flags to the current flags.
	public AppendFlags(flags: number /** uint */): void {
		this.m_drawFlags |= flags;
	}

	/// Clear flags from the current flags.
	public ClearFlags(flags: number /** uint */): void {
		this.m_drawFlags &= ~flags;
	}

	/// Draw a closed polygon provided in CCW order.
	public DrawPolygon(vertices: b2Vec2[], vertexCount: number /** int */, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		for (let i: number /** int */ = 1; i < vertexCount; i++) {
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);

	}

	/// Draw a solid closed polygon provided in CCW order.
	public DrawSolidPolygon(vertices: b2Vec2[], vertexCount: number /** int */, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
		for (let i: number /** int */ = 1; i < vertexCount; i++) {
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.endFill();

	}

	/// Draw a circle.
	public DrawCircle(center: b2Vec2, radius: number, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);

	}

	/// Draw a solid circle.
	public DrawSolidCircle(center: b2Vec2, radius: number, axis: b2Vec2, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(0,0);
		this.m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
		this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);
		this.m_sprite.graphics.endFill();
		this.m_sprite.graphics.moveTo(center.x * this.m_drawScale, center.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((center.x + axis.x * radius) * this.m_drawScale, (center.y + axis.y * radius) * this.m_drawScale);

	}

	/// Draw a line segment.
	public DrawSegment(p1: b2Vec2, p2: b2Vec2, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(p1.x * this.m_drawScale, p1.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo(p2.x * this.m_drawScale, p2.y * this.m_drawScale);

	}

	/// Draw a closed polygon provided in CCW order.
	public DrawPolyline(vertices:b2Vec2[], vertexCount:number /** uint */, color:b2Color) : void{
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		for (var i:number /** uint */ = 1; i < vertexCount; i++){
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		
	}
	
	/// Draw a circle arc
	public DrawCurve(p1:b2Vec2, p2: b2Vec2, center: b2Vec2, color:b2Color) : void{
		var s_numSegs:number = 16;
		var dx:number = p1.x-center.x;
		var dy:number = p1.y-center.y;
		var theta1:number = Math.atan2(dy,dx);
		var theta2:number = Math.atan2(p2.y-center.y,p2.x-center.x);
		var r:number = Math.sqrt(dx*dx+dy*dy);
		while(theta2<theta1)
			theta2 += Math.PI*2;
		var dtheta:number = (theta2-theta1)/s_numSegs;
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(p1.x * this.m_drawScale, p1.y * this.m_drawScale);
		for(var theta:number = theta1;theta<=theta2;theta+=dtheta){
			this.m_sprite.graphics.lineTo( (center.x + r*Math.cos(theta)) * this.m_drawScale, (center.y + r*Math.sin(theta)) * this.m_drawScale);
		}
	}
	
	public DrawConcaveArc(vertices:b2Vec2[], vertexCount:number /** uint */, center:b2Vec2, color:b2Color) : void {
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x, this.m_drawScale, vertices[0].y * this.m_drawScale);
		//Draw arc
		var s_numSegs:number = 16;
		var dx:number = vertices[0].x-center.x;
		var dy:number = vertices[0].y-center.y;
		var theta1:number = Math.atan2(dy,dx);
		var theta2:number = Math.atan2(vertices[1].y-center.y,vertices[1].x-center.x);
		var r:number = Math.sqrt(dx*dx+dy*dy);
		while(theta2>theta1)
			theta2 -= Math.PI*2;
		var dtheta:number = (theta1-theta2)/s_numSegs;
		
		for(var theta:number = theta1;theta>theta2;theta-=dtheta){
			this.m_sprite.graphics.lineTo( (center.x + r*Math.cos(theta)) * this.m_drawScale, (center.y + r*Math.sin(theta)) * this.m_drawScale);
		}		
		//Draw polyline
		for (var i:number /** uint */ = 1; i < vertexCount; i++){
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y *this.m_drawScale);	
	}
	
	public DrawSolidConcaveArc(vertices:b2Vec2[], vertexCount:number /** uint */, center:b2Vec2, color:b2Color) : void {
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.beginFill(color.color,this.m_fillAlpha);
		//Draw arc
		var s_numSegs:number = 16;
		var dx:number = vertices[0].x-center.x;
		var dy:number = vertices[0].y-center.y;
		var theta1:number = Math.atan2(dy,dx);
		var theta2:number = Math.atan2(vertices[1].y-center.y,vertices[1].x-center.x);
		var r:number = Math.sqrt(dx*dx+dy*dy);
		while(theta2>theta1)
			theta2 -= Math.PI*2;
		var dtheta:number = (theta1-theta2)/s_numSegs;
		
		for(var theta:number = theta1;theta>theta2;theta-=dtheta){
			this.m_sprite.graphics.lineTo( (center.x + r*Math.cos(theta)) * this.m_drawScale, (center.y + r*Math.sin(theta)) * this.m_drawScale);
		}			
		//Draw polyline
		for (var i:number /** uint */ = 1; i < vertexCount; i++){
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.endFill();		
	}

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	public DrawXForm(xf: b2XForm): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, 0xff0000, this.m_alpha);
		this.m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * this.m_drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * this.m_drawScale);

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, 0x00ff00, this.m_alpha);
		this.m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * this.m_drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * this.m_drawScale);

	}

	public m_drawFlags: number /** uint */;
	public m_sprite: any;
	public m_drawScale: number = 1.0;

	public m_lineThickness: number = 1.0;
	public m_alpha: number = 1.0;
	public m_fillAlpha: number = 1.0;
	public m_xformScale: number = 1.0;

}