import { b2Vec2, b2Transform } from '../Common/Math';
import { b2Color } from '../Common/b2Color';
import { Sprite } from '@awayjs/scene';

/**
* Implement and register this class with a b2World to provide debug drawing of physics
* entities in your game.
*/
export class b2DebugDraw {

	__fast__: boolean = true;

	constructor() {
		this.m_drawFlags = 0;
	}

	//virtual ~b2DebugDraw() {}

	//enum
	//{
	/** Draw shapes */
	public static e_shapeBit: number /** uint */ 			= 0x0001;
	/** Draw joint connections */
	public static e_jointBit: number /** uint */			= 0x0002;
	/** Draw axis aligned bounding boxes */
	public static e_aabbBit: number /** uint */			= 0x0004;
	/** Draw broad-phase pairs */
	public static e_pairBit: number /** uint */			= 0x0008;
	/** Draw center of mass frame */
	public static e_centerOfMassBit: number /** uint */	= 0x0010;
	/** Draw controllers */
	public static e_controllerBit: number /** uint */		= 0x0020;
	//};

	/**
	* Set the drawing flags.
	*/
	public SetFlags(flags: number /** uint */): void {
		this.m_drawFlags = flags;
	}

	/**
	* Get the drawing flags.
	*/
	public GetFlags(): number /** uint */{
		return this.m_drawFlags;
	}

	/**
	* Append flags to the current flags.
	*/
	public AppendFlags(flags: number /** uint */): void {
		this.m_drawFlags |= flags;
	}

	/**
	* Clear flags from the current flags.
	*/
	public ClearFlags(flags: number /** uint */): void {
		this.m_drawFlags &= ~flags;
	}

	/**
	* Set the sprite
	*/
	public SetSprite(sprite: Sprite): void {
		this.m_sprite = sprite;
	}

	/**
	* Get the sprite
	*/
	public GetSprite(): Sprite {
		return this.m_sprite;
	}

	/**
	* Set the draw scale
	*/
	public SetDrawScale(drawScale: number): void {
		this.m_drawScale = drawScale;
	}

	/**
	* Get the draw
	*/
	public GetDrawScale(): Number {
		return this.m_drawScale;
	}

	/**
	* Set the line thickness
	*/
	public SetLineThickness(lineThickness: number): void {
		this.m_lineThickness = lineThickness;
	}

	/**
	* Get the line thickness
	*/
	public GetLineThickness(): Number {
		return this.m_lineThickness;
	}

	/**
	* Set the alpha value used for lines
	*/
	public SetAlpha(alpha: number): void {
		this.m_alpha = alpha;
	}

	/**
	* Get the alpha value used for lines
	*/
	public GetAlpha(): Number {
		return this.m_alpha;
	}

	/**
	* Set the alpha value used for fills
	*/
	public SetFillAlpha(alpha: number): void {
		this.m_fillAlpha = alpha;
	}

	/**
	* Get the alpha value used for fills
	*/
	public GetFillAlpha(): Number {
		return this.m_fillAlpha;
	}

	/**
	* Set the scale used for drawing XForms
	*/
	public SetXFormScale(xformScale: number): void {
		this.m_xformScale = xformScale;
	}

	/**
	* Get the scale used for drawing XForms
	*/
	public GetXFormScale(): Number {
		return this.m_xformScale;
	}

	/**
	* Draw a closed polygon provided in CCW order.
	*/
	public DrawPolygon(vertices: Array<b2Vec2>, vertexCount: number /** int */, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		for (let i: number /** int */ = 1; i < vertexCount; i++) {
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);

	}

	/**
	* Draw a solid closed polygon provided in CCW order.
	*/
	public DrawSolidPolygon(vertices: Array<b2Vec2>, vertexCount: number /** int */, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
		for (let i: number /** int */ = 1; i < vertexCount; i++) {
			this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.endFill();

	}

	/**
	* Draw a circle.
	*/
	public DrawCircle(center: b2Vec2, radius: number, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);

	}

	/**
	* Draw a solid circle.
	*/
	public DrawSolidCircle(center: b2Vec2, radius: number, axis: b2Vec2, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(0,0);
		this.m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
		this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);
		this.m_sprite.graphics.endFill();
		this.m_sprite.graphics.moveTo(center.x * this.m_drawScale, center.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((center.x + axis.x * radius) * this.m_drawScale, (center.y + axis.y * radius) * this.m_drawScale);

	}

	/**
	* Draw a line segment.
	*/
	public DrawSegment(p1: b2Vec2, p2: b2Vec2, color: b2Color): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(p1.x * this.m_drawScale, p1.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo(p2.x * this.m_drawScale, p2.y * this.m_drawScale);

	}

	/**
	* Draw a transform. Choose your own length scale.
	* @param xf a transform.
	*/
	public DrawTransform(xf: b2Transform): void {

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, 0xff0000, this.m_alpha);
		this.m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * this.m_drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * this.m_drawScale);

		this.m_sprite.graphics.lineStyle(this.m_lineThickness, 0x00ff00, this.m_alpha);
		this.m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * this.m_drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * this.m_drawScale);

	}

	private m_drawFlags: number /** uint */;
	public m_sprite: Sprite;
	private m_drawScale: number = 1.0;

	private m_lineThickness: number = 1.0;
	private m_alpha: number = 1.0;
	private m_fillAlpha: number = 1.0;
	private m_xformScale: number = 1.0;

}
