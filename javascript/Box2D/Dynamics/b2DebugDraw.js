var b2DebugDraw = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2DebugDraw.prototype.__constructor = function () {
		this.m_drawFlags = 0;
	}
b2DebugDraw.prototype.__varz = function(){
}
// static attributes
b2DebugDraw.e_shapeBit =  0x0001;
b2DebugDraw.e_jointBit =  0x0002;
b2DebugDraw.e_coreShapeBit =  0x0004;
b2DebugDraw.e_aabbBit =  0x0008;
b2DebugDraw.e_obbBit =  0x0010;
b2DebugDraw.e_pairBit =  0x0020;
b2DebugDraw.e_centerOfMassBit =  0x0040;
// static methods
// attributes
b2DebugDraw.prototype.m_drawFlags =  0;
b2DebugDraw.prototype.m_sprite =  null;
b2DebugDraw.prototype.m_drawScale =  1.0;
b2DebugDraw.prototype.m_lineThickness =  1.0;
b2DebugDraw.prototype.m_alpha =  1.0;
b2DebugDraw.prototype.m_fillAlpha =  1.0;
b2DebugDraw.prototype.m_xformScale =  1.0;
// methods
b2DebugDraw.prototype.SetFlags = function (flags) {
		this.m_drawFlags = flags;
	}
b2DebugDraw.prototype.GetFlags = function () {
		return this.m_drawFlags;
	}
b2DebugDraw.prototype.AppendFlags = function (flags) {
		this.m_drawFlags |= flags;
	}
b2DebugDraw.prototype.ClearFlags = function (flags) {
		this.m_drawFlags &= ~flags;
	}
b2DebugDraw.prototype.DrawPolygon = function (vertices, vertexCount, color) {
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		for (var i = 1; i < vertexCount; i++){
				this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		
	}
b2DebugDraw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) {
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
		for (var i = 1; i < vertexCount; i++){
				this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
		}
		this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
		this.m_sprite.graphics.endFill();
		
	}
b2DebugDraw.prototype.DrawCircle = function (center, radius, color) {
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);
		
	}
b2DebugDraw.prototype.DrawSolidCircle = function (center, radius, axis, color) {
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(0,0);
		this.m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
		this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);
		this.m_sprite.graphics.endFill();
		this.m_sprite.graphics.moveTo(center.x * this.m_drawScale, center.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((center.x + axis.x*radius) * this.m_drawScale, (center.y + axis.y*radius) * this.m_drawScale);
		
	}
b2DebugDraw.prototype.DrawSegment = function (p1, p2, color) {
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
		this.m_sprite.graphics.moveTo(p1.x * this.m_drawScale, p1.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo(p2.x * this.m_drawScale, p2.y * this.m_drawScale);
		
	}
b2DebugDraw.prototype.DrawXForm = function (xf) {
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, 0xff0000, this.m_alpha);
		this.m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale*xf.R.col1.x) * this.m_drawScale, (xf.position.y + this.m_xformScale*xf.R.col1.y) * this.m_drawScale);
		
		this.m_sprite.graphics.lineStyle(this.m_lineThickness, 0x00ff00, this.m_alpha);
		this.m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
		this.m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale*xf.R.col2.x) * this.m_drawScale, (xf.position.y + this.m_xformScale*xf.R.col2.y) * this.m_drawScale);
		
	}