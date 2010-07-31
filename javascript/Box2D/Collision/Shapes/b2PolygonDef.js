var b2PolygonDef = function() {
b2ShapeDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PolygonDef.prototype, b2ShapeDef.prototype)
b2PolygonDef.prototype._super = function(){ b2ShapeDef.prototype.__constructor.apply(this, arguments) }
b2PolygonDef.prototype.__constructor = function () {
		this.type = b2Shape.e_polygonShape;
		this.vertexCount = 0;
		
		for (var i = 0; i < b2Settings.b2_maxPolygonVertices; i++){
			this.vertices[i] = new b2Vec2();
		}
	}
b2PolygonDef.prototype.__varz = function(){
this.vertices =  new Array(b2Settings.b2_maxPolygonVertices);
}
// static attributes
b2PolygonDef.s_mat =  new b2Mat22();
// static methods
// attributes
b2PolygonDef.prototype.vertices =  new Array(b2Settings.b2_maxPolygonVertices);
b2PolygonDef.prototype.vertexCount =  0;
// methods
b2PolygonDef.prototype.SetAsBox = function (hx, hy) {
		this.vertexCount = 4;
		this.vertices[0].Set(-hx, -hy);
		this.vertices[1].Set( hx, -hy);
		this.vertices[2].Set( hx, hy);
		this.vertices[3].Set(-hx, hy);
	}
b2PolygonDef.prototype.SetAsOrientedBox = function (hx, hy, center, angle) {
		
		{
			this.vertexCount = 4;
			this.vertices[0].Set(-hx, -hy);
			this.vertices[1].Set( hx, -hy);
			this.vertices[2].Set( hx, hy);
			this.vertices[3].Set(-hx, hy);
		}
		
		if (center){
			
			
			var xfPosition = center;
			
			var xfR = b2PolygonDef.s_mat;
			xfR.Set(angle);
			
			for (var i = 0; i < this.vertexCount; ++i)
			{
				
				
				center = this.vertices[i];
				hx = xfPosition.x + (xfR.col1.x * center.x + xfR.col2.x * center.y)
				center.y = xfPosition.y + (xfR.col1.y * center.x + xfR.col2.y * center.y)
				center.x = hx;
			}
		}
	}