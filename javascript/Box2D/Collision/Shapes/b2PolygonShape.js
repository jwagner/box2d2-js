var b2PolygonShape = function() {
b2Shape.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PolygonShape.prototype, b2Shape.prototype)
b2PolygonShape.prototype._super = function(){ b2Shape.prototype.__constructor.apply(this, arguments) }
b2PolygonShape.prototype.__constructor = function (def) {
		this._super(def);
		
		
		this.m_type = b2Shape.e_polygonShape;
		var poly = def;
		
		
		this.m_vertexCount = poly.vertexCount;
		
		
		var i = 0;
		var i1 = i;
		var i2 = i;
		
		
		for (i = 0; i < this.m_vertexCount; ++i)
		{
			this.m_vertices[i] = poly.vertices[i].Copy();
		}
		
		
		for (i = 0; i < this.m_vertexCount; ++i)
		{
			i1 = i;
			i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
			
			var edgeX = this.m_vertices[i2].x - this.m_vertices[i1].x;
			var edgeY = this.m_vertices[i2].y - this.m_vertices[i1].y;
			
			
			var len = Math.sqrt(edgeX*edgeX + edgeY*edgeY);
			
			this.m_normals[i] = new b2Vec2(edgeY/len, -edgeX/len);
		}
		
		
		
		
		this.m_centroid = b2PolygonShape.ComputeCentroid(poly.vertices, poly.vertexCount);
		
		
		b2PolygonShape.ComputeOBB(this.m_obb, this.m_vertices, this.m_vertexCount);
		
		
		
		for (i = 0; i < this.m_vertexCount; ++i)
		{
			i1 = i - 1 >= 0 ? i - 1 : this.m_vertexCount - 1;
			i2 = i;
			
			
			var n1X = this.m_normals[i1].x;
			var n1Y = this.m_normals[i1].y;
			
			var n2X = this.m_normals[i2].x;
			var n2Y = this.m_normals[i2].y;
			
			var vX = this.m_vertices[i].x - this.m_centroid.x;
			var vY = this.m_vertices[i].y - this.m_centroid.y;
			
			
			var dX = (n1X*vX + n1Y*vY) - b2Settings.b2_toiSlop;
			var dY = (n2X*vX + n2Y*vY) - b2Settings.b2_toiSlop;
			
			
			
			
			
			
			
			
			
			
			
			
			var det = 1.0/(n1X * n2Y - n1Y * n2X);
			
			this.m_coreVertices[i] = new b2Vec2(	det * (n2Y * dX - n1Y * dY) + this.m_centroid.x, 
											det * (n1X * dY - n2X * dX) + this.m_centroid.y);
		}
	}
b2PolygonShape.prototype.__varz = function(){
this.s_supportVec =  new b2Vec2();
this.m_obb =  new b2OBB();
this.m_vertices =  new Array(b2Settings.b2_maxPolygonVertices);
this.m_normals =  new Array(b2Settings.b2_maxPolygonVertices);
this.m_coreVertices =  new Array(b2Settings.b2_maxPolygonVertices);
}
// static attributes
b2PolygonShape.s_computeMat =  new b2Mat22();
b2PolygonShape.s_sweptAABB1 =  new b2AABB();
b2PolygonShape.s_sweptAABB2 =  new b2AABB();
// static methods
b2PolygonShape.ComputeCentroid = function (vs, count) {
		
		
		var c = new b2Vec2();
		var area = 0.0;
		
		
		
		
		var p1X = 0.0;
		var p1Y = 0.0;
	
		
		var inv3 = 1.0 / 3.0;
		
		for (var i = 0; i < count; ++i)
		{
			
			
				
			
			var p2 = vs[i];
			
			var p3 = i + 1 < count ? vs[parseInt(i+1)] : vs[0];
			
			
			var e1X = p2.x - p1X;
			var e1Y = p2.y - p1Y;
			
			var e2X = p3.x - p1X;
			var e2Y = p3.y - p1Y;
			
			
			var D = (e1X * e2Y - e1Y * e2X);
			
			
			var triangleArea = 0.5 * D;
			area += triangleArea;
			
			
			
			c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
			c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
		}
		
		
		
		
		c.x *= 1.0 / area;
		c.y *= 1.0 / area;
		return c;
	}
b2PolygonShape.ComputeOBB = function (obb, vs, count) {
		var i = 0;
		
		var p = new Array(b2Settings.b2_maxPolygonVertices + 1);
		for (i = 0; i < count; ++i)
		{
			p[i] = vs[i];
		}
		p[count] = p[0];
		
		var minArea = Number.MAX_VALUE;
		
		for (i = 1; i <= count; ++i)
		{
			var root = p[parseInt(i-1)];
			
			var uxX = p[i].x - root.x;
			var uxY = p[i].y - root.y;
			
			var length = Math.sqrt(uxX*uxX + uxY*uxY);
			uxX /= length;
			uxY /= length;
			
			
			var uyX = -uxY;
			var uyY = uxX;
			
			var lowerX = Number.MAX_VALUE;
			var lowerY = Number.MAX_VALUE;
			
			var upperX = -Number.MAX_VALUE;
			var upperY = -Number.MAX_VALUE;
			
			for (var j = 0; j < count; ++j)
			{
				
				var dX = p[j].x - root.x;
				var dY = p[j].y - root.y;
				
				
				var rX = (uxX*dX + uxY*dY);
				
				var rY = (uyX*dX + uyY*dY);
				
				if (rX < lowerX) lowerX = rX;
				if (rY < lowerY) lowerY = rY;
				
				if (rX > upperX) upperX = rX;
				if (rY > upperY) upperY = rY;
			}
			
			var area = (upperX - lowerX) * (upperY - lowerY);
			if (area < 0.95 * minArea)
			{
				minArea = area;
				
				obb.R.col1.x = uxX;
				obb.R.col1.y = uxY;
				
				obb.R.col2.x = uyX;
				obb.R.col2.y = uyY;
				
				var centerX = 0.5 * (lowerX + upperX);
				var centerY = 0.5 * (lowerY + upperY);
				
				var tMat = obb.R;
				obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
				obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
				
				obb.extents.x = 0.5 * (upperX - lowerX);
				obb.extents.y = 0.5 * (upperY - lowerY);
			}
		}
		
		
	}
// attributes
b2PolygonShape.prototype.s_supportVec =  new b2Vec2();
b2PolygonShape.prototype.m_centroid =  null;
b2PolygonShape.prototype.m_obb =  new b2OBB();
b2PolygonShape.prototype.m_vertices =  new Array(b2Settings.b2_maxPolygonVertices);
b2PolygonShape.prototype.m_normals =  new Array(b2Settings.b2_maxPolygonVertices);
b2PolygonShape.prototype.m_coreVertices =  new Array(b2Settings.b2_maxPolygonVertices);
b2PolygonShape.prototype.m_vertexCount =  0;
// methods
b2PolygonShape.prototype.TestPoint = function (xf, p) {
		var tVec;
		
		
		var tMat = xf.R;
		var tX = p.x - xf.position.x;
		var tY = p.y - xf.position.y;
		var pLocalX = (tX*tMat.col1.x + tY*tMat.col1.y);
		var pLocalY = (tX*tMat.col2.x + tY*tMat.col2.y);
		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			tVec = this.m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			tVec = this.m_normals[i];
			var dot = (tVec.x * tX + tVec.y * tY);
			if (dot > 0.0)
			{
				return false;
			}
		}
		
		return true;
	}
b2PolygonShape.prototype.TestSegment = function ( xf,
		lambda, 
		normal, 
		segment,
		maxLambda) {
		var lower = 0.0;
		var upper = maxLambda;
		
		var tX;
		var tY;
		var tMat;
		var tVec;
		
		
		tX = segment.p1.x - xf.position.x;
		tY = segment.p1.y - xf.position.y;
		tMat = xf.R;
		var p1X = (tX * tMat.col1.x + tY * tMat.col1.y);
		var p1Y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		tX = segment.p2.x - xf.position.x;
		tY = segment.p2.y - xf.position.y;
		tMat = xf.R;
		var p2X = (tX * tMat.col1.x + tY * tMat.col1.y);
		var p2Y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		var index = -1;
		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			
			
			
			
			tVec = this.m_vertices[i];
			tX = tVec.x - p1X;
			tY = tVec.y - p1Y;
			tVec = this.m_normals[i];
			var numerator = (tVec.x*tX + tVec.y*tY);
			
			var denominator = (tVec.x*dX + tVec.y*dY);
			
			
			
			
			
			
			if (denominator < 0.0 && numerator < lower * denominator)
			{
				
				
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0 && numerator < upper * denominator)
			{
				
				
				upper = numerator / denominator;
			}
			
			if (upper < lower)
			{
				return false;
			}
		}
		
		
		
		if (index >= 0)
		{
			
			lambda[0] = lower;
			
			tMat = xf.R;
			tVec = this.m_normals[index];
			normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			return true;
		}
		
		return false;
	}
b2PolygonShape.prototype.ComputeAABB = function (aabb, xf) {
		var tMat;
		var tVec;
		
		var R = b2PolygonShape.s_computeMat;
		
		tMat = xf.R;
		tVec = this.m_obb.R.col1;
		
		R.col1.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		R.col1.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tVec = this.m_obb.R.col2;
		
		R.col2.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		R.col2.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		
		R.Abs();
		var absR = R;
		
		tVec = this.m_obb.extents;
		var hX = (absR.col1.x * tVec.x + absR.col2.x * tVec.y);
		var hY = (absR.col1.y * tVec.x + absR.col2.y * tVec.y);
		
		tMat = xf.R;
		tVec = this.m_obb.center;
		var positionX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var positionY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		aabb.lowerBound.Set(positionX - hX, positionY - hY);
		
		aabb.upperBound.Set(positionX + hX, positionY + hY);
	}
b2PolygonShape.prototype.ComputeSweptAABB = function (	aabb,
		transform1,
		transform2) {
		
		var aabb1 = b2PolygonShape.s_sweptAABB1;
		var aabb2 = b2PolygonShape.s_sweptAABB2;
		this.ComputeAABB(aabb1, transform1);
		this.ComputeAABB(aabb2, transform2);
		
		aabb.lowerBound.Set((aabb1.lowerBound.x < aabb2.lowerBound.x ? aabb1.lowerBound.x : aabb2.lowerBound.x),
							(aabb1.lowerBound.y < aabb2.lowerBound.y ? aabb1.lowerBound.y : aabb2.lowerBound.y));
		
		aabb.upperBound.Set((aabb1.upperBound.x > aabb2.upperBound.x ? aabb1.upperBound.x : aabb2.upperBound.x),
							(aabb1.upperBound.y > aabb2.upperBound.y ? aabb1.upperBound.y : aabb2.upperBound.y));
	}
b2PolygonShape.prototype.ComputeMass = function (massData) {
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		var centerX = 0.0;
		var centerY = 0.0;
		var area = 0.0;
		var I = 0.0;
		
		
		
		
		var p1X = 0.0;
		var p1Y = 0.0;
		
		
		var k_inv3 = 1.0 / 3.0;
		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			
			
			
			var p2 = this.m_vertices[i];
			
			var p3 = i + 1 < this.m_vertexCount ? this.m_vertices[parseInt(i+1)] : this.m_vertices[0];
			
			
			var e1X = p2.x - p1X;
			var e1Y = p2.y - p1Y;
			
			var e2X = p3.x - p1X;
			var e2Y = p3.y - p1Y;
			
			
			var D = e1X * e2Y - e1Y * e2X;
			
			
			var triangleArea = 0.5 * D;
			area += triangleArea;
			
			
			
			centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
			centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
			
			
			var px = p1X;
			var py = p1Y;
			
			var ex1 = e1X;
			var ey1 = e1Y;
			
			var ex2 = e2X;
			var ey2 = e2Y;
			
			
			var intx2 = k_inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
			
			var inty2 = k_inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;
			
			I += D * (intx2 + inty2);
		}
		
		
		massData.mass = this.m_density * area;
		
		
		
		
		centerX *= 1.0 / area;
		centerY *= 1.0 / area;
		
		massData.center.Set(centerX, centerY);
		
		
		massData.I = this.m_density * I;
	}
b2PolygonShape.prototype.GetOBB = function () {
		return this.m_obb;
	}
b2PolygonShape.prototype.GetCentroid = function () {
		return this.m_centroid;
	}
b2PolygonShape.prototype.GetVertexCount = function () {
		return this.m_vertexCount;
	}
b2PolygonShape.prototype.GetVertices = function () {
		return this.m_vertices;
	}
b2PolygonShape.prototype.GetCoreVertices = function () {
		return this.m_coreVertices;
	}
b2PolygonShape.prototype.GetNormals = function () {
		return this.m_normals;
	}
b2PolygonShape.prototype.GetFirstVertex = function (xf) {
		return b2Math.b2MulX(xf, this.m_coreVertices[0]);
	}
b2PolygonShape.prototype.Centroid = function (xf) {
		return b2Math.b2MulX(xf, this.m_centroid);
	}
b2PolygonShape.prototype.Support = function (xf, dX, dY) {
		var tVec;
		
		var tMat;
		
		tMat = xf.R;
		var dLocalX = (dX * tMat.col1.x + dY * tMat.col1.y);
		var dLocalY = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		var bestIndex = 0;
		
		tVec = this.m_coreVertices[0];
		var bestValue = (tVec.x*dLocalX + tVec.y*dLocalY);
		for (var i = 1; i < this.m_vertexCount; ++i)
		{
			
			tVec = this.m_coreVertices[i];
			var value = (tVec.x*dLocalX + tVec.y*dLocalY);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		
		
		tMat = xf.R;
		tVec = this.m_coreVertices[bestIndex];
		this.s_supportVec.x = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		this.s_supportVec.y = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		return this.s_supportVec;
		
	}
b2PolygonShape.prototype.UpdateSweepRadius = function (center) {
		var tVec;
		
		
		
		this.m_sweepRadius = 0.0;
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			tVec = this.m_coreVertices[i];
			var dX = tVec.x - center.x;
			var dY = tVec.y - center.y;
			dX = Math.sqrt(dX*dX + dY*dY);
			
			if (dX > this.m_sweepRadius) this.m_sweepRadius = dX;
		}
	}
