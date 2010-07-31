var b2CircleShape = function() {
b2Shape.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2CircleShape.prototype, b2Shape.prototype)
b2CircleShape.prototype._super = function(){ b2Shape.prototype.__constructor.apply(this, arguments) }
b2CircleShape.prototype.__constructor = function (def) {
		this._super(def);
		
		
		var circleDef = def;
		
		this.m_type = b2Shape.e_circleShape;
		this.m_localPosition.SetV(circleDef.localPosition);
		this.m_radius = circleDef.radius;
		
	}
b2CircleShape.prototype.__varz = function(){
this.m_localPosition =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2CircleShape.prototype.m_localPosition =  new b2Vec2();
b2CircleShape.prototype.m_radius =  null;
// methods
b2CircleShape.prototype.TestPoint = function (transform, p) {
		
		var tMat = transform.R;
		var dX = transform.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		var dY = transform.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		
		dX = p.x - dX;
		dY = p.y - dY;
		
		return (dX*dX + dY*dY) <= this.m_radius * this.m_radius;
	}
b2CircleShape.prototype.TestSegment = function (	transform,
						lambda, 
						normal, 
						segment,
						maxLambda) {
		
		var tMat = transform.R;
		var positionX = transform.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		var positionY = transform.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		
		
		var sX = segment.p1.x - positionX;
		var sY = segment.p1.y - positionY;
		
		var b = (sX*sX + sY*sY) - this.m_radius * this.m_radius;
		
		
		if (b < 0.0)
		{
			return false;
		}
		
		
		
		var rX = segment.p2.x - segment.p1.x;
		var rY = segment.p2.y - segment.p1.y;
		
		var c = (sX*rX + sY*rY);
		
		var rr = (rX*rX + rY*rY);
		var sigma = c * c - rr * b;
		
		
		if (sigma < 0.0 || rr < Number.MIN_VALUE)
		{
			return false;
		}
		
		
		var a = -(c + Math.sqrt(sigma));
		
		
		if (0.0 <= a && a <= maxLambda * rr)
		{
			a /= rr;
			
			lambda[0] = a;
			
			normal.x = sX + a * rX;
			normal.y = sY + a * rY;
			normal.Normalize();
			return true;
		}
		
		return false;
	}
b2CircleShape.prototype.ComputeAABB = function (aabb, transform) {
		
		var tMat = transform.R;
		var pX = transform.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		var pY = transform.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
		aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
	}
b2CircleShape.prototype.ComputeSweptAABB = function (	aabb,
							transform1,
							transform2) {
		var tMat;
		
		tMat = transform1.R;
		var p1X = transform1.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		var p1Y = transform1.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		
		tMat = transform2.R;
		var p2X = transform2.position.x + (tMat.col1.x * this.m_localPosition.x + tMat.col2.x * this.m_localPosition.y);
		var p2Y = transform2.position.y + (tMat.col1.y * this.m_localPosition.x + tMat.col2.y * this.m_localPosition.y);
		
		
		
		
		
		aabb.lowerBound.Set((p1X < p2X ? p1X : p2X) - this.m_radius, (p1Y < p2Y ? p1Y : p2Y) - this.m_radius);
		
		aabb.upperBound.Set((p1X > p2X ? p1X : p2X) + this.m_radius, (p1Y > p2Y ? p1Y : p2Y) + this.m_radius);
	}
b2CircleShape.prototype.ComputeMass = function (massData) {
		massData.mass = this.m_density * b2Settings.b2_pi * this.m_radius * this.m_radius;
		massData.center.SetV(this.m_localPosition);
		
		
		
		massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_localPosition.x*this.m_localPosition.x + this.m_localPosition.y*this.m_localPosition.y));
	}
b2CircleShape.prototype.GetLocalPosition = function () {
		return this.m_localPosition;
	}
b2CircleShape.prototype.GetRadius = function () {
		return this.m_radius;
	}
b2CircleShape.prototype.UpdateSweepRadius = function (center) {
		
		
		
		var dX = this.m_localPosition.x - center.x;
		var dY = this.m_localPosition.y - center.y;
		dX = Math.sqrt(dX*dX + dY*dY); 
		
		this.m_sweepRadius = dX + this.m_radius - b2Settings.b2_toiSlop;
	}