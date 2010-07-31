var b2Shape = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Shape.prototype.__constructor = function (def) {
		
		this.m_userData = def.userData;
		this.m_friction = def.friction;
		this.m_restitution = def.restitution;
		this.m_density = def.density;
		this.m_body = null;
		this.m_sweepRadius = 0.0;
		
		this.m_next = null;
		
		this.m_proxyId = b2Pair.b2_nullProxy;
		
		this.m_filter = def.filter.Copy();
		
		this.m_isSensor = def.isSensor;
		
	}
b2Shape.prototype.__varz = function(){
}
// static attributes
b2Shape.e_unknownShape =  	-1;
b2Shape.e_circleShape =  	0;
b2Shape.e_polygonShape =  	1;
b2Shape.e_shapeTypeCount =  	2;
b2Shape.s_proxyAABB =  new b2AABB();
b2Shape.s_syncAABB =  new b2AABB();
b2Shape.s_resetAABB =  new b2AABB();
// static methods
b2Shape.Create = function (def, allocator) {
		switch (def.type)
		{
		case b2Shape.e_circleShape:
			{
				
				return new b2CircleShape(def);
			}
		
		case b2Shape.e_polygonShape:
			{
				
				return new b2PolygonShape(def);
			}
		
		default:
			
			return null;
		}
	}
b2Shape.Destroy = function (shape, allocator) {
		
	}
// attributes
b2Shape.prototype.m_type =  0;
b2Shape.prototype.m_next =  null;
b2Shape.prototype.m_body =  null;
b2Shape.prototype.m_sweepRadius =  null;
b2Shape.prototype.m_density =  null;
b2Shape.prototype.m_friction =  null;
b2Shape.prototype.m_restitution =  null;
b2Shape.prototype.m_proxyId =  0;
b2Shape.prototype.m_filter =  null;
b2Shape.prototype.m_isSensor =  null;
b2Shape.prototype.m_userData =  null;
// methods
b2Shape.prototype.GetType = function () {
		return this.m_type;
	}
b2Shape.prototype.IsSensor = function () {
		return this.m_isSensor;
	}
b2Shape.prototype.SetFilterData = function (filter) {
		this.m_filter = filter.Copy();
	}
b2Shape.prototype.GetFilterData = function () {
		return this.m_filter.Copy();
	}
b2Shape.prototype.GetBody = function () {
		return this.m_body;
	}
b2Shape.prototype.GetNext = function () {
		return this.m_next;
	}
b2Shape.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Shape.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Shape.prototype.TestPoint = function (xf, p) {return false}
b2Shape.prototype.TestSegment = function (xf,
								lambda, 
								normal, 
								segment,
								maxLambda) {return false}
b2Shape.prototype.ComputeAABB = function (aabb, xf) {}
b2Shape.prototype.ComputeSweptAABB = function (	aabb,
									xf1,
									xf2) {}
b2Shape.prototype.ComputeMass = function (massData) {}
b2Shape.prototype.GetSweepRadius = function () {
		return this.m_sweepRadius;
	}
b2Shape.prototype.GetFriction = function () {
		return this.m_friction;
	}
b2Shape.prototype.GetRestitution = function () {
		return this.m_restitution;
	}
b2Shape.prototype.CreateProxy = function (broadPhase, transform) {
		
		
		
		var aabb = b2Shape.s_proxyAABB;
		this.ComputeAABB(aabb, transform);
		
		var inRange = broadPhase.InRange(aabb);
		
		
		
		
		if (inRange)
		{
			this.m_proxyId = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			this.m_proxyId = b2Pair.b2_nullProxy;
		}
		
	}
b2Shape.prototype.DestroyProxy = function (broadPhase) {
		
		if (this.m_proxyId != b2Pair.b2_nullProxy)
		{
			broadPhase.DestroyProxy(this.m_proxyId);
			this.m_proxyId = b2Pair.b2_nullProxy;
		}
		
	}
b2Shape.prototype.Synchronize = function (broadPhase, transform1, transform2) {
		
		if (this.m_proxyId == b2Pair.b2_nullProxy)
		{	
			return false;
		}
		
		
		var aabb = b2Shape.s_syncAABB;
		this.ComputeSweptAABB(aabb, transform1, transform2);
		
		if (broadPhase.InRange(aabb))
		{
			broadPhase.MoveProxy(this.m_proxyId, aabb);
			return true;
		}
		else
		{
			return false;
		}
		
	}
b2Shape.prototype.RefilterProxy = function (broadPhase, transform) {
		
		if (this.m_proxyId == b2Pair.b2_nullProxy)
		{
			return;
		}
		
		broadPhase.DestroyProxy(this.m_proxyId);
		
		var aabb = b2Shape.s_resetAABB;
		this.ComputeAABB(aabb, transform);
		
		var inRange = broadPhase.InRange(aabb);
		
		if (inRange)
		{
			this.m_proxyId = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			this.m_proxyId = b2Pair.b2_nullProxy;
		}
		
	}
b2Shape.prototype.UpdateSweepRadius = function (center) {}