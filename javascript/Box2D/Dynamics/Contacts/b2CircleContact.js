var b2CircleContact = function() {
b2Contact.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2CircleContact.prototype, b2Contact.prototype)
b2CircleContact.prototype._super = function(){ b2Contact.prototype.__constructor.apply(this, arguments) }
b2CircleContact.prototype.__constructor = function (shape1, shape2) {
		this._super(shape1, shape2);
		
		this.m_manifold = this.m_manifolds[0];
		
		
		
		this.m_manifold.pointCount = 0;
		var point = this.m_manifold.points[0];
		point.normalImpulse = 0.0;
		point.tangentImpulse = 0.0;
	}
b2CircleContact.prototype.__varz = function(){
this.m_manifolds =  [new b2Manifold()];
this.m0 =  new b2Manifold();
}
// static attributes
b2CircleContact.s_evalCP =  new b2ContactPoint();
// static methods
b2CircleContact.Create = function (shape1, shape2, allocator) {
		return new b2CircleContact(shape1, shape2);
	}
b2CircleContact.Destroy = function (contact, allocator) {
		
	}
// attributes
b2CircleContact.prototype.m_manifolds =  [new b2Manifold()];
b2CircleContact.prototype.m0 =  new b2Manifold();
b2CircleContact.prototype.m_manifold =  null;
// methods
b2CircleContact.prototype.Evaluate = function (listener) {
		var v1;
		var v2;
		var mp0;
		
		var b1 = this.m_shape1.m_body;
		var b2 = this.m_shape2.m_body;
		
		
		
		
		this.m0.Set(this.m_manifold);
		
		b2Collision.b2CollideCircles(this.m_manifold, this.m_shape1, b1.m_xf, this.m_shape2, b2.m_xf);
		
		var cp = b2CircleContact.s_evalCP;
		cp.shape1 = this.m_shape1;
		cp.shape2 = this.m_shape2;
		cp.friction = this.m_friction;
		cp.restitution = this.m_restitution;
		
		if (this.m_manifold.pointCount > 0)
		{
			this.m_manifoldCount = 1;
			var mp = this.m_manifold.points[ 0 ];
			
			if (this.m0.pointCount == 0)
			{
				mp.normalImpulse = 0.0;
				mp.tangentImpulse = 0.0;
	
				if (listener)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(this.m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					listener.Add(cp);
				}
			} else
			{
				mp0 = this.m0.points[ 0 ];
				mp.normalImpulse = mp0.normalImpulse;
				mp.tangentImpulse = mp0.tangentImpulse;
				
				if (listener)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(this.m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					listener.Persist(cp);
				}
			}
		}
		else
		{
			this.m_manifoldCount = 0;
			if (this.m0.pointCount > 0 && listener)
			{
				mp0 = this.m0.points[ 0 ];
				cp.position = b1.GetWorldPoint(mp0.localPoint1);
				v1 = b1.GetLinearVelocityFromLocalPoint(mp0.localPoint1);
				v2 = b2.GetLinearVelocityFromLocalPoint(mp0.localPoint2);
				cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
				cp.normal.SetV(this.m0.normal);
				cp.separation = mp0.separation;
				cp.id.key = mp0.id._key;
				listener.Remove(cp);
			}
		}
	}
b2CircleContact.prototype.GetManifolds = function () {
		return this.m_manifolds;
	}