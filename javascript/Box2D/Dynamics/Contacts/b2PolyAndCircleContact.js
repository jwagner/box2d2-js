var b2PolyAndCircleContact = function() {
b2Contact.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PolyAndCircleContact.prototype, b2Contact.prototype)
b2PolyAndCircleContact.prototype._super = function(){ b2Contact.prototype.__constructor.apply(this, arguments) }
b2PolyAndCircleContact.prototype.__constructor = function (shape1, shape2) {
		this._super(shape1, shape2);
		
		this.m_manifold = this.m_manifolds[0];
		
		b2Settings.b2Assert(this.m_shape1.m_type == b2Shape.e_polygonShape);
		b2Settings.b2Assert(this.m_shape2.m_type == b2Shape.e_circleShape);
		this.m_manifold.pointCount = 0;
		var point = this.m_manifold.points[0];
		point.normalImpulse = 0.0;
		point.tangentImpulse = 0.0;
	}
b2PolyAndCircleContact.prototype.__varz = function(){
this.m_manifolds =  [new b2Manifold()];
this.m0 =  new b2Manifold();
}
// static attributes
b2PolyAndCircleContact.s_evalCP =  new b2ContactPoint();
// static methods
b2PolyAndCircleContact.Create = function (shape1, shape2, allocator) {
		return new b2PolyAndCircleContact(shape1, shape2);
	}
b2PolyAndCircleContact.Destroy = function (contact, allocator) {
	}
// attributes
b2PolyAndCircleContact.prototype.m_manifolds =  [new b2Manifold()];
b2PolyAndCircleContact.prototype.m0 =  new b2Manifold();
b2PolyAndCircleContact.prototype.m_manifold =  null;
// methods
b2PolyAndCircleContact.prototype.Evaluate = function (listener) {
		var i = 0;
		var v1;
		var v2;
		var mp0;
		
		var b1 = this.m_shape1.m_body;
		var b2 = this.m_shape2.m_body;
		
		
		
		
		this.m0.Set(this.m_manifold);
		
		b2Collision.b2CollidePolygonAndCircle(this.m_manifold, this.m_shape1, b1.m_xf, this.m_shape2, b2.m_xf);
		
		var persisted = [false, false];
		
		var cp = b2PolyAndCircleContact.s_evalCP;
		cp.shape1 = this.m_shape1;
		cp.shape2 = this.m_shape2;
		cp.friction = this.m_friction;
		cp.restitution = this.m_restitution;
		
		
		if (this.m_manifold.pointCount > 0)
		{
			
			
			for (i = 0; i < this.m_manifold.pointCount; ++i)
			{
				var mp = this.m_manifold.points[ i ];
				mp.normalImpulse = 0.0;
				mp.tangentImpulse = 0.0;
				var found = false;
				var idKey = mp.id._key;
	
				for (var j = 0; j < this.m0.pointCount; ++j)
				{
					if (persisted[j] == true)
					{
						continue;
					}
	
					mp0 = this.m0.points[ j ];
	
					if (mp0.id._key == idKey)
					{
						persisted[j] = true;
						mp.normalImpulse = mp0.normalImpulse;
						mp.tangentImpulse = mp0.tangentImpulse;
	
						
						found = true;
	
						
						if (listener != null)
						{
							cp.position = b1.GetWorldPoint(mp.localPoint1);
							v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
							v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
							cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
							cp.normal.SetV(this.m_manifold.normal);
							cp.separation = mp.separation;
							cp.id.key = idKey;
							listener.Persist(cp);
						}
						break;
					}
				}
	
				
				if (found == false && listener != null)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(this.m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = idKey;
					listener.Add(cp);
				}
			}
	
			this.m_manifoldCount = 1;
		}
		else
		{
			this.m_manifoldCount = 0;
		}
		
		if (listener == null)
		{
			return;
		}
		
		
		for (i = 0; i < this.m0.pointCount; ++i)
		{
			if (persisted[i])
			{
				continue;
			}
			
			mp0 = this.m0.points[ i ];
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
b2PolyAndCircleContact.prototype.GetManifolds = function () {
		return this.m_manifolds;
	}