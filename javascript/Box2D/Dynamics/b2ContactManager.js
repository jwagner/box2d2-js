var b2ContactManager = function() {
b2PairCallback.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2ContactManager.prototype, b2PairCallback.prototype)
b2ContactManager.prototype._super = function(){ b2PairCallback.prototype.__constructor.apply(this, arguments) }
b2ContactManager.prototype.__constructor = function () {
		this.m_world = null;
		this.m_destroyImmediate = false;
	}
b2ContactManager.prototype.__varz = function(){
this.m_nullContact =  new b2NullContact();
}
// static attributes
b2ContactManager.s_evalCP =  new b2ContactPoint();
// static methods
// attributes
b2ContactManager.prototype.m_world =  null;
b2ContactManager.prototype.m_nullContact =  new b2NullContact();
b2ContactManager.prototype.m_destroyImmediate =  null;
// methods
b2ContactManager.prototype.PairAdded = function (proxyUserData1, proxyUserData2) {
		var shape1 = proxyUserData1;
		var shape2 = proxyUserData2;
		
		var body1 = shape1.m_body;
		var body2 = shape2.m_body;
		
		if (body1.IsStatic() && body2.IsStatic())
		{
			return this.m_nullContact;
		}
		
		if (shape1.m_body == shape2.m_body)
		{
			return this.m_nullContact;
		}
		
		if (body2.IsConnected(body1))
		{
			return this.m_nullContact;
		}
		
		if (this.m_world.m_contactFilter != null && this.m_world.m_contactFilter.ShouldCollide(shape1, shape2) == false)
		{
			return this.m_nullContact;
		}
		
		
		var c = b2Contact.Create(shape1, shape2, this.m_world.m_blockAllocator);
		
		if (c == null)
		{
			return this.m_nullContact;
		}
		
		
		shape1 = c.m_shape1;
		shape2 = c.m_shape2;
		body1 = shape1.m_body;
		body2 = shape2.m_body;
		
		
		c.m_prev = null;
		c.m_next = this.m_world.m_contactList;
		if (this.m_world.m_contactList != null)
		{
			this.m_world.m_contactList.m_prev = c;
		}
		this.m_world.m_contactList = c;
		
		
		
		
		
		c.m_node1.contact = c;
		c.m_node1.other = body2;
		
		c.m_node1.prev = null;
		c.m_node1.next = body1.m_contactList;
		if (body1.m_contactList != null)
		{
			body1.m_contactList.prev = c.m_node1;
		}
		body1.m_contactList = c.m_node1;
		
		
		c.m_node2.contact = c;
		c.m_node2.other = body1;
		
		c.m_node2.prev = null;
		c.m_node2.next = body2.m_contactList;
		if (body2.m_contactList != null)
		{
			body2.m_contactList.prev = c.m_node2;
		}
		body2.m_contactList = c.m_node2;
		
		++this.m_world.m_contactCount;
		return c;
		
	}
b2ContactManager.prototype.PairRemoved = function (proxyUserData1, proxyUserData2, pairUserData) {
		
		if (pairUserData == null)
		{
			return;
		}
		
		var c = pairUserData;
		if (c == this.m_nullContact)
		{
			return;
		}
		
		
		
		this.Destroy(c);
	}
b2ContactManager.prototype.Destroy = function (c) {
		
		var shape1 = c.m_shape1;
		var shape2 = c.m_shape2;
		
		
		var manifoldCount = c.m_manifoldCount;
		if (manifoldCount > 0 && this.m_world.m_contactListener)
		{
			var b1 = shape1.m_body;
			var b2 = shape2.m_body;

			var manifolds = c.GetManifolds();
			var cp = b2ContactManager.s_evalCP;
			cp.shape1 = c.m_shape1;
			cp.shape2 = c.m_shape2;
			cp.friction = c.m_friction;
			cp.restitution = c.m_restitution;
			
			for (var i = 0; i < manifoldCount; ++i)
			{
				var manifold = manifolds[ i ];
				cp.normal.SetV(manifold.normal);
				
				for (var j = 0; j < manifold.pointCount; ++j)
				{
					var mp = manifold.points[j];
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					var v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					var v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					this.m_world.m_contactListener.Remove(cp);
				}
			}
		}
		
		
		if (c.m_prev)
		{
			c.m_prev.m_next = c.m_next;
		}
		
		if (c.m_next)
		{
			c.m_next.m_prev = c.m_prev;
		}
		
		if (c == this.m_world.m_contactList)
		{
			this.m_world.m_contactList = c.m_next;
		}
		
		var body1 = shape1.m_body;
		var body2 = shape2.m_body;
		
		
		if (c.m_node1.prev)
		{
			c.m_node1.prev.next = c.m_node1.next;
		}
		
		if (c.m_node1.next)
		{
			c.m_node1.next.prev = c.m_node1.prev;
		}
		
		if (c.m_node1 == body1.m_contactList)
		{
			body1.m_contactList = c.m_node1.next;
		}
		
		
		if (c.m_node2.prev)
		{
			c.m_node2.prev.next = c.m_node2.next;
		}
		
		if (c.m_node2.next)
		{
			c.m_node2.next.prev = c.m_node2.prev;
		}
		
		if (c.m_node2 == body2.m_contactList)
		{
			body2.m_contactList = c.m_node2.next;
		}
		
		
		b2Contact.Destroy(c, this.m_world.m_blockAllocator);
		--this.m_world.m_contactCount;
	}
b2ContactManager.prototype.Collide = function () {
		
		for (var c = this.m_world.m_contactList; c; c = c.m_next)
		{
			var body1 = c.m_shape1.m_body;
			var body2 = c.m_shape2.m_body;
			if (body1.IsSleeping() && body2.IsSleeping())
			{
				continue;
			}
			
			c.Update(this.m_world.m_contactListener);
		}
	}