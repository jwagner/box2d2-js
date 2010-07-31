var b2World = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2World.prototype.__constructor = function (worldAABB, gravity, doSleep) {
		
		this.m_destructionListener = null;
		this.m_boundaryListener = null;
		this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
		this.m_contactListener = null;
		this.m_debugDraw = null;
		
		this.m_bodyList = null;
		this.m_contactList = null;
		this.m_jointList = null;
		
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
		
		b2World.m_positionCorrection = true;
		b2World.m_warmStarting = true;
		b2World.m_continuousPhysics = true;
		
		this.m_allowSleep = doSleep;
		this.m_gravity = gravity;
		
		this.m_lock = false;
		
		this.m_inv_dt0 = 0.0;
		
		this.m_contactManager.m_world = this;
		
		this.m_broadPhase = new b2BroadPhase(worldAABB, this.m_contactManager);
		
		var bd = new b2BodyDef();
		this.m_groundBody = this.CreateBody(bd);
	}
b2World.prototype.__varz = function(){
this.m_contactManager =  new b2ContactManager();
}
// static attributes
b2World.m_positionCorrection =  null;
b2World.m_warmStarting =  null;
b2World.m_continuousPhysics =  null;
b2World.s_jointColor =  new b2Color(0.5, 0.8, 0.8);
b2World.s_coreColor =  new b2Color(0.9, 0.6, 0.6);
b2World.s_xf =  new b2XForm();
// static methods
// attributes
b2World.prototype.m_blockAllocator =  null;
b2World.prototype.m_stackAllocator =  null;
b2World.prototype.m_lock =  null;
b2World.prototype.m_broadPhase =  null;
b2World.prototype.m_contactManager =  new b2ContactManager();
b2World.prototype.m_bodyList =  null;
b2World.prototype.m_jointList =  null;
b2World.prototype.m_contactList =  null;
b2World.prototype.m_bodyCount =  0;
b2World.prototype.m_contactCount =  0;
b2World.prototype.m_jointCount =  0;
b2World.prototype.m_gravity =  null;
b2World.prototype.m_allowSleep =  null;
b2World.prototype.m_groundBody =  null;
b2World.prototype.m_destructionListener =  null;
b2World.prototype.m_boundaryListener =  null;
b2World.prototype.m_contactFilter =  null;
b2World.prototype.m_contactListener =  null;
b2World.prototype.m_debugDraw =  null;
b2World.prototype.m_inv_dt0 =  null;
b2World.prototype.m_positionIterationCount =  0;
// methods
b2World.prototype.SetDestructionListener = function (listener) {
		this.m_destructionListener = listener;
	}
b2World.prototype.SetBoundaryListener = function (listener) {
		this.m_boundaryListener = listener;
	}
b2World.prototype.SetContactFilter = function (filter) {
		this.m_contactFilter = filter;
	}
b2World.prototype.SetContactListener = function (listener) {
		this.m_contactListener = listener;
	}
b2World.prototype.SetDebugDraw = function (debugDraw) {
		this.m_debugDraw = debugDraw;
	}
b2World.prototype.Validate = function () {
		this.m_broadPhase.Validate();
	}
b2World.prototype.GetProxyCount = function () {
		return this.m_broadPhase.m_proxyCount;
	}
b2World.prototype.GetPairCount = function () {
		return this.m_broadPhase.m_pairManager.m_pairCount;
	}
b2World.prototype.CreateBody = function (def) {
		
		
		if (this.m_lock == true)
		{
			return null;
		}
		
		
		var b = new b2Body(def, this);
		
		
		b.m_prev = null;
		b.m_next = this.m_bodyList;
		if (this.m_bodyList)
		{
			this.m_bodyList.m_prev = b;
		}
		this.m_bodyList = b;
		++this.m_bodyCount;
		
		return b;
		
	}
b2World.prototype.DestroyBody = function (b) {
		
		
		
		if (this.m_lock == true)
		{
			return;
		}
		
		
		var jn = b.m_jointList;
		while (jn)
		{
			var jn0 = jn;
			jn = jn.next;
			
			if (this.m_destructionListener)
			{
				this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
			}
			
			this.DestroyJoint(jn0.joint);
		}
		
		
		
		var s = b.m_shapeList;
		while (s)
		{
			var s0 = s;
			s = s.m_next;
			
			if (this.m_destructionListener)
			{
				this.m_destructionListener.SayGoodbyeShape(s0);
			}
			
			s0.DestroyProxy(this.m_broadPhase);
			b2Shape.Destroy(s0, this.m_blockAllocator);
		}
		
		
		if (b.m_prev)
		{
			b.m_prev.m_next = b.m_next;
		}
		
		if (b.m_next)
		{
			b.m_next.m_prev = b.m_prev;
		}
		
		if (b == this.m_bodyList)
		{
			this.m_bodyList = b.m_next;
		}
		
		--this.m_bodyCount;
		
		
		
	}
b2World.prototype.CreateJoint = function (def) {
		
		
		
		var j = b2Joint.Create(def, this.m_blockAllocator);
		
		
		j.m_prev = null;
		j.m_next = this.m_jointList;
		if (this.m_jointList)
		{
			this.m_jointList.m_prev = j;
		}
		this.m_jointList = j;
		++this.m_jointCount;
		
		
		j.m_node1.joint = j;
		j.m_node1.other = j.m_body2;
		j.m_node1.prev = null;
		j.m_node1.next = j.m_body1.m_jointList;
		if (j.m_body1.m_jointList) j.m_body1.m_jointList.prev = j.m_node1;
		j.m_body1.m_jointList = j.m_node1;
		
		j.m_node2.joint = j;
		j.m_node2.other = j.m_body1;
		j.m_node2.prev = null;
		j.m_node2.next = j.m_body2.m_jointList;
		if (j.m_body2.m_jointList) j.m_body2.m_jointList.prev = j.m_node2;
		j.m_body2.m_jointList = j.m_node2;
		
		
		if (def.collideConnected == false)
		{
			
			var b = def.body1.m_shapeCount < def.body2.m_shapeCount ? def.body1 : def.body2;
			for (var s = b.m_shapeList; s; s = s.m_next)
			{
				s.RefilterProxy(this.m_broadPhase, b.m_xf);
			}
		}
		
		return j;
		
	}
b2World.prototype.DestroyJoint = function (j) {
		
		
		
		var collideConnected = j.m_collideConnected;
		
		
		if (j.m_prev)
		{
			j.m_prev.m_next = j.m_next;
		}
		
		if (j.m_next)
		{
			j.m_next.m_prev = j.m_prev;
		}
		
		if (j == this.m_jointList)
		{
			this.m_jointList = j.m_next;
		}
		
		
		var body1 = j.m_body1;
		var body2 = j.m_body2;
		
		
		body1.WakeUp();
		body2.WakeUp();
		
		
		if (j.m_node1.prev)
		{
			j.m_node1.prev.next = j.m_node1.next;
		}
		
		if (j.m_node1.next)
		{
			j.m_node1.next.prev = j.m_node1.prev;
		}
		
		if (j.m_node1 == body1.m_jointList)
		{
			body1.m_jointList = j.m_node1.next;
		}
		
		j.m_node1.prev = null;
		j.m_node1.next = null;
		
		
		if (j.m_node2.prev)
		{
			j.m_node2.prev.next = j.m_node2.next;
		}
		
		if (j.m_node2.next)
		{
			j.m_node2.next.prev = j.m_node2.prev;
		}
		
		if (j.m_node2 == body2.m_jointList)
		{
			body2.m_jointList = j.m_node2.next;
		}
		
		j.m_node2.prev = null;
		j.m_node2.next = null;
		
		b2Joint.Destroy(j, this.m_blockAllocator);
		
		
		--this.m_jointCount;
		
		
		if (collideConnected == false)
		{
			
			var b = body1.m_shapeCount < body2.m_shapeCount ? body1 : body2;
			for (var s = b.m_shapeList; s; s = s.m_next)
			{
				s.RefilterProxy(this.m_broadPhase, b.m_xf);
			}
		}
		
	}
b2World.prototype.Refilter = function (shape) {
		shape.RefilterProxy(this.m_broadPhase, shape.m_body.m_xf);
	}
b2World.prototype.SetWarmStarting = function (flag) { b2World.m_warmStarting = flag; }
b2World.prototype.SetPositionCorrection = function (flag) { b2World.m_positionCorrection = flag; }
b2World.prototype.SetContinuousPhysics = function (flag) { b2World.m_continuousPhysics = flag; }
b2World.prototype.GetBodyCount = function () {
		return this.m_bodyCount;
	}
b2World.prototype.GetJointCount = function () {
		return this.m_jointCount;
	}
b2World.prototype.GetContactCount = function () {
		return this.m_contactCount;
	}
b2World.prototype.SetGravity = function (gravity) {
		this.m_gravity = gravity;
	}
b2World.prototype.GetGroundBody = function () {
		return this.m_groundBody;
	}
b2World.prototype.Step = function (dt, iterations) {
		
		this.m_lock = true;
		
		var step = new b2TimeStep();
		step.dt = dt;
		step.maxIterations	= iterations;
		if (dt > 0.0)
		{
			step.inv_dt = 1.0 / dt;
		}
		else
		{
			step.inv_dt = 0.0;
		}
		
		step.dtRatio = this.m_inv_dt0 * dt;
		
		step.positionCorrection = b2World.m_positionCorrection;
		step.warmStarting = b2World.m_warmStarting;
		
		
		this.m_contactManager.Collide();
		
		
		if (step.dt > 0.0)
		{
			this.Solve(step);
		}
		
		
		if (b2World.m_continuousPhysics && step.dt > 0.0)
		{
			this.SolveTOI(step);
		}
		
		
		this.DrawDebugData();
		
		this.m_inv_dt0 = step.inv_dt;
		this.m_lock = false;
	}
b2World.prototype.Query = function (aabb, shapes, maxCount) {
		
		
		var results = new Array(maxCount);
		
		var count = this.m_broadPhase.QueryAABB(aabb, results, maxCount);
		
		for (var i = 0; i < count; ++i)
		{
			shapes[i] = results[i];
		}
		
		
		return count;
		
	}
b2World.prototype.GetBodyList = function () {
		return this.m_bodyList;
	}
b2World.prototype.GetJointList = function () {
		return this.m_jointList;
	}
b2World.prototype.Solve = function (step) {
		
		var b;
		
		this.m_positionIterationCount = 0;
		
		
		var island = new b2Island(this.m_bodyCount, this.m_contactCount, this.m_jointCount, this.m_stackAllocator, this.m_contactListener);
		
		
		for (b = this.m_bodyList; b; b = b.m_next)
		{
			b.m_flags &= ~b2Body.e_islandFlag;
		}
		for (var c = this.m_contactList; c; c = c.m_next)
		{
			c.m_flags &= ~b2Contact.e_islandFlag;
		}
		for (var j = this.m_jointList; j; j = j.m_next)
		{
			j.m_islandFlag = false;
		}
		
		
		var stackSize = this.m_bodyCount;
		
		var stack = new Array(stackSize);
		for (var seed = this.m_bodyList; seed; seed = seed.m_next)
		{
			if (seed.m_flags & (b2Body.e_islandFlag | b2Body.e_sleepFlag | b2Body.e_frozenFlag))
			{
				continue;
			}
			
			if (seed.IsStatic())
			{
				continue;
			}
			
			
			island.Clear();
			var stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;
			
			
			while (stackCount > 0)
			{
				
				b = stack[--stackCount];
				island.AddBody(b);
				
				
				b.m_flags &= ~b2Body.e_sleepFlag;
				
				
				
				if (b.IsStatic())
				{
					continue;
				}
				
				var other;
				
				for (var cn = b.m_contactList; cn; cn = cn.next)
				{
					
					if (cn.contact.m_flags & (b2Contact.e_islandFlag | b2Contact.e_nonSolidFlag))
					{
						continue;
					}
					
					
					if (cn.contact.m_manifoldCount == 0)
					{
						continue;
					}
					
					island.AddContact(cn.contact);
					cn.contact.m_flags |= b2Contact.e_islandFlag;
					
					
					other = cn.other;
					
					
					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}
					
					
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
				
				
				for (var jn = b.m_jointList; jn; jn = jn.next)
				{
					if (jn.joint.m_islandFlag == true)
					{
						continue;
					}
					
					island.AddJoint(jn.joint);
					jn.joint.m_islandFlag = true;
					
					
					other = jn.other;
					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}
					
					
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}
			
			island.Solve(step, this.m_gravity, b2World.m_positionCorrection, this.m_allowSleep);
			
			if (island.m_positionIterationCount > this.m_positionIterationCount) {
				this.m_positionIterationCount = island.m_positionIterationCount;
			}
			
			
			for (var i = 0; i < island.m_bodyCount; ++i)
			{
				
				b = island.m_bodies[i];
				if (b.IsStatic())
				{
					b.m_flags &= ~b2Body.e_islandFlag;
				}
			}
		}
		
		
		
		
		for (b = this.m_bodyList; b; b = b.m_next)
		{
			if (b.m_flags & (b2Body.e_sleepFlag | b2Body.e_frozenFlag))
			{
				continue;
			}
			
			if (b.IsStatic())
			{
				continue;
			}
			
			
			
			
			var inRange = b.SynchronizeShapes();
			
			
			if (inRange == false && this.m_boundaryListener != null)
			{
				this.m_boundaryListener.Violation(b);
			}
		}
		
		
		
		this.m_broadPhase.Commit();
		
	}
b2World.prototype.SolveTOI = function (step) {
		
		var b;
		var s1;
		var s2;
		var b1;
		var b2;
		var cn;
		
		
		var island = new b2Island(this.m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, 0, this.m_stackAllocator, this.m_contactListener);
		var stackSize = this.m_bodyCount;
		
		
		var stack = new Array(stackSize);
		
		for (b = this.m_bodyList; b; b = b.m_next)
		{
			b.m_flags &= ~b2Body.e_islandFlag;
			b.m_sweep.t0 = 0.0;
		}
		
		var c;
		for (c = this.m_contactList; c; c = c.m_next)
		{
			
			c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
		}
		
		
		for (;;)
		{
			
			var minContact = null;
			var minTOI = 1.0;
			
			for (c = this.m_contactList; c; c = c.m_next)
			{
				if (c.m_flags & (b2Contact.e_slowFlag | b2Contact.e_nonSolidFlag))
				{
					continue;
				}
				
				
				
				var toi = 1.0;
				if (c.m_flags & b2Contact.e_toiFlag)
				{
					
					toi = c.m_toi;
				}
				else
				{
					
					s1 = c.m_shape1;
					s2 = c.m_shape2;
					b1 = s1.m_body;
					b2 = s2.m_body;
					
					if ((b1.IsStatic() || b1.IsSleeping()) && (b2.IsStatic() || b2.IsSleeping()))
					{
						continue;
					}
					
					
					var t0 = b1.m_sweep.t0;
					
					if (b1.m_sweep.t0 < b2.m_sweep.t0)
					{
						t0 = b2.m_sweep.t0;
						b1.m_sweep.Advance(t0);
					}
					else if (b2.m_sweep.t0 < b1.m_sweep.t0)
					{
						t0 = b1.m_sweep.t0;
						b2.m_sweep.Advance(t0);
					}
					
					
					
					
					toi = b2TimeOfImpact.TimeOfImpact(c.m_shape1, b1.m_sweep, c.m_shape2, b2.m_sweep);
					
					
					if (toi > 0.0 && toi < 1.0)
					{
						
						toi = (1.0 - toi) * t0 + toi;
						if (toi > 1) toi = 1;
					}
					
					
					c.m_toi = toi;
					c.m_flags |= b2Contact.e_toiFlag;
				}
				
				if (Number.MIN_VALUE < toi && toi < minTOI)
				{
					
					minContact = c;
					minTOI = toi;
				}
			}
			
			if (minContact == null || 1.0 - 100.0 * Number.MIN_VALUE < minTOI)
			{
				
				break;
			}
			
			
			s1 = minContact.m_shape1;
			s2 = minContact.m_shape2;
			b1 = s1.m_body;
			b2 = s2.m_body;
			b1.Advance(minTOI);
			b2.Advance(minTOI);
			
			
			minContact.Update(this.m_contactListener);
			minContact.m_flags &= ~b2Contact.e_toiFlag;
			
			if (minContact.m_manifoldCount == 0)
			{
				
				
				continue;
			}
			
			
			var seed = b1;
			if (seed.IsStatic())
			{
				seed = b2;
			}
			
			
			island.Clear();
			var stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;
			
			
			while (stackCount > 0)
			{
				
				b = stack[--stackCount];
				island.AddBody(b);
				
				
				b.m_flags &= ~b2Body.e_sleepFlag;
				
				
				
				if (b.IsStatic())
				{
					continue;
				}
				
				
				for (cn = b.m_contactList; cn; cn = cn.next)
				{
					
					if (island.m_contactCount == island.m_contactCapacity)
					{
						continue;
					}
					
					
					if (cn.contact.m_flags & (b2Contact.e_islandFlag | b2Contact.e_slowFlag | b2Contact.e_nonSolidFlag))
					{
						continue;
					}
					
					
					if (cn.contact.m_manifoldCount == 0)
					{
						continue;
					}
					
					island.AddContact(cn.contact);
					cn.contact.m_flags |= b2Contact.e_islandFlag;
					
					
					var other = cn.other;
					
					
					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}
					
					
					if (other.IsStatic() == false)
					{
						other.Advance(minTOI);
						other.WakeUp();
					}
					
					
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}
			
			var subStep = new b2TimeStep();
			subStep.dt = (1.0 - minTOI) * step.dt;
			
			subStep.inv_dt = 1.0 / subStep.dt;
			subStep.maxIterations = step.maxIterations;
			
			island.SolveTOI(subStep);
			
			var i = 0;
			
			for (i = 0; i < island.m_bodyCount; ++i)
			{
				
				b = island.m_bodies[i];
				b.m_flags &= ~b2Body.e_islandFlag;
				
				if (b.m_flags & (b2Body.e_sleepFlag | b2Body.e_frozenFlag))
				{
					continue;
				}
				
				if (b.IsStatic())
				{
					continue;
				}
				
				
				
				
				var inRange = b.SynchronizeShapes();
				
				
				if (inRange == false && this.m_boundaryListener != null)
				{
					this.m_boundaryListener.Violation(b);
				}
				
				
				
				for (cn = b.m_contactList; cn; cn = cn.next)
				{
					cn.contact.m_flags &= ~b2Contact.e_toiFlag;
				}
			}
			
			for (i = 0; i < island.m_contactCount; ++i)
			{
				
				c = island.m_contacts[i];
				c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
			}
			
			
			
			this.m_broadPhase.Commit();
		}
		
		
		
	}
b2World.prototype.DrawJoint = function (joint) {
		
		var b1 = joint.m_body1;
		var b2 = joint.m_body2;
		var xf1 = b1.m_xf;
		var xf2 = b2.m_xf;
		var x1 = xf1.position;
		var x2 = xf2.position;
		var p1 = joint.GetAnchor1();
		var p2 = joint.GetAnchor2();
		
		
		var color = b2World.s_jointColor;
		
		switch (joint.m_type)
		{
		case b2Joint.e_distanceJoint:
			this.m_debugDraw.DrawSegment(p1, p2, color);
			break;
		
		case b2Joint.e_pulleyJoint:
			{
				var pulley = (joint);
				var s1 = pulley.GetGroundAnchor1();
				var s2 = pulley.GetGroundAnchor2();
				this.m_debugDraw.DrawSegment(s1, p1, color);
				this.m_debugDraw.DrawSegment(s2, p2, color);
				this.m_debugDraw.DrawSegment(s1, s2, color);
			}
			break;
		
		case b2Joint.e_mouseJoint:
			this.m_debugDraw.DrawSegment(p1, p2, color);
			break;
		
		default:
			if (b1 != this.m_groundBody)
				this.m_debugDraw.DrawSegment(x1, p1, color);
			this.m_debugDraw.DrawSegment(p1, p2, color);
			if (b2 != this.m_groundBody)
				this.m_debugDraw.DrawSegment(x2, p2, color);
		}
	}
b2World.prototype.DrawShape = function (shape, xf, color, core) {
		
		var coreColor = b2World.s_coreColor;
		
		switch (shape.m_type)
		{
		case b2Shape.e_circleShape:
			{
				var circle = (shape);
				
				var center = b2Math.b2MulX(xf, circle.m_localPosition);
				var radius = circle.m_radius;
				var axis = xf.R.col1;
				
				this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
				
				if (core)
				{
					this.m_debugDraw.DrawCircle(center, radius - b2Settings.b2_toiSlop, coreColor);
				}
			}
			break;
		
		case b2Shape.e_polygonShape:
			{
				var i = 0;
				var poly = (shape);
				var vertexCount = poly.GetVertexCount();
				var localVertices = poly.GetVertices();
				
				
				var vertices = new Array(b2Settings.b2_maxPolygonVertices);
				
				for (i = 0; i < vertexCount; ++i)
				{
					vertices[i] = b2Math.b2MulX(xf, localVertices[i]);
				}
				
				this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
				
				if (core)
				{
					var localCoreVertices = poly.GetCoreVertices();
					for (i = 0; i < vertexCount; ++i)
					{
						vertices[i] = b2Math.b2MulX(xf, localCoreVertices[i]);
					}
					this.m_debugDraw.DrawPolygon(vertices, vertexCount, coreColor);
				}
			}
			break;
		}
	}
b2World.prototype.DrawDebugData = function () {
		
		if (this.m_debugDraw == null)
		{
			return;
		}
		
		this.m_debugDraw.m_sprite.graphics.clear();
		
		var flags = this.m_debugDraw.GetFlags();
		
		var i = 0;
		var b;
		var s;
		var j;
		var bp;
		var invQ = new b2Vec2;
		var x1 = new b2Vec2;
		var x2 = new b2Vec2;
		var color = new b2Color(0,0,0);
		var xf;
		var b1 = new b2AABB();
		var b2 = new b2AABB();
		var vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
		
		if (flags & b2DebugDraw.e_shapeBit)
		{
			var core = (flags & b2DebugDraw.e_coreShapeBit) == b2DebugDraw.e_coreShapeBit;
			
			for (b = this.m_bodyList; b; b = b.m_next)
			{
				xf = b.m_xf;
				for (s = b.GetShapeList(); s; s = s.m_next)
				{
					if (b.IsStatic())
					{
						this.DrawShape(s, xf, new b2Color(0.5, 0.9, 0.5), core);
					}
					else if (b.IsSleeping())
					{
						this.DrawShape(s, xf, new b2Color(0.5, 0.5, 0.9), core);
					}
					else
					{
						this.DrawShape(s, xf, new b2Color(0.9, 0.9, 0.9), core);
					}
				}
			}
		}
		
		if (flags & b2DebugDraw.e_jointBit)
		{
			for (j = this.m_jointList; j; j = j.m_next)
			{
				
				
					this.DrawJoint(j);
				
			}
		}
		
		if (flags & b2DebugDraw.e_pairBit)
		{
			bp = this.m_broadPhase;
			
			invQ.Set(1.0 / bp.m_quantizationFactor.x, 1.0 / bp.m_quantizationFactor.y);
			
			color.Set(0.9, 0.9, 0.3);
			
			for (i = 0; i < b2Pair.b2_tableCapacity; ++i)
			{
				var index = bp.m_pairManager.m_hashTable[i];
				while (index != b2Pair.b2_nullPair)
				{
					var pair = bp.m_pairManager.m_pairs[ index ];
					var p1 = bp.m_proxyPool[ pair.proxyId1 ];
					var p2 = bp.m_proxyPool[ pair.proxyId2 ];
					
					
					b1.lowerBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p1.lowerBounds[0]].value;
					b1.lowerBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p1.lowerBounds[1]].value;
					b1.upperBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p1.upperBounds[0]].value;
					b1.upperBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p1.upperBounds[1]].value;
					b2.lowerBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p2.lowerBounds[0]].value;
					b2.lowerBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p2.lowerBounds[1]].value;
					b2.upperBound.x = bp.m_worldAABB.lowerBound.x + invQ.x * bp.m_bounds[0][p2.upperBounds[0]].value;
					b2.upperBound.y = bp.m_worldAABB.lowerBound.y + invQ.y * bp.m_bounds[1][p2.upperBounds[1]].value;
					
					
					x1.x = 0.5 * (b1.lowerBound.x + b1.upperBound.x);
					x1.y = 0.5 * (b1.lowerBound.y + b1.upperBound.y);
					
					x2.x = 0.5 * (b2.lowerBound.x + b2.upperBound.x);
					x2.y = 0.5 * (b2.lowerBound.y + b2.upperBound.y);
					
					this.m_debugDraw.DrawSegment(x1, x2, color);
					
					index = pair.next;
				}
			}
		}
		
		if (flags & b2DebugDraw.e_aabbBit)
		{
			bp = this.m_broadPhase;
			var worldLower = bp.m_worldAABB.lowerBound;
			var worldUpper = bp.m_worldAABB.upperBound;
			
			
			invQ.Set(1.0 / bp.m_quantizationFactor.x, 1.0 / bp.m_quantizationFactor.y);
			
			color.Set(0.9, 0.3, 0.9);
			for (i = 0; i < b2Settings.b2_maxProxies; ++i)
			{
				var p = bp.m_proxyPool[ i];
				if (p.IsValid() == false)
				{
					continue;
				}
				
				
				b1.lowerBound.x = worldLower.x + invQ.x * bp.m_bounds[0][p.lowerBounds[0]].value;
				b1.lowerBound.y = worldLower.y + invQ.y * bp.m_bounds[1][p.lowerBounds[1]].value;
				b1.upperBound.x = worldLower.x + invQ.x * bp.m_bounds[0][p.upperBounds[0]].value;
				b1.upperBound.y = worldLower.y + invQ.y * bp.m_bounds[1][p.upperBounds[1]].value;
				
				
				vs[0].Set(b1.lowerBound.x, b1.lowerBound.y);
				vs[1].Set(b1.upperBound.x, b1.lowerBound.y);
				vs[2].Set(b1.upperBound.x, b1.upperBound.y);
				vs[3].Set(b1.lowerBound.x, b1.upperBound.y);
				
				this.m_debugDraw.DrawPolygon(vs, 4, color);
			}
			
			
			vs[0].Set(worldLower.x, worldLower.y);
			vs[1].Set(worldUpper.x, worldLower.y);
			vs[2].Set(worldUpper.x, worldUpper.y);
			vs[3].Set(worldLower.x, worldUpper.y);
			this.m_debugDraw.DrawPolygon(vs, 4, new b2Color(0.3, 0.9, 0.9));
		}
		
		if (flags & b2DebugDraw.e_obbBit)
		{
			
			color.Set(0.5, 0.3, 0.5);
			
			for (b = this.m_bodyList; b; b = b.m_next)
			{
				xf = b.m_xf;
				for (s = b.GetShapeList(); s; s = s.m_next)
				{
					if (s.m_type != b2Shape.e_polygonShape)
					{
						continue;
					}
					
					var poly = (s);
					var obb = poly.GetOBB();
					var h = obb.extents;
					
					vs[0].Set(-h.x, -h.y);
					vs[1].Set( h.x, -h.y);
					vs[2].Set( h.x, h.y);
					vs[3].Set(-h.x, h.y);
					
					for (i = 0; i < 4; ++i)
					{
						
						var tMat = obb.R;
						var tVec = vs[i];
						var tX;
						tX = obb.center.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
						vs[i].y = obb.center.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
						vs[i].x = tX;
						
						tMat = xf.R;
						tX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
						vs[i].y = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
						vs[i].x = tX;
					}
					
					this.m_debugDraw.DrawPolygon(vs, 4, color);
				}
			}
		}
		
		if (flags & b2DebugDraw.e_centerOfMassBit)
		{
			for (b = this.m_bodyList; b; b = b.m_next)
			{
				xf = b2World.s_xf;
				xf.R = b.m_xf.R;
				xf.position = b.GetWorldCenter();
				this.m_debugDraw.DrawXForm(xf);
			}
		}
	}