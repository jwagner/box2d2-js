var b2Contact = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Contact.prototype.__constructor = function (s1, s2) {
		this.m_flags = 0;
		
		if (!s1 || !s2){
			this.m_shape1 = null;
			this.m_shape2 = null;
			return;
		}
		
		if (s1.IsSensor() || s2.IsSensor())
		{
			this.m_flags |= b2Contact.e_nonSolidFlag;
		}
		
		this.m_shape1 = s1;
		this.m_shape2 = s2;
		
		this.m_manifoldCount = 0;
		
		this.m_friction = Math.sqrt(this.m_shape1.m_friction * this.m_shape2.m_friction);
		this.m_restitution = b2Math.b2Max(this.m_shape1.m_restitution, this.m_shape2.m_restitution);
		
		this.m_prev = null;
		this.m_next = null;
		
		this.m_node1.contact = null;
		this.m_node1.prev = null;
		this.m_node1.next = null;
		this.m_node1.other = null;
		
		this.m_node2.contact = null;
		this.m_node2.prev = null;
		this.m_node2.next = null;
		this.m_node2.other = null;
	}
b2Contact.prototype.__varz = function(){
this.m_node1 =  new b2ContactEdge();
this.m_node2 =  new b2ContactEdge();
}
// static attributes
b2Contact.e_nonSolidFlag =  0x0001;
b2Contact.e_slowFlag =  0x0002;
b2Contact.e_islandFlag =  0x0004;
b2Contact.e_toiFlag =  0x0008;
b2Contact.s_registers =  null;
b2Contact.s_initialized =  false;
// static methods
b2Contact.AddType = function (createFcn, destroyFcn, type1, type2) {
		
		
		
		b2Contact.s_registers[type1][type2].createFcn = createFcn;
		b2Contact.s_registers[type1][type2].destroyFcn = destroyFcn;
		b2Contact.s_registers[type1][type2].primary = true;
		
		if (type1 != type2)
		{
			b2Contact.s_registers[type2][type1].createFcn = createFcn;
			b2Contact.s_registers[type2][type1].destroyFcn = destroyFcn;
			b2Contact.s_registers[type2][type1].primary = false;
		}
	}
b2Contact.InitializeRegisters = function () {
		b2Contact.s_registers = new Array(b2Shape.e_shapeTypeCount);
		for (var i = 0; i < b2Shape.e_shapeTypeCount; i++){
			b2Contact.s_registers[i] = new Array(b2Shape.e_shapeTypeCount);
			for (var j = 0; j < b2Shape.e_shapeTypeCount; j++){
				b2Contact.s_registers[i][j] = new b2ContactRegister();
			}
		}
		
		b2Contact.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		b2Contact.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		b2Contact.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
		
	}
b2Contact.Create = function (shape1, shape2, allocator) {
		if (b2Contact.s_initialized == false)
		{
			b2Contact.InitializeRegisters();
			b2Contact.s_initialized = true;
		}
		
		var type1 = shape1.m_type;
		var type2 = shape2.m_type;
		
		
		
		
		var reg = b2Contact.s_registers[type1][type2];
		var createFcn = reg.createFcn;
		if (createFcn != null)
		{
			if (reg.primary)
			{
				return createFcn(shape1, shape2, allocator);
			}
			else
			{
				var c = createFcn(shape2, shape1, allocator);
				for (var i = 0; i < c.m_manifoldCount; ++i)
				{
					var m = c.GetManifolds()[ i ];
					m.normal = m.normal.Negative();
				}
				return c;
			}
		}
		else
		{
			return null;
		}
	}
b2Contact.Destroy = function (contact, allocator) {
		
		
		if (contact.m_manifoldCount > 0)
		{
			contact.m_shape1.m_body.WakeUp();
			contact.m_shape2.m_body.WakeUp();
		}
		
		var type1 = contact.m_shape1.m_type;
		var type2 = contact.m_shape2.m_type;
		
		
		
		
		var reg = b2Contact.s_registers[type1][type2];
		var destroyFcn = reg.destroyFcn;
		destroyFcn(contact, allocator);
	}
// attributes
b2Contact.prototype.m_flags =  0;
b2Contact.prototype.m_prev =  null;
b2Contact.prototype.m_next =  null;
b2Contact.prototype.m_node1 =  new b2ContactEdge();
b2Contact.prototype.m_node2 =  new b2ContactEdge();
b2Contact.prototype.m_shape1 =  null;
b2Contact.prototype.m_shape2 =  null;
b2Contact.prototype.m_manifoldCount =  0;
b2Contact.prototype.m_friction =  null;
b2Contact.prototype.m_restitution =  null;
b2Contact.prototype.m_toi =  null;
// methods
b2Contact.prototype.GetManifolds = function () {return null}
b2Contact.prototype.GetManifoldCount = function () {
		return this.m_manifoldCount;
	}
b2Contact.prototype.IsSolid = function () {
		return (this.m_flags & b2Contact.e_nonSolidFlag) == 0;
	}
b2Contact.prototype.GetNext = function () {
		return this.m_next;
	}
b2Contact.prototype.GetShape1 = function () {
		return this.m_shape1;
	}
b2Contact.prototype.GetShape2 = function () {
		return this.m_shape2;
	}
b2Contact.prototype.Update = function (listener) {
		var oldCount = this.m_manifoldCount;
		
		this.Evaluate(listener);
		
		var newCount = this.m_manifoldCount;
		
		var body1 = this.m_shape1.m_body;
		var body2 = this.m_shape2.m_body;
		
		if (newCount == 0 && oldCount > 0)
		{
			body1.WakeUp();
			body2.WakeUp();
		}
		
		
		if (body1.IsStatic() || body1.IsBullet() || body2.IsStatic() || body2.IsBullet())
		{
			this.m_flags &= ~b2Contact.e_slowFlag;
		}
		else
		{
			this.m_flags |= b2Contact.e_slowFlag;
		}
	}
b2Contact.prototype.Evaluate = function (listener) {}