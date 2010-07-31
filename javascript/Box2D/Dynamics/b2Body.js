var b2Body = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Body.prototype.__constructor = function (bd, world) {
		
		
		this.m_flags = 0;
		
		if (bd.isBullet)
		{
			this.m_flags |= b2Body.e_bulletFlag;
		}
		if (bd.fixedRotation)
		{
			this.m_flags |= b2Body.e_fixedRotationFlag;
		}
		if (bd.allowSleep)
		{
			this.m_flags |= b2Body.e_allowSleepFlag;
		}
		if (bd.isSleeping)
		{
			this.m_flags |= b2Body.e_sleepFlag;
		}
		
		this.m_world = world;
		
		this.m_xf.position.SetV(bd.position);
		this.m_xf.R.Set(bd.angle);
		
		this.m_sweep.localCenter.SetV(bd.massData.center);
		this.m_sweep.t0 = 1.0;
		this.m_sweep.a0 = this.m_sweep.a = bd.angle;
		
		
		
		var tMat = this.m_xf.R;
		var tVec = this.m_sweep.localCenter;
		
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		
		this.m_sweep.c0.SetV(this.m_sweep.c);
		
		this.m_jointList = null;
		this.m_contactList = null;
		this.m_prev = null;
		this.m_next = null;
		
		this.m_linearDamping = bd.linearDamping;
		this.m_angularDamping = bd.angularDamping;
		
		this.m_force.Set(0.0, 0.0);
		this.m_torque = 0.0;
		
		this.m_linearVelocity.SetZero();
		this.m_angularVelocity = 0.0;
		
		this.m_sleepTime = 0.0;
		
		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;
		
		this.m_mass = bd.massData.mass;
		
		if (this.m_mass > 0.0)
		{
			this.m_invMass = 1.0 / this.m_mass;
		}
		
		if ((this.m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			this.m_I = bd.massData.I;
		}
		
		if (this.m_I > 0.0)
		{
			this.m_invI = 1.0 / this.m_I;
		}
		
		if (this.m_invMass == 0.0 && this.m_invI == 0.0)
		{
			this.m_type = b2Body.e_staticType;
		}
		else
		{
			this.m_type = b2Body.e_dynamicType;
		}
	
		this.m_userData = bd.userData;
		
		this.m_shapeList = null;
		this.m_shapeCount = 0;
	}
b2Body.prototype.__varz = function(){
this.m_xf =  new b2XForm();
this.m_sweep =  new b2Sweep();
this.m_linearVelocity =  new b2Vec2();
this.m_force =  new b2Vec2();
}
// static attributes
b2Body.e_frozenFlag =  0x0002;
b2Body.e_islandFlag =  0x0004;
b2Body.e_sleepFlag =  0x0008;
b2Body.e_allowSleepFlag =  0x0010;
b2Body.e_bulletFlag =  0x0020;
b2Body.e_fixedRotationFlag =  0x0040;
b2Body.e_staticType =  1;
b2Body.e_dynamicType =  2;
b2Body.e_maxTypes =  3;
b2Body.s_massData =  new b2MassData();
b2Body.s_xf1 =  new b2XForm();
// static methods
// attributes
b2Body.prototype.m_flags =  0;
b2Body.prototype.m_type =  0;
b2Body.prototype.m_xf =  new b2XForm();
b2Body.prototype.m_sweep =  new b2Sweep();
b2Body.prototype.m_linearVelocity =  new b2Vec2();
b2Body.prototype.m_angularVelocity =  null;
b2Body.prototype.m_force =  new b2Vec2();
b2Body.prototype.m_torque =  null;
b2Body.prototype.m_world =  null;
b2Body.prototype.m_prev =  null;
b2Body.prototype.m_next =  null;
b2Body.prototype.m_shapeList =  null;
b2Body.prototype.m_shapeCount =  0;
b2Body.prototype.m_jointList =  null;
b2Body.prototype.m_contactList =  null;
b2Body.prototype.m_mass =  null;
b2Body.prototype.m_invMass =  null;
b2Body.prototype.m_I =  null;
b2Body.prototype.m_invI =  null;
b2Body.prototype.m_linearDamping =  null;
b2Body.prototype.m_angularDamping =  null;
b2Body.prototype.m_sleepTime =  null;
b2Body.prototype.m_userData =  null;
// methods
b2Body.prototype.CreateShape = function (def) {
		
		if (this.m_world.m_lock == true)
		{
			return null;
		}
		
		var s = b2Shape.Create(def, this.m_world.m_blockAllocator);
		
		s.m_next = this.m_shapeList;
		this.m_shapeList = s;
		++this.m_shapeCount;
		
		s.m_body = this;
		
		
		s.CreateProxy(this.m_world.m_broadPhase, this.m_xf);
		
		
		s.UpdateSweepRadius(this.m_sweep.localCenter);
		
		return s;
	}
b2Body.prototype.DestroyShape = function (s) {
		
		if (this.m_world.m_lock == true)
		{
			return;
		}
		
		
		s.DestroyProxy(this.m_world.m_broadPhase);
		
		
		
		var node = this.m_shapeList;
		var ppS = null; 
		var found = false;
		while (node != null)
		{
			if (node == s)
			{
				if (ppS)
					ppS.m_next = s.m_next;
				else
					this.m_shapeList = s.m_next;
				
				found = true;
				break;
			}
			
			ppS = node;
			node = node.m_next;
		}
		
		
		
		
		s.m_body = null;
		s.m_next = null;
		
		--this.m_shapeCount;
		
		b2Shape.Destroy(s, this.m_world.m_blockAllocator);
	}
b2Body.prototype.SetMass = function (massData) {
		var s;
		
		
		if (this.m_world.m_lock == true)
		{
			return;
		}
		
		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;
		
		this.m_mass = massData.mass;
		
		if (this.m_mass > 0.0)
		{
			this.m_invMass = 1.0 / this.m_mass;
		}
		
		if ((this.m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			this.m_I = massData.I;
		}
		
		if (this.m_I > 0.0)
		{
			this.m_invI = 1.0 / this.m_I;
		}
		
		
		this.m_sweep.localCenter.SetV(massData.center);
		
		
		var tMat = this.m_xf.R;
		var tVec = this.m_sweep.localCenter;
		
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		
		this.m_sweep.c0.SetV(this.m_sweep.c);
		
		
		for (s = this.m_shapeList; s; s = s.m_next)
		{
			s.UpdateSweepRadius(this.m_sweep.localCenter);
		}

		var oldType = this.m_type;
		if (this.m_invMass == 0.0 && this.m_invI == 0.0)
		{
			this.m_type = b2Body.e_staticType;
		}
		else
		{
			this.m_type = b2Body.e_dynamicType;
		}
	
		
		if (oldType != this.m_type)
		{
			for (s = this.m_shapeList; s; s = s.m_next)
			{
				s.RefilterProxy(this.m_world.m_broadPhase, this.m_xf);
			}
		}
	}
b2Body.prototype.SetMassFromShapes = function () {
		
		var s;
		
		
		if (this.m_world.m_lock == true)
		{
			return;
		}
		
		
		this.m_mass = 0.0;
		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;
		
		
		var centerX = 0.0;
		var centerY = 0.0;
		var massData = b2Body.s_massData;
		for (s = this.m_shapeList; s; s = s.m_next)
		{
			s.ComputeMass(massData);
			this.m_mass += massData.mass;
			
			centerX += massData.mass * massData.center.x;
			centerY += massData.mass * massData.center.y;
			this.m_I += massData.I;
		}
		
		
		if (this.m_mass > 0.0)
		{
			this.m_invMass = 1.0 / this.m_mass;
			centerX *= this.m_invMass;
			centerY *= this.m_invMass;
		}
		
		if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			
			
			this.m_I -= this.m_mass * (centerX * centerX + centerY * centerY);
			
			this.m_invI = 1.0 / this.m_I;
		}
		else
		{
			this.m_I = 0.0;
			this.m_invI = 0.0;
		}
		
		
		this.m_sweep.localCenter.Set(centerX, centerY);
		
		
		var tMat = this.m_xf.R;
		var tVec = this.m_sweep.localCenter;
		
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		
		this.m_sweep.c0.SetV(this.m_sweep.c);
		
		
		for (s = this.m_shapeList; s; s = s.m_next)
		{
			s.UpdateSweepRadius(this.m_sweep.localCenter);
		}
		
		var oldType = this.m_type;
		if (this.m_invMass == 0.0 && this.m_invI == 0.0)
		{
			this.m_type = b2Body.e_staticType;
		}
		else
		{
			this.m_type = b2Body.e_dynamicType;
		}
		
		
		if (oldType != this.m_type)
		{
			for (s = this.m_shapeList; s; s = s.m_next)
			{
				s.RefilterProxy(this.m_world.m_broadPhase, this.m_xf);
			}
		}
	}
b2Body.prototype.SetXForm = function (position, angle) {
		
		var s;
		
		
		if (this.m_world.m_lock == true)
		{
			return true;
		}
		
		if (this.IsFrozen())
		{
			return false;
		}
		
		this.m_xf.R.Set(angle);
		this.m_xf.position.SetV(position);
		
		
		
		var tMat = this.m_xf.R;
		var tVec = this.m_sweep.localCenter;
		
		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		
		this.m_sweep.c0.SetV(this.m_sweep.c);
		
		this.m_sweep.a0 = this.m_sweep.a = angle;
		
		var freeze = false;
		for (s = this.m_shapeList; s; s = s.m_next)
		{
			var inRange = s.Synchronize(this.m_world.m_broadPhase, this.m_xf, this.m_xf);
			
			if (inRange == false)
			{
				freeze = true;
				break;
			}
		}
		
		if (freeze == true)
		{
			this.m_flags |= b2Body.e_frozenFlag;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			for (s = this.m_shapeList; s; s = s.m_next)
			{
				s.DestroyProxy(this.m_world.m_broadPhase);
			}
			
			
			return false;
		}
		
		
		this.m_world.m_broadPhase.Commit();
		return true;
		
	}
b2Body.prototype.GetXForm = function () {
		return this.m_xf;
	}
b2Body.prototype.GetPosition = function () {
		return this.m_xf.position;
	}
b2Body.prototype.GetAngle = function () {
		return this.m_sweep.a;
	}
b2Body.prototype.GetWorldCenter = function () {
		return this.m_sweep.c;
	}
b2Body.prototype.GetLocalCenter = function () {
		return this.m_sweep.localCenter;
	}
b2Body.prototype.SetLinearVelocity = function (v) {
		this.m_linearVelocity.SetV(v);
	}
b2Body.prototype.GetLinearVelocity = function () {
		return this.m_linearVelocity;
	}
b2Body.prototype.SetAngularVelocity = function (omega) {
		this.m_angularVelocity = omega;
	}
b2Body.prototype.GetAngularVelocity = function () {
		return this.m_angularVelocity;
	}
b2Body.prototype.ApplyForce = function (force, point) {
		if (this.IsSleeping())
		{
			this.WakeUp();
		}
		
		this.m_force.x += force.x;
		this.m_force.y += force.y;
		
		this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
	}
b2Body.prototype.ApplyTorque = function (torque) {
		if (this.IsSleeping())
		{
			this.WakeUp();
		}
		this.m_torque += torque;
	}
b2Body.prototype.ApplyImpulse = function (impulse, point) {
		if (this.IsSleeping())
		{
			this.WakeUp();
		}
		
		this.m_linearVelocity.x += this.m_invMass * impulse.x;
		this.m_linearVelocity.y += this.m_invMass * impulse.y;
		
		this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
	}
b2Body.prototype.GetMass = function () {
		return this.m_mass;
	}
b2Body.prototype.GetInertia = function () {
		return this.m_I;
	}
b2Body.prototype.GetWorldPoint = function (localPoint) {
		
		var A = this.m_xf.R;
		var u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, 
								 A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		u.x += this.m_xf.position.x;
		u.y += this.m_xf.position.y;
		return u;
	}
b2Body.prototype.GetWorldVector = function (localVector) {
		return b2Math.b2MulMV(this.m_xf.R, localVector);
	}
b2Body.prototype.GetLocalPoint = function (worldPoint) {
		return b2Math.b2MulXT(this.m_xf, worldPoint);
	}
b2Body.prototype.GetLocalVector = function (worldVector) {
		return b2Math.b2MulTMV(this.m_xf.R, worldVector);
	}
b2Body.prototype.GetLinearVelocityFromWorldPoint = function (worldPoint) {
		
		return new b2Vec2(	this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), 
							this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
	}
b2Body.prototype.GetLinearVelocityFromLocalPoint = function (localPoint) {
		
		var A = this.m_xf.R;
		var worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, 
								 A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		worldPoint.x += this.m_xf.position.x;
		worldPoint.y += this.m_xf.position.y;
		return new b2Vec2(this.m_linearVelocity.x + this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), 
		 this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
	}
b2Body.prototype.IsBullet = function () {
		return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
	}
b2Body.prototype.SetBullet = function (flag) {
		if (flag)
		{
			this.m_flags |= b2Body.e_bulletFlag;
		}
		else
		{
			this.m_flags &= ~b2Body.e_bulletFlag;
		}
	}
b2Body.prototype.IsStatic = function () {
		return this.m_type == b2Body.e_staticType;
	}
b2Body.prototype.IsDynamic = function () {
		return this.m_type == b2Body.e_dynamicType;
	}
b2Body.prototype.IsFrozen = function () {
		return (this.m_flags & b2Body.e_frozenFlag) == b2Body.e_frozenFlag;
	}
b2Body.prototype.IsSleeping = function () {
		return (this.m_flags & b2Body.e_sleepFlag) == b2Body.e_sleepFlag;
	}
b2Body.prototype.AllowSleeping = function (flag) {
		if (flag)
		{
			this.m_flags |= b2Body.e_allowSleepFlag;
		}
		else
		{
			this.m_flags &= ~b2Body.e_allowSleepFlag;
			this.WakeUp();
		}
	}
b2Body.prototype.WakeUp = function () {
		this.m_flags &= ~b2Body.e_sleepFlag;
		this.m_sleepTime = 0.0;
	}
b2Body.prototype.PutToSleep = function () {
		this.m_flags |= b2Body.e_sleepFlag;
		this.m_sleepTime = 0.0;
		this.m_linearVelocity.SetZero();
		this.m_angularVelocity = 0.0;
		this.m_force.SetZero();
		this.m_torque = 0.0;
	}
b2Body.prototype.GetShapeList = function () {
		return this.m_shapeList;
	}
b2Body.prototype.GetJointList = function () {
		return this.m_jointList;
	}
b2Body.prototype.GetNext = function () {
		return this.m_next;
	}
b2Body.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Body.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Body.prototype.GetWorld = function () {
		return this.m_world;
	}
b2Body.prototype.SynchronizeShapes = function () {
		
		var xf1 = b2Body.s_xf1;
		xf1.R.Set(this.m_sweep.a0);
		
		var tMat = xf1.R;
		var tVec = this.m_sweep.localCenter;
		xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		var s;
		
		var inRange = true;
		for (s = this.m_shapeList; s; s = s.m_next)
		{
			inRange = s.Synchronize(this.m_world.m_broadPhase, xf1, this.m_xf);
			if (inRange == false)
			{
				break;
			}
		}
		
		if (inRange == false)
		{
			this.m_flags |= b2Body.e_frozenFlag;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			for (s = this.m_shapeList; s; s = s.m_next)
			{
				s.DestroyProxy(this.m_world.m_broadPhase);
			}
			
			
			return false;
		}
		
		
		return true;
		
	}
b2Body.prototype.SynchronizeTransform = function () {
		this.m_xf.R.Set(this.m_sweep.a);
		
		var tMat = this.m_xf.R;
		var tVec = this.m_sweep.localCenter;
		this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	}
b2Body.prototype.IsConnected = function (other) {
		for (var jn = this.m_jointList; jn; jn = jn.next)
		{
			if (jn.other == other)
				return jn.joint.m_collideConnected == false;
		}
		
		return false;
	}
b2Body.prototype.Advance = function (t) {
		
		this.m_sweep.Advance(t);
		this.m_sweep.c.SetV(this.m_sweep.c0);
		this.m_sweep.a = this.m_sweep.a0;
		this.SynchronizeTransform();
	}