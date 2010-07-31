var b2PulleyJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PulleyJoint.prototype, b2Joint.prototype)
b2PulleyJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2PulleyJoint.prototype.__constructor = function (def) {
		
		
		this._super(def);
		
		var tMat;
		var tX;
		var tY;
		
		this.m_ground = this.m_body1.m_world.m_groundBody;
		
		this.m_groundAnchor1.x = def.groundAnchor1.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor1.y = def.groundAnchor1.y - this.m_ground.m_xf.position.y;
		
		this.m_groundAnchor2.x = def.groundAnchor2.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor2.y = def.groundAnchor2.y - this.m_ground.m_xf.position.y;
		
		this.m_localAnchor1.SetV(def.localAnchor1);
		
		this.m_localAnchor2.SetV(def.localAnchor2);
		
		
		this.m_ratio = def.ratio;
		
		this.m_constant = def.length1 + this.m_ratio * def.length2;
		
		this.m_maxLength1 = b2Math.b2Min(def.maxLength1, this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength);
		this.m_maxLength2 = b2Math.b2Min(def.maxLength2, (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio);
		
		this.m_force = 0.0;
		this.m_limitForce1 = 0.0;
		this.m_limitForce2 = 0.0;
		
	}
b2PulleyJoint.prototype.__varz = function(){
this.m_groundAnchor1 =  new b2Vec2();
this.m_groundAnchor2 =  new b2Vec2();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_u1 =  new b2Vec2();
this.m_u2 =  new b2Vec2();
}
// static attributes
b2PulleyJoint.b2_minPulleyLength =  2.0;
// static methods
// attributes
b2PulleyJoint.prototype.m_ground =  null;
b2PulleyJoint.prototype.m_groundAnchor1 =  new b2Vec2();
b2PulleyJoint.prototype.m_groundAnchor2 =  new b2Vec2();
b2PulleyJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2PulleyJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2PulleyJoint.prototype.m_u1 =  new b2Vec2();
b2PulleyJoint.prototype.m_u2 =  new b2Vec2();
b2PulleyJoint.prototype.m_constant =  null;
b2PulleyJoint.prototype.m_ratio =  null;
b2PulleyJoint.prototype.m_maxLength1 =  null;
b2PulleyJoint.prototype.m_maxLength2 =  null;
b2PulleyJoint.prototype.m_pulleyMass =  null;
b2PulleyJoint.prototype.m_limitMass1 =  null;
b2PulleyJoint.prototype.m_limitMass2 =  null;
b2PulleyJoint.prototype.m_force =  null;
b2PulleyJoint.prototype.m_limitForce1 =  null;
b2PulleyJoint.prototype.m_limitForce2 =  null;
b2PulleyJoint.prototype.m_positionImpulse =  null;
b2PulleyJoint.prototype.m_limitPositionImpulse1 =  null;
b2PulleyJoint.prototype.m_limitPositionImpulse2 =  null;
b2PulleyJoint.prototype.m_state =  0;
b2PulleyJoint.prototype.m_limitState1 =  0;
b2PulleyJoint.prototype.m_limitState2 =  0;
// methods
b2PulleyJoint.prototype.GetAnchor1 = function () {
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}
b2PulleyJoint.prototype.GetAnchor2 = function () {
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}
b2PulleyJoint.prototype.GetReactionForce = function () {
		
		var F = this.m_u2.Copy();
		F.Multiply(this.m_force);
		return F;
	}
b2PulleyJoint.prototype.GetReactionTorque = function () {
		return 0.0;
	}
b2PulleyJoint.prototype.GetGroundAnchor1 = function () {
		
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor1);
		return a;
	}
b2PulleyJoint.prototype.GetGroundAnchor2 = function () {
		
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor2);
		return a;
	}
b2PulleyJoint.prototype.GetLength1 = function () {
		var p = this.m_body1.GetWorldPoint(this.m_localAnchor1);
		
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var dX = p.x - sX;
		var dY = p.y - sY;
		
		return Math.sqrt(dX*dX + dY*dY);
	}
b2PulleyJoint.prototype.GetLength2 = function () {
		var p = this.m_body2.GetWorldPoint(this.m_localAnchor2);
		
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		var dX = p.x - sX;
		var dY = p.y - sY;
		
		return Math.sqrt(dX*dX + dY*dY);
	}
b2PulleyJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
b2PulleyJoint.prototype.InitVelocityConstraints = function (step) {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var tMat;
		
		
		tMat = b1.m_xf.R;
		var r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		
		tMat = b2.m_xf.R;
		var r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		
		var p1X = b1.m_sweep.c.x + r1X;
		var p1Y = b1.m_sweep.c.y + r1Y;
		
		var p2X = b2.m_sweep.c.x + r2X;
		var p2Y = b2.m_sweep.c.y + r2Y;
		
		
		var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		
		
		this.m_u1.Set(p1X - s1X, p1Y - s1Y);
		
		this.m_u2.Set(p2X - s2X, p2Y - s2Y);
		
		var length1 = this.m_u1.Length();
		var length2 = this.m_u2.Length();
		
		if (length1 > b2Settings.b2_linearSlop)
		{
			
			this.m_u1.Multiply(1.0 / length1);
		}
		else
		{
			this.m_u1.SetZero();
		}
		
		if (length2 > b2Settings.b2_linearSlop)
		{
			
			this.m_u2.Multiply(1.0 / length2);
		}
		else
		{
			this.m_u2.SetZero();
		}
		
		var C = this.m_constant - length1 - this.m_ratio * length2;
		if (C > 0.0)
		{
			this.m_state = b2Joint.e_inactiveLimit;
			this.m_force = 0.0;
		}
		else
		{
			this.m_state = b2Joint.e_atUpperLimit;
			this.m_positionImpulse = 0.0;
		}
		
		if (length1 < this.m_maxLength1)
		{
			this.m_limitState1 = b2Joint.e_inactiveLimit;
			this.m_limitForce1 = 0.0;
		}
		else
		{
			this.m_limitState1 = b2Joint.e_atUpperLimit;
			this.m_limitPositionImpulse1 = 0.0;
		}
		
		if (length2 < this.m_maxLength2)
		{
			this.m_limitState2 = b2Joint.e_inactiveLimit;
			this.m_limitForce2 = 0.0;
		}
		else
		{
			this.m_limitState2 = b2Joint.e_atUpperLimit;
			this.m_limitPositionImpulse2 = 0.0;
		}
		
		
		
		var cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
		
		var cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;
		
		this.m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		this.m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
		
		
		
		this.m_limitMass1 = 1.0 / this.m_limitMass1;
		this.m_limitMass2 = 1.0 / this.m_limitMass2;
		this.m_pulleyMass = 1.0 / this.m_pulleyMass;
		
		if (step.warmStarting)
		{
			
			
			
			var P1X = step.dt * (-this.m_force - this.m_limitForce1) * this.m_u1.x;
			var P1Y = step.dt * (-this.m_force - this.m_limitForce1) * this.m_u1.y;
			
			
			var P2X = step.dt * (-this.m_ratio * this.m_force - this.m_limitForce2) * this.m_u2.x;
			var P2Y = step.dt * (-this.m_ratio * this.m_force - this.m_limitForce2) * this.m_u2.y;
			
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		else
		{
			this.m_force = 0.0;
			this.m_limitForce1 = 0.0;
			this.m_limitForce2 = 0.0;
		}
	}
b2PulleyJoint.prototype.SolveVelocityConstraints = function (step) {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var tMat;
		
		
		tMat = b1.m_xf.R;
		var r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		
		tMat = b2.m_xf.R;
		var r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		
		var v1X;
		var v1Y;
		var v2X;
		var v2Y;
		var P1X;
		var P1Y;
		var P2X;
		var P2Y;
		var Cdot;
		var force;
		var oldForce;
		
		if (this.m_state == b2Joint.e_atUpperLimit)
		{
			
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
			
			
			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
			force = -step.inv_dt * this.m_pulleyMass * Cdot;
			oldForce = this.m_force;
			this.m_force = b2Math.b2Max(0.0, this.m_force + force);
			force = this.m_force - oldForce;
			
			
			P1X = -step.dt * force * this.m_u1.x;
			P1Y = -step.dt * force * this.m_u1.y;
			
			P2X = -step.dt * this.m_ratio * force * this.m_u2.x;
			P2Y = -step.dt * this.m_ratio * force * this.m_u2.y;
			
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		
		if (this.m_limitState1 == b2Joint.e_atUpperLimit)
		{
			
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			
			
			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y);
			force = -step.inv_dt * this.m_limitMass1 * Cdot;
			oldForce = this.m_limitForce1;
			this.m_limitForce1 = b2Math.b2Max(0.0, this.m_limitForce1 + force);
			force = this.m_limitForce1 - oldForce;
			
			
			P1X = -step.dt * force * this.m_u1.x;
			P1Y = -step.dt * force * this.m_u1.y;
			
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
		}
		
		if (this.m_limitState2 == b2Joint.e_atUpperLimit)
		{
			
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
			
			
			Cdot = -(this.m_u2.x * v2X + this.m_u2.y * v2Y);
			force = -step.inv_dt * this.m_limitMass2 * Cdot;
			oldForce = this.m_limitForce2;
			this.m_limitForce2 = b2Math.b2Max(0.0, this.m_limitForce2 + force);
			force = this.m_limitForce2 - oldForce;
			
			
			P2X = -step.dt * force * this.m_u2.x;
			P2Y = -step.dt * force * this.m_u2.y;
			
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	}
b2PulleyJoint.prototype.SolvePositionConstraints = function () {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var tMat;
		
		
		var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		
		var r1X;
		var r1Y;
		var r2X;
		var r2Y;
		var p1X;
		var p1Y;
		var p2X;
		var p2Y;
		var length1;
		var length2;
		var C;
		var impulse;
		var oldImpulse;
		var oldLimitPositionImpulse;
		
		var tX;
		
		var linearError = 0.0;
		
		if (this.m_state == b2Joint.e_atUpperLimit)
		{
			
			tMat = b1.m_xf.R;
			r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			
			tMat = b2.m_xf.R;
			r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			
			
			p1X = b1.m_sweep.c.x + r1X;
			p1Y = b1.m_sweep.c.y + r1Y;
			
			p2X = b2.m_sweep.c.x + r2X;
			p2Y = b2.m_sweep.c.y + r2Y;
			
			
			
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length1 = this.m_u1.Length();
			length2 = this.m_u2.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				
				this.m_u1.Multiply( 1.0 / length1 );
			}
			else
			{
				this.m_u1.SetZero();
			}
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				
				this.m_u2.Multiply( 1.0 / length2 );
			}
			else
			{
				this.m_u2.SetZero();
			}
			
			C = this.m_constant - length1 - this.m_ratio * length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_pulleyMass * C;
			
			oldImpulse = this.m_positionImpulse;
			this.m_positionImpulse = b2Math.b2Max(0.0, this.m_positionImpulse + impulse);
			impulse = this.m_positionImpulse - oldImpulse;
			
			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;
			p2X = -this.m_ratio * impulse * this.m_u2.x;
			p2Y = -this.m_ratio * impulse * this.m_u2.y;
			
			b1.m_sweep.c.x += b1.m_invMass * p1X;
			b1.m_sweep.c.y += b1.m_invMass * p1Y;
			b1.m_sweep.a += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			b2.m_sweep.c.x += b2.m_invMass * p2X;
			b2.m_sweep.c.y += b2.m_invMass * p2Y;
			b2.m_sweep.a += b2.m_invI * (r2X * p2Y - r2Y * p2X);
			
			b1.SynchronizeTransform();
			b2.SynchronizeTransform();
		}
		
		if (this.m_limitState1 == b2Joint.e_atUpperLimit)
		{
			
			tMat = b1.m_xf.R;
			r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			
			p1X = b1.m_sweep.c.x + r1X;
			p1Y = b1.m_sweep.c.y + r1Y;
			
			
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			
			length1 = this.m_u1.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				
				this.m_u1.x *= 1.0 / length1;
				this.m_u1.y *= 1.0 / length1;
			}
			else
			{
				this.m_u1.SetZero();
			}
			
			C = this.m_maxLength1 - length1;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_limitMass1 * C;
			oldLimitPositionImpulse = this.m_limitPositionImpulse1;
			this.m_limitPositionImpulse1 = b2Math.b2Max(0.0, this.m_limitPositionImpulse1 + impulse);
			impulse = this.m_limitPositionImpulse1 - oldLimitPositionImpulse;
			
			
			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;
			
			b1.m_sweep.c.x += b1.m_invMass * p1X;
			b1.m_sweep.c.y += b1.m_invMass * p1Y;
			
			b1.m_sweep.a += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			
			b1.SynchronizeTransform();
		}
		
		if (this.m_limitState2 == b2Joint.e_atUpperLimit)
		{
			
			tMat = b2.m_xf.R;
			r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			
			p2X = b2.m_sweep.c.x + r2X;
			p2Y = b2.m_sweep.c.y + r2Y;
			
			
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length2 = this.m_u2.Length();
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				
				this.m_u2.x *= 1.0 / length2;
				this.m_u2.y *= 1.0 / length2;
			}
			else
			{
				this.m_u2.SetZero();
			}
			
			C = this.m_maxLength2 - length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_limitMass2 * C;
			oldLimitPositionImpulse = this.m_limitPositionImpulse2;
			this.m_limitPositionImpulse2 = b2Math.b2Max(0.0, this.m_limitPositionImpulse2 + impulse);
			impulse = this.m_limitPositionImpulse2 - oldLimitPositionImpulse;
			
			
			p2X = -impulse * this.m_u2.x;
			p2Y = -impulse * this.m_u2.y;
			
			
			b2.m_sweep.c.x += b2.m_invMass * p2X;
			b2.m_sweep.c.y += b2.m_invMass * p2Y;
			
			b2.m_sweep.a += b2.m_invI * (r2X * p2Y - r2Y * p2X);
			
			b2.SynchronizeTransform();
		}
		
		return linearError < b2Settings.b2_linearSlop;
	}