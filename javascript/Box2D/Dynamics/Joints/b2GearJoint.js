var b2GearJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2GearJoint.prototype, b2Joint.prototype)
b2GearJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2GearJoint.prototype.__constructor = function (def) {
		
		this._super(def);
		
		var type1 = def.joint1.m_type;
		var type2 = def.joint2.m_type;
		
		
		
		
		
		
		this.m_revolute1 = null;
		this.m_prismatic1 = null;
		this.m_revolute2 = null;
		this.m_prismatic2 = null;
		
		var coordinate1;
		var coordinate2;
		
		this.m_ground1 = def.joint1.m_body1;
		this.m_body1 = def.joint1.m_body2;
		if (type1 == b2Joint.e_revoluteJoint)
		{
			this.m_revolute1 = def.joint1;
			this.m_groundAnchor1.SetV( this.m_revolute1.m_localAnchor1 );
			this.m_localAnchor1.SetV( this.m_revolute1.m_localAnchor2 );
			coordinate1 = this.m_revolute1.GetJointAngle();
		}
		else
		{
			this.m_prismatic1 = def.joint1;
			this.m_groundAnchor1.SetV( this.m_prismatic1.m_localAnchor1 );
			this.m_localAnchor1.SetV( this.m_prismatic1.m_localAnchor2 );
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}
		
		this.m_ground2 = def.joint2.m_body1;
		this.m_body2 = def.joint2.m_body2;
		if (type2 == b2Joint.e_revoluteJoint)
		{
			this.m_revolute2 = def.joint2;
			this.m_groundAnchor2.SetV( this.m_revolute2.m_localAnchor1 );
			this.m_localAnchor2.SetV( this.m_revolute2.m_localAnchor2 );
			coordinate2 = this.m_revolute2.GetJointAngle();
		}
		else
		{
			this.m_prismatic2 = def.joint2;
			this.m_groundAnchor2.SetV( this.m_prismatic2.m_localAnchor1 );
			this.m_localAnchor2.SetV( this.m_prismatic2.m_localAnchor2 );
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}
		
		this.m_ratio = def.ratio;
		
		this.m_constant = coordinate1 + this.m_ratio * coordinate2;
		
		this.m_force = 0.0;
		
	}
b2GearJoint.prototype.__varz = function(){
this.m_groundAnchor1 =  new b2Vec2();
this.m_groundAnchor2 =  new b2Vec2();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_J =  new b2Jacobian();
}
// static attributes
// static methods
// attributes
b2GearJoint.prototype.m_ground1 =  null;
b2GearJoint.prototype.m_ground2 =  null;
b2GearJoint.prototype.m_revolute1 =  null;
b2GearJoint.prototype.m_prismatic1 =  null;
b2GearJoint.prototype.m_revolute2 =  null;
b2GearJoint.prototype.m_prismatic2 =  null;
b2GearJoint.prototype.m_groundAnchor1 =  new b2Vec2();
b2GearJoint.prototype.m_groundAnchor2 =  new b2Vec2();
b2GearJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2GearJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2GearJoint.prototype.m_J =  new b2Jacobian();
b2GearJoint.prototype.m_constant =  null;
b2GearJoint.prototype.m_ratio =  null;
b2GearJoint.prototype.m_mass =  null;
b2GearJoint.prototype.m_force =  null;
// methods
b2GearJoint.prototype.GetAnchor1 = function () {
		
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}
b2GearJoint.prototype.GetAnchor2 = function () {
		
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}
b2GearJoint.prototype.GetReactionForce = function () {
		
		var F = new b2Vec2(this.m_force * this.m_J.linear2.x, this.m_force * this.m_J.linear2.y);
		return F;
	}
b2GearJoint.prototype.GetReactionTorque = function () {
		
		
		var tMat = this.m_body2.m_xf.R;
		var rX = this.m_localAnchor1.x - this.m_body2.m_sweep.localCenter.x;
		var rY = this.m_localAnchor1.y - this.m_body2.m_sweep.localCenter.y;
		var tX = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		
		
		tX = this.m_force * this.m_J.angular2 - (rX * (this.m_force * this.m_J.linear2.y) - rY * (this.m_force * this.m_J.linear2.x));
		return tX;
	}
b2GearJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
b2GearJoint.prototype.InitVelocityConstraints = function (step) {
		var g1 = this.m_ground1;
		var g2 = this.m_ground2;
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		
		var ugX;
		var ugY;
		var rX;
		var rY;
		var tMat;
		var tVec;
		var crug;
		var tX;
		
		var K = 0.0;
		this.m_J.SetZero();
		
		if (this.m_revolute1)
		{
			this.m_J.angular1 = -1.0;
			K += b1.m_invI;
		}
		else
		{
			
			tMat = g1.m_xf.R;
			tVec = this.m_prismatic1.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			
			tMat = b1.m_xf.R;
			rX = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
			rY = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			
			
			crug = rX * ugY - rY * ugX;
			
			this.m_J.linear1.Set(-ugX, -ugY);
			this.m_J.angular1 = -crug;
			K += b1.m_invMass + b1.m_invI * crug * crug;
		}
		
		if (this.m_revolute2)
		{
			this.m_J.angular2 = -this.m_ratio;
			K += this.m_ratio * this.m_ratio * b2.m_invI;
		}
		else
		{
			
			tMat = g2.m_xf.R;
			tVec = this.m_prismatic2.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			
			tMat = b2.m_xf.R;
			rX = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
			rY = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			
			
			crug = rX * ugY - rY * ugX;
			
			this.m_J.linear2.Set(-this.m_ratio*ugX, -this.m_ratio*ugY);
			this.m_J.angular2 = -this.m_ratio * crug;
			K += this.m_ratio * this.m_ratio * (b2.m_invMass + b2.m_invI * crug * crug);
		}
		
		
		
		this.m_mass = 1.0 / K;
		
		if (step.warmStarting)
		{
			
			var P = step.dt * this.m_force;
			
			b1.m_linearVelocity.x += b1.m_invMass * P * this.m_J.linear1.x;
			b1.m_linearVelocity.y += b1.m_invMass * P * this.m_J.linear1.y;
			b1.m_angularVelocity += b1.m_invI * P * this.m_J.angular1;
			
			b2.m_linearVelocity.x += b2.m_invMass * P * this.m_J.linear2.x;
			b2.m_linearVelocity.y += b2.m_invMass * P * this.m_J.linear2.y;
			b2.m_angularVelocity += b2.m_invI * P * this.m_J.angular2;
		}
		else
		{
			this.m_force = 0.0;
		}
	}
b2GearJoint.prototype.SolveVelocityConstraints = function (step) {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var Cdot = this.m_J.Compute(	b1.m_linearVelocity, b1.m_angularVelocity,
										b2.m_linearVelocity, b2.m_angularVelocity);
		
		var force = -step.inv_dt * this.m_mass * Cdot;
		this.m_force += force;
		
		var P = step.dt * force;
		b1.m_linearVelocity.x += b1.m_invMass * P * this.m_J.linear1.x;
		b1.m_linearVelocity.y += b1.m_invMass * P * this.m_J.linear1.y;
		b1.m_angularVelocity += b1.m_invI * P * this.m_J.angular1;
		b2.m_linearVelocity.x += b2.m_invMass * P * this.m_J.linear2.x;
		b2.m_linearVelocity.y += b2.m_invMass * P * this.m_J.linear2.y;
		b2.m_angularVelocity += b2.m_invI * P * this.m_J.angular2;
	}
b2GearJoint.prototype.SolvePositionConstraints = function () {
		var linearError = 0.0;
		
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var coordinate1;
		var coordinate2;
		if (this.m_revolute1)
		{
			coordinate1 = this.m_revolute1.GetJointAngle();
		}
		else
		{
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}
		
		if (this.m_revolute2)
		{
			coordinate2 = this.m_revolute2.GetJointAngle();
		}
		else
		{
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}
		
		var C = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);
		
		var impulse = -this.m_mass * C;
		
		b1.m_sweep.c.x += b1.m_invMass * impulse * this.m_J.linear1.x;
		b1.m_sweep.c.y += b1.m_invMass * impulse * this.m_J.linear1.y;
		b1.m_sweep.a += b1.m_invI * impulse * this.m_J.angular1;
		b2.m_sweep.c.x += b2.m_invMass * impulse * this.m_J.linear2.x;
		b2.m_sweep.c.y += b2.m_invMass * impulse * this.m_J.linear2.y;
		b2.m_sweep.a += b2.m_invI * impulse * this.m_J.angular2;
		
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		return linearError < b2Settings.b2_linearSlop;
	}