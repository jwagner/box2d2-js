var b2MouseJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2MouseJoint.prototype, b2Joint.prototype)
b2MouseJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2MouseJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		this.m_target.SetV(def.target);
		
		var tX = this.m_target.x - this.m_body2.m_xf.position.x;
		var tY = this.m_target.y - this.m_body2.m_xf.position.y;
		var tMat = this.m_body2.m_xf.R;
		this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
		this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		this.m_maxForce = def.maxForce;
		this.m_impulse.SetZero();
		
		var mass = this.m_body2.m_mass;
		
		
		var omega = 2.0 * b2Settings.b2_pi * def.frequencyHz;
		
		
		var d = 2.0 * mass * def.dampingRatio * omega;
		
		
		var k = (def.timeStep * mass) * (omega * omega);
		
		
		
		this.m_gamma = 1.0 / (d + k);
		this.m_beta = k / (d + k);
	}
b2MouseJoint.prototype.__varz = function(){
this.K =  new b2Mat22();
this.K1 =  new b2Mat22();
this.K2 =  new b2Mat22();
this.m_localAnchor =  new b2Vec2();
this.m_target =  new b2Vec2();
this.m_impulse =  new b2Vec2();
this.m_mass =  new b2Mat22();
this.m_C =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2MouseJoint.prototype.K =  new b2Mat22();
b2MouseJoint.prototype.K1 =  new b2Mat22();
b2MouseJoint.prototype.K2 =  new b2Mat22();
b2MouseJoint.prototype.m_localAnchor =  new b2Vec2();
b2MouseJoint.prototype.m_target =  new b2Vec2();
b2MouseJoint.prototype.m_impulse =  new b2Vec2();
b2MouseJoint.prototype.m_mass =  new b2Mat22();
b2MouseJoint.prototype.m_C =  new b2Vec2();
b2MouseJoint.prototype.m_maxForce =  null;
b2MouseJoint.prototype.m_beta =  null;
b2MouseJoint.prototype.m_gamma =  null;
// methods
b2MouseJoint.prototype.GetAnchor1 = function () {
		return this.m_target;
	}
b2MouseJoint.prototype.GetAnchor2 = function () {
		return this.m_body2.GetWorldPoint(this.m_localAnchor);
	}
b2MouseJoint.prototype.GetReactionForce = function () {
		return this.m_impulse;
	}
b2MouseJoint.prototype.GetReactionTorque = function () {
		return 0.0;
	}
b2MouseJoint.prototype.SetTarget = function (target) {
		if (this.m_body2.IsSleeping()){
			this.m_body2.WakeUp();
		}
		this.m_target = target;
	}
b2MouseJoint.prototype.InitVelocityConstraints = function (step) {
		var b = this.m_body2;
		
		var tMat;
		
		
		
		tMat = b.m_xf.R;
		var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;
		
		
		
		
		var invMass = b.m_invMass;
		var invI = b.m_invI;
		
		
		this.K1.col1.x = invMass;	this.K1.col2.x = 0.0;
		this.K1.col1.y = 0.0;		this.K1.col2.y = invMass;
		
		
		this.K2.col1.x = invI * rY * rY;	this.K2.col2.x = -invI * rX * rY;
		this.K2.col1.y = -invI * rX * rY;	this.K2.col2.y = invI * rX * rX;
		
		
		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.col1.x += this.m_gamma;
		this.K.col2.y += this.m_gamma;
		
		
		this.K.Invert(this.m_mass);
		
		
		this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
		this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;
		
		
		b.m_angularVelocity *= 0.98;
		
		
		
		var PX = step.dt * this.m_impulse.x;
		var PY = step.dt * this.m_impulse.y;
		
		b.m_linearVelocity.x += invMass * PX;
		b.m_linearVelocity.y += invMass * PY;
		
		b.m_angularVelocity += invI * (rX * PY - rY * PX);
	}
b2MouseJoint.prototype.SolveVelocityConstraints = function (step) {
		var b = this.m_body2;
		
		var tMat;
		var tX;
		var tY;
		
		
		
		tMat = b.m_xf.R;
		var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;
		
		
		
		var CdotX = b.m_linearVelocity.x + (-b.m_angularVelocity * rY);
		var CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
		
		tMat = this.m_mass;
		tX = CdotX + (this.m_beta * step.inv_dt) * this.m_C.x + this.m_gamma * step.dt * this.m_impulse.x;
		tY = CdotY + (this.m_beta * step.inv_dt) * this.m_C.y + this.m_gamma * step.dt * this.m_impulse.y;
		var forceX = -step.inv_dt * (tMat.col1.x * tX + tMat.col2.x * tY);
		var forceY = -step.inv_dt * (tMat.col1.y * tX + tMat.col2.y * tY);
		
		var oldForceX = this.m_impulse.x;
		var oldForceY = this.m_impulse.y;
		
		this.m_impulse.x += forceX;
		this.m_impulse.y += forceY;
		var forceMagnitude = this.m_impulse.Length();
		if (forceMagnitude > this.m_maxForce)
		{
			
			this.m_impulse.Multiply(this.m_maxForce / forceMagnitude);
		}
		
		forceX = this.m_impulse.x - oldForceX;
		forceY = this.m_impulse.y - oldForceY;
		
		
		var PX = step.dt * forceX;
		var PY = step.dt * forceY;
		
		b.m_linearVelocity.x += b.m_invMass * PX;
		b.m_linearVelocity.y += b.m_invMass * PY;
		
		b.m_angularVelocity += b.m_invI * (rX * PY - rY * PX);
	}
b2MouseJoint.prototype.SolvePositionConstraints = function () { 
		return true; 
	}