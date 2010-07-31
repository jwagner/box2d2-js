var b2DistanceJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2DistanceJoint.prototype, b2Joint.prototype)
b2DistanceJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2DistanceJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		var tMat;
		var tX;
		var tY;
		
		this.m_localAnchor1.SetV(def.localAnchor1);
		
		this.m_localAnchor2.SetV(def.localAnchor2);
		
		this.m_length = def.length;
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		this.m_impulse = 0.0;
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
		this.m_inv_dt = 0.0;
	}
b2DistanceJoint.prototype.__varz = function(){
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_u =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2DistanceJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2DistanceJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2DistanceJoint.prototype.m_u =  new b2Vec2();
b2DistanceJoint.prototype.m_frequencyHz =  null;
b2DistanceJoint.prototype.m_dampingRatio =  null;
b2DistanceJoint.prototype.m_gamma =  null;
b2DistanceJoint.prototype.m_bias =  null;
b2DistanceJoint.prototype.m_impulse =  null;
b2DistanceJoint.prototype.m_mass =  null;
b2DistanceJoint.prototype.m_length =  null;
// methods
b2DistanceJoint.prototype.InitVelocityConstraints = function (step) {
		
		var tMat;
		var tX;
		
		this.m_inv_dt = step.inv_dt;

		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		
		
		tMat = b1.m_xf.R;
		var r1X = this.m_localAnchor1.x - b1.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - b1.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		
		tMat = b2.m_xf.R;
		var r2X = this.m_localAnchor2.x - b2.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - b2.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		
		this.m_u.x = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x - r1X;
		this.m_u.y = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y - r1Y;
		
		
		
		var length = Math.sqrt(this.m_u.x*this.m_u.x + this.m_u.y*this.m_u.y);
		if (length > b2Settings.b2_linearSlop)
		{
			
			this.m_u.Multiply( 1.0 / length );
		}
		else
		{
			this.m_u.SetZero();
		}
		
		
		var cr1u = (r1X * this.m_u.y - r1Y * this.m_u.x);
		
		var cr2u = (r2X * this.m_u.y - r2Y * this.m_u.x);
		
		var invMass = b1.m_invMass + b1.m_invI * cr1u * cr1u + b2.m_invMass + b2.m_invI * cr2u * cr2u;
		
		this.m_mass = 1.0 / invMass;
		
		if (this.m_frequencyHz > 0.0)
		{
			var C = length - this.m_length;
	
			
			var omega = 2.0 * Math.PI * this.m_frequencyHz;
	
			
			var d = 2.0 * this.m_mass * this.m_dampingRatio * omega;
	
			
			var k = this.m_mass * omega * omega;
	
			
			this.m_gamma = 1.0 / (step.dt * (d + step.dt * k));
			this.m_bias = C * step.dt * k * this.m_gamma;
	
			this.m_mass = 1.0 / (invMass + this.m_gamma);
		}
		
		if (step.warmStarting)
		{
			this.m_impulse *= step.dtRatio;
			
			var PX = this.m_impulse * this.m_u.x;
			var PY = this.m_impulse * this.m_u.y;
			
			b1.m_linearVelocity.x -= b1.m_invMass * PX;
			b1.m_linearVelocity.y -= b1.m_invMass * PY;
			
			b1.m_angularVelocity -= b1.m_invI * (r1X * PY - r1Y * PX);
			
			b2.m_linearVelocity.x += b2.m_invMass * PX;
			b2.m_linearVelocity.y += b2.m_invMass * PY;
			
			b2.m_angularVelocity += b2.m_invI * (r2X * PY - r2Y * PX);
		}
		else
		{
			this.m_impulse = 0.0;
		}
	}
b2DistanceJoint.prototype.SolveVelocityConstraints = function (step) {
		
		var tMat;
		
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		
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
		
		
		
		var v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
		var v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
		
		var v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
		var v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
		
		var Cdot = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y));
		
		var impulse = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
		this.m_impulse += impulse;
		
		
		var PX = impulse * this.m_u.x;
		var PY = impulse * this.m_u.y;
		
		b1.m_linearVelocity.x -= b1.m_invMass * PX;
		b1.m_linearVelocity.y -= b1.m_invMass * PY;
		
		b1.m_angularVelocity -= b1.m_invI * (r1X * PY - r1Y * PX);
		
		b2.m_linearVelocity.x += b2.m_invMass * PX;
		b2.m_linearVelocity.y += b2.m_invMass * PY;
		
		b2.m_angularVelocity += b2.m_invI * (r2X * PY - r2Y * PX);
	}
b2DistanceJoint.prototype.SolvePositionConstraints = function () {
		
		var tMat;
		
		if (this.m_frequencyHz > 0.0)
		{
			return true;
		}
		
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		
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
		
		
		var dX = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x - r1X;
		var dY = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y - r1Y;
		
		
		var length = Math.sqrt(dX*dX + dY*dY);
		dX /= length;
		dY /= length;
		
		var C = length - this.m_length;
		C = b2Math.b2Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
		
		var impulse = -this.m_mass * C;
		
		this.m_u.Set(dX, dY);
		
		var PX = impulse * this.m_u.x;
		var PY = impulse * this.m_u.y;
		
		
		b1.m_sweep.c.x -= b1.m_invMass * PX;
		b1.m_sweep.c.y -= b1.m_invMass * PY;
		
		b1.m_sweep.a -= b1.m_invI * (r1X * PY - r1Y * PX);
		
		b2.m_sweep.c.x += b2.m_invMass * PX;
		b2.m_sweep.c.y += b2.m_invMass * PY;
		
		b2.m_sweep.a += b2.m_invI * (r2X * PY - r2Y * PX);
		
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		return b2Math.b2Abs(C) < b2Settings.b2_linearSlop;
		
	}
b2DistanceJoint.prototype.GetAnchor1 = function () {
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}
b2DistanceJoint.prototype.GetAnchor2 = function () {
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}
b2DistanceJoint.prototype.GetReactionForce = function () {
		
		var F = new b2Vec2();
		F.SetV(this.m_u);
		F.Multiply(this.m_inv_dt * this.m_impulse);
		return F;
	}
b2DistanceJoint.prototype.GetReactionTorque = function () {
		
		return 0.0;
	}