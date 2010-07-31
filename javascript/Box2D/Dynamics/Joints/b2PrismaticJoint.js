var b2PrismaticJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PrismaticJoint.prototype, b2Joint.prototype)
b2PrismaticJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2PrismaticJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		var tMat;
		var tX;
		var tY;
		
		this.m_localAnchor1.SetV(def.localAnchor1);
		this.m_localAnchor2.SetV(def.localAnchor2);
		this.m_localXAxis1.SetV(def.localAxis1);
		
		
		this.m_localYAxis1.x = -this.m_localXAxis1.y;
		this.m_localYAxis1.y = this.m_localXAxis1.x;
		
		this.m_refAngle = def.referenceAngle;
		
		this.m_linearJacobian.SetZero();
		this.m_linearMass = 0.0;
		this.m_force = 0.0;
		
		this.m_angularMass = 0.0;
		this.m_torque = 0.0;
		
		this.m_motorJacobian.SetZero();
		this.m_motorMass = 0.0;
		this.m_motorForce = 0.0;
		this.m_limitForce = 0.0;
		this.m_limitPositionImpulse = 0.0;
		
		this.m_lowerTranslation = def.lowerTranslation;
		this.m_upperTranslation = def.upperTranslation;
		this.m_maxMotorForce = def.maxMotorForce;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
	}
b2PrismaticJoint.prototype.__varz = function(){
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_localXAxis1 =  new b2Vec2();
this.m_localYAxis1 =  new b2Vec2();
this.m_linearJacobian =  new b2Jacobian();
this.m_motorJacobian =  new b2Jacobian();
}
// static attributes
// static methods
// attributes
b2PrismaticJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2PrismaticJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2PrismaticJoint.prototype.m_localXAxis1 =  new b2Vec2();
b2PrismaticJoint.prototype.m_localYAxis1 =  new b2Vec2();
b2PrismaticJoint.prototype.m_refAngle =  null;
b2PrismaticJoint.prototype.m_linearJacobian =  new b2Jacobian();
b2PrismaticJoint.prototype.m_linearMass =  null;
b2PrismaticJoint.prototype.m_force =  null;
b2PrismaticJoint.prototype.m_angularMass =  null;
b2PrismaticJoint.prototype.m_torque =  null;
b2PrismaticJoint.prototype.m_motorJacobian =  new b2Jacobian();
b2PrismaticJoint.prototype.m_motorMass =  null;
b2PrismaticJoint.prototype.m_motorForce =  null;
b2PrismaticJoint.prototype.m_limitForce =  null;
b2PrismaticJoint.prototype.m_limitPositionImpulse =  null;
b2PrismaticJoint.prototype.m_lowerTranslation =  null;
b2PrismaticJoint.prototype.m_upperTranslation =  null;
b2PrismaticJoint.prototype.m_maxMotorForce =  null;
b2PrismaticJoint.prototype.m_motorSpeed =  null;
b2PrismaticJoint.prototype.m_enableLimit =  null;
b2PrismaticJoint.prototype.m_enableMotor =  null;
b2PrismaticJoint.prototype.m_limitState =  0;
// methods
b2PrismaticJoint.prototype.GetAnchor1 = function () {
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}
b2PrismaticJoint.prototype.GetAnchor2 = function () {
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}
b2PrismaticJoint.prototype.GetReactionForce = function () {
		var tMat = this.m_body1.m_xf.R;
		
		var ax1X = this.m_limitForce* (tMat.col1.x * this.m_localXAxis1.x + tMat.col2.x * this.m_localXAxis1.y);
		var ax1Y = this.m_limitForce* (tMat.col1.y * this.m_localXAxis1.x + tMat.col2.y * this.m_localXAxis1.y);
		
		var ay1X = this.m_force* (tMat.col1.x * this.m_localYAxis1.x + tMat.col2.x * this.m_localYAxis1.y);
		var ay1Y = this.m_force* (tMat.col1.y * this.m_localYAxis1.x + tMat.col2.y * this.m_localYAxis1.y);
		
		
		return new b2Vec2( this.m_limitForce*ax1X + this.m_force*ay1X, this.m_limitForce*ax1Y + this.m_force*ay1Y);
	}
b2PrismaticJoint.prototype.GetReactionTorque = function () {
		return this.m_torque;
	}
b2PrismaticJoint.prototype.GetJointTranslation = function () {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var tMat;
		
		var p1 = b1.GetWorldPoint(this.m_localAnchor1);
		var p2 = b2.GetWorldPoint(this.m_localAnchor2);
		
		var dX = p2.x - p1.x;
		var dY = p2.y - p1.y;
		
		var axis = b1.GetWorldVector(this.m_localXAxis1);
		
		
		var translation = axis.x*dX + axis.y*dY;
		return translation;
	}
b2PrismaticJoint.prototype.GetJointSpeed = function () {
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
		
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		
		var axis = b1.GetWorldVector(this.m_localXAxis1);
		
		var v1 = b1.m_linearVelocity;
		var v2 = b2.m_linearVelocity;
		var w1 = b1.m_angularVelocity;
		var w2 = b2.m_angularVelocity;
		
		
		
		
		var speed = (dX*(-w1 * axis.y) + dY*(w1 * axis.x)) + (axis.x * ((( v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * ((( v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
		
		return speed;
	}
b2PrismaticJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
b2PrismaticJoint.prototype.EnableLimit = function (flag) {
		this.m_enableLimit = flag;
	}
b2PrismaticJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerTranslation;
	}
b2PrismaticJoint.prototype.GetUpperLimit = function () {
		return this.m_upperTranslation;
	}
b2PrismaticJoint.prototype.SetLimits = function (lower, upper) {
		
		this.m_lowerTranslation = lower;
		this.m_upperTranslation = upper;
	}
b2PrismaticJoint.prototype.IsMotorEnabled = function () {
		return this.m_enableMotor;
	}
b2PrismaticJoint.prototype.EnableMotor = function (flag) {
		this.m_enableMotor = flag;
	}
b2PrismaticJoint.prototype.SetMotorSpeed = function (speed) {
		this.m_motorSpeed = speed;
	}
b2PrismaticJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
b2PrismaticJoint.prototype.SetMaxMotorForce = function (force) {
		this.m_maxMotorForce = force;
	}
b2PrismaticJoint.prototype.GetMotorForce = function () {
		return this.m_motorForce;
	}
b2PrismaticJoint.prototype.InitVelocityConstraints = function (step) {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var tMat;
		var tX;
		
		
		
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
		
		
		var invMass1 = b1.m_invMass;
		var invMass2 = b2.m_invMass;
		
		var invI1 = b1.m_invI;
		var invI2 = b2.m_invI;
		
		
		
		
		tMat = b1.m_xf.R;
		var ay1X = tMat.col1.x * this.m_localYAxis1.x + tMat.col2.x * this.m_localYAxis1.y;
		var ay1Y = tMat.col1.y * this.m_localYAxis1.x + tMat.col2.y * this.m_localYAxis1.y;
		
		var eX = b2.m_sweep.c.x + r2X - b1.m_sweep.c.x;
		var eY = b2.m_sweep.c.y + r2Y - b1.m_sweep.c.y;
		
		
		this.m_linearJacobian.linear1.x = -ay1X; 
		this.m_linearJacobian.linear1.y = -ay1Y;
		this.m_linearJacobian.linear2.x = ay1X; 
		this.m_linearJacobian.linear2.y = ay1Y;
		this.m_linearJacobian.angular1 = -(eX * ay1Y - eY * ay1X); 
		this.m_linearJacobian.angular2 = r2X * ay1Y - r2Y * ay1X; 
		
		this.m_linearMass =	invMass1 + invI1 * this.m_linearJacobian.angular1 * this.m_linearJacobian.angular1 +
						invMass2 + invI2 * this.m_linearJacobian.angular2 * this.m_linearJacobian.angular2;
		
		this.m_linearMass = 1.0 / this.m_linearMass;
		
		
		this.m_angularMass = invI1 + invI2;
		if (this.m_angularMass > Number.MIN_VALUE)
		{
			this.m_angularMass = 1.0 / this.m_angularMass;
		}
		
		
		if (this.m_enableLimit || this.m_enableMotor)
		{
			
			
			tMat = b1.m_xf.R;
			var ax1X = tMat.col1.x * this.m_localXAxis1.x + tMat.col2.x * this.m_localXAxis1.y;
			var ax1Y = tMat.col1.y * this.m_localXAxis1.x + tMat.col2.y * this.m_localXAxis1.y;
			
			this.m_motorJacobian.linear1.x = -ax1X; this.m_motorJacobian.linear1.y = -ax1Y;
			this.m_motorJacobian.linear2.x = ax1X; this.m_motorJacobian.linear2.y = ax1Y;
			this.m_motorJacobian.angular1 = -(eX * ax1Y - eY * ax1X); 
			this.m_motorJacobian.angular2 = r2X * ax1Y - r2Y * ax1X; 
			
			this.m_motorMass =	invMass1 + invI1 * this.m_motorJacobian.angular1 * this.m_motorJacobian.angular1 +
							invMass2 + invI2 * this.m_motorJacobian.angular2 * this.m_motorJacobian.angular2;
			
			this.m_motorMass = 1.0 / this.m_motorMass;
			
			if (this.m_enableLimit)
			{
				
				var dX = eX - r1X;
				var dY = eY - r1Y;
				
				var jointTranslation = ax1X * dX + ax1Y * dY;
				if (b2Math.b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop)
				{
					this.m_limitState = b2Joint.e_equalLimits;
				}
				else if (jointTranslation <= this.m_lowerTranslation)
				{
					if (this.m_limitState != b2Joint.e_atLowerLimit)
					{
						this.m_limitForce = 0.0;
					}
					this.m_limitState = b2Joint.e_atLowerLimit;
				}
				else if (jointTranslation >= this.m_upperTranslation)
				{
					if (this.m_limitState != b2Joint.e_atUpperLimit)
					{
						this.m_limitForce = 0.0;
					}
					this.m_limitState = b2Joint.e_atUpperLimit;
				}
				else
				{
					this.m_limitState = b2Joint.e_inactiveLimit;
					this.m_limitForce = 0.0;
				}
			}
		}
		
		if (this.m_enableMotor == false)
		{
			this.m_motorForce = 0.0;
		}
		
		if (this.m_enableLimit == false)
		{
			this.m_limitForce = 0.0;
		}
		
		if (step.warmStarting)
		{
			
			var P1X = step.dt * (this.m_force * this.m_linearJacobian.linear1.x + (this.m_motorForce + this.m_limitForce) * this.m_motorJacobian.linear1.x);
			var P1Y = step.dt * (this.m_force * this.m_linearJacobian.linear1.y + (this.m_motorForce + this.m_limitForce) * this.m_motorJacobian.linear1.y);
			
			var P2X = step.dt * (this.m_force * this.m_linearJacobian.linear2.x + (this.m_motorForce + this.m_limitForce) * this.m_motorJacobian.linear2.x);
			var P2Y = step.dt * (this.m_force * this.m_linearJacobian.linear2.y + (this.m_motorForce + this.m_limitForce) * this.m_motorJacobian.linear2.y);
			
			var L1 = step.dt * (this.m_force * this.m_linearJacobian.angular1 - this.m_torque + (this.m_motorForce + this.m_limitForce) * this.m_motorJacobian.angular1);
			
			var L2 = step.dt * (this.m_force * this.m_linearJacobian.angular2 + this.m_torque + (this.m_motorForce + this.m_limitForce) * this.m_motorJacobian.angular2);
			
			
			b1.m_linearVelocity.x += invMass1 * P1X;
			b1.m_linearVelocity.y += invMass1 * P1Y;
			
			b1.m_angularVelocity += invI1 * L1;
			
			
			b2.m_linearVelocity.x += invMass2 * P2X;
			b2.m_linearVelocity.y += invMass2 * P2Y;
			
			b2.m_angularVelocity += invI2 * L2;
		}
		else
		{
			this.m_force = 0.0;
			this.m_torque = 0.0;
			this.m_limitForce = 0.0;
			this.m_motorForce = 0.0;
		}
		
		this.m_limitPositionImpulse = 0.0;
		
	}
b2PrismaticJoint.prototype.SolveVelocityConstraints = function (step) {
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var invMass1 = b1.m_invMass;
		var invMass2 = b2.m_invMass;
		var invI1 = b1.m_invI;
		var invI2 = b2.m_invI;
		
		var oldLimitForce;
		
		
		var linearCdot = this.m_linearJacobian.Compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity);
		var force = -step.inv_dt * this.m_linearMass * linearCdot;
		this.m_force += force;
		
		var P = step.dt * force;
		
		b1.m_linearVelocity.x += (invMass1 * P) * this.m_linearJacobian.linear1.x;
		b1.m_linearVelocity.y += (invMass1 * P) * this.m_linearJacobian.linear1.y;
		
		b1.m_angularVelocity += invI1 * P * this.m_linearJacobian.angular1;
		
		
		b2.m_linearVelocity.x += (invMass2 * P) * this.m_linearJacobian.linear2.x;
		b2.m_linearVelocity.y += (invMass2 * P) * this.m_linearJacobian.linear2.y;
		
		b2.m_angularVelocity += invI2 * P * this.m_linearJacobian.angular2;
		
		
		var angularCdot = b2.m_angularVelocity - b1.m_angularVelocity;
		var torque = -step.inv_dt * this.m_angularMass * angularCdot;
		this.m_torque += torque;
		
		var L = step.dt * torque;
		b1.m_angularVelocity -= invI1 * L;
		b2.m_angularVelocity += invI2 * L;
		
		
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits)
		{
			var motorCdot = this.m_motorJacobian.Compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity) - this.m_motorSpeed;
			var motorForce = -step.inv_dt * this.m_motorMass * motorCdot;
			var oldMotorForce = this.m_motorForce;
			this.m_motorForce = b2Math.b2Clamp(this.m_motorForce + motorForce, -this.m_maxMotorForce, this.m_maxMotorForce);
			motorForce = this.m_motorForce - oldMotorForce;
			
			P = step.dt * motorForce;
			
			b1.m_linearVelocity.x += (invMass1 * P) * this.m_motorJacobian.linear1.x;
			b1.m_linearVelocity.y += (invMass1 * P) * this.m_motorJacobian.linear1.y;
			
			b1.m_angularVelocity += invI1 * P * this.m_motorJacobian.angular1;
			
			
			b2.m_linearVelocity.x += (invMass2 * P) * this.m_motorJacobian.linear2.x;
			b2.m_linearVelocity.y += (invMass2 * P) * this.m_motorJacobian.linear2.y;
			
			b2.m_angularVelocity += invI2 * P * this.m_motorJacobian.angular2;
		}
		
		
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit)
		{
			var limitCdot = this.m_motorJacobian.Compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity);
			var limitForce = -step.inv_dt * this.m_motorMass * limitCdot;
			
			if (this.m_limitState == b2Joint.e_equalLimits)
			{
				this.m_limitForce += limitForce;
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit)
			{
				oldLimitForce = this.m_limitForce;
				this.m_limitForce = b2Math.b2Max(this.m_limitForce + limitForce, 0.0);
				limitForce = this.m_limitForce - oldLimitForce;
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit)
			{
				oldLimitForce = this.m_limitForce;
				this.m_limitForce = b2Math.b2Min(this.m_limitForce + limitForce, 0.0);
				limitForce = this.m_limitForce - oldLimitForce;
			}
			
			P = step.dt * limitForce;
			
			b1.m_linearVelocity.x += (invMass1 * P) * this.m_motorJacobian.linear1.x;
			b1.m_linearVelocity.y += (invMass1 * P) * this.m_motorJacobian.linear1.y;
			
			b1.m_angularVelocity += invI1 * P * this.m_motorJacobian.angular1;
			
			
			b2.m_linearVelocity.x += (invMass2 * P) * this.m_motorJacobian.linear2.x;
			b2.m_linearVelocity.y += (invMass2 * P) * this.m_motorJacobian.linear2.y;
			
			b2.m_angularVelocity += invI2 * P * this.m_motorJacobian.angular2;
		}
	}
b2PrismaticJoint.prototype.SolvePositionConstraints = function () {
		
		var limitC;
		var oldLimitImpulse;
		
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var invMass1 = b1.m_invMass;
		var invMass2 = b2.m_invMass;
		var invI1 = b1.m_invI;
		var invI2 = b2.m_invI;
		
		var tMat;
		var tX;
		
		
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
		
		
		var p1X = b1.m_sweep.c.x + r1X;
		var p1Y = b1.m_sweep.c.y + r1Y;
		
		var p2X = b2.m_sweep.c.x + r2X;
		var p2Y = b2.m_sweep.c.y + r2Y;
		
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		
		tMat = b1.m_xf.R;
		var ay1X = tMat.col1.x * this.m_localYAxis1.x + tMat.col2.x * this.m_localYAxis1.y;
		var ay1Y = tMat.col1.y * this.m_localYAxis1.x + tMat.col2.y * this.m_localYAxis1.y;
		
		
		
		var linearC = ay1X*dX + ay1Y*dY;
		
		linearC = b2Math.b2Clamp(linearC, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
		var linearImpulse = -this.m_linearMass * linearC;
		
		
		b1.m_sweep.c.x += (invMass1 * linearImpulse) * this.m_linearJacobian.linear1.x;
		b1.m_sweep.c.y += (invMass1 * linearImpulse) * this.m_linearJacobian.linear1.y;
		
		b1.m_sweep.a += invI1 * linearImpulse * this.m_linearJacobian.angular1;
		
		
		
		b2.m_sweep.c.x += (invMass2 * linearImpulse) * this.m_linearJacobian.linear2.x;
		b2.m_sweep.c.y += (invMass2 * linearImpulse) * this.m_linearJacobian.linear2.y;
		
		b2.m_sweep.a += invI2 * linearImpulse * this.m_linearJacobian.angular2;
		
		
		var positionError = b2Math.b2Abs(linearC);
		
		
		var angularC = b2.m_sweep.a - b1.m_sweep.a - this.m_refAngle;
		
		angularC = b2Math.b2Clamp(angularC, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
		var angularImpulse = -this.m_angularMass * angularC;
		
		b1.m_sweep.a -= b1.m_invI * angularImpulse;
		b2.m_sweep.a += b2.m_invI * angularImpulse;
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		var angularError = b2Math.b2Abs(angularC);
		
		
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit)
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
			
			dX = p2X - p1X;
			dY = p2Y - p1Y;
			
			tMat = b1.m_xf.R;
			var ax1X = tMat.col1.x * this.m_localXAxis1.x + tMat.col2.x * this.m_localXAxis1.y;
			var ax1Y = tMat.col1.y * this.m_localXAxis1.x + tMat.col2.y * this.m_localXAxis1.y;
			
			
			var translation = (ax1X*dX + ax1Y*dY);
			var limitImpulse = 0.0;
			
			if (this.m_limitState == b2Joint.e_equalLimits)
			{
				
				limitC = b2Math.b2Clamp(translation, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
				limitImpulse = -this.m_motorMass * limitC;
				positionError = b2Math.b2Max(positionError, b2Math.b2Abs(angularC));
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit)
			{
				limitC = translation - this.m_lowerTranslation;
				positionError = b2Math.b2Max(positionError, -limitC);
				
				
				limitC = b2Math.b2Clamp(limitC + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
				limitImpulse = -this.m_motorMass * limitC;
				oldLimitImpulse = this.m_limitPositionImpulse;
				this.m_limitPositionImpulse = b2Math.b2Max(this.m_limitPositionImpulse + limitImpulse, 0.0);
				limitImpulse = this.m_limitPositionImpulse - oldLimitImpulse;
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit)
			{
				limitC = translation - this.m_upperTranslation;
				positionError = b2Math.b2Max(positionError, limitC);
				
				
				limitC = b2Math.b2Clamp(limitC - b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
				limitImpulse = -this.m_motorMass * limitC;
				oldLimitImpulse = this.m_limitPositionImpulse;
				this.m_limitPositionImpulse = b2Math.b2Min(this.m_limitPositionImpulse + limitImpulse, 0.0);
				limitImpulse = this.m_limitPositionImpulse - oldLimitImpulse;
			}
			
			
			b1.m_sweep.c.x += (invMass1 * limitImpulse) * this.m_motorJacobian.linear1.x;
			b1.m_sweep.c.y += (invMass1 * limitImpulse) * this.m_motorJacobian.linear1.y;
			
			b1.m_sweep.a += invI1 * limitImpulse * this.m_motorJacobian.angular1;
			
			
			b2.m_sweep.c.x += (invMass2 * limitImpulse) * this.m_motorJacobian.linear2.x;
			b2.m_sweep.c.y += (invMass2 * limitImpulse) * this.m_motorJacobian.linear2.y;
			
			b2.m_sweep.a += invI2 * limitImpulse * this.m_motorJacobian.angular2;
			
			b1.SynchronizeTransform();
			b2.SynchronizeTransform();
			
		}
		
		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
		
	}