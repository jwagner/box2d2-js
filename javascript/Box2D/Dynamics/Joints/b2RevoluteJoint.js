var b2RevoluteJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2RevoluteJoint.prototype, b2Joint.prototype)
b2RevoluteJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2RevoluteJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		
		this.m_localAnchor1.SetV(def.localAnchor1);
		
		this.m_localAnchor2.SetV(def.localAnchor2);
		
		this.m_referenceAngle = def.referenceAngle;
		
		this.m_pivotForce.Set(0.0, 0.0);
		this.m_motorForce = 0.0;
		this.m_limitForce = 0.0;
		this.m_limitPositionImpulse = 0.0;
		
		this.m_lowerAngle = def.lowerAngle;
		this.m_upperAngle = def.upperAngle;
		this.m_maxMotorTorque = def.maxMotorTorque;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
	}
b2RevoluteJoint.prototype.__varz = function(){
this.K =  new b2Mat22();
this.K1 =  new b2Mat22();
this.K2 =  new b2Mat22();
this.K3 =  new b2Mat22();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_pivotForce =  new b2Vec2();
this.m_pivotMass =  new b2Mat22();
}
// static attributes
b2RevoluteJoint.tImpulse =  new b2Vec2();
// static methods
// attributes
b2RevoluteJoint.prototype.K =  new b2Mat22();
b2RevoluteJoint.prototype.K1 =  new b2Mat22();
b2RevoluteJoint.prototype.K2 =  new b2Mat22();
b2RevoluteJoint.prototype.K3 =  new b2Mat22();
b2RevoluteJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2RevoluteJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2RevoluteJoint.prototype.m_pivotForce =  new b2Vec2();
b2RevoluteJoint.prototype.m_motorForce =  null;
b2RevoluteJoint.prototype.m_limitForce =  null;
b2RevoluteJoint.prototype.m_limitPositionImpulse =  null;
b2RevoluteJoint.prototype.m_pivotMass =  new b2Mat22();
b2RevoluteJoint.prototype.m_motorMass =  null;
b2RevoluteJoint.prototype.m_enableMotor =  null;
b2RevoluteJoint.prototype.m_maxMotorTorque =  null;
b2RevoluteJoint.prototype.m_motorSpeed =  null;
b2RevoluteJoint.prototype.m_enableLimit =  null;
b2RevoluteJoint.prototype.m_referenceAngle =  null;
b2RevoluteJoint.prototype.m_lowerAngle =  null;
b2RevoluteJoint.prototype.m_upperAngle =  null;
b2RevoluteJoint.prototype.m_limitState =  0;
// methods
b2RevoluteJoint.prototype.GetAnchor1 = function () {
		return this.m_body1.GetWorldPoint(this.m_localAnchor1);
	}
b2RevoluteJoint.prototype.GetAnchor2 = function () {
		return this.m_body2.GetWorldPoint(this.m_localAnchor2);
	}
b2RevoluteJoint.prototype.GetReactionForce = function () {
		return this.m_pivotForce;
	}
b2RevoluteJoint.prototype.GetReactionTorque = function () {
		return this.m_limitForce;
	}
b2RevoluteJoint.prototype.GetJointAngle = function () {
		
		
		return this.m_body2.m_sweep.a - this.m_body1.m_sweep.a - this.m_referenceAngle;
	}
b2RevoluteJoint.prototype.GetJointSpeed = function () {
		
		
		return this.m_body2.m_angularVelocity - this.m_body1.m_angularVelocity;
	}
b2RevoluteJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
b2RevoluteJoint.prototype.EnableLimit = function (flag) {
		this.m_enableLimit = flag;
	}
b2RevoluteJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerAngle;
	}
b2RevoluteJoint.prototype.GetUpperLimit = function () {
		return this.m_upperAngle;
	}
b2RevoluteJoint.prototype.SetLimits = function (lower, upper) {
		
		this.m_lowerAngle = lower;
		this.m_upperAngle = upper;
	}
b2RevoluteJoint.prototype.IsMotorEnabled = function () {
		return this.m_enableMotor;
	}
b2RevoluteJoint.prototype.EnableMotor = function (flag) {
		this.m_enableMotor = flag;
	}
b2RevoluteJoint.prototype.SetMotorSpeed = function (speed) {
		this.m_motorSpeed = speed;
	}
b2RevoluteJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
b2RevoluteJoint.prototype.SetMaxMotorTorque = function (torque) {
		this.m_maxMotorTorque = torque;
	}
b2RevoluteJoint.prototype.GetMotorTorque = function () {
		return this.m_motorForce;
	}
b2RevoluteJoint.prototype.InitVelocityConstraints = function (step) {
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
		
		
		this.K1.col1.x = invMass1 + invMass2;	this.K1.col2.x = 0.0;
		this.K1.col1.y = 0.0;					this.K1.col2.y = invMass1 + invMass2;
		
		
		this.K2.col1.x = invI1 * r1Y * r1Y;	this.K2.col2.x = -invI1 * r1X * r1Y;
		this.K2.col1.y = -invI1 * r1X * r1Y;	this.K2.col2.y = invI1 * r1X * r1X;
		
		
		this.K3.col1.x = invI2 * r2Y * r2Y;	this.K3.col2.x = -invI2 * r2X * r2Y;
		this.K3.col1.y = -invI2 * r2X * r2Y;	this.K3.col2.y = invI2 * r2X * r2X;
		
		
		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.AddM(this.K3);
		
		
		this.K.Invert(this.m_pivotMass);
		
		this.m_motorMass = 1.0 / (invI1 + invI2);
		
		if (this.m_enableMotor == false)
		{
			this.m_motorForce = 0.0;
		}
		
		if (this.m_enableLimit)
		{
			
			var jointAngle = b2.m_sweep.a - b1.m_sweep.a - this.m_referenceAngle;
			if (b2Math.b2Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop)
			{
				this.m_limitState = b2Joint.e_equalLimits;
			}
			else if (jointAngle <= this.m_lowerAngle)
			{
				if (this.m_limitState != b2Joint.e_atLowerLimit)
				{
					this.m_limitForce = 0.0;
				}
				this.m_limitState = b2Joint.e_atLowerLimit;
			}
			else if (jointAngle >= this.m_upperAngle)
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
		else
		{
			this.m_limitForce = 0.0;
		}
		
		
		if (step.warmStarting)
		{
			
			b1.m_linearVelocity.x -= step.dt * invMass1 * this.m_pivotForce.x;
			b1.m_linearVelocity.y -= step.dt * invMass1 * this.m_pivotForce.y;
			
			b1.m_angularVelocity -= step.dt * invI1 * ((r1X * this.m_pivotForce.y - r1Y * this.m_pivotForce.x) + this.m_motorForce + this.m_limitForce);
			
			
			b2.m_linearVelocity.x += step.dt * invMass2 * this.m_pivotForce.x;
			b2.m_linearVelocity.y += step.dt * invMass2 * this.m_pivotForce.y;
			
			b2.m_angularVelocity += step.dt * invI2 * ((r2X * this.m_pivotForce.y - r2Y * this.m_pivotForce.x) + this.m_motorForce + this.m_limitForce);
		}
		else{
			this.m_pivotForce.SetZero();
			this.m_motorForce = 0.0;
			this.m_limitForce = 0.0;
		}
		
		this.m_limitPositionImpulse = 0.0;
	}
b2RevoluteJoint.prototype.SolveVelocityConstraints = function (step) {
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
		
		var oldLimitForce;
		
		
		
		var pivotCdotX = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y) - b1.m_linearVelocity.x - (-b1.m_angularVelocity * r1Y);
		var pivotCdotY = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X) - b1.m_linearVelocity.y - (b1.m_angularVelocity * r1X);
		
		
		var pivotForceX = -step.inv_dt * (this.m_pivotMass.col1.x * pivotCdotX + this.m_pivotMass.col2.x * pivotCdotY);
		var pivotForceY = -step.inv_dt * (this.m_pivotMass.col1.y * pivotCdotX + this.m_pivotMass.col2.y * pivotCdotY);
		this.m_pivotForce.x += pivotForceX;
		this.m_pivotForce.y += pivotForceY;
		
		
		var PX = step.dt * pivotForceX;
		var PY = step.dt * pivotForceY;
		
		
		b1.m_linearVelocity.x -= b1.m_invMass * PX;
		b1.m_linearVelocity.y -= b1.m_invMass * PY;
		
		b1.m_angularVelocity -= b1.m_invI * (r1X * PY - r1Y * PX);
		
		
		b2.m_linearVelocity.x += b2.m_invMass * PX;
		b2.m_linearVelocity.y += b2.m_invMass * PY;
		
		b2.m_angularVelocity += b2.m_invI * (r2X * PY - r2Y * PX);
		
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits)
		{
			var motorCdot = b2.m_angularVelocity - b1.m_angularVelocity - this.m_motorSpeed;
			var motorForce = -step.inv_dt * this.m_motorMass * motorCdot;
			var oldMotorForce = this.m_motorForce;
			this.m_motorForce = b2Math.b2Clamp(this.m_motorForce + motorForce, -this.m_maxMotorTorque, this.m_maxMotorTorque);
			motorForce = this.m_motorForce - oldMotorForce;
			
			b1.m_angularVelocity -= b1.m_invI * step.dt * motorForce;
			b2.m_angularVelocity += b2.m_invI * step.dt * motorForce;
		}
		
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit)
		{
			var limitCdot = b2.m_angularVelocity - b1.m_angularVelocity;
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
			
			b1.m_angularVelocity -= b1.m_invI * step.dt * limitForce;
			b2.m_angularVelocity += b2.m_invI * step.dt * limitForce;
		}
	}
b2RevoluteJoint.prototype.SolvePositionConstraints = function () {
		
		var oldLimitImpulse;
		var limitC;
		
		var b1 = this.m_body1;
		var b2 = this.m_body2;
		
		var positionError = 0.0;
		
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
		
		
		var ptpCX = p2X - p1X;
		var ptpCY = p2Y - p1Y;
		
		
		positionError = Math.sqrt(ptpCX*ptpCX + ptpCY*ptpCY);
		
		
		
		
		
		
		var invMass1 = b1.m_invMass;
		var invMass2 = b2.m_invMass;
		
		var invI1 = b1.m_invI;
		var invI2 = b2.m_invI;
		
		
		this.K1.col1.x = invMass1 + invMass2;	this.K1.col2.x = 0.0;
		this.K1.col1.y = 0.0;					this.K1.col2.y = invMass1 + invMass2;
		
		
		this.K2.col1.x = invI1 * r1Y * r1Y;	this.K2.col2.x = -invI1 * r1X * r1Y;
		this.K2.col1.y = -invI1 * r1X * r1Y;	this.K2.col2.y = invI1 * r1X * r1X;
		
		
		this.K3.col1.x = invI2 * r2Y * r2Y;		this.K3.col2.x = -invI2 * r2X * r2Y;
		this.K3.col1.y = -invI2 * r2X * r2Y;		this.K3.col2.y = invI2 * r2X * r2X;
		
		
		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.AddM(this.K3);
		
		this.K.Solve(b2RevoluteJoint.tImpulse, -ptpCX, -ptpCY);
		var impulseX = b2RevoluteJoint.tImpulse.x;
		var impulseY = b2RevoluteJoint.tImpulse.y;
		
		
		b1.m_sweep.c.x -= b1.m_invMass * impulseX;
		b1.m_sweep.c.y -= b1.m_invMass * impulseY;
		
		b1.m_sweep.a -= b1.m_invI * (r1X * impulseY - r1Y * impulseX);
		
		
		b2.m_sweep.c.x += b2.m_invMass * impulseX;
		b2.m_sweep.c.y += b2.m_invMass * impulseY;
		
		b2.m_sweep.a += b2.m_invI * (r2X * impulseY - r2Y * impulseX);
		
		b1.SynchronizeTransform();
		b2.SynchronizeTransform();
		
		
		
		var angularError = 0.0;
		
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit)
		{
			var angle = b2.m_sweep.a - b1.m_sweep.a - this.m_referenceAngle;
			var limitImpulse = 0.0;
			
			if (this.m_limitState == b2Joint.e_equalLimits)
			{
				
				limitC = b2Math.b2Clamp(angle, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * limitC;
				angularError = b2Math.b2Abs(limitC);
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit)
			{
				limitC = angle - this.m_lowerAngle;
				angularError = b2Math.b2Max(0.0, -limitC);
				
				
				limitC = b2Math.b2Clamp(limitC + b2Settings.b2_angularSlop, -b2Settings.b2_maxAngularCorrection, 0.0);
				limitImpulse = -this.m_motorMass * limitC;
				oldLimitImpulse = this.m_limitPositionImpulse;
				this.m_limitPositionImpulse = b2Math.b2Max(this.m_limitPositionImpulse + limitImpulse, 0.0);
				limitImpulse = this.m_limitPositionImpulse - oldLimitImpulse;
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit)
			{
				limitC = angle - this.m_upperAngle;
				angularError = b2Math.b2Max(0.0, limitC);
				
				
				limitC = b2Math.b2Clamp(limitC - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * limitC;
				oldLimitImpulse = this.m_limitPositionImpulse;
				this.m_limitPositionImpulse = b2Math.b2Min(this.m_limitPositionImpulse + limitImpulse, 0.0);
				limitImpulse = this.m_limitPositionImpulse - oldLimitImpulse;
			}
			
			b1.m_sweep.a -= b1.m_invI * limitImpulse;
			b2.m_sweep.a += b2.m_invI * limitImpulse;
			
			b1.SynchronizeTransform();
			b2.SynchronizeTransform();
		}
		
		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}