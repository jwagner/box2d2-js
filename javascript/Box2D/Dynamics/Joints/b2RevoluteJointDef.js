var b2RevoluteJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2RevoluteJointDef.prototype, b2JointDef.prototype)
b2RevoluteJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2RevoluteJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_revoluteJoint;
		this.localAnchor1.Set(0.0, 0.0);
		this.localAnchor2.Set(0.0, 0.0);
		this.referenceAngle = 0.0;
		this.lowerAngle = 0.0;
		this.upperAngle = 0.0;
		this.maxMotorTorque = 0.0;
		this.motorSpeed = 0.0;
		this.enableLimit = false;
		this.enableMotor = false;
	}
b2RevoluteJointDef.prototype.__varz = function(){
this.localAnchor1 =  new b2Vec2();
this.localAnchor2 =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2RevoluteJointDef.prototype.localAnchor1 =  new b2Vec2();
b2RevoluteJointDef.prototype.localAnchor2 =  new b2Vec2();
b2RevoluteJointDef.prototype.referenceAngle =  null;
b2RevoluteJointDef.prototype.enableLimit =  null;
b2RevoluteJointDef.prototype.lowerAngle =  null;
b2RevoluteJointDef.prototype.upperAngle =  null;
b2RevoluteJointDef.prototype.enableMotor =  null;
b2RevoluteJointDef.prototype.motorSpeed =  null;
b2RevoluteJointDef.prototype.maxMotorTorque =  null;
// methods
b2RevoluteJointDef.prototype.Initialize = function (b1, b2, anchor) {
		this.body1 = b1;
		this.body2 = b2;
		this.localAnchor1 = this.body1.GetLocalPoint(anchor);
		this.localAnchor2 = this.body2.GetLocalPoint(anchor);
		this.referenceAngle = this.body2.GetAngle() - this.body1.GetAngle();
	}