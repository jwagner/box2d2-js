var b2PrismaticJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PrismaticJointDef.prototype, b2JointDef.prototype)
b2PrismaticJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2PrismaticJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_prismaticJoint;
		
		
		this.localAxis1.Set(1.0, 0.0);
		this.referenceAngle = 0.0;
		this.enableLimit = false;
		this.lowerTranslation = 0.0;
		this.upperTranslation = 0.0;
		this.enableMotor = false;
		this.maxMotorForce = 0.0;
		this.motorSpeed = 0.0;
	}
b2PrismaticJointDef.prototype.__varz = function(){
this.localAnchor1 =  new b2Vec2();
this.localAnchor2 =  new b2Vec2();
this.localAxis1 =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2PrismaticJointDef.prototype.localAnchor1 =  new b2Vec2();
b2PrismaticJointDef.prototype.localAnchor2 =  new b2Vec2();
b2PrismaticJointDef.prototype.localAxis1 =  new b2Vec2();
b2PrismaticJointDef.prototype.referenceAngle =  null;
b2PrismaticJointDef.prototype.enableLimit =  null;
b2PrismaticJointDef.prototype.lowerTranslation =  null;
b2PrismaticJointDef.prototype.upperTranslation =  null;
b2PrismaticJointDef.prototype.enableMotor =  null;
b2PrismaticJointDef.prototype.maxMotorForce =  null;
b2PrismaticJointDef.prototype.motorSpeed =  null;
// methods
b2PrismaticJointDef.prototype.Initialize = function (b1, b2, anchor, axis) {
		this.body1 = b1;
		this.body2 = b2;
		this.localAnchor1 = this.body1.GetLocalPoint(anchor);
		this.localAnchor2 = this.body2.GetLocalPoint(anchor);
		this.localAxis1 = this.body1.GetLocalVector(axis);
		this.referenceAngle = this.body2.GetAngle() - this.body1.GetAngle();
	}