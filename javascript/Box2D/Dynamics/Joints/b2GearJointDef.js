var b2GearJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2GearJointDef.prototype, b2JointDef.prototype)
b2GearJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2GearJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_gearJoint;
		this.joint1 = null;
		this.joint2 = null;
		this.ratio = 1.0;
	}
b2GearJointDef.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2GearJointDef.prototype.joint1 =  null;
b2GearJointDef.prototype.joint2 =  null;
b2GearJointDef.prototype.ratio =  null;
// methods