var b2MouseJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2MouseJointDef.prototype, b2JointDef.prototype)
b2MouseJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2MouseJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_mouseJoint;
		this.maxForce = 0.0;
		this.frequencyHz = 5.0;
		this.dampingRatio = 0.7;
		this.timeStep = 1.0 / 60.0;
	}
b2MouseJointDef.prototype.__varz = function(){
this.target =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2MouseJointDef.prototype.target =  new b2Vec2();
b2MouseJointDef.prototype.maxForce =  null;
b2MouseJointDef.prototype.frequencyHz =  null;
b2MouseJointDef.prototype.dampingRatio =  null;
b2MouseJointDef.prototype.timeStep =  null;
// methods