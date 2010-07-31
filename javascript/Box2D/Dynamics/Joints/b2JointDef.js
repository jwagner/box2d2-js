var b2JointDef = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2JointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_unknownJoint;
		this.userData = null;
		this.body1 = null;
		this.body2 = null;
		this.collideConnected = false;
	}
b2JointDef.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2JointDef.prototype.type =  0;
b2JointDef.prototype.userData =  null;
b2JointDef.prototype.body1 =  null;
b2JointDef.prototype.body2 =  null;
b2JointDef.prototype.collideConnected =  null;
// methods