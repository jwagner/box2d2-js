var b2DistanceJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2DistanceJointDef.prototype, b2JointDef.prototype)
b2DistanceJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2DistanceJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_distanceJoint;
		
		
		this.length = 1.0;
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}
b2DistanceJointDef.prototype.__varz = function(){
this.localAnchor1 =  new b2Vec2();
this.localAnchor2 =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2DistanceJointDef.prototype.localAnchor1 =  new b2Vec2();
b2DistanceJointDef.prototype.localAnchor2 =  new b2Vec2();
b2DistanceJointDef.prototype.length =  null;
b2DistanceJointDef.prototype.frequencyHz =  null;
b2DistanceJointDef.prototype.dampingRatio =  null;
// methods
b2DistanceJointDef.prototype.Initialize = function (b1, b2,
								anchor1, anchor2) {
		this.body1 = b1;
		this.body2 = b2;
		this.localAnchor1.SetV( this.body1.GetLocalPoint(anchor1));
		this.localAnchor2.SetV( this.body2.GetLocalPoint(anchor2));
		var dX = anchor2.x - anchor1.x;
		var dY = anchor2.y - anchor1.y;
		this.length = Math.sqrt(dX*dX + dY*dY);
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}