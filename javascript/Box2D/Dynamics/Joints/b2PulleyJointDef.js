var b2PulleyJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PulleyJointDef.prototype, b2JointDef.prototype)
b2PulleyJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2PulleyJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_pulleyJoint;
		this.groundAnchor1.Set(-1.0, 1.0);
		this.groundAnchor2.Set(1.0, 1.0);
		this.localAnchor1.Set(-1.0, 0.0);
		this.localAnchor2.Set(1.0, 0.0);
		this.length1 = 0.0;
		this.maxLength1 = 0.0;
		this.length2 = 0.0;
		this.maxLength2 = 0.0;
		this.ratio = 1.0;
		this.collideConnected = true;
	}
b2PulleyJointDef.prototype.__varz = function(){
this.groundAnchor1 =  new b2Vec2();
this.groundAnchor2 =  new b2Vec2();
this.localAnchor1 =  new b2Vec2();
this.localAnchor2 =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2PulleyJointDef.prototype.groundAnchor1 =  new b2Vec2();
b2PulleyJointDef.prototype.groundAnchor2 =  new b2Vec2();
b2PulleyJointDef.prototype.localAnchor1 =  new b2Vec2();
b2PulleyJointDef.prototype.localAnchor2 =  new b2Vec2();
b2PulleyJointDef.prototype.length1 =  null;
b2PulleyJointDef.prototype.maxLength1 =  null;
b2PulleyJointDef.prototype.length2 =  null;
b2PulleyJointDef.prototype.maxLength2 =  null;
b2PulleyJointDef.prototype.ratio =  null;
// methods
b2PulleyJointDef.prototype.Initialize = function (b1, b2,
				ga1, ga2,
				anchor1, anchor2,
				r) {
		this.body1 = b1;
		this.body2 = b2;
		this.groundAnchor1.SetV( ga1 );
		this.groundAnchor2.SetV( ga2 );
		this.localAnchor1 = this.body1.GetLocalPoint(anchor1);
		this.localAnchor2 = this.body2.GetLocalPoint(anchor2);
		
		var d1X = anchor1.x - ga1.x;
		var d1Y = anchor1.y - ga1.y;
		
		this.length1 = Math.sqrt(d1X*d1X + d1Y*d1Y);
		
		
		var d2X = anchor2.x - ga2.x;
		var d2Y = anchor2.y - ga2.y;
		
		this.length2 = Math.sqrt(d2X*d2X + d2Y*d2Y);
		
		this.ratio = r;
		
		var C = this.length1 + this.ratio * this.length2;
		this.maxLength1 = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
		this.maxLength2 = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
	}