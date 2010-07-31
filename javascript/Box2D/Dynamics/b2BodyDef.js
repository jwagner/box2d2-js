var b2BodyDef = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2BodyDef.prototype.__constructor = function () {
		this.massData.center.SetZero();
		this.massData.mass = 0.0;
		this.massData.I = 0.0;
		this.userData = null;
		this.position.Set(0.0, 0.0);
		this.angle = 0.0;
		this.linearDamping = 0.0;
		this.angularDamping = 0.0;
		this.allowSleep = true;
		this.isSleeping = false;
		this.fixedRotation = false;
		this.isBullet = false;
	}
b2BodyDef.prototype.__varz = function(){
this.massData =  new b2MassData();
this.position =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2BodyDef.prototype.massData =  new b2MassData();
b2BodyDef.prototype.userData =  null;
b2BodyDef.prototype.position =  new b2Vec2();
b2BodyDef.prototype.angle =  null;
b2BodyDef.prototype.linearDamping =  null;
b2BodyDef.prototype.angularDamping =  null;
b2BodyDef.prototype.allowSleep =  null;
b2BodyDef.prototype.isSleeping =  null;
b2BodyDef.prototype.fixedRotation =  null;
b2BodyDef.prototype.isBullet =  null;
// methods