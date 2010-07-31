var b2Jacobian = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Jacobian.prototype.__constructor = function(){}
b2Jacobian.prototype.__varz = function(){
this.linear1 =  new b2Vec2();
this.linear2 =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2Jacobian.prototype.linear1 =  new b2Vec2();
b2Jacobian.prototype.angular1 =  null;
b2Jacobian.prototype.linear2 =  new b2Vec2();
b2Jacobian.prototype.angular2 =  null;
// methods
b2Jacobian.prototype.SetZero = function () {
		this.linear1.SetZero(); this.angular1 = 0.0;
		this.linear2.SetZero(); this.angular2 = 0.0;
	}
b2Jacobian.prototype.Set = function (x1, a1, x2, a2) {
		this.linear1.SetV(x1); this.angular1 = a1;
		this.linear2.SetV(x2); this.angular2 = a2;
	}
b2Jacobian.prototype.Compute = function (x1, a1, x2, a2) {
		
		
		return (this.linear1.x*x1.x + this.linear1.y*x1.y) + this.angular1 * a1 + (this.linear2.x*x2.x + this.linear2.y*x2.y) + this.angular2 * a2;
	}