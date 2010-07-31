var b2XForm = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2XForm.prototype.__constructor = function (pos, r) {
		if (pos){
			this.position.SetV(pos);
			this.R.SetM(r);

		}
	}
b2XForm.prototype.__varz = function(){
this.position =  new b2Vec2;
this.R =  new b2Mat22();
}
// static attributes
// static methods
// attributes
b2XForm.prototype.position =  new b2Vec2;
b2XForm.prototype.R =  new b2Mat22();
// methods
b2XForm.prototype.Initialize = function (pos, r) {
		this.position.SetV(pos);
		this.R.SetM(r);
	}
b2XForm.prototype.SetIdentity = function () {
		this.position.SetZero();
		this.R.SetIdentity();
	}
b2XForm.prototype.Set = function (x) {

		this.position.SetV(x.position);

		this.R.SetM(x.R);

	}