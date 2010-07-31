var b2Color = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Color.prototype.__constructor = function (rr, gg, bb) {
		this._r = parseInt(255 * b2Math.b2Clamp(rr, 0.0, 1.0));
		this._g = parseInt(255 * b2Math.b2Clamp(gg, 0.0, 1.0));
		this._b = parseInt(255 * b2Math.b2Clamp(bb, 0.0, 1.0));
	}
b2Color.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2Color.prototype._r =  0;
b2Color.prototype._g =  0;
b2Color.prototype._b =  0;
// methods
b2Color.prototype.Set = function (rr, gg, bb) {
		this._r = parseInt(255 * b2Math.b2Clamp(rr, 0.0, 1.0));
		this._g = parseInt(255 * b2Math.b2Clamp(gg, 0.0, 1.0));
		this._b = parseInt(255 * b2Math.b2Clamp(bb, 0.0, 1.0));
	}
b2Color.prototype.set = function (rr) {
		this._r = parseInt(255 * b2Math.b2Clamp(rr, 0.0, 1.0));
	}
b2Color.prototype.set = function (gg) {
		this._g = parseInt(255 * b2Math.b2Clamp(gg, 0.0, 1.0));
	}
b2Color.prototype.set = function (bb) {
		this._b = parseInt(255 * b2Math.b2Clamp(bb, 0.0, 1.0));
	}
b2Color.prototype.get = function () {
		return (this._r) | (this._g << 8) | (this._b << 16);
	}