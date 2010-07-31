var b2Math = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Math.prototype.__constructor = function(){}
b2Math.prototype.__varz = function(){
}
// static attributes
b2Math.b2Vec2_zero =  new b2Vec2(0.0, 0.0);
b2Math.b2Mat22_identity =  new b2Mat22(0, new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
b2Math.b2XForm_identity =  new b2XForm(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
// static methods
b2Math.b2IsValid = function (x) {
		return isFinite(x);
	}
b2Math.b2Dot = function (a, b) {
		return a.x * b.x + a.y * b.y;
	}
b2Math.b2CrossVV = function (a, b) {
		return a.x * b.y - a.y * b.x;
	}
b2Math.b2CrossVF = function (a, s) {
		var v = new b2Vec2(s * a.y, -s * a.x);
		return v;
	}
b2Math.b2CrossFV = function (s, a) {
		var v = new b2Vec2(-s * a.y, s * a.x);
		return v;
	}
b2Math.b2MulMV = function (A, v) {
		
		
		var u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
		return u;
	}
b2Math.b2MulTMV = function (A, v) {
		
		
		var u = new b2Vec2(b2Math.b2Dot(v, A.col1), b2Math.b2Dot(v, A.col2));
		return u;
	}
b2Math.b2MulX = function (T, v) {
		var a = b2Math.b2MulMV(T.R, v);
		a.x += T.position.x;
		a.y += T.position.y;
		
		return a;
	}
b2Math.b2MulXT = function (T, v) {
		var a = b2Math.SubtractVV(v, T.position);
		
		var tX = (a.x * T.R.col1.x + a.y * T.R.col1.y );
		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y );
		a.x = tX;
		return a;
	}
b2Math.AddVV = function (a, b) {
		var v = new b2Vec2(a.x + b.x, a.y + b.y);
		return v;
	}
b2Math.SubtractVV = function (a, b) {
		var v = new b2Vec2(a.x - b.x, a.y - b.y);
		return v;
	}
b2Math.b2Distance = function (a, b) {
		var cX = a.x-b.x;
		var cY = a.y-b.y;
		return Math.sqrt(cX*cX + cY*cY);
	}
b2Math.b2DistanceSquared = function (a, b) {
		var cX = a.x-b.x;
		var cY = a.y-b.y;
		return (cX*cX + cY*cY);
	}
b2Math.MulFV = function (s, a) {
		var v = new b2Vec2(s * a.x, s * a.y);
		return v;
	}
b2Math.AddMM = function (A, B) {
		var C = new b2Mat22(0, b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
		return C;
	}
b2Math.b2MulMM = function (A, B) {
		var C = new b2Mat22(0, b2Math.b2MulMV(A, B.col1), b2Math.b2MulMV(A, B.col2));
		return C;
	}
b2Math.b2MulTMM = function (A, B) {
		var c1 = new b2Vec2(b2Math.b2Dot(A.col1, B.col1), b2Math.b2Dot(A.col2, B.col1));
		var c2 = new b2Vec2(b2Math.b2Dot(A.col1, B.col2), b2Math.b2Dot(A.col2, B.col2));
		var C = new b2Mat22(0, c1, c2);
		return C;
	}
b2Math.b2Abs = function (a) {
		return a > 0.0 ? a : -a;
	}
b2Math.b2AbsV = function (a) {
		var b = new b2Vec2(b2Math.b2Abs(a.x), b2Math.b2Abs(a.y));
		return b;
	}
b2Math.b2AbsM = function (A) {
		var B = new b2Mat22(0, b2Math.b2AbsV(A.col1), b2Math.b2AbsV(A.col2));
		return B;
	}
b2Math.b2Min = function (a, b) {
		return a < b ? a : b;
	}
b2Math.b2MinV = function (a, b) {
		var c = new b2Vec2(b2Math.b2Min(a.x, b.x), b2Math.b2Min(a.y, b.y));
		return c;
	}
b2Math.b2Max = function (a, b) {
		return a > b ? a : b;
	}
b2Math.b2MaxV = function (a, b) {
		var c = new b2Vec2(b2Math.b2Max(a.x, b.x), b2Math.b2Max(a.y, b.y));
		return c;
	}
b2Math.b2Clamp = function (a, low, high) {
		return b2Math.b2Max(low, b2Math.b2Min(a, high));
	}
b2Math.b2ClampV = function (a, low, high) {
		return b2Math.b2MaxV(low, b2Math.b2MinV(a, high));
	}
b2Math.b2Swap = function (a, b) {
		var tmp = a[0];
		a[0] = b[0];
		b[0] = tmp;
	}
b2Math.b2Random = function () {
		return Math.random() * 2 - 1;
	}
b2Math.b2RandomRange = function (lo, hi) {
		var r = Math.random();
		r = (hi - lo) * r + lo;
		return r;
	}
b2Math.b2NextPowerOfTwo = function (x) {
		x |= (x >> 1) & 0x7FFFFFFF;
		x |= (x >> 2) & 0x3FFFFFFF;
		x |= (x >> 4) & 0x0FFFFFFF;
		x |= (x >> 8) & 0x00FFFFFF;
		x |= (x >> 16)& 0x0000FFFF;
		return x + 1;
	}
b2Math.b2IsPowerOfTwo = function (x) {
		var result = x > 0 && (x & (x - 1)) == 0;
		return result;
	}
// attributes
// methods