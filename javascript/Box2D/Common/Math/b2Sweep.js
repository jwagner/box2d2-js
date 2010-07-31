var b2Sweep = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Sweep.prototype.__constructor = function(){}
b2Sweep.prototype.__varz = function(){
this.localCenter =  new b2Vec2();
this.c0 =  new b2Vec2;
this.c =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2Sweep.prototype.localCenter =  new b2Vec2();
b2Sweep.prototype.c0 =  new b2Vec2;
b2Sweep.prototype.c =  new b2Vec2();
b2Sweep.prototype.a0 =  null;
b2Sweep.prototype.a =  null;
b2Sweep.prototype.t0 =  null;
// methods
b2Sweep.prototype.GetXForm = function (xf, t) {
		
		
		if (1.0 - this.t0 > Number.MIN_VALUE)
		{
			var alpha = (t - this.t0) / (1.0 - this.t0);
			xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
			xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
			var angle = (1.0 - alpha) * this.a0 + alpha * this.a;
			xf.R.Set(angle);
		}
		else
		{
			xf.position.SetV(this.c);
			xf.R.Set(this.a);
		}
		
		
		
		var tMat = xf.R;
		xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
		xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);
		
	}
b2Sweep.prototype.Advance = function (t) {
		if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE)
		{
			var alpha = (t - this.t0) / (1.0 - this.t0);
			
			this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
			this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
			this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
			this.t0 = t;
		}
	}