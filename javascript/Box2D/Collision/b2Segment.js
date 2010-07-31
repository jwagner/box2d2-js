var b2Segment = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Segment.prototype.__constructor = function(){}
b2Segment.prototype.__varz = function(){
this.p1 =  new b2Vec2();
this.p2 =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2Segment.prototype.p1 =  new b2Vec2();
b2Segment.prototype.p2 =  new b2Vec2();
// methods
b2Segment.prototype.TestSegment = function (lambda, 
								normal, 
								segment, 
								maxLambda) {
		
		var s = segment.p1;
		
		var rX = segment.p2.x - s.x;
		var rY = segment.p2.y - s.y;
		
		var dX = this.p2.x - this.p1.x;
		var dY = this.p2.y - this.p1.y;
		
		var nX = dY;
		var nY = -dX;
		
		var k_slop = 100.0 * Number.MIN_VALUE;
		
		var denom = -(rX*nX + rY*nY);
		
		
		if (denom > k_slop)
		{
			
			
			var bX = s.x - this.p1.x;
			var bY = s.y - this.p1.y;
			
			var a = (bX*nX + bY*nY);
			
			if (0.0 <= a && a <= maxLambda * denom)
			{
				var mu2 = -rX * bY + rY * bX;
				
				
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop))
				{
					a /= denom;
					
					var nLen = Math.sqrt(nX*nX + nY*nY);
					nX /= nLen;
					nY /= nLen;
					
					lambda[0] = a;
					
					normal.Set(nX, nY);
					return true;
				}
			}
		}
		
		return false;
	}