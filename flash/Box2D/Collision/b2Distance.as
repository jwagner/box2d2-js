/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package Box2D.Collision{
	
import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.Shapes.*;
import Box2D.Collision.*;

public class b2Distance
{

// GJK using Voronoi regions (Christer Ericson) and region selection
// optimizations (Casey Muratori).

// The origin is either in the region of points[1] or in the edge region. The origin is
// not in region of points[0] because that is the old point.
static public function ProcessTwo(x1:b2Vec2, x2:b2Vec2, p1s:Array, p2s:Array, points:Array):int
{
	var points_0:b2Vec2 = points[0];
	var points_1:b2Vec2 = points[1];
	var p1s_0:b2Vec2 = p1s[0];
	var p1s_1:b2Vec2 = p1s[1];
	var p2s_0:b2Vec2 = p2s[0];
	var p2s_1:b2Vec2 = p2s[1];
	
	// If in point[1] region
	//b2Vec2 r = -points[1];
	var rX:Number = -points_1.x;
	var rY:Number = -points_1.y;
	//b2Vec2 d = points[1] - points[0];
	var dX:Number = points_0.x - points_1.x;
	var dY:Number = points_0.y - points_1.y;
	//float32 length = d.Normalize();
	var length:Number = Math.sqrt(dX*dX + dY*dY);
	dX /= length;
	dY /= length;
	
	//float32 lambda = b2Dot(r, d);
	var lambda:Number = rX * dX + rY * dY;
	if (lambda <= 0.0 || length < Number.MIN_VALUE)
	{
		// The simplex is reduced to a point.
		//*p1Out = p1s[1];
		x1.SetV(p1s_1);
		//*p2Out = p2s[1];
		x2.SetV(p2s_1);
		//p1s[0] = p1s[1];
		p1s_0.SetV(p1s_1);
		//p2s[0] = p2s[1];
		p2s_0.SetV(p2s_1);
		points_0.SetV(points_1);
		return 1;
	}

	// Else in edge region
	lambda /= length;
	//*x1 = p1s[1] + lambda * (p1s[0] - p1s[1]);
	x1.x = p1s_1.x + lambda * (p1s_0.x - p1s_1.x);
	x1.y = p1s_1.y + lambda * (p1s_0.y - p1s_1.y);
	//*x2 = p2s[1] + lambda * (p2s[0] - p2s[1]);
	x2.x = p2s_1.x + lambda * (p2s_0.x - p2s_1.x);
	x2.y = p2s_1.y + lambda * (p2s_0.y - p2s_1.y);
	return 2;
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
static public function ProcessThree(x1:b2Vec2, x2:b2Vec2, p1s:Array, p2s:Array, points:Array):int
{
	var points_0:b2Vec2 = points[0];
	var points_1:b2Vec2 = points[1];
	var points_2:b2Vec2 = points[2];
	var p1s_0:b2Vec2 = p1s[0];
	var p1s_1:b2Vec2 = p1s[1];
	var p1s_2:b2Vec2 = p1s[2];
	var p2s_0:b2Vec2 = p2s[0];
	var p2s_1:b2Vec2 = p2s[1];
	var p2s_2:b2Vec2 = p2s[2];
	
	//b2Vec2 a = points[0];
	var aX:Number = points_0.x;
	var aY:Number = points_0.y;
	//b2Vec2 b = points[1];
	var bX:Number = points_1.x;
	var bY:Number = points_1.y;
	//b2Vec2 c = points[2];
	var cX:Number = points_2.x;
	var cY:Number = points_2.y;

	//b2Vec2 ab = b - a;
	var abX:Number = bX - aX;
	var abY:Number = bY - aY;
	//b2Vec2 ac = c - a;
	var acX:Number = cX - aX;
	var acY:Number = cY - aY;
	//b2Vec2 bc = c - b;
	var bcX:Number = cX - bX;
	var bcY:Number = cY - bY;

	//float32 sn = -b2Dot(a, ab), sd = b2Dot(b, ab);
	var sn:Number = -(aX * abX + aY * abY);
	var sd:Number = (bX * abX + bY * abY);
	//float32 tn = -b2Dot(a, ac), td = b2Dot(c, ac);
	var tn:Number = -(aX * acX + aY * acY);
	var td:Number = (cX * acX + cY * acY);
	//float32 un = -b2Dot(b, bc), ud = b2Dot(c, bc);
	var un:Number = -(bX * bcX + bY * bcY);
	var ud:Number = (cX * bcX + cY * bcY);

	// In vertex c region?
	if (td <= 0.0 && ud <= 0.0)
	{
		// Single point
		//*x1 = p1s[2];
		x1.SetV(p1s_2);
		//*x2 = p2s[2];
		x2.SetV(p2s_2);
		//p1s[0] = p1s[2];
		p1s_0.SetV(p1s_2);
		//p2s[0] = p2s[2];
		p2s_0.SetV(p2s_2);
		points_0.SetV(points_2);
		return 1;
	}

	// Should not be in vertex a or b region.
	//b2Settings.b2Assert(sn > 0.0 || tn > 0.0);
	//b2Settings.b2Assert(sd > 0.0 || un > 0.0);

	//float32 n = b2Cross(ab, ac);
	var n:Number = abX * acY - abY * acX;

	// Should not be in edge ab region.
	//float32 vc = n * b2Cross(a, b);
	var vc:Number = n * (aX * bY - aY * bX); 
	//b2Settings.b2Assert(vc > 0.0 || sn > 0.0 || sd > 0.0);
	var lambda:Number;
	
	// In edge bc region?
	//float32 va = n * b2Cross(b, c);
	var va:Number = n * (bX * cY - bY * cX); 
	if (va <= 0.0 && un >= 0.0 && ud >= 0.0 && (un+ud) > 0.0)
	{
		//b2Settings.b2Assert(un + ud > 0.0);
		
		//float32 lambda = un / (un + ud);
		lambda = un / (un + ud);
		//*x1 = p1s[1] + lambda * (p1s[2] - p1s[1]);
		x1.x = p1s_1.x + lambda * (p1s_2.x - p1s_1.x);
		x1.y = p1s_1.y + lambda * (p1s_2.y - p1s_1.y);
		//*x2 = p2s[1] + lambda * (p2s[2] - p2s[1]);
		x2.x = p2s_1.x + lambda * (p2s_2.x - p2s_1.x);
		x2.y = p2s_1.y + lambda * (p2s_2.y - p2s_1.y);
		//p1s[0] = p1s[2];
		p1s_0.SetV(p1s_2);
		//p2s[0] = p2s[2];
		p2s_0.SetV(p2s_2);
		//points[0] = points[2];
		points_0.SetV(points_2);
		return 2;
	}

	// In edge ac region?
	//float32 vb = n * b2Cross(c, a);
	var vb:Number = n * (cX * aY - cY * aX);
	if (vb <= 0.0 && tn >= 0.0 && td >= 0.0 && (tn+td) > 0.0)
	{
		//b2Settings.b2Assert(tn + td > 0.0);
		
		//float32 lambda = tn / (tn + td);
		lambda = tn / (tn + td);
		//*x1 = p1s[0] + lambda * (p1s[2] - p1s[0]);
		x1.x = p1s_0.x + lambda * (p1s_2.x - p1s_0.x);
		x1.y = p1s_0.y + lambda * (p1s_2.y - p1s_0.y);
		//*x2 = p2s[0] + lambda * (p2s[2] - p2s[0]);
		x2.x = p2s_0.x + lambda * (p2s_2.x - p2s_0.x);
		x2.y = p2s_0.y + lambda * (p2s_2.y - p2s_0.y);
		//p1s[1] = p1s[2];
		p1s_1.SetV(p1s_2);
		//p2s[1] = p2s[2];
		p2s_1.SetV(p2s_2);
		//points[1] = points[2];
		points_1.SetV(points_2);
		return 2;
	}

	// Inside the triangle, compute barycentric coordinates
	//float32 denom = va + vb + vc;
	var denom:Number = va + vb + vc;
	//b2Settings.b2Assert(denom > 0.0);
	denom = 1.0 / denom;
	//float32 u = va * denom;
	var u:Number = va * denom;
	//float32 v = vb * denom;
	var v:Number = vb * denom;
	//float32 w = 1.0f - u - v;
	var w:Number = 1.0 - u - v;
	//*x1 = u * p1s[0] + v * p1s[1] + w * p1s[2];
	x1.x = u * p1s_0.x + v * p1s_1.x + w * p1s_2.x;
	x1.y = u * p1s_0.y + v * p1s_1.y + w * p1s_2.y;
	//*x2 = u * p2s[0] + v * p2s[1] + w * p2s[2];
	x2.x = u * p2s_0.x + v * p2s_1.x + w * p2s_2.x;
	x2.y = u * p2s_0.y + v * p2s_1.y + w * p2s_2.y;
	return 3;
}

static public function InPoints(w:b2Vec2, points:Array, pointCount:int):Boolean
{
	const k_tolerance:Number = 100.0 * Number.MIN_VALUE;
	for (var i:int = 0; i < pointCount; ++i)
	{
		var points_i:b2Vec2 = points[i];
		//b2Vec2 d = b2Abs(w - points[i]);
		var dX:Number = Math.abs(w.x - points_i.x);
		var dY:Number = Math.abs(w.y - points_i.y);
		//b2Vec2 m = b2Max(b2Abs(w), b2Abs(points[i]));
		var mX:Number = Math.max(Math.abs(w.x), Math.abs(points_i.x));
		var mY:Number = Math.max(Math.abs(w.y), Math.abs(points_i.y));
		
		if (dX < k_tolerance * (mX + 1.0) &&
			dY < k_tolerance * (mY + 1.0)){
			return true;
		}
	}

	return false;
}

// 
static private var s_p1s:Array = [new b2Vec2(), new b2Vec2(), new b2Vec2()];
static private var s_p2s:Array = [new b2Vec2(), new b2Vec2(), new b2Vec2()];
static private var s_points:Array = [new b2Vec2(), new b2Vec2(), new b2Vec2()];
//

static public function DistanceGeneric(x1:b2Vec2, x2:b2Vec2, 
										shape1:*, xf1:b2XForm, 
										shape2:*, xf2:b2XForm):Number
{
	var tVec: b2Vec2;
	
	//b2Vec2 p1s[3], p2s[3];
	var p1s:Array = s_p1s;
	var p2s:Array = s_p2s;
	//b2Vec2 points[3];
	var points:Array = s_points;
	//int32 pointCount = 0;
	var pointCount:int = 0;

	//*x1 = shape1->GetFirstVertex(xf1);
	x1.SetV(shape1.GetFirstVertex(xf1));
	//*x2 = shape2->GetFirstVertex(xf2);
	x2.SetV(shape2.GetFirstVertex(xf2));

	var vSqr:Number = 0.0;
	const maxIterations:int = 20;
	for (var iter:int = 0; iter < maxIterations; ++iter)
	{
		//b2Vec2 v = *x2 - *x1;
		var vX:Number = x2.x - x1.x;
		var vY:Number = x2.y - x1.y;
		//b2Vec2 w1 = shape1->Support(xf1, v);
		var w1:b2Vec2 = shape1.Support(xf1, vX, vY);
		//b2Vec2 w2 = shape2->Support(xf2, -v);
		var w2:b2Vec2 = shape2.Support(xf2, -vX, -vY);
		//float32 vSqr = b2Dot(v, v);
		vSqr = (vX*vX + vY*vY);
		//b2Vec2 w = w2 - w1;
		var wX:Number = w2.x - w1.x;
		var wY:Number = w2.y - w1.y;
		//float32 vw = b2Dot(v, w);
		var vw:Number = (vX*wX + vY*wY);
		//if (vSqr - b2Dot(v, w) <= 0.01f * vSqr) // or w in points
		if (vSqr - vw <= 0.01 * vSqr) // or w in points
		{
			if (pointCount == 0)
			{
				//*x1 = w1;
				x1.SetV(w1);
				//*x2 = w2;
				x2.SetV(w2);
			}
			g_GJK_Iterations = iter;
			return Math.sqrt(vSqr);
		}
		
		switch (pointCount)
		{
		case 0:
			//p1s[0] = w1;
			tVec = p1s[0];
			tVec.SetV(w1);
			//p2s[0] = w2;
			tVec = p2s[0];
			tVec.SetV(w2);
			//points[0] = w;
			tVec = points[0];
			tVec.x = wX;
			tVec.y = wY;
			//*x1 = p1s[0];
			x1.SetV(p1s[0]);
			//*x2 = p2s[0];
			x2.SetV(p2s[0]);
			++pointCount;
			break;
			
		case 1:
			//p1s[1] = w1;
			tVec = p1s[1];
			tVec.SetV(w1);
			//p2s[1] = w2;
			tVec = p2s[1];
			tVec.SetV(w2);
			//points[1] = w;
			tVec = points[1];
			tVec.x = wX;
			tVec.y = wY;
			pointCount = ProcessTwo(x1, x2, p1s, p2s, points);
			break;
			
		case 2:
			//p1s[2] = w1;
			tVec = p1s[2];
			tVec.SetV(w1);
			//p2s[2] = w2;
			tVec = p2s[2];
			tVec.SetV(w2);
			//points[2] = w;
			tVec = points[2];
			tVec.x = wX;
			tVec.y = wY;
			pointCount = ProcessThree(x1, x2, p1s, p2s, points);
			break;
		}
		
		// If we have three points, then the origin is in the corresponding triangle.
		if (pointCount == 3)
		{
			g_GJK_Iterations = iter;
			return 0.0;
		}
		
		//float32 maxSqr = -FLT_MAX;
		var maxSqr:Number = -Number.MAX_VALUE;
		for (var i:int = 0; i < pointCount; ++i)
		{
			//maxSqr = b2Math.b2Max(maxSqr, b2Dot(points[i], points[i]));
			tVec = points[i];
			maxSqr = b2Math.b2Max(maxSqr, (tVec.x*tVec.x + tVec.y*tVec.y));
		}
		
		if (pointCount == 3 || vSqr <= 100.0 * Number.MIN_VALUE * maxSqr)
		{
			g_GJK_Iterations = iter;
			//v = *x2 - *x1;
			vX = x2.x - x1.x;
			vY = x2.y - x1.y;
			//vSqr = b2Dot(v, v);
			vSqr = (vX*vX + vY*vY);
			return Math.sqrt(vSqr);
		}
	}

	g_GJK_Iterations = maxIterations;
	return Math.sqrt(vSqr);
}


static public function DistanceCC(
	x1:b2Vec2, x2:b2Vec2,
	circle1:b2CircleShape, xf1:b2XForm,
	circle2:b2CircleShape, xf2:b2XForm) : Number
{
	var tMat:b2Mat22;
	var tVec:b2Vec2;
	//b2Vec2 p1 = b2Mul(xf1, circle1->m_localPosition);
	tMat = xf1.R;
	tVec = circle1.m_localPosition;
	var p1X:Number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	var p1Y:Number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	//b2Vec2 p2 = b2Mul(xf2, circle2->m_localPosition);
	tMat = xf2.R;
	tVec = circle2.m_localPosition;
	var p2X:Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	var p2Y:Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

	//b2Vec2 d = p2 - p1;
	var dX:Number = p2X - p1X;
	var dY:Number = p2Y - p1Y;
	var dSqr:Number = (dX*dX + dY*dY);
	var r1:Number = circle1.m_radius - b2Settings.b2_toiSlop;
	var r2:Number = circle2.m_radius - b2Settings.b2_toiSlop;
	var r:Number = r1 + r2;
	if (dSqr > r * r)
	{
		//var dLen:Number = d.Normalize();
		var dLen:Number = Math.sqrt(dSqr);
		dX /= dLen;
		dY /= dLen;
		var distance:Number = dLen - r;
		//*x1 = p1 + r1 * d;
		x1.x = p1X + r1 * dX;
		x1.y = p1Y + r1 * dY;
		//*x2 = p2 - r2 * d;
		x2.x = p2X - r2 * dX;
		x2.y = p2Y - r2 * dY;
		return distance;
	}
	else if (dSqr > Number.MIN_VALUE * Number.MIN_VALUE)
	{
		//d.Normalize();
		dLen = Math.sqrt(dSqr);
		dX /= dLen;
		dY /= dLen;
		//*x1 = p1 + r1 * d;
		x1.x = p1X + r1 * dX;
		x1.y = p1Y + r1 * dY;
		//*x2 = *x1;
		x2.x = x1.x;
		x2.y = x1.y;
		return 0.0;
	}

	//*x1 = p1;
	x1.x = p1X;
	x1.y = p1Y;
	//*x2 = *x1;
	x2.x = x1.x;
	x2.y = x1.y;
	return 0.0;
}



// GJK is more robust with polygon-vs-point than polygon-vs-circle.
// So we convert polygon-vs-circle to polygon-vs-point.
static private var gPoint:b2Point = new b2Point();
///
static public function DistancePC(
	x1:b2Vec2, x2:b2Vec2,
	polygon:b2PolygonShape, xf1:b2XForm,
	circle:b2CircleShape, xf2:b2XForm) : Number
{
	
	var tMat:b2Mat22;
	var tVec:b2Vec2;
	
	var point:b2Point = gPoint;
	//point.p = b2Mul(xf2, circle->m_localPosition);
	tVec = circle.m_localPosition;
	tMat = xf2.R;
	point.p.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	point.p.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

	// Create variation of function to replace template
	var distance:Number = DistanceGeneric(x1, x2, polygon, xf1, point, b2Math.b2XForm_identity);

	var r:Number = circle.m_radius - b2Settings.b2_toiSlop;

	if (distance > r)
	{
		distance -= r;
		//b2Vec2 d = *x2 - *x1;
		var dX:Number = x2.x - x1.x;
		var dY:Number = x2.y - x1.y;
		//d.Normalize();
		var dLen:Number = Math.sqrt(dX*dX + dY*dY);
		dX /= dLen;
		dY /= dLen;
		//*x2 -= r * d;
		x2.x -= r * dX;
		x2.y -= r * dY;
	}
	else
	{
		distance = 0.0;
		//*x2 = *x1;
		x2.x = x1.x;
		x2.y = x1.y;
	}
	
	return distance;
}


static public function Distance(x1:b2Vec2, x2:b2Vec2,
				   shape1:b2Shape, xf1:b2XForm,
				   shape2:b2Shape, xf2:b2XForm) : Number
{
	//b2ShapeType type1 = shape1->GetType();
	var type1:int = shape1.m_type;
	//b2ShapeType type2 = shape2->GetType();
	var type2:int = shape2.m_type;

	if (type1 == b2Shape.e_circleShape && type2 == b2Shape.e_circleShape)
	{
		//return DistanceCC(x1, x2, (b2CircleShape*)shape1, xf1, (b2CircleShape*)shape2, xf2);
		return DistanceCC(x1, x2, shape1 as b2CircleShape, xf1, shape2 as b2CircleShape, xf2);
	}
	
	if (type1 == b2Shape.e_polygonShape && type2 == b2Shape.e_circleShape)
	{
		//return DistancePC(x1, x2, (b2PolygonShape*)shape1, xf1, (b2CircleShape*)shape2, xf2);
		return DistancePC(x1, x2, shape1 as b2PolygonShape, xf1, shape2 as b2CircleShape, xf2);
	}

	if (type1 == b2Shape.e_circleShape && type2 == b2Shape.e_polygonShape)
	{
		return DistancePC(x2, x1, shape2 as b2PolygonShape, xf2, shape1 as b2CircleShape, xf1);
	}

	if (type1 == b2Shape.e_polygonShape && type2 == b2Shape.e_polygonShape)
	{
		return DistanceGeneric(x1, x2, shape1 as b2PolygonShape, xf1, shape2 as b2PolygonShape, xf2);
	}
	
	return 0.0;
}



static public var g_GJK_Iterations:int = 0;



};


}