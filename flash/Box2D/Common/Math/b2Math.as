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

package Box2D.Common.Math{

import Box2D.Common.b2Settings;
import Box2D.Collision.b2AABB;

public class b2Math{

	/// This function is used to ensure that a floating point number is
	/// not a NaN or infinity.
	static public function b2IsValid(x:Number) : Boolean
	{
		return isFinite(x);
	}
	
	/*static public function b2InvSqrt(x:Number):Number{
		union
		{
			float32 x;
			int32 i;
		} convert;
		
		convert.x = x;
		float32 xhalf = 0.5f * x;
		convert.i = 0x5f3759df - (convert.i >> 1);
		x = convert.x;
		x = x * (1.5f - xhalf * x * x);
		return x;
	}*/

	static public function b2Dot(a:b2Vec2, b:b2Vec2):Number
	{
		return a.x * b.x + a.y * b.y;
	}

	static public function b2CrossVV(a:b2Vec2, b:b2Vec2):Number
	{
		return a.x * b.y - a.y * b.x;
	}

	static public function b2CrossVF(a:b2Vec2, s:Number):b2Vec2
	{
		var v:b2Vec2 = new b2Vec2(s * a.y, -s * a.x);
		return v;
	}

	static public function b2CrossFV(s:Number, a:b2Vec2):b2Vec2
	{
		var v:b2Vec2 = new b2Vec2(-s * a.y, s * a.x);
		return v;
	}

	static public function b2MulMV(A:b2Mat22, v:b2Vec2):b2Vec2
	{
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		var u:b2Vec2 = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
		return u;
	}

	static public function b2MulTMV(A:b2Mat22, v:b2Vec2):b2Vec2
	{
		// (tVec.x * tMat.col1.x + tVec.y * tMat.col1.y)
		// (tVec.x * tMat.col2.x + tVec.y * tMat.col2.y)
		var u:b2Vec2 = new b2Vec2(b2Dot(v, A.col1), b2Dot(v, A.col2));
		return u;
	}
	
	static public function b2MulX(T:b2XForm, v:b2Vec2) : b2Vec2
	{
		var a:b2Vec2 = b2MulMV(T.R, v);
		a.x += T.position.x;
		a.y += T.position.y;
		//return T.position + b2Mul(T.R, v);
		return a;
	}

	static public function b2MulXT(T:b2XForm, v:b2Vec2):b2Vec2
	{
		var a:b2Vec2 = SubtractVV(v, T.position);
		//return b2MulT(T.R, v - T.position);
		var tX:Number = (a.x * T.R.col1.x + a.y * T.R.col1.y );
		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y );
		a.x = tX;
		return a;
	}

	static public function AddVV(a:b2Vec2, b:b2Vec2):b2Vec2
	{
		var v:b2Vec2 = new b2Vec2(a.x + b.x, a.y + b.y);
		return v;
	}

	static public function SubtractVV(a:b2Vec2, b:b2Vec2):b2Vec2
	{
		var v:b2Vec2 = new b2Vec2(a.x - b.x, a.y - b.y);
		return v;
	}
	
	static public function b2Distance(a:b2Vec2, b:b2Vec2) : Number{
		var cX:Number = a.x-b.x;
		var cY:Number = a.y-b.y;
		return Math.sqrt(cX*cX + cY*cY);
	}
	
	static public function b2DistanceSquared(a:b2Vec2, b:b2Vec2) : Number{
		var cX:Number = a.x-b.x;
		var cY:Number = a.y-b.y;
		return (cX*cX + cY*cY);
	}

	static public function MulFV(s:Number, a:b2Vec2):b2Vec2
	{
		var v:b2Vec2 = new b2Vec2(s * a.x, s * a.y);
		return v;
	}

	static public function AddMM(A:b2Mat22, B:b2Mat22):b2Mat22
	{
		var C:b2Mat22 = new b2Mat22(0, AddVV(A.col1, B.col1), AddVV(A.col2, B.col2));
		return C;
	}

	// A * B
	static public function b2MulMM(A:b2Mat22, B:b2Mat22):b2Mat22
	{
		var C:b2Mat22 = new b2Mat22(0, b2MulMV(A, B.col1), b2MulMV(A, B.col2));
		return C;
	}

	// A^T * B
	static public function b2MulTMM(A:b2Mat22, B:b2Mat22):b2Mat22
	{
		var c1:b2Vec2 = new b2Vec2(b2Dot(A.col1, B.col1), b2Dot(A.col2, B.col1));
		var c2:b2Vec2 = new b2Vec2(b2Dot(A.col1, B.col2), b2Dot(A.col2, B.col2));
		var C:b2Mat22 = new b2Mat22(0, c1, c2);
		return C;
	}

	static public function b2Abs(a:Number):Number
	{
		return a > 0.0 ? a : -a;
	}

	static public function b2AbsV(a:b2Vec2):b2Vec2
	{
		var b:b2Vec2 = new b2Vec2(b2Abs(a.x), b2Abs(a.y));
		return b;
	}

	static public function b2AbsM(A:b2Mat22):b2Mat22
	{
		var B:b2Mat22 = new b2Mat22(0, b2AbsV(A.col1), b2AbsV(A.col2));
		return B;
	}

	static public function b2Min(a:Number, b:Number):Number
	{
		return a < b ? a : b;
	}

	static public function b2MinV(a:b2Vec2, b:b2Vec2):b2Vec2
	{
		var c:b2Vec2 = new b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
		return c;
	}

	static public function b2Max(a:Number, b:Number):Number
	{
		return a > b ? a : b;
	}

	static public function b2MaxV(a:b2Vec2, b:b2Vec2):b2Vec2
	{
		var c:b2Vec2 = new b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
		return c;
	}

	static public function b2Clamp(a:Number, low:Number, high:Number):Number
	{
		return b2Max(low, b2Min(a, high));
	}

	static public function b2ClampV(a:b2Vec2, low:b2Vec2, high:b2Vec2):b2Vec2
	{
		return b2MaxV(low, b2MinV(a, high));
	}

	static public function b2Swap(a:Array, b:Array) : void
	{
		var tmp:* = a[0];
		a[0] = b[0];
		b[0] = tmp;
	}

	// b2Random number in range [-1,1]
	static public function b2Random():Number
	{
		return Math.random() * 2 - 1;
	}

	static public function b2RandomRange(lo:Number, hi:Number) : Number
	{
		var r:Number = Math.random();
		r = (hi - lo) * r + lo;
		return r;
	}

	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	static public function b2NextPowerOfTwo(x:uint):uint
	{
		x |= (x >> 1) & 0x7FFFFFFF;
		x |= (x >> 2) & 0x3FFFFFFF;
		x |= (x >> 4) & 0x0FFFFFFF;
		x |= (x >> 8) & 0x00FFFFFF;
		x |= (x >> 16)& 0x0000FFFF;
		return x + 1;
	}

	static public function b2IsPowerOfTwo(x:uint):Boolean
	{
		var result:Boolean = x > 0 && (x & (x - 1)) == 0;
		return result;
	}
	
	
	// Temp vector functions to reduce calls to 'new'
	/*static public var tempVec:b2Vec2 = new b2Vec2();
	static public var tempVec2:b2Vec2 = new b2Vec2();
	static public var tempVec3:b2Vec2 = new b2Vec2();
	static public var tempVec4:b2Vec2 = new b2Vec2();
	static public var tempVec5:b2Vec2 = new b2Vec2();
	
	static public var tempMat:b2Mat22 = new b2Mat22();	
	
	static public var tempAABB:b2AABB = new b2AABB();	*/
	
	static public const b2Vec2_zero:b2Vec2 = new b2Vec2(0.0, 0.0);
	static public const b2Mat22_identity:b2Mat22 = new b2Mat22(0, new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
	static public const b2XForm_identity:b2XForm = new b2XForm(b2Vec2_zero, b2Mat22_identity);
	

}
}
