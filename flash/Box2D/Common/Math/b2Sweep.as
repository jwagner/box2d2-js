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

	
import Box2D.Common.*;
	
	
/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
public class b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param t the normalized time in [0,1].
	public function GetXForm(xf:b2XForm, t:Number) : void{
		
		// center = p + R * localCenter
		if (1.0 - t0 > Number.MIN_VALUE)
		{
			var alpha:Number = (t - t0) / (1.0 - t0);
			xf.position.x = (1.0 - alpha) * c0.x + alpha * c.x;
			xf.position.y = (1.0 - alpha) * c0.y + alpha * c.y;
			var angle:Number = (1.0 - alpha) * a0 + alpha * a;
			xf.R.Set(angle);
		}
		else
		{
			xf.position.SetV(c);
			xf.R.Set(a);
		}
		
		// Shift to origin
		//xf->position -= b2Mul(xf->R, localCenter);
		var tMat:b2Mat22 = xf.R;
		xf.position.x -= (tMat.col1.x * localCenter.x + tMat.col2.x * localCenter.y);
		xf.position.y -= (tMat.col1.y * localCenter.x + tMat.col2.y * localCenter.y);
		
	}

	/// Advance the sweep forward, yielding a new initial state.
	/// @param t the new initial time.
	public function Advance(t:Number) : void{
		if (t0 < t && 1.0 - t0 > Number.MIN_VALUE)
		{
			var alpha:Number = (t - t0) / (1.0 - t0);
			//c0 = (1.0f - alpha) * c0 + alpha * c;
			c0.x = (1.0 - alpha) * c0.x + alpha * c.x;
			c0.y = (1.0 - alpha) * c0.y + alpha * c.y;
			a0 = (1.0 - alpha) * a0 + alpha * a;
			t0 = t;
		}
	}

	public var localCenter:b2Vec2 = new b2Vec2();	///< local center of mass position
	public var c0:b2Vec2 = new b2Vec2;				///< center world positions
	public var c:b2Vec2 = new b2Vec2();
	public var a0:Number, a:Number;					///< world angles
	public var t0:Number;							///< time interval = [t0,1], where t0 is in [0,1]
};

}