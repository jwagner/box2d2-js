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

package Box2D.Common{
	
	
import Box2D.Common.Math.*;
	
	
public class b2Settings{

	static public const USHRT_MAX:int = 0x0000ffff;

	static public const b2_pi:Number = Math.PI;



	// Collision
	static public const b2_maxManifoldPoints:int = 2;
	static public const b2_maxPolygonVertices:int = 8;
	static public const b2_maxProxies:int = 512;				// this must be a power of two
	static public const b2_maxPairs:int = 8 * b2_maxProxies;	// this must be a power of two



	// Dynamics
	
	/// A small length used as a collision and constraint tolerance. Usually it is
	/// chosen to be numerically significant, but visually insignificant.
	static public const b2_linearSlop:Number = 0.005;	// 0.5 cm
	
	/// A small angle used as a collision and constraint tolerance. Usually it is
	/// chosen to be numerically significant, but visually insignificant.
	static public const b2_angularSlop:Number = 2.0 / 180.0 * b2_pi;			// 2 degrees
	
	/// Continuous collision detection (CCD) works with core, shrunken shapes. This is the
	/// amount by which shapes are automatically shrunk to work with CCD. This must be
	/// larger than b2_linearSlop.
	static public const b2_toiSlop:Number = 8.0 * b2_linearSlop;
	
	/// Maximum number of contacts to be handled to solve a TOI island.
	static public const b2_maxTOIContactsPerIsland:int = 32;
	
	/// A velocity threshold for elastic collisions. Any collision with a relative linear
	/// velocity below this threshold will be treated as inelastic.
	static public const b2_velocityThreshold:Number = 1.0;		// 1 m/s
	
	/// The maximum linear position correction used when solving constraints. This helps to
	/// prevent overshoot.
	static public const b2_maxLinearCorrection:Number = 0.2;	// 20 cm
	
	/// The maximum angular position correction used when solving constraints. This helps to
	/// prevent overshoot.
	static public const b2_maxAngularCorrection:Number = 8.0 / 180.0 * b2_pi;			// 8 degrees
	
	/// The maximum linear velocity of a body. This limit is very large and is used
	/// to prevent numerical problems. You shouldn't need to adjust this.
	static public const b2_maxLinearVelocity:Number = 200.0;
	static public const b2_maxLinearVelocitySquared:Number = b2_maxLinearVelocity * b2_maxLinearVelocity;

	/// The maximum angular velocity of a body. This limit is very large and is used
	/// to prevent numerical problems. You shouldn't need to adjust this.
	static public const b2_maxAngularVelocity:Number = 250.0;
	static public const b2_maxAngularVelocitySquared:Number = b2_maxAngularVelocity * b2_maxAngularVelocity;
	
	/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
	/// that overlap is removed in one time step. However using values close to 1 often lead
	/// to overshoot.
	static public const b2_contactBaumgarte:Number = 0.2;



	// Sleep
	
	/// The time that a body must be still before it will go to sleep.
	static public const b2_timeToSleep:Number = 0.5;					// half a second
	/// A body cannot sleep if its linear velocity is above this tolerance.
	static public const b2_linearSleepTolerance:Number = 0.01;			// 1 cm/s
	/// A body cannot sleep if its angular velocity is above this tolerance.
	static public const b2_angularSleepTolerance:Number = 2.0 / 180.0;	// 2 degrees/s
	
	// assert
	static public function b2Assert(a:Boolean) : void
	{
		if (!a){
			var nullVec:b2Vec2;
			nullVec.x++;
		}
	}
}

}