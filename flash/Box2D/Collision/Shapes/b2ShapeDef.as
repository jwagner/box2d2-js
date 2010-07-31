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

package Box2D.Collision.Shapes{



import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.Shapes.b2Shape;



/// A shape definition is used to construct a shape. This class defines an
/// abstract shape definition. You can reuse shape definitions safely.
public class b2ShapeDef
{
	/// Holds the shape type for down-casting.
	public var type:int = b2Shape.e_unknownShape;
	
	/// Use this to store application specify shape data.
	public var userData:* = null;
	
	/// The shape's friction coefficient, usually in the range [0,1].
	public var friction:Number = 0.2;
	
	/// The shape's restitution (elasticity) usually in the range [0,1].
	public var restitution:Number = 0.0;
	
	/// The shape's density, usually in kg/m^2.
	public var density:Number = 0.0;
	
	/// A sensor shape collects contact information but never generates a collision
	/// response.
	public var isSensor:Boolean = false;
	
	/// Contact filtering data.
	public var filter: b2FilterData = new b2FilterData();
};

}