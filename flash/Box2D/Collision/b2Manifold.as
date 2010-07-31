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
	
import Box2D.Collision.*
import Box2D.Common.Math.*
import Box2D.Common.*

// A manifold for two touching convex shapes.
public class b2Manifold
{
	public function b2Manifold(){
		points = new Array(b2Settings.b2_maxManifoldPoints);
		for (var i:int = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			points[i] = new b2ManifoldPoint();
		}
		normal = new b2Vec2();
	}
	public function Reset() : void{
		for (var i:int = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			(points[i] as b2ManifoldPoint).Reset();
		}
		normal.SetZero();
		pointCount = 0;
	}
	public function Set(m:b2Manifold) : void{
		pointCount = m.pointCount;
		for (var i:int = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			(points[i] as b2ManifoldPoint).Set(m.points[i]);
		}
		normal.SetV(m.normal);
	}
	public var points:Array;	///< the points of contact
	public var normal:b2Vec2;	///< the shared unit normal vector
	public var pointCount:int = 0;	///< the number of manifold points
};


}