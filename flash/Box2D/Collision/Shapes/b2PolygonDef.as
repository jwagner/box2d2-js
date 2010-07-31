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
import Box2D.Collision.Shapes.*;



public class b2PolygonDef extends b2ShapeDef
{
	public function b2PolygonDef()
	{
		type = b2Shape.e_polygonShape;
		vertexCount = 0;
		
		for (var i:int = 0; i < b2Settings.b2_maxPolygonVertices; i++){
			vertices[i] = new b2Vec2();
		}
	}
	
	/// Build vertices to represent an axis-aligned box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	public function SetAsBox(hx:Number, hy:Number) : void{
		vertexCount = 4;
		vertices[0].Set(-hx, -hy);
		vertices[1].Set( hx, -hy);
		vertices[2].Set( hx,  hy);
		vertices[3].Set(-hx,  hy);
	}

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	static private var s_mat:b2Mat22 = new b2Mat22();
	public function SetAsOrientedBox(hx:Number, hy:Number, center:b2Vec2=null, angle:Number=0.0) : void{
		//SetAsBox(hx, hy);
		{
			vertexCount = 4;
			vertices[0].Set(-hx, -hy);
			vertices[1].Set( hx, -hy);
			vertices[2].Set( hx,  hy);
			vertices[3].Set(-hx,  hy);
		}
		
		if (center){
			//b2XForm xf;
			//xf.position = center;
			var xfPosition:b2Vec2 = center;
			//xf.R.Set(angle);
			var xfR:b2Mat22 = s_mat;
			xfR.Set(angle);
			
			for (var i:int = 0; i < vertexCount; ++i)
			{
				//vertices[i] = b2Mul(xf, vertices[i]);
				//var a:b2Vec2 = b2MulMV(T.R, v);
				center = vertices[i];
				hx = xfPosition.x + (xfR.col1.x * center.x + xfR.col2.x * center.y)
				center.y = xfPosition.y + (xfR.col1.y * center.x + xfR.col2.y * center.y)
				center.x = hx;
			}
		}
	}

	/// The polygon vertices in local coordinates.
	public var vertices:Array = new Array(b2Settings.b2_maxPolygonVertices);
	/// The number of polygon vertices.
	public var vertexCount:int;
};

}