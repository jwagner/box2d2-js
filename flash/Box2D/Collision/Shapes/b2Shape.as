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
import Box2D.Dynamics.*;
import Box2D.Collision.*;



/// A shape is used for collision detection. Shapes are created in b2World.
/// You can use shape for collision detection before they are attached to the world.
/// @warning you cannot reuse shapes.
public class b2Shape
{
	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	public function GetType() : int{
		return m_type;
	}

	/// Is this shape a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	public function IsSensor() : Boolean{
		return m_isSensor;
	}

	/// Set the contact filtering data. You must call b2World::Refilter to correct
	/// existing contacts/non-contacts.
	public function SetFilterData(filter:b2FilterData) : void
	{
		m_filter = filter.Copy();
	}

	/// Get the contact filtering data.
	public function GetFilterData() : b2FilterData
	{
		return m_filter.Copy();
	}

	/// Get the parent body of this shape. This is NULL if the shape is not attached.
	/// @return the parent body.
	public function GetBody() : b2Body{
		return m_body;
	}

	/// Get the next shape in the parent body's shape list.
	/// @return the next shape.
	public function GetNext() : b2Shape{
		return m_next;
	}

	/// Get the user data that was assigned in the shape definition. Use this to
	/// store your application specific data.
	public function GetUserData() : *{
		return m_userData;
	}

	/// Set the user data. Use this to store your application specific data.
	public function SetUserData(data:*) : void
	{
		m_userData = data;
	}

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	public virtual function TestPoint(xf:b2XForm, p:b2Vec2) : Boolean {return false};

	/// Perform a ray cast against this shape.
	/// @param xf the shape world transform.
	/// @param lambda returns the hit fraction. You can use this to compute the contact point
	/// p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	/// @param normal returns the normal at the contact point. If there is no intersection, the normal
	/// is not set.
	/// @param segment defines the begin and end point of the ray cast.
	/// @param maxLambda a number typically in the range [0,1].
	/// @return true if there was an intersection.
	public virtual function  TestSegment(xf:b2XForm,
								lambda:Array, // float pointer
								normal:b2Vec2, // pointer
								segment:b2Segment,
								maxLambda:Number) : Boolean {return false};

	/// Given a transform, compute the associated axis aligned bounding box for this shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	public virtual function  ComputeAABB(aabb:b2AABB, xf:b2XForm) : void {};

	/// Given two transforms, compute the associated swept axis aligned bounding box for this shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf1 the starting shape world transform.
	/// @param xf2 the ending shape world transform.
	public virtual function  ComputeSweptAABB(	aabb:b2AABB,
									xf1:b2XForm,
									xf2:b2XForm) : void {};

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin, not the centroid.
	/// @param massData returns the mass data for this shape.
	public virtual function  ComputeMass(massData:b2MassData) : void {};

	/// Get the maximum radius about the parent body's center of mass.
	public function GetSweepRadius() : Number
	{
		return m_sweepRadius;
	}

	/// Get the coefficient of friction.
	public function GetFriction() : Number
	{
		return m_friction;
	}

	/// Get the coefficient of restitution.
	public function GetRestitution() : Number
	{
		return m_restitution;
	}
	
	//--------------- Internals Below -------------------

	static public function Create(def:b2ShapeDef, allocator:*) : b2Shape
	{
		switch (def.type)
		{
		case e_circleShape:
			{
				//void* mem = allocator->Allocate(sizeof(b2CircleShape));
				return new b2CircleShape(def);
			}
		
		case e_polygonShape:
			{
				//void* mem = allocator->Allocate(sizeof(b2PolygonShape));
				return new b2PolygonShape(def);
			}
		
		default:
			//b2Settings.b2Assert(false);
			return null;
		}
	}
	
	static public function Destroy(shape:b2Shape, allocator:*) : void
	{
		/*switch (s.m_type)
		{
		case e_circleShape:
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2CircleShape));
			break;
		
		case e_polygonShape:
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2PolygonShape));
			break;
		
		default:
			//b2Settings.b2Assert(false);
		}*/
	}

	public function b2Shape(def:b2ShapeDef){
		
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		m_density = def.density;
		m_body = null;
		m_sweepRadius = 0.0;
		
		m_next = null;
		
		m_proxyId = b2Pair.b2_nullProxy;
		
		m_filter = def.filter.Copy();
		
		m_isSensor = def.isSensor;
		
	}
	
	//virtual ~b2Shape();

	//
	static private var s_proxyAABB:b2AABB = new b2AABB();
	public function CreateProxy(broadPhase:b2BroadPhase, transform:b2XForm) : void{
		
		//b2Settings.b2Assert(m_proxyId == b2_nullProxy);
		
		var aabb:b2AABB = s_proxyAABB;
		ComputeAABB(aabb, transform);
		
		var inRange:Boolean = broadPhase.InRange(aabb);
		
		// You are creating a shape outside the world box.
		//b2Settings.b2Assert(inRange);
		
		if (inRange)
		{
			m_proxyId = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
	}
	
	public function DestroyProxy(broadPhase:b2BroadPhase) : void{
		
		if (m_proxyId != b2Pair.b2_nullProxy)
		{
			broadPhase.DestroyProxy(m_proxyId);
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
	}
	
	//
	static private var s_syncAABB:b2AABB = new b2AABB();
	//
	public function Synchronize(broadPhase:b2BroadPhase, transform1:b2XForm, transform2:b2XForm) : Boolean{
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return false;
		}
		
		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		var aabb:b2AABB = s_syncAABB;
		ComputeSweptAABB(aabb, transform1, transform2);
		
		if (broadPhase.InRange(aabb))
		{
			broadPhase.MoveProxy(m_proxyId, aabb);
			return true;
		}
		else
		{
			return false;
		}
		
	}
	
	static private var s_resetAABB:b2AABB = new b2AABB();
	public function RefilterProxy(broadPhase:b2BroadPhase, transform:b2XForm) : void{
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{
			return;
		}
		
		broadPhase.DestroyProxy(m_proxyId);
		
		var aabb:b2AABB = s_resetAABB;
		ComputeAABB(aabb, transform);
		
		var inRange:Boolean = broadPhase.InRange(aabb);
		
		if (inRange)
		{
			m_proxyId = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
	}

	public virtual function UpdateSweepRadius(center:b2Vec2) : void{};

	public var m_type:int;
	public var m_next:b2Shape;
	public var m_body:b2Body;

	// Sweep radius relative to the parent body's center of mass.
	public var m_sweepRadius:Number;

	public var m_density:Number;
	public var m_friction:Number;
	public var m_restitution:Number;

	public var m_proxyId:uint;
	public var m_filter:b2FilterData;

	public var m_isSensor:Boolean;

	public var m_userData:*;

	
	
	
	/// The various collision shape types supported by Box2D.
	//enum b2ShapeType
	//{
		static public const e_unknownShape:int = 	-1;
		static public const e_circleShape:int = 	0;
		static public const e_polygonShape:int = 	1;
		static public const e_shapeTypeCount:int = 	2;
	//};
	
	
	
};

	
}
