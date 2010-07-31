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

package Box2D.Dynamics.Joints{


import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Dynamics.*;


/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
public class b2Joint
{
	/// Get the type of the concrete joint.
	public function GetType():int{
		return m_type;
	}
	
	/// Get the anchor point on body1 in world coordinates.
	public virtual function GetAnchor1():b2Vec2{return null};
	/// Get the anchor point on body2 in world coordinates.
	public virtual function GetAnchor2():b2Vec2{return null};
	
	/// Get the reaction force on body2 at the joint anchor.
	public virtual function GetReactionForce():b2Vec2 {return null};
	/// Get the reaction torque on body2.
	public virtual function GetReactionTorque():Number {return 0.0}
	
	/// Get the first body attached to this joint.
	public function GetBody1():b2Body
	{
		return m_body1;
	}
	
	/// Get the second body attached to this joint.
	public function GetBody2():b2Body
	{
		return m_body2;
	}

	/// Get the next joint the world joint list.
	public function GetNext():b2Joint{
		return m_next;
	}

	/// Get the user data pointer.
	public function GetUserData():*{
		return m_userData;
	}

	/// Set the user data pointer.
	public function SetUserData(data:*):void{
		m_userData = data;
	}

	//--------------- Internals Below -------------------

	static public function Create(def:b2JointDef, allocator:*):b2Joint{
		var joint:b2Joint = null;
		
		switch (def.type)
		{
		case e_distanceJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
				joint = new b2DistanceJoint(def as b2DistanceJointDef);
			}
			break;
		
		case e_mouseJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2MouseJoint));
				joint = new b2MouseJoint(def as b2MouseJointDef);
			}
			break;
		
		case e_prismaticJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
				joint = new b2PrismaticJoint(def as b2PrismaticJointDef);
			}
			break;
		
		case e_revoluteJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
				joint = new b2RevoluteJoint(def as b2RevoluteJointDef);
			}
			break;
		
		case e_pulleyJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
				joint = new b2PulleyJoint(def as b2PulleyJointDef);
			}
			break;
		
		case e_gearJoint:
			{
				//void* mem = allocator->Allocate(sizeof(b2GearJoint));
				joint = new b2GearJoint(def as b2GearJointDef);
			}
			break;
		
		default:
			//b2Settings.b2Assert(false);
			break;
		}
		
		return joint;
	}
	
	static public function Destroy(joint:b2Joint, allocator:*) : void{
		/*joint->~b2Joint();
		switch (joint.m_type)
		{
		case e_distanceJoint:
			allocator->Free(joint, sizeof(b2DistanceJoint));
			break;
		
		case e_mouseJoint:
			allocator->Free(joint, sizeof(b2MouseJoint));
			break;
		
		case e_prismaticJoint:
			allocator->Free(joint, sizeof(b2PrismaticJoint));
			break;
		
		case e_revoluteJoint:
			allocator->Free(joint, sizeof(b2RevoluteJoint));
			break;
		
		case e_pulleyJoint:
			allocator->Free(joint, sizeof(b2PulleyJoint));
			break;
		
		case e_gearJoint:
			allocator->Free(joint, sizeof(b2GearJoint));
			break;
		
		default:
			b2Assert(false);
			break;
		}*/
	}

	public function b2Joint(def:b2JointDef){
		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_body1 = def.body1;
		m_body2 = def.body2;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;
	}
	//virtual ~b2Joint() {}

	public virtual function InitVelocityConstraints(step:b2TimeStep) : void{};
	public virtual function SolveVelocityConstraints(step:b2TimeStep) : void{};

	// This returns true if the position errors are within tolerance.
	public virtual function InitPositionConstraints() : void{};
	public virtual function SolvePositionConstraints():Boolean{return false};

	public var m_type:int;
	public var m_prev:b2Joint;
	public var m_next:b2Joint;
	public var m_node1:b2JointEdge = new b2JointEdge();
	public var m_node2:b2JointEdge = new b2JointEdge();
	public var m_body1:b2Body;
	public var m_body2:b2Body;

	public var m_inv_dt:Number;

	public var m_islandFlag:Boolean;
	public var m_collideConnected:Boolean;

	public var m_userData:*;
	
	
	// ENUMS
	
	// enum b2JointType
	static public const e_unknownJoint:int = 0;
	static public const e_revoluteJoint:int = 1;
	static public const e_prismaticJoint:int = 2;
	static public const e_distanceJoint:int = 3;
	static public const e_pulleyJoint:int = 4;
	static public const e_mouseJoint:int = 5;
	static public const e_gearJoint:int = 6;

	// enum b2LimitState
	static public const e_inactiveLimit:int = 0;
	static public const e_atLowerLimit:int = 1;
	static public const e_atUpperLimit:int = 2;
	static public const e_equalLimits:int = 3;
	
};



}
