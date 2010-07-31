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

package Box2D.Dynamics{


import Box2D.Dynamics.*;
import Box2D.Dynamics.Joints.*;
import Box2D.Dynamics.Contacts.*;
import Box2D.Collision.Shapes.*;
import Box2D.Common.Math.*;
import Box2D.Common.*;



/// A rigid body.
public class b2Body
{
	/// Creates a shape and attach it to this body.
	/// @param shapeDef the shape definition.
	/// @warning This function is locked during callbacks.
	public function CreateShape(def:b2ShapeDef) : b2Shape{
		//b2Settings.b2Assert(m_world.m_lock == false);
		if (m_world.m_lock == true)
		{
			return null;
		}
		
		var s:b2Shape = b2Shape.Create(def, m_world.m_blockAllocator);
		
		s.m_next = m_shapeList;
		m_shapeList = s;
		++m_shapeCount;
		
		s.m_body = this;
		
		// Add the shape to the world's broad-phase.
		s.CreateProxy(m_world.m_broadPhase, m_xf);
		
		// Compute the sweep radius for CCD.
		s.UpdateSweepRadius(m_sweep.localCenter);
		
		return s;
	}

	/// Destroy a shape. This removes the shape from the broad-phase and
	/// therefore destroys any contacts associated with this shape. All shapes
	/// attached to a body are implicitly destroyed when the body is destroyed.
	/// @param shape the shape to be removed.
	/// @warning This function is locked during callbacks.
	public function DestroyShape(s:b2Shape) : void{
		//b2Settings.b2Assert(m_world.m_lock == false);
		if (m_world.m_lock == true)
		{
			return;
		}
		
		//b2Settings.b2Assert(s.m_body == this);
		s.DestroyProxy(m_world.m_broadPhase);
		
		//b2Settings.b2Assert(m_shapeCount > 0);
		//b2Shape** node = &m_shapeList;
		var node:b2Shape = m_shapeList;
		var ppS:b2Shape = null; // Fix pointer-pointer stuff
		var found:Boolean = false;
		while (node != null)
		{
			if (node == s)
			{
				if (ppS)
					ppS.m_next = s.m_next;
				else
					m_shapeList = s.m_next;
				//node = s.m_next;
				found = true;
				break;
			}
			
			ppS = node;
			node = node.m_next;
		}
		
		// You tried to remove a shape that is not attached to this body.
		//b2Settings.b2Assert(found);
		
		s.m_body = null;
		s.m_next = null;
		
		--m_shapeCount;
		
		b2Shape.Destroy(s, m_world.m_blockAllocator);
	}

	/// Set the mass properties. Note that this changes the center of mass position.
	/// If you are not sure how to compute mass properties, use SetMassFromShapes.
	/// The inertia tensor is assumed to be relative to the center of mass.
	/// @param massData the mass properties.
	public function SetMass(massData:b2MassData) : void{
		var s:b2Shape;
		
		//b2Settings.b2Assert(m_world.m_lock == false);
		if (m_world.m_lock == true)
		{
			return;
		}
		
		m_invMass = 0.0;
		m_I = 0.0;
		m_invI = 0.0;
		
		m_mass = massData.mass;
		
		if (m_mass > 0.0)
		{
			m_invMass = 1.0 / m_mass;
		}
		
		if ((m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			m_I = massData.I;
		}
		
		if (m_I > 0.0)
		{
			m_invI = 1.0 / m_I;
		}
		
		// Move center of mass.
		m_sweep.localCenter.SetV(massData.center);
		//m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
		//b2MulMV(m_xf.R, m_sweep.localCenter);
		var tMat:b2Mat22 = m_xf.R;
		var tVec:b2Vec2 = m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		m_sweep.c.x += m_xf.position.x;
		m_sweep.c.y += m_xf.position.y;
		//m_sweep.c0 = m_sweep.c
		m_sweep.c0.SetV(m_sweep.c);
		
		// Update the sweep radii of all child shapes.
		for (s = m_shapeList; s; s = s.m_next)
		{
			s.UpdateSweepRadius(m_sweep.localCenter);
		}

		var oldType:int = m_type;
		if (m_invMass == 0.0 && m_invI == 0.0)
		{
			m_type = e_staticType;
		}
		else
		{
			m_type = e_dynamicType;
		}
	
		// If the body type changed, we need to refilter the broad-phase proxies.
		if (oldType != m_type)
		{
			for (s = m_shapeList; s; s = s.m_next)
			{
				s.RefilterProxy(m_world.m_broadPhase, m_xf);
			}
		}
	}

	/// Compute the mass properties from the attached shapes. You typically call this
	/// after adding all the shapes. If you add or remove shapes later, you may want
	/// to call this again. Note that this changes the center of mass position.
	static private var s_massData:b2MassData = new b2MassData();
	public function SetMassFromShapes() : void{
		
		var s:b2Shape;
		
		//b2Settings.b2Assert(m_world.m_lock == false);
		if (m_world.m_lock == true)
		{
			return;
		}
		
		// Compute mass data from shapes. Each shape has its own density.
		m_mass = 0.0;
		m_invMass = 0.0;
		m_I = 0.0;
		m_invI = 0.0;
		
		//b2Vec2 center = b2Vec2_zero;
		var centerX:Number = 0.0;
		var centerY:Number = 0.0;
		var massData:b2MassData = s_massData;
		for (s = m_shapeList; s; s = s.m_next)
		{
			s.ComputeMass(massData);
			m_mass += massData.mass;
			//center += massData.mass * massData.center;
			centerX += massData.mass * massData.center.x;
			centerY += massData.mass * massData.center.y;
			m_I += massData.I;
		}
		
		// Compute center of mass, and shift the origin to the COM.
		if (m_mass > 0.0)
		{
			m_invMass = 1.0 / m_mass;
			centerX *= m_invMass;
			centerY *= m_invMass;
		}
		
		if (m_I > 0.0 && (m_flags & e_fixedRotationFlag) == 0)
		{
			// Center the inertia about the center of mass.
			//m_I -= m_mass * b2Dot(center, center);
			m_I -= m_mass * (centerX * centerX + centerY * centerY);
			//b2Settings.b2Assert(m_I > 0.0);
			m_invI = 1.0 / m_I;
		}
		else
		{
			m_I = 0.0;
			m_invI = 0.0;
		}
		
		// Move center of mass.
		m_sweep.localCenter.Set(centerX, centerY);
		//m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
		//b2MulMV(m_xf.R, m_sweep.localCenter);
		var tMat:b2Mat22 = m_xf.R;
		var tVec:b2Vec2 = m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		m_sweep.c.x += m_xf.position.x;
		m_sweep.c.y += m_xf.position.y;
		//m_sweep.c0 = m_sweep.c
		m_sweep.c0.SetV(m_sweep.c);
		
		// Update the sweep radii of all child shapes.
		for (s = m_shapeList; s; s = s.m_next)
		{
			s.UpdateSweepRadius(m_sweep.localCenter);
		}
		
		var oldType:int = m_type;
		if (m_invMass == 0.0 && m_invI == 0.0)
		{
			m_type = e_staticType;
		}
		else
		{
			m_type = e_dynamicType;
		}
		
		// If the body type changed, we need to refilter the broad-phase proxies.
		if (oldType != m_type)
		{
			for (s = m_shapeList; s; s = s.m_next)
			{
				s.RefilterProxy(m_world.m_broadPhase, m_xf);
			}
		}
	}

	/// Set the position of the body's origin and rotation (radians).
	/// This breaks any contacts and wakes the other bodies.
	/// @param position the new world position of the body's origin (not necessarily
	/// the center of mass).
	/// @param angle the new world rotation angle of the body in radians.
	/// @return false if the movement put a shape outside the world. In this case the
	/// body is automatically frozen.
	public function SetXForm(position:b2Vec2, angle:Number) : Boolean{
		
		var s:b2Shape;
		
		//b2Settings.b2Assert(m_world.m_lock == false);
		if (m_world.m_lock == true)
		{
			return true;
		}
		
		if (IsFrozen())
		{
			return false;
		}
		
		m_xf.R.Set(angle);
		m_xf.position.SetV(position);
		
		//m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
		//b2MulMV(m_xf.R, m_sweep.localCenter);
		var tMat:b2Mat22 = m_xf.R;
		var tVec:b2Vec2 = m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		m_sweep.c.x += m_xf.position.x;
		m_sweep.c.y += m_xf.position.y;
		//m_sweep.c0 = m_sweep.c
		m_sweep.c0.SetV(m_sweep.c);
		
		m_sweep.a0 = m_sweep.a = angle;
		
		var freeze:Boolean = false;
		for (s = m_shapeList; s; s = s.m_next)
		{
			var inRange:Boolean = s.Synchronize(m_world.m_broadPhase, m_xf, m_xf);
			
			if (inRange == false)
			{
				freeze = true;
				break;
			}
		}
		
		if (freeze == true)
		{
			m_flags |= e_frozenFlag;
			m_linearVelocity.SetZero();
			m_angularVelocity = 0.0;
			for (s = m_shapeList; s; s = s.m_next)
			{
				s.DestroyProxy(m_world.m_broadPhase);
			}
			
			// Failure
			return false;
		}
		
		// Success
		m_world.m_broadPhase.Commit();
		return true;
		
	}

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	public function GetXForm() : b2XForm{
		return m_xf;
	}

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	public function GetPosition() : b2Vec2{
		return m_xf.position;
	}

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	public function GetAngle() : Number{
		return m_sweep.a;
	}

	/// Get the world position of the center of mass.
	public function GetWorldCenter() : b2Vec2{
		return m_sweep.c;
	}

	/// Get the local position of the center of mass.
	public function GetLocalCenter() : b2Vec2{
		return m_sweep.localCenter;
	}

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	public function SetLinearVelocity(v:b2Vec2) : void{
		m_linearVelocity.SetV(v);
	}

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	public function GetLinearVelocity() : b2Vec2{
		return m_linearVelocity;
	}

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	public function SetAngularVelocity(omega:Number) : void{
		m_angularVelocity = omega;
	}

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	public function GetAngularVelocity() : Number{
		return m_angularVelocity;
	}

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	public function ApplyForce(force:b2Vec2, point:b2Vec2) : void{
		if (IsSleeping())
		{
			WakeUp();
		}
		//m_force += force;
		m_force.x += force.x;
		m_force.y += force.y;
		//m_torque += b2Cross(point - m_sweep.c, force);
		m_torque += ((point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x);
	}

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	public function ApplyTorque(torque:Number) : void{
		if (IsSleeping())
		{
			WakeUp();
		}
		m_torque += torque;
	}

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	public function ApplyImpulse(impulse:b2Vec2, point:b2Vec2) : void{
		if (IsSleeping())
		{
			WakeUp();
		}
		//m_linearVelocity += m_invMass * impulse;
		m_linearVelocity.x += m_invMass * impulse.x;
		m_linearVelocity.y += m_invMass * impulse.y;
		//m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
		m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x);
	}

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	public function GetMass() : Number{
		return m_mass;
	}

	/// Get the central rotational inertia of the body.
	/// @return the rotational inertia, usually in kg-m^2.
	public function GetInertia() : Number{
		return m_I;
	}

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	public function GetWorldPoint(localPoint:b2Vec2) : b2Vec2{
		//return b2Math.b2MulX(m_xf, localPoint);
		var A:b2Mat22 = m_xf.R;
		var u:b2Vec2 = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, 
								  A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		u.x += m_xf.position.x;
		u.y += m_xf.position.y;
		return u;
	}

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	public function GetWorldVector(localVector:b2Vec2) : b2Vec2{
		return b2Math.b2MulMV(m_xf.R, localVector);
	}

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	public function GetLocalPoint(worldPoint:b2Vec2) : b2Vec2{
		return b2Math.b2MulXT(m_xf, worldPoint);
	}

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	public function GetLocalVector(worldVector:b2Vec2) : b2Vec2{
		return b2Math.b2MulTMV(m_xf.R, worldVector);
	}
	
	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	public function GetLinearVelocityFromWorldPoint(worldPoint:b2Vec2) : b2Vec2
	{
		//return          m_linearVelocity   + b2Cross(m_angularVelocity,   worldPoint   - m_sweep.c);
		return new b2Vec2(	m_linearVelocity.x -         m_angularVelocity * (worldPoint.y - m_sweep.c.y), 
							m_linearVelocity.y +         m_angularVelocity * (worldPoint.x - m_sweep.c.x));
	}
	
	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	public function GetLinearVelocityFromLocalPoint(localPoint:b2Vec2) : b2Vec2
	{
		//return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
		var A:b2Mat22 = m_xf.R;
		var worldPoint:b2Vec2 = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, 
								  A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		worldPoint.x += m_xf.position.x;
		worldPoint.y += m_xf.position.y;
		return new b2Vec2(m_linearVelocity.x +         m_angularVelocity * (worldPoint.y - m_sweep.c.y), 
		                  m_linearVelocity.x -         m_angularVelocity * (worldPoint.x - m_sweep.c.x));
	}
	
	/// Is this body treated like a bullet for continuous collision detection?
	public function IsBullet() : Boolean{
		return (m_flags & e_bulletFlag) == e_bulletFlag;
	}

	/// Should this body be treated like a bullet for continuous collision detection?
	public function SetBullet(flag:Boolean) : void{
		if (flag)
		{
			m_flags |= e_bulletFlag;
		}
		else
		{
			m_flags &= ~e_bulletFlag;
		}
	}

	/// Is this body static (immovable)?
	public function IsStatic() : Boolean{
		return m_type == e_staticType;
	}

	/// Is this body dynamic (movable)?
	public function IsDynamic() :Boolean{
		return m_type == e_dynamicType;
	}

	/// Is this body frozen?
	public function IsFrozen() : Boolean{
		return (m_flags & e_frozenFlag) == e_frozenFlag;
	}

	/// Is this body sleeping (not simulating).
	public function IsSleeping() : Boolean{
		return (m_flags & e_sleepFlag) == e_sleepFlag;
	}

	/// You can disable sleeping on this body.
	public function AllowSleeping(flag:Boolean) : void{
		if (flag)
		{
			m_flags |= e_allowSleepFlag;
		}
		else
		{
			m_flags &= ~e_allowSleepFlag;
			WakeUp();
		}
	}

	/// Wake up this body so it will begin simulating.
	public function WakeUp() : void{
		m_flags &= ~e_sleepFlag;
		m_sleepTime = 0.0;
	}

	/// Put this body to sleep so it will stop simulating.
	/// This also sets the velocity to zero.
	public function PutToSleep() : void
	{
		m_flags |= e_sleepFlag;
		m_sleepTime = 0.0;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0;
		m_force.SetZero();
		m_torque = 0.0;
	}

	/// Get the list of all shapes attached to this body.
	public function GetShapeList() : b2Shape{
		return m_shapeList;
	}

	/// Get the list of all joints attached to this body.
	public function GetJointList() : b2JointEdge{
		return m_jointList;
	}

	/// Get the next body in the world's body list.
	public function GetNext() : b2Body{
		return m_next;
	}

	/// Get the user data pointer that was provided in the body definition.
	public function GetUserData() : *{
		return m_userData;
	}

	/// Set the user data. Use this to store your application specific data.
	public function SetUserData(data:*) : void
	{
		m_userData = data;
	}

	/// Get the parent world of this body.
	public function GetWorld(): b2World
	{
		return m_world;
	}

	//--------------- Internals Below -------------------

	
	// Constructor
	public function b2Body(bd:b2BodyDef, world:b2World){
		//b2Settings.b2Assert(world.m_lock == false);
		
		m_flags = 0;
		
		if (bd.isBullet)
		{
			m_flags |= e_bulletFlag;
		}
		if (bd.fixedRotation)
		{
			m_flags |= e_fixedRotationFlag;
		}
		if (bd.allowSleep)
		{
			m_flags |= e_allowSleepFlag;
		}
		if (bd.isSleeping)
		{
			m_flags |= e_sleepFlag;
		}
		
		m_world = world;
		
		m_xf.position.SetV(bd.position);
		m_xf.R.Set(bd.angle);
		
		m_sweep.localCenter.SetV(bd.massData.center);
		m_sweep.t0 = 1.0;
		m_sweep.a0 = m_sweep.a = bd.angle;
		
		//m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
		//b2MulMV(m_xf.R, m_sweep.localCenter);
		var tMat:b2Mat22 = m_xf.R;
		var tVec:b2Vec2 = m_sweep.localCenter;
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//return T.position + b2Mul(T.R, v);
		m_sweep.c.x += m_xf.position.x;
		m_sweep.c.y += m_xf.position.y;
		//m_sweep.c0 = m_sweep.c
		m_sweep.c0.SetV(m_sweep.c);
		
		m_jointList = null;
		m_contactList = null;
		m_prev = null;
		m_next = null;
		
		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;
		
		m_force.Set(0.0, 0.0);
		m_torque = 0.0;
		
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0;
		
		m_sleepTime = 0.0;
		
		m_invMass = 0.0;
		m_I = 0.0;
		m_invI = 0.0;
		
		m_mass = bd.massData.mass;
		
		if (m_mass > 0.0)
		{
			m_invMass = 1.0 / m_mass;
		}
		
		if ((m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			m_I = bd.massData.I;
		}
		
		if (m_I > 0.0)
		{
			m_invI = 1.0 / m_I;
		}
		
		if (m_invMass == 0.0 && m_invI == 0.0)
		{
			m_type = e_staticType;
		}
		else
		{
			m_type = e_dynamicType;
		}
	
		m_userData = bd.userData;
		
		m_shapeList = null;
		m_shapeCount = 0;
	}
	
	// Destructor
	//~b2Body();

	//
	static private var s_xf1:b2XForm = new b2XForm();
	//
	public function SynchronizeShapes() : Boolean{
		
		var xf1:b2XForm = s_xf1;
		xf1.R.Set(m_sweep.a0);
		//xf1.position = m_sweep.c0 - b2Mul(xf1.R, m_sweep.localCenter);
		var tMat:b2Mat22 = xf1.R;
		var tVec:b2Vec2 = m_sweep.localCenter;
		xf1.position.x = m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		xf1.position.y = m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		var s:b2Shape;
		
		var inRange:Boolean = true;
		for (s = m_shapeList; s; s = s.m_next)
		{
			inRange = s.Synchronize(m_world.m_broadPhase, xf1, m_xf);
			if (inRange == false)
			{
				break;
			}
		}
		
		if (inRange == false)
		{
			m_flags |= e_frozenFlag;
			m_linearVelocity.SetZero();
			m_angularVelocity = 0.0;
			for (s = m_shapeList; s; s = s.m_next)
			{
				s.DestroyProxy(m_world.m_broadPhase);
			}
			
			// Failure
			return false;
		}
		
		// Success
		return true;
		
	}

	public function SynchronizeTransform() : void{
		m_xf.R.Set(m_sweep.a);
		//m_xf.position = m_sweep.c - b2Mul(m_xf.R, m_sweep.localCenter);
		var tMat:b2Mat22 = m_xf.R;
		var tVec:b2Vec2 = m_sweep.localCenter;
		m_xf.position.x = m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		m_xf.position.y = m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	}

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	public function IsConnected(other:b2Body) : Boolean{
		for (var jn:b2JointEdge = m_jointList; jn; jn = jn.next)
		{
			if (jn.other == other)
				return jn.joint.m_collideConnected == false;
		}
		
		return false;
	}

	public function Advance(t:Number) : void{
		// Advance to the new safe time.
		m_sweep.Advance(t);
		m_sweep.c.SetV(m_sweep.c0);
		m_sweep.a = m_sweep.a0;
		SynchronizeTransform();
	}

	public var m_flags:uint;
	public var m_type:int;

	public var m_xf:b2XForm = new b2XForm();		// the body origin transform

	public var m_sweep:b2Sweep = new b2Sweep();	// the swept motion for CCD

	public var m_linearVelocity:b2Vec2 = new b2Vec2();
	public var m_angularVelocity:Number;

	public var m_force:b2Vec2 = new b2Vec2();
	public var m_torque:Number;

	public var m_world:b2World;
	public var m_prev:b2Body;
	public var m_next:b2Body;

	public var m_shapeList:b2Shape;
	public var m_shapeCount:int;

	public var m_jointList:b2JointEdge;
	public var m_contactList:b2ContactEdge;

	public var m_mass:Number, m_invMass:Number;
	public var m_I:Number, m_invI:Number;

	public var m_linearDamping:Number;
	public var m_angularDamping:Number;

	public var m_sleepTime:Number;

	public var m_userData:*;
	
	
	// m_flags
	//enum
	//{
		static public var e_frozenFlag:uint			= 0x0002;
		static public var e_islandFlag:uint			= 0x0004;
		static public var e_sleepFlag:uint			= 0x0008;
		static public var e_allowSleepFlag:uint		= 0x0010;
		static public var e_bulletFlag:uint			= 0x0020;
		static public var e_fixedRotationFlag:uint	= 0x0040;
	//};

	// m_type
	//enum
	//{
		static public var e_staticType:uint 	= 1;
		static public var e_dynamicType:uint 	= 2;
		static public var e_maxTypes:uint 		= 3;
	//};
	
};

}
