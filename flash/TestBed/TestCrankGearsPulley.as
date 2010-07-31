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


package TestBed{
	
	
	
	import Box2D.Dynamics.*;
	import Box2D.Collision.*;
	import Box2D.Collision.Shapes.*;
	import Box2D.Dynamics.Joints.*;
	import Box2D.Dynamics.Contacts.*;
	import Box2D.Common.*;
	import Box2D.Common.Math.*;
	
	
	
	public class TestCrankGearsPulley extends Test{
		
		public function TestCrankGearsPulley(){
			
			// Set Text field
			Main.m_aboutText.text = "Crank/Gears/Pulley";
			
			var ground:b2Body = m_world.GetGroundBody();
			
			var body:b2Body;
			var sd:b2PolygonDef;
			var bd:b2BodyDef;
			
			//
			// CRANK
			//
			{
				// Define crank.
				sd = new b2PolygonDef();
				sd.SetAsBox(7.5/m_physScale, 30.0/m_physScale);
				sd.density = 1.0;
				
				var rjd:b2RevoluteJointDef = new b2RevoluteJointDef();
				
				var prevBody:b2Body = ground;
				
				bd = new b2BodyDef();
				bd.position.Set(100.0/m_physScale, (360.0-105.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateShape(sd);
				body.SetMassFromShapes();
				
				rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (360.0-75.0)/m_physScale));
				rjd.motorSpeed = 1.0 * -Math.PI;
				rjd.maxMotorTorque = 5000.0;
				rjd.enableMotor = true;
				m_joint1 = m_world.CreateJoint(rjd) as b2RevoluteJoint;
				
				prevBody = body;
				
				// Define follower.
				sd.SetAsBox(7.5/m_physScale, 60.0/m_physScale);
				bd.position.Set(100.0/m_physScale, (360.0-195.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateShape(sd);
				body.SetMassFromShapes();
				
				rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (360.0-135.0)/m_physScale));
				rjd.enableMotor = false;
				m_world.CreateJoint(rjd);
				
				prevBody = body;
				
				// Define piston
				sd.SetAsBox(22.5/m_physScale, 22.5/m_physScale);
				bd.position.Set(100.0/m_physScale, (360.0-255.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateShape(sd);
				body.SetMassFromShapes();
				
				rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (360.0-255.0)/m_physScale));
				m_world.CreateJoint(rjd);
				
				var pjd:b2PrismaticJointDef = new b2PrismaticJointDef();
				pjd.Initialize(ground, body, new b2Vec2(100.0/m_physScale, (360.0-255.0)/m_physScale), new b2Vec2(0.0, 1.0));
				
				pjd.maxMotorForce = 500.0;
				pjd.enableMotor = true;
				
				m_joint2 = m_world.CreateJoint(pjd) as b2PrismaticJoint;
				
				// Create a payload
				sd.density = 2.0;
				bd.position.Set(100.0/m_physScale, (360.0-345.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateShape(sd);
				body.SetMassFromShapes();
			}
			
			
			// 
			// GEARS
			//
			{
				var circle1:b2CircleDef = new b2CircleDef();
				circle1.radius = 25 / m_physScale;
				circle1.density = 5.0;
				
				var circle2:b2CircleDef = new b2CircleDef();
				circle2.radius = 50 / m_physScale;
				circle2.density = 5.0;
				
				var box:b2PolygonDef = new b2PolygonDef();
				box.SetAsBox(10 / m_physScale, 100 / m_physScale);
				box.density = 5.0;
				
				var bd1:b2BodyDef = new b2BodyDef();
				bd1.position.Set(200 / m_physScale, 360/2 / m_physScale);
				var body1:b2Body = m_world.CreateBody(bd1);
				body1.CreateShape(circle1);
				body1.SetMassFromShapes();
				
				var jd1:b2RevoluteJointDef = new b2RevoluteJointDef();
				jd1.Initialize(ground, body1, bd1.position);
				//jd1.anchorPoint.SetV(bd1.position);
				//jd1.body1 = ground;
				//jd1.body2 = body1;
				m_gJoint1 = m_world.CreateJoint(jd1) as b2RevoluteJoint;
				
				var bd2:b2BodyDef = new b2BodyDef();
				bd2.position.Set(275 / m_physScale, 360/2 / m_physScale);
				var body2:b2Body = m_world.CreateBody(bd2);
				body2.CreateShape(circle2);
				body2.SetMassFromShapes();
				
				var jd2:b2RevoluteJointDef = new b2RevoluteJointDef();
				jd2.Initialize(ground, body2, bd2.position);
				//jd2.body1 = ground;
				//jd2.body2 = body2;
				//jd2.anchorPoint.SetV(bd2.position);
				m_gJoint2 = m_world.CreateJoint(jd2) as b2RevoluteJoint;
				
				var bd3:b2BodyDef = new b2BodyDef();
				bd3.position.Set(335 / m_physScale, 360/2 / m_physScale);
				var body3:b2Body = m_world.CreateBody(bd3);
				body3.CreateShape(box);
				body3.SetMassFromShapes();
				
				var jd3:b2PrismaticJointDef = new b2PrismaticJointDef();
				jd3.Initialize(ground, body3, bd3.position, new b2Vec2(0,1));
				//jd3.body1 = ground;
				//jd3.body2 = body3;
				//jd3.anchorPoint.SetV(bd3.position);
				//jd3.axis.Set(0.0, 1.0);
				jd3.lowerTranslation = -25.0 / m_physScale;
				jd3.upperTranslation = 100.0 / m_physScale;
				jd3.enableLimit = true;
				
				m_gJoint3 = m_world.CreateJoint(jd3) as b2PrismaticJoint;
				
				var jd4:b2GearJointDef = new b2GearJointDef();
				jd4.body1 = body1;
				jd4.body2 = body2;
				jd4.joint1 = m_gJoint1;
				jd4.joint2 = m_gJoint2;
				jd4.ratio = circle2.radius / circle1.radius;
				m_gJoint4 = m_world.CreateJoint(jd4) as b2GearJoint;
				
				var jd5:b2GearJointDef = new b2GearJointDef();
				jd5.body1 = body2;
				jd5.body2 = body3;
				jd5.joint1 = m_gJoint2;
				jd5.joint2 = m_gJoint3;
				jd5.ratio = -1.0 / circle2.radius;
				m_gJoint5 = m_world.CreateJoint(jd5) as b2GearJoint;
			}
			
			
			
			//
			// PULLEY
			//
			{
				sd = new b2PolygonDef();
				sd.SetAsBox(50 / m_physScale, 20 / m_physScale);
				sd.density = 5.0;
				
				bd = new b2BodyDef();
				
				bd.position.Set(480 / m_physScale, 200 / m_physScale);
				body2 = m_world.CreateBody(bd);
				body2.CreateShape(sd);
				body2.SetMassFromShapes();
				
				var pulleyDef:b2PulleyJointDef = new b2PulleyJointDef();
				
				var anchor1:b2Vec2 = new b2Vec2(335 / m_physScale, 180 / m_physScale);
				var anchor2:b2Vec2 = new b2Vec2(480 / m_physScale, 180 / m_physScale);
				var groundAnchor1:b2Vec2 = new b2Vec2(335 / m_physScale, 50 / m_physScale);
				var groundAnchor2:b2Vec2 = new b2Vec2(480 / m_physScale, 50 / m_physScale);
				pulleyDef.Initialize(body3, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0);
				
				pulleyDef.maxLength1 = 200 / m_physScale;
				pulleyDef.maxLength2 = 150 / m_physScale;
				
				//m_joint1 = m_world.CreateJoint(pulleyDef) as b2PulleyJoint;
				m_world.CreateJoint(pulleyDef) as b2PulleyJoint;
				
				
				// Add a circle to weigh down the pulley
				var circ:b2CircleDef = new b2CircleDef();
				circ.radius = 40 / m_physScale;
				circ.friction = 0.3;
				circ.restitution = 0.3;
				circ.density = 5.0;
				bd.position.Set(485 / m_physScale, 100 / m_physScale);
				body1 = m_world.CreateBody(bd);
				body1.CreateShape(circ);
				body1.SetMassFromShapes();
			}
			
		}
		
		
		//======================
		// Member Data 
		//======================
		private var m_joint1:b2RevoluteJoint;
		private var m_joint2:b2PrismaticJoint;
		
		public var m_gJoint1:b2RevoluteJoint;
		public var m_gJoint2:b2RevoluteJoint;
		public var m_gJoint3:b2PrismaticJoint;
		public var m_gJoint4:b2GearJoint;
		public var m_gJoint5:b2GearJoint;
		
	}
	
}