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
	
	
	
	public class TestCCD extends Test{
		
		public function TestCCD(){
			
			// Set Text field
			Main.m_aboutText.text = "Continuous Collision Detection";
			
			var bd:b2BodyDef;
			var body:b2Body;
			{
				var sd_bottom:b2PolygonDef = new b2PolygonDef();
				sd_bottom.SetAsBox( 45.0/m_physScale, 4.5/m_physScale );
				sd_bottom.density = 4.0;
				sd_bottom.restitution = 1.4;
				
				var sd_left:b2PolygonDef = new b2PolygonDef();
				sd_left.SetAsOrientedBox(4.5/m_physScale, 81.0/m_physScale, new b2Vec2(-43.5/m_physScale, -70.5/m_physScale), -0.2);
				sd_left.density = 4.0;
				sd_left.restitution = 1.4;
				
				var sd_right:b2PolygonDef = new b2PolygonDef();
				sd_right.SetAsOrientedBox(4.5/m_physScale, 81.0/m_physScale, new b2Vec2(43.5/m_physScale, -70.5/m_physScale), 0.2);
				sd_right.density = 4.0;
				sd_right.restitution = 1.4;
				
				bd = new b2BodyDef();
				bd.isBullet = true;
				bd.position.Set( 150.0/m_physScale, 100.0/m_physScale );
				body = m_world.CreateBody(bd);
				body.CreateShape(sd_bottom);
				body.CreateShape(sd_left);
				body.CreateShape(sd_right);
				body.SetMassFromShapes();
			}
			
			// add some small circles for effect
			for (var i:int = 0; i < 5; i++){
				var cd:b2CircleDef = new b2CircleDef();
				cd.radius = (Math.random() * 10 + 5) / m_physScale;
				cd.friction = 0.3;
				cd.density = 1.0;
				cd.restitution = 1.1;
				bd = new b2BodyDef();
				bd.isBullet = true;
				bd.position.Set( (Math.random()*300 + 250)/m_physScale, (Math.random()*320 + 20)/m_physScale );
				body = m_world.CreateBody(bd);
				body.CreateShape(cd);
				body.SetMassFromShapes();
			}
			
		}
		
		
		//======================
		// Member Data 
		//======================
		
	}
	
}