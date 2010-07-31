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


import Box2D.Collision.*;
import Box2D.Collision.Shapes.*;
import Box2D.Dynamics.Contacts.*;
import Box2D.Dynamics.*;
import Box2D.Common.Math.*;
import Box2D.Common.*;


/// Implement this class to get collision results. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// @warning The contact separation is the last computed value.
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
public class b2ContactListener
{

	/// Called when a contact point is added. This includes the geometry
	/// and the forces.
	public virtual function Add(point:b2ContactPoint) : void{};

	/// Called when a contact point persists. This includes the geometry
	/// and the forces.
	public virtual function Persist(point:b2ContactPoint) : void{};

	/// Called when a contact point is removed. This includes the last
	/// computed geometry and forces.
	public virtual function Remove(point:b2ContactPoint) : void{};
	
	/// Called after a contact point is solved.
	public virtual function Result(point:b2ContactResult) : void{};
};

}
