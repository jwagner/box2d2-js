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

import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.*;
import Box2D.Collision.Shapes.*;


public class b2Collision{
	
	// Null feature
	static public const b2_nullFeature:uint = 0x000000ff;//UCHAR_MAX;
	
	
	static public function ClipSegmentToLine(vOut:Array, vIn:Array, normal:b2Vec2, offset:Number):int
	{
		var cv: ClipVertex;
		
		// Start with no output points
		var numOut:int = 0;
		
		cv = vIn[0];
		var vIn0:b2Vec2 = cv.v;
		cv = vIn[1];
		var vIn1:b2Vec2 = cv.v;
		
		// Calculate the distance of end points to the line
		var distance0:Number = b2Math.b2Dot(normal, vIn0) - offset;
		var distance1:Number = b2Math.b2Dot(normal, vIn1) - offset;
		
		// If the points are behind the plane
		if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
		if (distance1 <= 0.0) vOut[numOut++] = vIn[1];
		
		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0)
		{
			// Find intersection point of edge and plane
			var interp:Number = distance0 / (distance0 - distance1);
			// expanded for performance 
			// vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			cv = vOut[numOut];
			var tVec:b2Vec2 = cv.v;
			tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
			tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
			cv = vOut[numOut];
			var cv2: ClipVertex;
			if (distance0 > 0.0)
			{
				cv2 = vIn[0];
				cv.id = cv2.id;
			}
			else
			{
				cv2 = vIn[1];
				cv.id = cv2.id;
			}
			++numOut;
		}
		
		return numOut;
	}
	
	
	// Find the separation between poly1 and poly2 for a give edge normal on poly1.
	static public function EdgeSeparation(	poly1:b2PolygonShape, xf1:b2XForm, edge1:int, 
											poly2:b2PolygonShape, xf2:b2XForm):Number
	{
		var count1:int = poly1.m_vertexCount;
		var vertices1:Array = poly1.m_vertices;
		var normals1:Array = poly1.m_normals;
		
		var count2:int = poly2.m_vertexCount;
		var vertices2:Array = poly2.m_vertices;
		
		//b2Assert(0 <= edge1 && edge1 < count1);
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		// Convert normal from poly1's frame into poly2's frame.
		//b2Vec2 normal1World = b2Mul(xf1.R, normals1[edge1]);
		tMat = xf1.R;
		tVec = normals1[edge1];
		var normal1WorldX:Number = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var normal1WorldY:Number = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 normal1 = b2MulT(xf2.R, normal1World);
		tMat = xf2.R;
		var normal1X:Number = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
		var normal1Y:Number = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);
		
		// Find support vertex on poly2 for -normal.
		var index:int = 0;
		var minDot:Number = Number.MAX_VALUE;
		for (var i:int = 0; i < count2; ++i)
		{
			//float32 dot = b2Dot(poly2->m_vertices[i], normal1);
			tVec = vertices2[i];
			var dot:Number = tVec.x * normal1X + tVec.y * normal1Y;
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}
		
		//b2Vec2 v1 = b2Mul(xf1, vertices1[edge1]);
		tVec = vertices1[edge1];
		tMat = xf1.R;
		var v1X:Number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var v1Y:Number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 v2 = b2Mul(xf2, vertices2[index]);
		tVec = vertices2[index];
		tMat = xf2.R;
		var v2X:Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var v2Y:Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//var separation:Number = b2Math.b2Dot( b2Math.SubtractVV( v2, v1 ) , normal);
		v2X -= v1X;
		v2Y -= v1Y;
		//float32 separation = b2Dot(v2 - v1, normal1World);
		var separation:Number = v2X * normal1WorldX + v2Y * normal1WorldY;
		return separation;
	}
	
	
	
	
	// Find the max separation between poly1 and poly2 using edge normals
	// from poly1.
	static public function FindMaxSeparation(edgeIndex:Array /*int ptr*/, 
											poly1:b2PolygonShape, xf1:b2XForm, 
											poly2:b2PolygonShape, xf2:b2XForm):Number
	{
		var count1:int = poly1.m_vertexCount;
		var normals1:Array = poly1.m_normals;
		
		var tVec:b2Vec2;
		var tMat:b2Mat22;
		
		// Vector pointing from the centroid of poly1 to the centroid of poly2.
		//b2Vec2 d = b2Mul(xf2, poly2->m_centroid) - b2Mul(xf1, poly1->m_centroid);
		tMat = xf2.R;
		tVec = poly2.m_centroid;
		var dX:Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var dY:Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf1.R;
		tVec = poly1.m_centroid;
		dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//b2Vec2 dLocal1 = b2MulT(xf1.R, d);
		var dLocal1X:Number = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
		var dLocal1Y:Number = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
		
		// Get support vertex as a hint for our search
		var edge:int = 0;
		var maxDot:Number = -Number.MAX_VALUE;
		for (var i:int = 0; i < count1; ++i)
		{
			//var dot:Number = b2Math.b2Dot(normals1[i], dLocal1);
			tVec = normals1[i];
			var dot:Number = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
			if (dot > maxDot)
			{
				maxDot = dot;
				edge = i;
			}
		}
		
		// Get the separation for the edge normal.
		var s:Number = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0)
		{
			return s;
		}
		
		// Check the separation for the previous edge normal.
		var prevEdge:int = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		var sPrev:Number = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
		if (sPrev > 0.0)
		{
			return sPrev;
		}
		
		// Check the separation for the next edge normal.
		var nextEdge:int = edge + 1 < count1 ? edge + 1 : 0;
		var sNext:Number = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
		if (sNext > 0.0)
		{
			return sNext;
		}
		
		// Find the best edge and the search direction.
		var bestEdge:int;
		var bestSeparation:Number;
		var increment:int;
		if (sPrev > s && sPrev > sNext)
		{
			increment = -1;
			bestEdge = prevEdge;
			bestSeparation = sPrev;
		}
		else if (sNext > s)
		{
			increment = 1;
			bestEdge = nextEdge;
			bestSeparation = sNext;
		}
		else
		{
			// pointer out
			edgeIndex[0] = edge;
			return s;
		}
		
		// Perform a local search for the best edge normal.
		while (true)
		{
			
			if (increment == -1)
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			else
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			
			s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
			if (s > 0.0)
			{
				return s;
			}
			
			if (s > bestSeparation)
			{
				bestEdge = edge;
				bestSeparation = s;
			}
			else
			{
				break;
			}
		}
		
		// pointer out
		edgeIndex[0] = bestEdge;
		return bestSeparation;
	}
	
	
	
	static public function FindIncidentEdge(c:Array, 
											poly1:b2PolygonShape, xf1:b2XForm, edge1:int, 
											poly2:b2PolygonShape, xf2:b2XForm) : void
	{
		var count1:int = poly1.m_vertexCount;
		var normals1:Array = poly1.m_normals;
		
		var count2:int = poly2.m_vertexCount;
		var vertices2:Array = poly2.m_vertices;
		var normals2:Array = poly2.m_normals;
		
		//b2Assert(0 <= edge1 && edge1 < count1);
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		// Get the normal of the reference edge in poly2's frame.
		//b2Vec2 normal1 = b2MulT(xf2.R, b2Mul(xf1.R, normals1[edge1]));
		tMat = xf1.R;
		tVec = normals1[edge1];
		var normal1X:Number = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var normal1Y:Number = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf2.R;
		var tX:Number = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
		normal1Y = 		(tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
		normal1X = tX;
		
		// Find the incident edge on poly2.
		var index:int = 0;
		var minDot:Number = Number.MAX_VALUE;
		for (var i:int = 0; i < count2; ++i)
		{
			//var dot:Number = b2Dot(normal1, normals2[i]);
			tVec = normals2[i];
			var dot:Number = (normal1X * tVec.x + normal1Y * tVec.y);
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}
		
		var tClip:ClipVertex;
		// Build the clip vertices for the incident edge.
		var i1:int = index;
		var i2:int = i1 + 1 < count2 ? i1 + 1 : 0;
		
		tClip = c[0];
		//c[0].v = b2Mul(xf2, vertices2[i1]);
		tVec = vertices2[i1];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i1;
		tClip.id.features.incidentVertex = 0;
		
		tClip = c[1];
		//c[1].v = b2Mul(xf2, vertices2[i2]);
		tVec = vertices2[i2];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i2;
		tClip.id.features.incidentVertex = 1;
	}
	
	
	

	// Find edge normal of max separation on A - return if separating axis is found
	// Find edge normal of max separation on B - return if separation axis is found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip
	static private var b2CollidePolyTempVec:b2Vec2 = new b2Vec2();
	// The normal points from 1 to 2
	static public function b2CollidePolygons(manifold:b2Manifold, 
											polyA:b2PolygonShape, xfA:b2XForm,
											polyB:b2PolygonShape, xfB:b2XForm) : void
	{
		var cv: ClipVertex;
		
		manifold.pointCount = 0;

		var edgeA:int = 0;
		var edgeAO:Array = [edgeA];
		var separationA:Number = FindMaxSeparation(edgeAO, polyA, xfA, polyB, xfB);
		edgeA = edgeAO[0];
		if (separationA > 0.0)
			return;

		var edgeB:int = 0;
		var edgeBO:Array = [edgeB];
		var separationB:Number = FindMaxSeparation(edgeBO, polyB, xfB, polyA, xfA);
		edgeB = edgeBO[0];
		if (separationB > 0.0)
			return;

		var poly1:b2PolygonShape;	// reference poly
		var poly2:b2PolygonShape;	// incident poly
		var xf1:b2XForm = new b2XForm();
		var xf2:b2XForm = new b2XForm();
		var edge1:int;		// reference edge
		var flip:uint;
		const k_relativeTol:Number = 0.98;
		const k_absoluteTol:Number = 0.001;

		// TODO_ERIN use "radius" of poly for absolute tolerance.
		if (separationB > k_relativeTol * separationA + k_absoluteTol)
		{
			poly1 = polyB;
			poly2 = polyA;
			xf1.Set(xfB);
			xf2.Set(xfA);
			edge1 = edgeB;
			flip = 1;
		}
		else
		{
			poly1 = polyA;
			poly2 = polyB;
			xf1.Set(xfA);
			xf2.Set(xfB);
			edge1 = edgeA;
			flip = 0;
		}

		var incidentEdge:Array = [new ClipVertex(), new ClipVertex()];
		FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		var count1:int = poly1.m_vertexCount;
		var vertices1:Array = poly1.m_vertices;

		var tVec: b2Vec2 = vertices1[edge1];
		var v11:b2Vec2 = tVec.Copy();
		if (edge1 + 1 < count1) {
			tVec = vertices1[int(edge1+1)];
			var v12:b2Vec2 = tVec.Copy();
		} else {
			tVec = vertices1[0];
			v12 = tVec.Copy();
		}

		var dv:b2Vec2 = b2Math.SubtractVV(v12 , v11);
		var sideNormal:b2Vec2 = b2Math.b2MulMV(xf1.R, b2Math.SubtractVV(v12 , v11));
		sideNormal.Normalize();
		var frontNormal:b2Vec2 = b2Math.b2CrossVF(sideNormal, 1.0);
		
		v11 = b2Math.b2MulX(xf1, v11);
		v12 = b2Math.b2MulX(xf1, v12);

		var frontOffset:Number = b2Math.b2Dot(frontNormal, v11);
		var sideOffset1:Number = -b2Math.b2Dot(sideNormal, v11);
		var sideOffset2:Number = b2Math.b2Dot(sideNormal, v12);

		// Clip incident edge against extruded edge1 side edges.
		var clipPoints1:Array = [new ClipVertex(), new ClipVertex()];
		var clipPoints2:Array = [new ClipVertex(), new ClipVertex()];
		var np:int;

		// Clip to box side 1
		//np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);
		np = ClipSegmentToLine(clipPoints1, incidentEdge, sideNormal.Negative(), sideOffset1);

		if (np < 2)
			return;

		// Clip to negative box side 1
		np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, sideOffset2);

		if (np < 2)
			return;

		// Now clipPoints2 contains the clipped points.
		manifold.normal = flip ? frontNormal.Negative() : frontNormal.Copy();

		var pointCount:int = 0;
		for (var i:int = 0; i < b2Settings.b2_maxManifoldPoints; ++i)
		{
			cv = clipPoints2[i];
			var separation:Number = b2Math.b2Dot(frontNormal, cv.v) - frontOffset;

			if (separation <= 0.0)
			{
				var cp:b2ManifoldPoint = manifold.points[ pointCount ];
				cp.separation = separation;
				cp.localPoint1 = b2Math.b2MulXT(xfA, cv.v);
				cp.localPoint2 = b2Math.b2MulXT(xfB, cv.v);
				cp.id.key = cv.id._key;
				cp.id.features.flip = flip;
				++pointCount;
			}
		}

		manifold.pointCount = pointCount;
	}
	
	
	
	static public function b2CollideCircles(
		manifold:b2Manifold, 
		circle1:b2CircleShape, xf1:b2XForm, 
		circle2:b2CircleShape, xf2:b2XForm) : void
	{
		manifold.pointCount = 0;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		//b2Vec2 p1 = b2Mul(xf1, circle1->m_localPosition);
		tMat = xf1.R; tVec = circle1.m_localPosition;
		var p1X:Number = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var p1Y:Number = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 p2 = b2Mul(xf2, circle2->m_localPosition);
		tMat = xf2.R; tVec = circle2.m_localPosition;
		var p2X:Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var p2Y:Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		//b2Vec2 d = p2 - p1;
		var dX:Number = p2X - p1X;
		var dY:Number = p2Y - p1Y;
		//var distSqr:Number = b2Math.b2Dot(d, d);
		var distSqr:Number = dX * dX + dY * dY;
		var r1:Number = circle1.m_radius;
		var r2:Number = circle2.m_radius;
		var radiusSum:Number = r1 + r2;
		if (distSqr > radiusSum * radiusSum)
		{
			return;
		}
		
		var separation:Number;
		if (distSqr < Number.MIN_VALUE)
		{
			separation = -radiusSum;
			manifold.normal.Set(0.0, 1.0);
		}
		else
		{
			var dist:Number = Math.sqrt(distSqr);
			separation = dist - radiusSum;
			var a:Number = 1.0 / dist;
			manifold.normal.x = a * dX;
			manifold.normal.y = a * dY;
		}
		
		manifold.pointCount = 1;
		var tPoint:b2ManifoldPoint = manifold.points[0];
		tPoint.id.key = 0;
		tPoint.separation = separation;
		
		p1X += r1 * manifold.normal.x;
		p1Y += r1 * manifold.normal.y;
		p2X -= r2 * manifold.normal.x;
		p2Y -= r2 * manifold.normal.y;
		
		//b2Vec2 p = 0.5f * (p1 + p2);
		var pX:Number = 0.5 * (p1X + p2X);
		var pY:Number = 0.5 * (p1Y + p2Y);
		
		//tPoint.localPoint1 = b2MulT(xf1, p);
		var tX:Number = pX - xf1.position.x;
		var tY:Number = pY - xf1.position.y;
		tPoint.localPoint1.x = (tX * xf1.R.col1.x + tY * xf1.R.col1.y );
		tPoint.localPoint1.y = (tX * xf1.R.col2.x + tY * xf1.R.col2.y );
		//tPoint.localPoint2 = b2MulT(xf2, p);
		tX = pX - xf2.position.x;
		tY = pY - xf2.position.y;
		tPoint.localPoint2.x = (tX * xf2.R.col1.x + tY * xf2.R.col1.y );
		tPoint.localPoint2.y = (tX * xf2.R.col2.x + tY * xf2.R.col2.y );
	}
	
	
	
	static public function b2CollidePolygonAndCircle(
		manifold:b2Manifold, 
		polygon:b2PolygonShape, xf1:b2XForm,
		circle:b2CircleShape, xf2:b2XForm) : void
	{
		manifold.pointCount = 0;
		var tPoint:b2ManifoldPoint;
		
		var dX:Number;
		var dY:Number;
		var positionX:Number;
		var positionY:Number;
		
		var tVec:b2Vec2;
		var tMat:b2Mat22;
		
		// Compute circle position in the frame of the polygon.
		//b2Vec2 c = b2Mul(xf2, circle->m_localPosition);
		tMat = xf2.R;
		tVec = circle.m_localPosition;
		var cX:Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY:Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//b2Vec2 cLocal = b2MulT(xf1, c);
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;
		tMat = xf1.R;
		var cLocalX:Number = (dX * tMat.col1.x + dY * tMat.col1.y);
		var cLocalY:Number = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		var dist:Number;
		
		// Find the min separating edge.
		var normalIndex:int = 0;
		var separation:Number = -Number.MAX_VALUE;
		var radius:Number = circle.m_radius;
		var vertexCount:int = polygon.m_vertexCount;
		var vertices:Array = polygon.m_vertices;
		var normals:Array = polygon.m_normals;

		for (var i:int = 0; i < vertexCount; ++i)
		{
			//float32 s = b2Dot(normals[i], cLocal - vertices[i]);
			tVec = vertices[i];
			dX = cLocalX-tVec.x;
			dY = cLocalY-tVec.y;
			tVec = normals[i];
			var s:Number = tVec.x * dX + tVec.y * dY;
			
			if (s > radius)
			{
				// Early out.
				return;
			}
			
			if (s > separation)
			{
				separation = s;
				normalIndex = i;
			}
		}
		
		// If the center is inside the polygon ...
		if (separation < Number.MIN_VALUE)
		{
			manifold.pointCount = 1;
			//manifold->normal = b2Mul(xf1.R, normals[normalIndex]);
			tVec = normals[normalIndex];
			tMat = xf1.R;
			manifold.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			manifold.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			
			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			//b2Vec2 position = c - radius * manifold->normal;
			positionX = cX - radius * manifold.normal.x;
			positionY = cY - radius * manifold.normal.y;
			//manifold->points[0].localPoint1 = b2MulT(xf1, position);
			dX = positionX - xf1.position.x;
			dY = positionY - xf1.position.y;
			tMat = xf1.R;
			tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			//manifold->points[0].localPoint2 = b2MulT(xf2, position);
			dX = positionX - xf2.position.x;
			dY = positionY - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			
			tPoint.separation = separation - radius;
			return;
		}
		
		// Project the circle center onto the edge segment.
		var vertIndex1:int = normalIndex;
		var vertIndex2:int = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		tVec = vertices[vertIndex1];
		var tVec2:b2Vec2 = vertices[vertIndex2];
		//var e:b2Vec2 = b2Math.SubtractVV(vertices[vertIndex2] , polygon.vertices[vertIndex1]);
		var eX:Number = tVec2.x - tVec.x;
		var eY:Number = tVec2.y - tVec.y;
		
		//var length:Number = e.Normalize();
		var length:Number = Math.sqrt(eX*eX + eY*eY);
		eX /= length;
		eY /= length;
		//b2Assert(length > B2_FLT_EPSILON);
		
		// Project the center onto the edge.
		//float32 u = b2Dot(cLocal - polygon->m_vertices[vertIndex1], e);
		dX = cLocalX - tVec.x;
		dY = cLocalY - tVec.y;
		var u:Number = dX*eX + dY*eY;
		
		tPoint = manifold.points[0];
		
		var pX:Number, pY:Number;
		if (u <= 0.0)
		{
			pX = tVec.x;
			pY = tVec.y;
			tPoint.id.features.incidentEdge = b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex1;
		}
		else if (u >= length)
		{
			pX = tVec2.x;
			pY = tVec2.y;
			tPoint.id.features.incidentEdge = b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex2;
		}
		else
		{
			//p = vertices[vertIndex1] + u * e;
			pX = eX * u + tVec.x;
			pY = eY * u + tVec.y;
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = b2_nullFeature;
		}
		
		//d = b2Math.SubtractVV(xLocal , p);
		dX = cLocalX - pX;
		dY = cLocalY - pY;
		//dist = d.Normalize();
		dist = Math.sqrt(dX*dX + dY*dY);
		dX /= dist;
		dY /= dist;
		if (dist > radius)
		{
			return;
		}
		
		manifold.pointCount = 1;
		//manifold->normal = b2Mul(xf1.R, d);
		tMat = xf1.R;
		manifold.normal.x = tMat.col1.x * dX + tMat.col2.x * dY;
		manifold.normal.y = tMat.col1.y * dX + tMat.col2.y * dY;
		//b2Vec2 position = c - radius * manifold->normal;
		positionX = cX - radius * manifold.normal.x;
		positionY = cY - radius * manifold.normal.y;
		//manifold->points[0].localPoint1 = b2MulT(xf1, position);
		dX = positionX - xf1.position.x;
		dY = positionY - xf1.position.y;
		tMat = xf1.R;
		tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
		tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
		//manifold->points[0].localPoint2 = b2MulT(xf2, position);
		dX = positionX - xf2.position.x;
		dY = positionY - xf2.position.y;
		tMat = xf2.R;
		tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
		tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
		tPoint.separation = dist - radius;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
	}




	static public function b2TestOverlap(a:b2AABB, b:b2AABB):Boolean
	{
		var t1:b2Vec2 = b.lowerBound;
		var t2:b2Vec2 = a.upperBound;
		//d1 = b2Math.SubtractVV(b.lowerBound, a.upperBound);
		var d1X:Number = t1.x - t2.x;
		var d1Y:Number = t1.y - t2.y;
		//d2 = b2Math.SubtractVV(a.lowerBound, b.upperBound);
		t1 = a.lowerBound;
		t2 = b.upperBound;
		var d2X:Number = t1.x - t2.x;
		var d2Y:Number = t1.y - t2.y;
		
		if (d1X > 0.0 || d1Y > 0.0)
			return false;
		
		if (d2X > 0.0 || d2Y > 0.0)
			return false;
		
		return true;
	}
	
	
	

}

}
