var b2Collision = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Collision.prototype.__constructor = function(){}
b2Collision.prototype.__varz = function(){
}
// static attributes
b2Collision.b2_nullFeature =  0x000000ff;
b2Collision.b2CollidePolyTempVec =  new b2Vec2();
// static methods
b2Collision.ClipSegmentToLine = function (vOut, vIn, normal, offset) {
		var cv;
		
		
		var numOut = 0;
		
		cv = vIn[0];
		var vIn0 = cv.v;
		cv = vIn[1];
		var vIn1 = cv.v;
		
		
		var distance0 = b2Math.b2Dot(normal, vIn0) - offset;
		var distance1 = b2Math.b2Dot(normal, vIn1) - offset;
		
		
		if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
		if (distance1 <= 0.0) vOut[numOut++] = vIn[1];
		
		
		if (distance0 * distance1 < 0.0)
		{
			
			var interp = distance0 / (distance0 - distance1);
			
			
			cv = vOut[numOut];
			var tVec = cv.v;
			tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
			tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
			cv = vOut[numOut];
			var cv2;
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
b2Collision.EdgeSeparation = function (	poly1, xf1, edge1, 
											poly2, xf2) {
		var count1 = poly1.m_vertexCount;
		var vertices1 = poly1.m_vertices;
		var normals1 = poly1.m_normals;
		
		var count2 = poly2.m_vertexCount;
		var vertices2 = poly2.m_vertices;
		
		
		
		var tMat;
		var tVec;
		
		
		
		tMat = xf1.R;
		tVec = normals1[edge1];
		var normal1WorldX = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var normal1WorldY = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tMat = xf2.R;
		var normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
		var normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);
		
		
		var index = 0;
		var minDot = Number.MAX_VALUE;
		for (var i = 0; i < count2; ++i)
		{
			
			tVec = vertices2[i];
			var dot = tVec.x * normal1X + tVec.y * normal1Y;
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}
		
		
		tVec = vertices1[edge1];
		tMat = xf1.R;
		var v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tVec = vertices2[index];
		tMat = xf2.R;
		var v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		
		v2X -= v1X;
		v2Y -= v1Y;
		
		var separation = v2X * normal1WorldX + v2Y * normal1WorldY;
		return separation;
	}
b2Collision.FindMaxSeparation = function (edgeIndex , 
											poly1, xf1, 
											poly2, xf2) {
		var count1 = poly1.m_vertexCount;
		var normals1 = poly1.m_normals;
		
		var tVec;
		var tMat;
		
		
		
		tMat = xf2.R;
		tVec = poly2.m_centroid;
		var dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf1.R;
		tVec = poly1.m_centroid;
		dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		
		var dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
		var dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
		
		
		var edge = 0;
		var maxDot = -Number.MAX_VALUE;
		for (var i = 0; i < count1; ++i)
		{
			
			tVec = normals1[i];
			var dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
			if (dot > maxDot)
			{
				maxDot = dot;
				edge = i;
			}
		}
		
		
		var s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0)
		{
			return s;
		}
		
		
		var prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		var sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
		if (sPrev > 0.0)
		{
			return sPrev;
		}
		
		
		var nextEdge = edge + 1 < count1 ? edge + 1 : 0;
		var sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
		if (sNext > 0.0)
		{
			return sNext;
		}
		
		
		var bestEdge = 0;
		var bestSeparation;
		var increment = 0;
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
			
			edgeIndex[0] = edge;
			return s;
		}
		
		
		while (true)
		{
			
			if (increment == -1)
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			else
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			
			s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
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
		
		
		edgeIndex[0] = bestEdge;
		return bestSeparation;
	}
b2Collision.FindIncidentEdge = function (c, 
											poly1, xf1, edge1, 
											poly2, xf2) {
		var count1 = poly1.m_vertexCount;
		var normals1 = poly1.m_normals;
		
		var count2 = poly2.m_vertexCount;
		var vertices2 = poly2.m_vertices;
		var normals2 = poly2.m_normals;
		
		
		
		var tMat;
		var tVec;
		
		
		
		tMat = xf1.R;
		tVec = normals1[edge1];
		var normal1X = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var normal1Y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tMat = xf2.R;
		var tX = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
		normal1Y = 		(tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
		normal1X = tX;
		
		
		var index = 0;
		var minDot = Number.MAX_VALUE;
		for (var i = 0; i < count2; ++i)
		{
			
			tVec = normals2[i];
			var dot = (normal1X * tVec.x + normal1Y * tVec.y);
			if (dot < minDot)
			{
				minDot = dot;
				index = i;
			}
		}
		
		var tClip;
		
		var i1 = index;
		var i2 = i1 + 1 < count2 ? i1 + 1 : 0;
		
		tClip = c[0];
		
		tVec = vertices2[i1];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i1;
		tClip.id.features.incidentVertex = 0;
		
		tClip = c[1];
		
		tVec = vertices2[i2];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i2;
		tClip.id.features.incidentVertex = 1;
	}
b2Collision.b2CollidePolygons = function (manifold, 
											polyA, xfA,
											polyB, xfB) {
		var cv;
		
		manifold.pointCount = 0;

		var edgeA = 0;
		var edgeAO = [edgeA];
		var separationA = b2Collision.FindMaxSeparation(edgeAO, polyA, xfA, polyB, xfB);
		edgeA = edgeAO[0];
		if (separationA > 0.0)
			return;

		var edgeB = 0;
		var edgeBO = [edgeB];
		var separationB = b2Collision.FindMaxSeparation(edgeBO, polyB, xfB, polyA, xfA);
		edgeB = edgeBO[0];
		if (separationB > 0.0)
			return;

		var poly1;	
		var poly2;	
		var xf1 = new b2XForm();
		var xf2 = new b2XForm();
		var edge1 = 0;		
		var flip = 0;
		var k_relativeTol = 0.98;
		var k_absoluteTol = 0.001;

		
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

		var incidentEdge = [new ClipVertex(), new ClipVertex()];
		b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		var count1 = poly1.m_vertexCount;
		var vertices1 = poly1.m_vertices;

		var tVec = vertices1[edge1];
		var v11 = tVec.Copy();
		if (edge1 + 1 < count1) {
			tVec = vertices1[parseInt(edge1+1)];
			var v12 = tVec.Copy();
		} else {
			tVec = vertices1[0];
			v12 = tVec.Copy();
		}

		var dv = b2Math.SubtractVV(v12 , v11);
		var sideNormal = b2Math.b2MulMV(xf1.R, b2Math.SubtractVV(v12 , v11));
		sideNormal.Normalize();
		var frontNormal = b2Math.b2CrossVF(sideNormal, 1.0);
		
		v11 = b2Math.b2MulX(xf1, v11);
		v12 = b2Math.b2MulX(xf1, v12);

		var frontOffset = b2Math.b2Dot(frontNormal, v11);
		var sideOffset1 = -b2Math.b2Dot(sideNormal, v11);
		var sideOffset2 = b2Math.b2Dot(sideNormal, v12);

		
		var clipPoints1 = [new ClipVertex(), new ClipVertex()];
		var clipPoints2 = [new ClipVertex(), new ClipVertex()];
		var np = 0;

		
		
		np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, sideNormal.Negative(), sideOffset1);

		if (np < 2)
			return;

		
		np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, sideOffset2);

		if (np < 2)
			return;

		
		manifold.normal = flip ? frontNormal.Negative() : frontNormal.Copy();

		var pointCount = 0;
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; ++i)
		{
			cv = clipPoints2[i];
			var separation = b2Math.b2Dot(frontNormal, cv.v) - frontOffset;

			if (separation <= 0.0)
			{
				var cp = manifold.points[ pointCount ];
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
b2Collision.b2CollideCircles = function (
		manifold, 
		circle1, xf1, 
		circle2, xf2) {
		manifold.pointCount = 0;
		
		var tMat;
		var tVec;
		
		
		tMat = xf1.R; tVec = circle1.m_localPosition;
		var p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		tMat = xf2.R; tVec = circle2.m_localPosition;
		var p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		
		var distSqr = dX * dX + dY * dY;
		var r1 = circle1.m_radius;
		var r2 = circle2.m_radius;
		var radiusSum = r1 + r2;
		if (distSqr > radiusSum * radiusSum)
		{
			return;
		}
		
		var separation;
		if (distSqr < Number.MIN_VALUE)
		{
			separation = -radiusSum;
			manifold.normal.Set(0.0, 1.0);
		}
		else
		{
			var dist = Math.sqrt(distSqr);
			separation = dist - radiusSum;
			var a = 1.0 / dist;
			manifold.normal.x = a * dX;
			manifold.normal.y = a * dY;
		}
		
		manifold.pointCount = 1;
		var tPoint = manifold.points[0];
		tPoint.id.key = 0;
		tPoint.separation = separation;
		
		p1X += r1 * manifold.normal.x;
		p1Y += r1 * manifold.normal.y;
		p2X -= r2 * manifold.normal.x;
		p2Y -= r2 * manifold.normal.y;
		
		
		var pX = 0.5 * (p1X + p2X);
		var pY = 0.5 * (p1Y + p2Y);
		
		
		var tX = pX - xf1.position.x;
		var tY = pY - xf1.position.y;
		tPoint.localPoint1.x = (tX * xf1.R.col1.x + tY * xf1.R.col1.y );
		tPoint.localPoint1.y = (tX * xf1.R.col2.x + tY * xf1.R.col2.y );
		
		tX = pX - xf2.position.x;
		tY = pY - xf2.position.y;
		tPoint.localPoint2.x = (tX * xf2.R.col1.x + tY * xf2.R.col1.y );
		tPoint.localPoint2.y = (tX * xf2.R.col2.x + tY * xf2.R.col2.y );
	}
b2Collision.b2CollidePolygonAndCircle = function (
		manifold, 
		polygon, xf1,
		circle, xf2) {
		manifold.pointCount = 0;
		var tPoint;
		
		var dX;
		var dY;
		var positionX;
		var positionY;
		
		var tVec;
		var tMat;
		
		
		
		tMat = xf2.R;
		tVec = circle.m_localPosition;
		var cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;
		tMat = xf1.R;
		var cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y);
		var cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y);
		
		var dist;
		
		
		var normalIndex = 0;
		var separation = -Number.MAX_VALUE;
		var radius = circle.m_radius;
		var vertexCount = polygon.m_vertexCount;
		var vertices = polygon.m_vertices;
		var normals = polygon.m_normals;

		for (var i = 0; i < vertexCount; ++i)
		{
			
			tVec = vertices[i];
			dX = cLocalX-tVec.x;
			dY = cLocalY-tVec.y;
			tVec = normals[i];
			var s = tVec.x * dX + tVec.y * dY;
			
			if (s > radius)
			{
				
				return;
			}
			
			if (s > separation)
			{
				separation = s;
				normalIndex = i;
			}
		}
		
		
		if (separation < Number.MIN_VALUE)
		{
			manifold.pointCount = 1;
			
			tVec = normals[normalIndex];
			tMat = xf1.R;
			manifold.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			manifold.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			
			tPoint = manifold.points[0];
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			
			positionX = cX - radius * manifold.normal.x;
			positionY = cY - radius * manifold.normal.y;
			
			dX = positionX - xf1.position.x;
			dY = positionY - xf1.position.y;
			tMat = xf1.R;
			tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			
			dX = positionX - xf2.position.x;
			dY = positionY - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
			tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
			
			tPoint.separation = separation - radius;
			return;
		}
		
		
		var vertIndex1 = normalIndex;
		var vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		tVec = vertices[vertIndex1];
		var tVec2 = vertices[vertIndex2];
		
		var eX = tVec2.x - tVec.x;
		var eY = tVec2.y - tVec.y;
		
		
		var length = Math.sqrt(eX*eX + eY*eY);
		eX /= length;
		eY /= length;
		
		
		
		
		dX = cLocalX - tVec.x;
		dY = cLocalY - tVec.y;
		var u = dX*eX + dY*eY;
		
		tPoint = manifold.points[0];
		
		var pX, pY;
		if (u <= 0.0)
		{
			pX = tVec.x;
			pY = tVec.y;
			tPoint.id.features.incidentEdge = b2Collision.b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex1;
		}
		else if (u >= length)
		{
			pX = tVec2.x;
			pY = tVec2.y;
			tPoint.id.features.incidentEdge = b2Collision.b2_nullFeature;
			tPoint.id.features.incidentVertex = vertIndex2;
		}
		else
		{
			
			pX = eX * u + tVec.x;
			pY = eY * u + tVec.y;
			tPoint.id.features.incidentEdge = normalIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
		}
		
		
		dX = cLocalX - pX;
		dY = cLocalY - pY;
		
		dist = Math.sqrt(dX*dX + dY*dY);
		dX /= dist;
		dY /= dist;
		if (dist > radius)
		{
			return;
		}
		
		manifold.pointCount = 1;
		
		tMat = xf1.R;
		manifold.normal.x = tMat.col1.x * dX + tMat.col2.x * dY;
		manifold.normal.y = tMat.col1.y * dX + tMat.col2.y * dY;
		
		positionX = cX - radius * manifold.normal.x;
		positionY = cY - radius * manifold.normal.y;
		
		dX = positionX - xf1.position.x;
		dY = positionY - xf1.position.y;
		tMat = xf1.R;
		tPoint.localPoint1.x = (dX*tMat.col1.x + dY*tMat.col1.y);
		tPoint.localPoint1.y = (dX*tMat.col2.x + dY*tMat.col2.y);
		
		dX = positionX - xf2.position.x;
		dY = positionY - xf2.position.y;
		tMat = xf2.R;
		tPoint.localPoint2.x = (dX*tMat.col1.x + dY*tMat.col1.y);
		tPoint.localPoint2.y = (dX*tMat.col2.x + dY*tMat.col2.y);
		tPoint.separation = dist - radius;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
	}
b2Collision.b2TestOverlap = function (a, b) {
		var t1 = b.lowerBound;
		var t2 = a.upperBound;
		
		var d1X = t1.x - t2.x;
		var d1Y = t1.y - t2.y;
		
		t1 = a.lowerBound;
		t2 = b.upperBound;
		var d2X = t1.x - t2.x;
		var d2Y = t1.y - t2.y;
		
		if (d1X > 0.0 || d1Y > 0.0)
			return false;
		
		if (d2X > 0.0 || d2Y > 0.0)
			return false;
		
		return true;
	}
// attributes
// methods