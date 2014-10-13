#coding=utf-8

# Copyright 2014 Jérémie Hornus
# Copyright 2014 Samuel Hornus
# It is not permitted to derive new software from this work in any way.
# THIS SOFTWARE COMES WITH NO GUARANTEE WHATSOEVER.
# USE AT YOUR OWN RISK.

from vanilla import *
from defconAppKit.windows.baseWindow import BaseWindowController
from mojo.events import addObserver, removeObserver
from lib.tools.bezierTools import curveConverter
from robofab.world import *
from robofab.pens.reverseContourPointPen import ReverseContourPointPen
import robofab.interface.all.dialogs as Dialogs
from math import sqrt, log, exp
from AppKit import *
from mojo.drawingTools import drawGlyph, save, restore, stroke, fill, strokeWidth
from mojo.UI import UpdateCurrentGlyphView
from os import path as ospath
import sys, tempfile, shutil
import robofab.misc.bezierTools as rbt

class Point(object):
	__slots__ = ('x', 'y')
	def __init__(self, ix=0.0, iy=0.0):
		self.x = ix
		self.y = iy
	def __len__(self): return 2
	def __getitem__(self, i):
		if i == 0:
			return self.x
		elif i == 1:
			return self.y
		else:
			print "ERROR ON INDEX", i
			assert(False)
	def __repr__(self):
		return "({:f},{:f})".format(self.x, self.y)
	def __add__(self, rhs): # rhs = right hand side
		return Point(self.x + rhs.x, self.y + rhs.y)
	def __sub__(self, rhs):
		return Point(self.x - rhs.x, self.y - rhs.y)
	def __or__(self, rhs):
		return (self.x * rhs.x + self.y * rhs.y)
	def __mul__(self, s): # 's' is a number, not a point
		return Point(s * self.x, s * self.y)
	def __rmul__(self, s): # 's' is a number, not a point
		return Point(s * self.x, s * self.y)
	def squaredLength(self):
		return self.x * self.x + self.y * self.y
	def length(self):
		return sqrt(self.squaredLength())

def roundPair(p):
	return int(round(p.x)), int(round(p.y))

def lerp(t, a, b):
	return (t * b) + ((1.0 - t) * a)

def det2x2(a, b):
	return a.x * b.y - a.y * b.x

def dot(a, b):
	return (a.x * b.x + a.y * b.y)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

eps = 1.0e-5

def solveQuadratic(a, b, c):
	if abs(a) < eps:
		if abs(b) < eps: return []
		return [- c / b]
	disc = b * b - 4.0 * a * c
	if disc < 0.0: return []
	if disc < eps:
		t = - b / (2.0 * a)
		return [t, t]
	disc = sqrt(disc)
	root1 = ( - b - disc ) / (2.0 * a)
	root2 = ( - b + disc ) / (2.0 * a)
	if root2 < root1:
		return [root2, root1]
	return [root1, root2]

#def cardanMethod(p, q): # from Wikipedia
#	delta = - (4.0 * p*p*p + 27.0 * q*q)
#	sq = cmath.sqrt(-delta/27.0)
#	ubase = cmath.pow(0.5 * (+sq - q), 0.333333333333333)
#	vbase = cmath.pow(0.5 * (-sq - q), 0.333333333333333)
#	ts = []
#	j = cmath.exp(cmath.pi * 0.6666666666666666)
#	jup = 1	# 1  j  j^2
#	jdo = 1	# 1  j^-1  j^-2
#	if delta < 0.0: pass
#	for i in range(3):
#		t = jup*ubase + jdo*vbase
#		if abs(t.imag) < 1.0e-3 * abs(t.real): ts.append(t.real)
#		jup *= j
#		jdo /= j

def solveCubic(a, b, c, d): # from Wikipedia
	return rbt.solveCubic(a,b,c,d)
	#if abs(a) < eps: return solveQuadratic(b, c, d)
	#offset = b/(3.0*a)
	#p = (c - b*offset)/a
	#q = 2.0*offset*offset*offset + (d - offset*c)/a
	#zs = cardanMethod(p, q)
	#if zs = None: return None
	#return [z-offset for z in zs]

def cubicPolyCoeffs((a,b,c,d)):
	return 3.0*(b-a), 3.0*(c - 2.0*b + a), d + 3.0*(b - c) - a

def cubicInflectionParams((p1, c1, c2, p2)):
	"""Returns the parameter value(s) of inflection points, if any.

	Many thanks to Adrian Colomitchi.
	http://caffeineowl.com/graphics/2d/vectorial/cubic-inflexion.html"""
	va = c1 - p1
	vb = c2 - (2.0 * c1) + p1
	vc = p2 - (3.0 * c2) + (3.0 * c1) - p1
	(a, b, c) = (det2x2(vb, vc), det2x2(va, vc), det2x2(va, vb))
	# now we have to solve [ a t^2 + b t + c = 0 ]
	return [t for t in solveQuadratic(a, b, c) if (t>=eps) and (t<=1.0-eps)]

def splitQuadratic(t, quadBez):
	"""Splits a quadratic Bezier into two quadratic Bezier at the given parameter t.

	Uses de Casteljau algorithm."""
	a0 = lerp(t, quadBez[0], quadBez[1])
	a1 = lerp(t, quadBez[1], quadBez[2])
	c0 = lerp(t, a0, a1)
	return (quadBez[0], a0, c0), (c0, a1, quadBez[2])

def splitCubic(t, cubic):
	"""Splits a cubic Bezier into two cubic Bezier at the given parameter t.

	Uses de Casteljau algorithm."""
	a0 = lerp(t, cubic[0], cubic[1])
	a1 = lerp(t, cubic[1], cubic[2])
	a2 = lerp(t, cubic[2], cubic[3])
	b0 = lerp(t, a0, a1)
	b1 = lerp(t, a1, a2)
	c0 = lerp(t, b0, b1)
	return (cubic[0], a0, b0, c0), (c0, b1, a2, cubic[3])

def splitCubicParamUniformly(cubic, n):
	if n <= 1:
		return [cubic]
	c_list = [None] * n
	c_list[0], c_list[1] = splitCubic(1.0/n, cubic)
	for i in range(1, n-1):
		c_list[i], c_list[i+1] = splitCubic(1.0/(n-i), c_list[i])
	return c_list

def splitCubicAtParams(cubic, ts):
	n = len(ts)
	c_list = [cubic] * (n+1)
	prev_t = 0.0
	for i in range(n):
		t = (ts[i] - prev_t) / (1.0 - prev_t)
		c_list[i], c_list[i+1] = splitCubic(t, c_list[i])
		prev_t = ts[i]
	return c_list

def splitCubicOnInflection(cubic, minLength):
	"""Splits a cubic bezier at inflection points.
	
	Returns one, two or three cubic bezier, in a list."""
	if lengthOfCubic(cubic) <= minLength:
		return [cubic]
	# if the two antennas are on the same side, we don't add the
	# inflection point (might be dangerous, we'll see in time...)
	if det2x2(cubic[1]-cubic[0], cubic[3]-cubic[0]) * det2x2(cubic[0]-cubic[3], cubic[2]-cubic[3]) >= 0.0:
		return [cubic]

	t = cubicInflectionParams(cubic)
	if t == []:
		return [cubic]
	cub0, tempcubic = splitCubic(t[0], cubic)
	if len(t) == 1 or t[0] == t[1]:
		return [cub0, tempcubic]
	t2 = (t[1] - t[0]) / (1.0 - t[0])
	cub1, cub2 = splitCubic(t2, tempcubic)
	return [cub0, cub1, cub2]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def positiveHitInterval((a,b,c,d)):
	bc = c-b
	ab = b-a
	other = d - 3.0 * c + 2.0 * b
	denom = det2x2(ab, other)
	if abs(denom) < eps: return []
	ret = ( - 2.0 * det2x2(ab, bc) ) / denom
	if ret <= eps or ret >= 1.0-eps: return []
	return [ret]

def tangentCrossing(cubic):
	c1,c2,c3 = cubicPolyCoeffs(cubic)
	A = det2x2(c2,c3)
	B = 2.0 * det2x2(c1,c3)
	C = det2x2(c1,c2)
	return [t for t in solveQuadratic(A, B, C) if (t>=eps) and (t<=1.0-eps)]

def makeIntervals(evts):
	if evts == []: return [(0.0, 1.0)]
	evts.sort()
	left = 0.0
	intervals = []
	for e in evts:
		if left >= 0.0:
			intervals.append((left, e))
			left = -1.0
		else:
			left = e
	if left >= 0.0 and left < 1.0:
		intervals.append((left, 1.0))
	return intervals

def intersectIntervals(left, right):
	if left == []: return []
	if right == []: return []
	l0, l1 = left[0]
	r0, r1 = right[0]
	if l1 < r1: # l0,l1 will disappear
		rest = intersectIntervals(left[1:], right)
	else:
		rest = intersectIntervals(left, right[1:])
	inter = max(l0,r0), min(l1,r1)
	if inter[0] > inter[1]: return rest
	return [inter]+rest

debug_X = 999999.0

def tangentRatioAt(cubic, T):
	debug = (cubic[0].x == debug_X)
	(l0, l1, l2, l3), (r3, r2, r1, r0) = splitCubic(T, cubic)
	u = det2x2(l1-l0, l2-l3)
	if abs(u) < eps:
		a = l0
	else:
		t = - det2x2(l0-l2, l2-l3) / u
		a = l0 + ( t * (l1-l0) )
	left_len = (a-l3).length()
	u = det2x2(r1-r0, r2-r3)
	if abs(u) < eps:
		b = r0
	else:
		v = det2x2(r0-r2, r2-r3)
		t = - v / u
		b = r0 + ( t * (r1-r0) )
	right_len = (b-r3).length()
	retval = (sys.float_info.max, T, a, b)
	if T < 1.0:
		if right_len > eps:
			retval = (left_len / right_len - 1.0, T, a, b)
	if debug:
		print "ratio", retval[0], "at", retval[1]
	return retval

def sign(f):
	if f > 0.0: return 1
	if f == 0.0: return 0
	return -1

def findZeroAux(cubic, tLeft, ratLeft, tRight, ratRight):
	tMid = 0.5 * ( tLeft + tRight )
	midRat = tangentRatioAt(cubic, tMid)
	if abs(midRat[0]) < 0.001: return midRat
	if abs(tLeft-tRight) < 0.0001: return midRat
	if sign(ratLeft[0]) == sign(midRat[0]):
		return findZeroAux(cubic, tMid, midRat, tRight, ratRight)
	else:
		return findZeroAux(cubic, tLeft, ratLeft, tMid, midRat)

def findZero(cubic, left, right):
	debug = cubic[0].x == debug_X
	if debug:
		print " ***** Finding zero in Interval <",
		print left, right, ">"
	ratLeft = tangentRatioAt(cubic, left)
	ratRight = tangentRatioAt(cubic, right)
	if abs(ratLeft[0]) < 0.001: return ratLeft
	if abs(ratRight[0]) < 0.001: return ratRight
	if sign(ratLeft[0]) != sign(ratRight[0]):
		return findZeroAux(cubic, left, ratLeft, right, ratRight)
	else: return None

def splitIntervalsAtT(intervals, T):
	newint = []
	for (l,r) in intervals:
		if l < T and T < r:
			newint.append((l,T))
			newint.append((T,r))
		else:
			newint.append((l,r))
	return newint

def coolQuad(cubic):
	(a,b,c,d) = cubic
	ab = b-a
	bc = c-b
	cd = d-c
	lab = ab.squaredLength()
	lcd = cd.squaredLength()
	if lab < 0.1:
		if lcd < 0.1:
			#print """I found a cubic segment that has two handles of length zero;
			#You might want to turn it into a simple line."""
			return splitQuadratic(0.5, (a, 0.5*(a+d), d))
		else:
			return splitQuadratic(0.5, (a, c, d))
	else:
		if lcd < 0.1: return splitQuadratic(0.5, (a, b, d))
	
	if det2x2(ab,bc) == 0 and det2x2(bc,cd) == 0:
		mid = 0.5 * (a+d)
		return splitQuadratic(0.5, (a, mid, d))

	hi = positiveHitInterval(cubic) + tangentCrossing(cubic)
	reverse = (d,c,b,a)
	lo = positiveHitInterval(reverse) + tangentCrossing(reverse)
	his = makeIntervals(hi)
	los = makeIntervals(lo)
	los.reverse()
	los = [(1.0-b, 1.0-a) for (a,b) in los]
	intervals = intersectIntervals(his, los)
	for t in cubicInflectionParams(cubic):
		intervals = splitIntervalsAtT(intervals, t)
	debug = cubic[0].x == debug_X
	if debug:
		print "\nIntervals for", cubic
		print intervals
	sols = []
	for l, r in intervals:
		d = 0.0#(r-l) / 10000.0
		s = findZero(cubic, l+d, r-d)
		if s != None: sols.append((abs(s[1]-0.5), s))
	if sols == []:
		return splitQuadratic(0.5, uniqueQuadraticWithSameTangentsAsCubic(cubic))
	sols.sort()
	sol = sols[0][1]
	if debug:
		print sol
	mid = 0.5 * ( sol[2] + sol[3] )
	shortAntenna = ((cubic[0]-sol[2]).squaredLength() <= 9.0 or
		(cubic[3]-sol[3]).squaredLength() <= 9.0)
	#if shortAntenna: print "SHORT ANTENNA"
	return (cubic[0], sol[2], mid), (mid, sol[3], cubic[3])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def distanceToQuadratic(m, quad):
	p0, p1, p2 = quad
	# From http://blog.gludion.com/2009/08/distance-to-quadratic-bezier-curve.html
	A = p1 - p0
	B = p2 - p1 - A
	# coeffs of third degree polynomial
	a = B.squaredLength()
	b = 3.0 * dot(A, B)
	mp = p0 - m
	c = 2.0 * A.squaredLength() + dot(mp, B)
	d = dot(mp, A)
	pts = [splitQuadratic(t, quad)[0][2] for t in solveCubic(a,b,c,d) if (t>=eps) and (t<=1.0-eps)]
	dists = [(m-p).length() for p in (pts+[p0,p2])]
	return min(dists)

def QuadCubicDistance(quad, cubic):
	ts = [0.2, 0.4, 0.5, 0.6, 0.8]
	d = [distanceToQuadratic(splitCubic(t, cubic)[0][3], quad) for t in ts]
	return max(d)

def TwoQuadsCubicDistance(q1, q2, cubic):
	ts = [0.2, 0.4, 0.5, 0.6, 0.8]
	pts = [splitCubic(t, cubic)[0][3] for t in ts]
	d = [min(distanceToQuadratic(p, q1), distanceToQuadratic(p, q2)) for p in pts]
	return max(d)

def lengthOfCubic(cubic, err = 1.0):
	l03 = (cubic[0]-cubic[3]).length()
	l0123 = (cubic[0]-cubic[1]).length() + (cubic[1]-cubic[2]).length() + (cubic[2]-cubic[3]).length()
	if abs(l03-l0123) < err:
		return 0.5 * (l03+l0123)
	a, b = splitCubic(0.5, cubic)
	return lengthOfCubic(a, 0.5*err) + lengthOfCubic(b, 0.5*err)

def fpflAux(cubic, l0, tleft, lleft, tright, lright):
	rat = (l0 - lleft) / (lright - lleft)
	guess = tleft * (1.0 - rat) + tright * rat
	(left, right) = splitCubic(guess, cubic)
	ll = lengthOfCubic(left, 0.1)
	if abs(ll-l0) < 1.0:
		return guess
	if ll < l0:
		return fpflAux(cubic, l0, guess, ll, tright, lright)
	return fpflAux(cubic, l0, tleft, lleft, guess, ll)

def findParamForLength(cubic, cubicLength, l0):
	return fpflAux(cubic, l0, 0.0, 0.0, 1.0, cubicLength)

def gcd(a,b):
	while b:
		a, b = b, a%b
	return a

def refine(paramStack, initLength):
	# (cubic, tRight)
	n = len(paramStack)
	assert(n > 0)
	denominator = n + 1
	cubic = paramStack[0][0][0]
	tOrg = 0
	ts = []
	prevLength = initLength / float(n)
	prevCubics = paramStack[-1]
	for numerator in range(1,denominator):
		prevCubic, prevTRight = prevCubics[numerator-1]
		if numerator == 1:
			prevTLeft = 0.0
		else:
			prevTLeft = prevCubics[numerator-2][1]
		divi = gcd(numerator, denominator)
		if divi == 1:
			t = findParamForLength(prevCubic, prevLength, (1.0-float(numerator)/denominator)*prevLength)
			t = prevTLeft + t * (prevTRight - prevTLeft)
		else:
			numer = int(numerator / divi)
			denom = int(denominator / divi)
			t = paramStack[denom-1][numer-1][1]
		a, cubic = splitCubic((t-tOrg)/(1.0-tOrg), cubic)
		ts.append((a, t))
		tOrg = t
	ts.append((cubic, 1.0))
	paramStack.append(ts)

def quadraticMidPointApprox((p1, c1, c2, p2)):
	"""Returns the midpoint quadratic approximation of a cubic Bezier, disregarding the quality of approximation."""
	#d0 = 0.5 * ((3.0 * c1) - p1)
	#d1 = 0.5 * ((3.0 * c2) - p2)
	#c = 0.5 * (d0 + d1)
	c = 0.25 * ((3.0*(c1 + c2)) - p1 - p2)
	return (p1, c, p2)

def uniqueQuadraticWithSameTangentsAsCubic(cubic):
	a, b, c, d = cubic
	ab = b - a
	cd = d - c
	lab = ab.squaredLength()
	lcd = cd.squaredLength()
	if lab < 0.1:
		if lcd < 0.1:
			#print """I found a cubic segment that has two handles of length zero;
			#You might want to turn it into a simple line."""
			return (a, 0.5*(a+d), d)
		else:
			return (a, c, d)
	else:
		if lcd < 0.1: return (a, b, d)
	u = det2x2(ab, cd)
	v = det2x2(a - c, cd)
	if abs( u ) < 1.0e-5:
		# we have parallel antennas
		return quadraticMidPointApprox(cubic) # (a, 0.5*(a+d), d)
	t = - v / u
	if t < 0.0: # Line (c,d) crosses the line (a,b) on the wrong side: at point p where a is in the middle of b and p.
		return quadraticMidPointApprox(cubic) # (a, 0.5*(a+d), d)
	w = det2x2(ab, d - b)
	if w * u < 0.0: # Line (a,b) crosses the line (c,d) on the wrong side: at point p where d is in the middle of c and p.
		return quadraticMidPointApprox(cubic) # (a, 0.5*(a+d), d)
	x = a + ( t * ab )
	return (a, x, d)

def hasGoodSmoothQuadraticApprox(cubic, dmax, minLength):
	q1, q2 = coolQuad(cubic)
	return (TwoQuadsCubicDistance(q1, q2, cubic) <= dmax, (q1,q2))
	#quad = uniqueQuadraticWithSameTangentsAsCubic(cubic)
	#return (QuadCubicDistance(quad, cubic) <= dmax, quad)

def oneHasBadApprox(cubics, dmax, minLength):
	quads = []
	for c in cubics:
		(good, q) =  hasGoodSmoothQuadraticApprox(c, dmax, minLength)
		if not good:
			return (True, [])
		quads.append(q)
	return (False, quads)

def adaptiveSmoothCubicSplit(cubic, dmax, minLength, useArcLength):
	l = lengthOfCubic(cubic)
	if minLength > 0.0:
		maxN = min(10, int(l / minLength))
	else:
		maxN = 10
	n = 1
	cubics = [cubic]
	paramStack = [[(cubic, 1.0)]]
	(badApprox, quads) = oneHasBadApprox(cubics, dmax, minLength)
	while (n <= maxN) and badApprox:
		n += 1
		if useArcLength:
			refine(paramStack, l)
			cubics = [ct[0] for ct in paramStack[-1]]
		else:
			cubics = splitCubicParamUniformly(cubic, n)
		(badApprox, quads) = oneHasBadApprox(cubics, dmax, minLength)
	if badApprox:
		return [coolQuad(c) for c in cubics]
	else:
		return quads

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def getFirstOnPoint(contour):
	firstSeg = contour[0]
	if firstSeg.type == 'line':
		return firstSeg.points[0]
	return contour[-1].points[-1]

def lineto(pen, p):
	pen.addPoint(roundPair(p), segmentType='line',	smooth=False)

def curveto(pen, (a, b, p, s)):
	pen.addPoint(roundPair(a), segmentType=None,	smooth=False)
	pen.addPoint(roundPair(b), segmentType=None,	smooth=False)
	pen.addPoint(roundPair(p), segmentType='qcurve',	smooth=s)

def convert(glyph, maxDistance, minLength, useArcLength):
	nbPoints = 0
	conts = []
	for contour in glyph:
		conts.append([])
		cmds = conts[-1]
		p0 = getFirstOnPoint(contour)
		nseg = len(contour)
		prevSeg = contour[nseg-1]
		for s in range(nseg):
			seg = contour[s]
			if seg.type == 'line':
				p1 = seg.points[0]
				cmds.append((lineto, p1))
				nbPoints += 1
				p0 = p1
			elif seg.type == 'qcurve':
				#print "Should not have quadratic segment in here. Skipping.",
				p0 = seg.points[-1]
			elif seg.type == 'curve':
				p1, p2, p3 = seg.points
				pt0 = Point(p0.x, p0.y)
				pt1 = Point(p1.x, p1.y)
				pt2 = Point(p2.x, p2.y)
				pt3 = Point(p3.x, p3.y)
				qsegs = []
				inputCubic = (pt0, pt1, pt2, pt3)
				for cubic in splitCubicOnInflection(inputCubic, minLength):
					qsegs = qsegs + adaptiveSmoothCubicSplit(cubic, maxDistance, minLength, useArcLength)
				nbQSegMinusOne = len(qsegs) - 1
				smooth = True
				for i, qseg in enumerate(qsegs):
					# We have to split the quad segment because Robofont does not (seem to) support
					# ON-OFF-ON quadratic bezier curves. If ever Robofont can handle this,
					# then it would suffice to write something like:
					#	(a0, a1, a2) = qseg
					#	cmds.append((curveto, (a1, a2)))

					if i == nbQSegMinusOne: smooth = seg.smooth
					q1, q2 = qseg
					cmds.append((curveto, (q1[1], q2[1], q2[2], smooth)))
					#ql, qr = splitQuadratic(0.5, qseg)
					#cmds.append((curveto, (ql[1], qr[1], qr[2], smooth)))
					nbPoints += 3
				p0 = p3
			else:
				#print "Unknown segment type: "+seg.type+". Skipping.",
				p0 = seg.points[-1]
			prevSeg = seg
	glyph.clearContours()
	glyph.preferredSegmentStyle = 'qcurve'
	pen = ReverseContourPointPen(glyph.getPointPen())
	for cmds in conts:
		if cmds == []: continue
		pen.beginPath()
		for action, args in cmds:
			action(pen, args)
		pen.endPath()
	# Now, we make sure that each contour starts with a ON control point
	for contour in glyph:
		contour.setStartSegment(0)
	glyph.update()
	return nbPoints

# - - - - - - - - - - - - - - - - -

OnColor = NSColor.colorWithCalibratedRed_green_blue_alpha_(1, .3, .94, 1)
OffColor = NSColor.colorWithCalibratedRed_green_blue_alpha_(0.2, .8, .8, 1)

class InterfaceWindow(BaseWindowController):
	def __init__(self):
		BaseWindowController.__init__(self)
		self.w = FloatingWindow((340, 220), 'Quadratic Converter')
		# ---------------------------
		top = 10
		self.w.maxDistanceTitle = TextBox((10, top, 100, 20), "Max Distance: ")
		minMaxDist  = 0.01
		maxMaxDist  = 10.0
		initMaxDist = 1.0
		self.maxDistanceValue = initMaxDist
		self.w.maxDistanceValueText = TextBox((110, top, -10, 22), str(initMaxDist))
		self.w.maxDistanceSlider = Slider( (10, top+20, -10, 20),
				minValue=log(minMaxDist), maxValue=log(maxMaxDist),
				value=log(initMaxDist), callback=self.maxDistanceSliderCallback )
		# ---------------------------
		top = 60
		self.w.minLengthTitle = TextBox((10, top, 150, 20), "Min Segment Length: ")
		minMinLen  = 0
		maxMinLen  = 100
		initMinLen = 30
		self.minLengthValue = initMinLen
		self.w.minLengthValueText = TextBox((160, top, -10, 22), str(initMinLen))
		self.w.minLengthSlider = Slider( (10, top+20, -10, 20),
				minValue=minMinLen, maxValue=maxMinLen,
				value=initMinLen, callback=self.minLengthSliderCallback )
		# ---------------------------
		top = 110
		self.useArcLength = True
		#self.w.arclencheckbox = CheckBox((10, top, 90, 20), "Arc length", callback=self.arcLengthCheckBoxCallback, value=self.useArcLength)
		self.calculatePreview = True
		self.w.previewCheckBox = CheckBox((10, top, 70, 20), "Preview", callback=self.previewCheckBoxCallback, value=self.calculatePreview)
		self.w.closeButton = SquareButton((120, top, 70, 20), "Close", callback=self.closeCallBack)
		self.w.convertCurrentFont = Button((210, top, 120, 20), "Convert Font", callback=self.convertCurrentFontCallback)
		# ---------------------------
		top = 150
		self.layers = ["foreground"]+CurrentFont().layerOrder
		self.w.layerText = TextBox((10, top, 120, 20), "Layer (per-glyph): ")
		self.w.layerPopup = PopUpButton((130, top, 90, 20), self.layers, callback=self.arcLengthCheckBoxCallback)
		self.w.convertCurrentGlyph = Button((225, top, 105, 20), "Convert Glyph", callback=self.convertCurrentGlyphCallback)
		# ---------------------------
		self.w.infoText = TextBox((10, -38, -10, 34), "WARNING. Un-saved modifications in a UFO will not be converted.")
		# ---------------------------
		self.w.open()
		self.w.bind("close", self.windowClosed)
		addObserver(self, "draw", "draw")
		UpdateCurrentGlyphView()

	def convertFont(self, f, progressBar):
		if f == None:
			return False
		if f.path != None:
			root, tail = ospath.split(f.path)
			name, ext = ospath.splitext(tail)
			tail = 'Quadratic_' + name + '.ufo'
			quadPath = ospath.join(root, tail)
			if ospath.exists(quadPath):
				ret = Dialogs.AskYesNoCancel('The UFO "'+quadPath+'" already exists.\nShall we overwrite?',
						default=1) # default value is not taken into account :-(
				if ret != 1: return False
				shutil.rmtree(quadPath)
			shutil.copytree(f.path, quadPath)
			nf = RFont(quadPath, showUI=False)
		else:
			ret = Dialogs.AskYesNoCancel("The font will be modified in place.\nThen you can save it in the UFO format.\nShall we proceed?",
					default=1) # default value is not taken into account :-(
			if ret != 1: return False
			nf = f
		nf.lib['com.typemytype.robofont.segmentType'] = 'qcurve'
		if f.path != None: progressBar.setTickCount(21)
		else: progressBar.setTickCount(20)
		tenth = int(len(nf)/20)
		count = 0
		nbPoints = 0
		badGlyphNames = []
		for g in nf:
			layerName = "Cubic contour"
			cubicLayer = g.getLayer(layerName, clear=True)
			g.copyToLayer(layerName, clear=True)
			if len(g.components) > 0 and len(g) > 0:
				badGlyphNames.append(g.name)
			nbPts = convert(g, self.maxDistanceValue, self.minLengthValue, self.useArcLength)
			nbPoints += nbPts
			if (count % tenth) == 0: progressBar.update(text=u"Converting glyphs…")
			count += 1
		print nbPoints, "points created."
		if badGlyphNames != []:
			if len(badGlyphNames) == 1:
				print "WARNING: The glyph '"+g.name+"' has at least one contour AND one component."
			else:
				print "WARNING: The following glyphs have at least one contour AND one component:"
				for n in badGlyphNames: print n+", ",
				print '\n'
		if f.path != None:
			progressBar.update(text=u'Saving, then opening…')
			nf.save()
			OpenFont(quadPath, showUI=True)
		else:
			nf.update()
		return True

	def drawDiscAtPoint(self, r, x, y, color):
		color.set()
		NSBezierPath.bezierPathWithOvalInRect_(((x-r, y-r), (r*2, r*2))).fill()

	def draw(self, info):
		if not self.calculatePreview:
			return
		cur = CurrentGlyph()
		if cur == None:
			return;

		scale = info['scale']
		layerToConvert = self.layers[self.w.layerPopup.get()]
		otherLayer = layerToConvert != 'foreground'
		if (not otherLayer) and (CurrentFont().lib['com.typemytype.robofont.segmentType'] == 'qcurve'):
			return
		if otherLayer: cur.flipLayers('foreground', layerToConvert)
		copy = cur.copy()
		if otherLayer: cur.flipLayers('foreground', layerToConvert)
		convert(copy, self.maxDistanceValue, self.minLengthValue, self.useArcLength)

		for c in copy:
			for p in c.points:
				if p.type == 'offCurve':
					color = OffColor
					r = 4*scale
				else:
					color = OnColor
					r = 6*scale
				self.drawDiscAtPoint(r, p.x, p.y, color)
		save()
		stroke(0.2, .8, .8, 1)
		fill(None)
		strokeWidth(scale)
		drawGlyph(copy)
		restore()

	def windowClosed(self, sender):
		removeObserver(self, "draw")
		UpdateCurrentGlyphView()

	def maxDistanceSliderCallback(self, sender):
		old = self.maxDistanceValue
		self.maxDistanceValue = round(exp(sender.get()), 3)
		self.w.maxDistanceValueText.set(self.maxDistanceValue)
		if old != self.maxDistanceValue:
			UpdateCurrentGlyphView()

	def minLengthSliderCallback(self, sender):
		old = self.minLengthValue
		self.minLengthValue = int(round(sender.get(), 0))
		self.w.minLengthValueText.set(self.minLengthValue)
		if old != self.minLengthValue:
			UpdateCurrentGlyphView()

	def arcLengthCheckBoxCallback(self, sender):
		self.useArcLength = sender.get()
		UpdateCurrentGlyphView()

	def previewCheckBoxCallback(self, sender):
		self.calculatePreview = sender.get()
		UpdateCurrentGlyphView()

	def convertCurrentGlyphCallback(self, sender):
		g = CurrentGlyph()
		if None == g: return
		layerToConvert = self.layers[self.w.layerPopup.get()]
		if layerToConvert == 'foreground':
			Dialogs.Message("I can only convert contours from a layer different from 'foreground'.")
			return
		g.flipLayers('foreground', layerToConvert)
		g.copyToLayer(layerToConvert)
		convert(g, self.maxDistanceValue, self.minLengthValue, self.useArcLength)
		CurrentFont().update()
		UpdateCurrentGlyphView()

	def convertCurrentFontCallback(self, sender):
		f = CurrentFont()
		if f.lib['com.typemytype.robofont.segmentType'] == 'qcurve':
			Dialogs.Message("I can only convert cubic fonts.")
			return
		progress = self.startProgress(u'Copying font…')
		closeWindow = False
		try:
			closeWindow = self.convertFont(f, progress)
		except:
			print "Unexpected error in QuadraticConverter:", sys.exc_info()
		progress.close()
		#if closeWindow:
		#	self.w.close()
	
	def closeCallBack(self, sender):
		self.w.close()

InterfaceWindow()

