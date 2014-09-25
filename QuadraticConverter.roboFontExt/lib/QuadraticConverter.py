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
from math import sqrt, log, exp
from AppKit import *
from mojo.drawingTools import drawGlyph, save, restore, stroke, fill, strokeWidth
from mojo.UI import UpdateCurrentGlyphView
from os import path as ospath
import sys

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
		return sqrt(self.squaredLength());

def lerp(t, a, b):
	return (t * b) + ((1.0 - t) * a)

def det2x2(a, b):
	return a.x * b.y - a.y * b.x

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def cubicInflectionParam((p1, c1, c2, p2)):
	"""Returns the parameter value(s) of inflection points, if any.

	Many thanks to Adrian Colomitchi.
	http://caffeineowl.com/graphics/2d/vectorial/cubic-inflexion.html"""
	va = c1 - p1
	vb = c2 - (2.0 * c1) + p1
	vc = p2 - (3.0 * c2) + (3.0 * c1) - p1
	(a, b, c) = (det2x2(vb, vc), det2x2(va, vc), det2x2(va, vb))
	# now we have to solve [ a t^2 + b t + c = 0 ]
	debug = False
	eps = 1.0e-6
	if abs(a) < eps:
		if abs(b) < eps:
			if debug: print "Case A", b
			return None
		t = - c / b
		if t < eps or t > 1.0-eps:
			if debug: print "Case B", t
			return None
		return (t, t)
	disc = b * b - 4.0 * a * c
	if disc < 0.0:
		if debug: print "Case C", disc
		return None
	if disc < eps:
		t = - b / (2.0 * a)
		if t < eps or t > 1.0-eps:
			if debug: print "Case D", t
			return None
		return (t, t)
	disc = sqrt(disc)
	root1 = ( - b - disc ) / (2.0 * a)
	root2 = ( - b + disc ) / (2.0 * a)
	if root2 < root1:
		root1, root2 = root2, root1
	if root1 < eps:
		if root2 < eps:
			if debug: print "Case E", (root1, root2)
			return None
		if root2 <= 1.0 - eps:
			return (root2, root2)
		if debug: print "Case F", (root1, root2)
		return None
	if root1 <= 1.0 - eps:
		if root2 <= 1.0 - eps:
			return (root1, root2)
		return (root1, root1)
	return None

def splitQuadratic(t, (p0, p1, p2)):
	"""Splits a quadratic Bezier into two quadratic Bezier at the given parameter t.

	Uses de Casteljau algorithm."""
	a0 = lerp(t, p0, p1)
	a1 = lerp(t, p1, p2)
	c0 = lerp(t, a0, a1)
	return (p0, a0, c0), (c0, a1, p2)

def splitCubic(t, (p0, p1, p2, p3)):
	"""Splits a cubic Bezier into two cubic Bezier at the given parameter t.

	Uses de Casteljau algorithm."""
	a0 = lerp(t, p0, p1)
	a1 = lerp(t, p1, p2)
	a2 = lerp(t, p2, p3)
	b0 = lerp(t, a0, a1)
	b1 = lerp(t, a1, a2)
	c0 = lerp(t, b0, b1)
	return (p0, a0, b0, c0), (c0, b1, a2, p3)

def splitCubicOnInflection(cubic):
	"""Splits a cubic bezier at inflection points.
	
	Returns one, two or three cubic bezier, in a list."""
	t = cubicInflectionParam(cubic)
	if t == None:
		return [cubic]
	t1, t2 = t
	cub0, tempcubic = splitCubic(t1, cubic)
	if t1 == t2:
		return [cub0, tempcubic]
	t2 = (t2 - t1) / (1.0 - t1)
	cub1, cub2 = splitCubic(t2, tempcubic)
	return [cub0, cub1, cub2]

def quadraticMidPointApprox((p1, c1, c2, p2)):
	"""Returns the midpoint quadratic approximation of a cubic Bezier, disregarding the quality of approximation."""
	#d0 = 0.5 * ((3.0 * c1) - p1)
	#d1 = 0.5 * ((3.0 * c2) - p2)
	#c = 0.5 * (d0 + d1)
	c = 0.25 * ((3.0*(c1 + c2)) - p1 - p2)
	return (p1, c, p2)

def simpleSplitCubic(cubic, n):
	if n <= 1:
		return [cubic]
	c_list = [None] * n
	c_list[0], c_list[1] = splitCubic(1.0/n, cubic)
	for i in range(1, n-1):
		c_list[i], c_list[i+1] = splitCubic(1.0/(n-i), c_list[i])
	return c_list

def adaptiveSimpleCubicSplit(cubic, dmax, n):
	scaleddmax = dmax * 10.3923048454132637612
	(p1, c1, c2, p2) = cubic
	d = 0.5 * ((3.0*(c1 - c2)) + p2 - p1).length()
	if d <= scaleddmax:
		return simpleSplitCubic(cubic, 1)
	t = pow(scaleddmax / d, 0.33333333333333333333333)
	if t >= 0.5:
		return simpleSplitCubic(cubic, 2)
	return simpleSplitCubic(cubic, n)

def uniqueQuadraticWithSameTangentsAsCubic((a, b, c, d)):
	ab = b - a
	ca = a - c
	cd = d - c
	u = det2x2(ab, cd)
	v = det2x2(ca, cd)
	if abs( u ) < 1.0e-5:
		# we have parallel antennas
		return (a, 0.5*(a+d), d)
	t = - v / u
	x = a + ( t * ab )
	return (a, x, d)

def adaptiveConvexCubicSplit(cubic, dmax):
	"""Returns an approximation of a cubic Bezier as a list of quadratic Bezier.

	Assumes the cubic curve has no inflection point.
	Many thanks to Adrian Colomitchi.
	http://caffeineowl.com/graphics/2d/vectorial/cubic2quad01.html"""
	def halve():
		c1, c2 = splitCubic(0.5, cubic)
		return [c1, c2]
	(p1, c1, c2, p2) = cubic
	#d0 = 0.5 * ((3.0 * c1) - p1)
	#d1 = 0.5 * ((3.0 * c2) - p2)
	#d = (d0 - d1).length()
	scaleddmax = dmax * 10.3923048454132637612 # = dmax * 18/√3
	d = 0.5 * ((3.0*(c1 - c2)) + p2 - p1).length()
	if d <= scaleddmax:
		return [cubic]
	t = pow(scaleddmax / d, 0.33333333333333333333333)
	if t > 1.0:
		return [cubic] # should not happen but just in case
	if t >= 0.5:
		return halve()
	#print "adaptive split at", t, (1.0-t)
	cub0, tempcubic = splitCubic(t, cubic)
	t2 = (1.0 - t - t) / (1.0 - t)
	cub1, cub2 = splitCubic(t2, tempcubic)
	(m0, m1, m2, m3) = cub1
	if (m0 - m3).length() < 30:
		return halve()
	return [cub0] + adaptiveConvexCubicSplit(cub1, dmax) + [cub2]

def adaptiveCubicSplit(cubic, dmax):
	"""Returns an approximation of a cubic Bezier as a list of quadratic Bezier."""
	return sum([adaptiveConvexCubicSplit(c, dmax) for c in splitCubicOnInflection(cubic)], [])

def hasGoodSmoothQuadraticApprox(cubic, dmax):
	(p1, c1, c2, p2) = cubic
	scaleddmax = dmax * 10.3923048454132637612
	d0 = 0.5 * ((3.0 * c1) - p1)
	d1 = 0.5 * ((3.0 * c2) - p2)
	if (d0 - d1).length() > scaleddmax:
		return False
	A = 0.5 * (d0 + d1)
	pt0, B, pt1 = uniqueQuadraticWithSameTangentsAsCubic(cubic)
	v = p2 - p1
	l = v.length()
	if l < 1.0e-3:
		return False
	v = (1.0/l) * v
	return abs(det2x2(A - B, v)) <= 2.0 * dmax

def oneHasBadApprox(cubics, dmax):
	for c in cubics:
		if not hasGoodSmoothQuadraticApprox(c, dmax):
			return True
	return False

def adaptiveSmoothCubicSplit(cubic, dmax):
	n = 1
	cubics = [cubic]
	while (n < 10) and oneHasBadApprox(cubics, dmax):
		n += 1
		cubics = simpleSplitCubic(cubic, n)
	return cubics

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def getFirstOnPoint(contour):
	firstSeg = contour[0]
	if firstSeg.type == 'line':
		return firstSeg.points[0]
	return contour[-1].points[-1]

def convert(glyph, maxDistance):
	nbPoints = 0
	def lineto(pen, p):
		pen.addPoint(p, 'line')
	def curveto(pen,(a,b,c)):
		pen.addPoint(a)
		pen.addPoint(b)
		pen.addPoint(c, 'qcurve', True)
	conts = []
	glyph.extremePoints()
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
				cmds.append((lineto, (p1.x, p1.y)))
				nbPoints += 1
				p0 = p1
			elif seg.type == 'qCurve':
				print "Should not have quadratic segment in here. Skipping."
				p0 = seg.points[-1]
			elif seg.type == 'curve':
				p1, p2, p3 = seg.points
				pt0 = Point(p0.x, p0.y)
				pt1 = Point(p1.x, p1.y)
				pt2 = Point(p2.x, p2.y)
				pt3 = Point(p3.x, p3.y)
				cubicSegment = (pt0, pt1, pt2, pt3)
				qsegs = []
				if seg.smooth == False and prevSeg.smooth == False:
					qsegs = [quadraticMidPointApprox(c)
							for c in adaptiveCubicSplit(cubicSegment, maxDistance)]
				else:
					for cubic in splitCubicOnInflection(cubicSegment):
						subs = adaptiveSmoothCubicSplit(cubic, maxDistance)
						qsegs = qsegs + [uniqueQuadraticWithSameTangentsAsCubic(c)
						for c in subs]
				for qseg in qsegs:
					subs = splitQuadratic(0.5, qseg)
					a0, a1, a2 = subs[0]
					a3, a4, a5 = subs[1]
					cmds.append((curveto, ((a1.x, a1.y), (a4.x, a4.y), (a5.x, a5.y))))
					nbPoints += 3
				p0 = p3
			else:
				print "Unknown segment type: "+seg.type+". Skipping."
				p0 = seg.points[-1]
			prevSeg = seg
	ng = RGlyph()
	ng.width = glyph.width
	ng.preferredSegmentStyle = 'qcurve'
	pen = ReverseContourPointPen(ng.getPointPen())
	for cmds in conts:
		pen.beginPath()
		for cmd in cmds:
			action, args = cmd
			action(pen, args)
		pen.endPath()
	ng.update()
	return ng

def convertFont(f, maxDistanceValue, progressBar):
	if f == None:
		return
	hasPath = (f.path != None)
	if hasPath:
		root, tail = ospath.split(f.path)
		QuadraticUFOTail = 'Quadratic_' + tail.split('.')[0] + '.ufo'
		QuadraticUFOPath = ospath.join(root, QuadraticUFOTail)
		f.save(QuadraticUFOPath)
		nf = RFont(QuadraticUFOPath)
	else:
		nf = f
	nf.lib['com.typemytype.robofont.segmentType'] = 'qCurve'
	componentGlyphs = []
	progressBar.setTickCount((len(nf)+9)/10)
	count = 0
	def progress(count):
		if count % 10 == 0:
			progressBar.update()
		return count + 1
	for g in nf:
		if len(g.components) > 0:
			componentGlyphs.append(g)
			if len(g) > 0:
				print "Warning: glyph '"+g.name+"' has", len(g.components), "components and", len(g), "contours."
		else:
			nf[g.name] = convert(g, maxDistanceValue)
			count = progress(count)
	for g in componentGlyphs:
		for component in g.components:
			nf[g.name].components.append(component)
		count = progress(count)
	nf.update()
	if hasPath:
		nf.save()

# - - - - - - - - - - - - - - - - -

OnColor = NSColor.colorWithCalibratedRed_green_blue_alpha_(1, .3, .94, 1)
OffColor = NSColor.colorWithCalibratedRed_green_blue_alpha_(0.2, .8, .8, 1)

class InterfaceWindow(BaseWindowController):
	def __init__(self):
		BaseWindowController.__init__(self)
		self.maxDistanceValue = 0.600
		self.calculatePreview = True
		self.tempGlyph = RGlyph()
		self.w = FloatingWindow((340, 100), 'Quadratic Converter')
		self.w.maxDistanceTitle = TextBox((10, 10, 100, 20), "Max Distance: ")
		minMaxDist = 0.01
		initMaxDist = 0.6
		maxMaxDist = 10.0
		self.w.maxDistanceValueText = TextBox((110, 10, -10, 22), str(initMaxDist))
		self.w.maxDistanceSlider = Slider( (10, 30, -10, 20),
				minValue=log(minMaxDist), maxValue=log(maxMaxDist),
				value=log(initMaxDist), callback=self.maxDistanceSliderCallback )
		self.w.previewCheckBox = CheckBox((10, 60, 70, 20), "Preview", callback=self.previewCheckBoxCallback, value=True)
		self.w.convertCurrentFont = Button((210, 60, 120, 20), "Convert Font", callback=self.convertCurrentFontCallback)
		self.w.open()
		self.w.bind("close", self.windowClosed)
		addObserver(self, "draw", "draw")
		UpdateCurrentGlyphView()

	def drawDiscAtPoint(self, r, x, y, color):
		color.set()
		NSBezierPath.bezierPathWithOvalInRect_(((x-r, y-r), (r*2, r*2))).fill()

	def draw(self, info):
		if not self.calculatePreview:
			return
		scale        = info['scale']
		doodle_glyph = info['glyph']

		if doodle_glyph == None:
			return

		convertedGlyph = convert(CurrentGlyph().copy(), self.maxDistanceValue)

		for c in convertedGlyph:
			for p in c.points:
				if p.type == 'offCurve':
					color = OffColor
					r = 4*scale
				else:
					color = OnColor
					r = 6*scale
				self.drawDiscAtPoint(r, p.x, p.y, color)
		self.tempGlyph.clear()
		self.tempGlyph.appendGlyph(doodle_glyph)
		save()
		stroke(0.2, .8, .8, 1)
		fill(None)
		strokeWidth(scale)
		drawGlyph(convertedGlyph)
		restore()

	def windowClosed(self, sender):
		removeObserver(self, "draw")
		UpdateCurrentGlyphView()

	def maxDistanceSliderCallback(self, sender):
		self.maxDistanceValue = round(exp(sender.get()), 3)
		self.w.maxDistanceValueText.set(self.maxDistanceValue)
		UpdateCurrentGlyphView()

	def previewCheckBoxCallback(self, sender):
		self.calculatePreview = sender.get()
		UpdateCurrentGlyphView()

	# def updateViewCallback(self, sender):
	# 	UpdateCurrentGlyphView()

	def convertCurrentFontCallback(self, sender):
		f = CurrentFont()
		progress = self.startProgress(u"Converting current font…")
		try:
			convertFont(f, self.maxDistanceValue, progress)
		except:
			print "Unexpected error in QuadraticConverter:", sys.exc_info()[0]
		progress.close()
		self.w.close()

InterfaceWindow()

