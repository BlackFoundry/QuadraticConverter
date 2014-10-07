## QuadraticConverter

### Extension for RoboFont

**QuadraticConverter** is an extension for RoboFont that converts Cubic (PostScript) UFO to Quadratic (TrueType) UFO.

## Changes

- **0.6**
  - Faster by not using the slow `contour.autoStartSegment()`
  - Optimized code
  - Slightly better approximation in non-smooth segments
- **0.5.9**
  - The conversion avoids inserting some inflection points
  - The approximation of a cubic by a single quadratic is more robust
- **0.5.8**
  - Fixed wrong approximation when a handle has length zero
- **0.5.7**
  - Improved behavior when glyph has both contours and components
- **0.5.6**
  - Added option to subdivide the smooth segments by arc-length (enabled by default)
- **0.5.5**
  - The contours are correctly orderer for the hinting to perform correctly
- **0.5.4**
  - Converting a UFO will create a new UFO. Converting a non-UFO modifies it in place
  - In any case, the original contour is saved in the layer *Cubic contour*
  - New control for the minimum length of a quadratic segment
