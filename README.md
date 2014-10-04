## QuadraticConverter

### Extension for RoboFont

**QuadraticConverter** is an extension for RoboFont that converts Cubic (PostScript) UFO to Quadratic (TrueType) UFO.

## Changes

- **0.5.6**
  - Added option to subdivide the smooth segments by arc-length (enabled by default)
- **0.5.5**
  - The contours are correctly orderer for the hinting to perform correctly.
- **0.5.4**
  - Converting a UFO will create a new UFO. Converting a non-UFO modifies it in place.
  - In any case, the original contour is saved in the layer *Cubic contour*.
  - New control for the minimum length of a quadratic segment.
