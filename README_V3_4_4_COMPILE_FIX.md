# LivoxHmi v3.4.4 - Static Touch compile fix

Fixes the SurfaceDefinition geometry hash in StaticTouchDetector to use the current model properties:
- CurvatureRadiusMeters
- CurvatureSign
- OcctBulgeMeters
- OcctTwistMeters

This replaces stale property names from an older surface model and fixes CS1061 errors in v3.4.3.
Touch logic remains unchanged: 5-frame confirm, 5-frame release, one-shot trigger, spatial Zone index.
