// Separate PlatformIO entry point for the F2 autonomous G1 test image.
// The build filter excludes main.cpp as an independent compilation unit; it is
// included here so F2 shares the tested F1 control core without duplicating it.
#include "main.cpp"
