// Wrapper to compile production source files in the native test build.
// PlatformIO's test_build_src doesn't compile src/ files for the native platform,
// so we include them explicitly here.
#include "gran_analysis.cpp"
#include "scheduler.cpp"
