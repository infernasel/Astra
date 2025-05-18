#include "options.h"

namespace astra {

CompilerOptions::CompilerOptions()
    : outputFile(""),
      compileOnly(false),
      generateAssembly(false),
      preprocessOnly(false),
      generateDebugInfo(false),
      optimizationLevel(0),
      enableVerification(false),
      analyzeResources(false),
      timingAnalysis(false),
      targetArchitecture("x86_64") {}

} // namespace astra