#include "options_minimal.h"

namespace astra {

CompilerOptions::CompilerOptions()
    : outputFile(""),
      optimizationLevel(0),
      targetArchitecture("x86_64") {}

} // namespace astra