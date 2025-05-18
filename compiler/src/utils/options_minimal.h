#ifndef ASTRA_OPTIONS_MINIMAL_H
#define ASTRA_OPTIONS_MINIMAL_H

#include <string>

namespace astra {

/**
 * Compiler options class
 */
class CompilerOptions {
public:
    /**
     * Constructor with default values
     */
    CompilerOptions();
    
    // Output file
    std::string outputFile;
    
    // Optimization level (0-3)
    int optimizationLevel;
    
    // Target architecture
    std::string targetArchitecture;
};

} // namespace astra

#endif // ASTRA_OPTIONS_MINIMAL_H