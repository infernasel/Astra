/**
 * ASTRA Programming Language Compiler
 * Compiler options
 */

#ifndef ASTRA_OPTIONS_H
#define ASTRA_OPTIONS_H

#include <string>
#include <vector>

namespace astra {

/**
 * Compiler options
 */
struct CompilerOptions {
    std::string outputFile;
    std::string targetArchitecture = "x86_64";
    std::vector<std::string> includePaths;
    std::vector<std::string> libraryPaths;
    std::vector<std::string> libraries;
    int optimizationLevel = 0;
    bool compileOnly = false;
    bool generateAssembly = false;
    bool preprocessOnly = false;
    bool generateDebugInfo = false;
    bool enableVerification = false;
    bool analyzeResources = false;
    bool timingAnalysis = false;
};

} // namespace astra

#endif // ASTRA_OPTIONS_H