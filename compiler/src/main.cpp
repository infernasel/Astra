/**
 * ASTRA Programming Language Compiler
 * Main entry point
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <cstring>

#include "lexer/lexer.h"
#include "parser/parser.h"
#include "semantic/analyzer.h"
#include "ir/generator.h"
#include "optimizer/optimizer.h"
#include "codegen/generator.h"
#include "utils/error_handler.h"
#include "utils/options.h"

void printVersion() {
    std::cout << "ASTRA Compiler version 0.1.0" << std::endl;
    std::cout << "Copyright (c) 2025 ASTRA Language Team" << std::endl;
}

void printHelp() {
    std::cout << "Usage: astrac [options] file..." << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help                Display this help message" << std::endl;
    std::cout << "  -v, --version             Display compiler version information" << std::endl;
    std::cout << "  -o <file>                 Write output to <file>" << std::endl;
    std::cout << "  -c                        Compile only, do not link" << std::endl;
    std::cout << "  -S                        Generate assembly code" << std::endl;
    std::cout << "  -E                        Preprocess only" << std::endl;
    std::cout << "  -g                        Generate debug information" << std::endl;
    std::cout << "  -O<level>                 Set optimization level (0-3)" << std::endl;
    std::cout << "  -W<warning>               Enable specific warning" << std::endl;
    std::cout << "  -f<feature>               Enable specific feature" << std::endl;
    std::cout << "  -I<dir>                   Add directory to include search path" << std::endl;
    std::cout << "  -L<dir>                   Add directory to library search path" << std::endl;
    std::cout << "  -l<library>               Link with library" << std::endl;
    std::cout << "  --target=<target>         Specify target architecture" << std::endl;
    std::cout << "  --verify                  Enable formal verification" << std::endl;
    std::cout << "  --analyze-resources       Analyze resource usage" << std::endl;
    std::cout << "  --timing-analysis         Perform timing analysis" << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse command line options
    astra::CompilerOptions options;
    std::vector<std::string> inputFiles;
    
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-') {
            // Handle options
            if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
                printHelp();
                return 0;
            } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--version") == 0) {
                printVersion();
                return 0;
            } else if (strcmp(argv[i], "-o") == 0) {
                if (i + 1 < argc) {
                    options.outputFile = argv[++i];
                } else {
                    std::cerr << "Error: -o option requires an argument" << std::endl;
                    return 1;
                }
            } else if (strcmp(argv[i], "-c") == 0) {
                options.compileOnly = true;
            } else if (strcmp(argv[i], "-S") == 0) {
                options.generateAssembly = true;
            } else if (strcmp(argv[i], "-E") == 0) {
                options.preprocessOnly = true;
            } else if (strcmp(argv[i], "-g") == 0) {
                options.generateDebugInfo = true;
            } else if (strncmp(argv[i], "-O", 2) == 0) {
                options.optimizationLevel = atoi(argv[i] + 2);
            } else if (strcmp(argv[i], "--verify") == 0) {
                options.enableVerification = true;
            } else if (strcmp(argv[i], "--analyze-resources") == 0) {
                options.analyzeResources = true;
            } else if (strcmp(argv[i], "--timing-analysis") == 0) {
                options.timingAnalysis = true;
            } else if (strncmp(argv[i], "--target=", 9) == 0) {
                options.targetArchitecture = argv[i] + 9;
            } else {
                std::cerr << "Warning: Unrecognized option: " << argv[i] << std::endl;
            }
        } else {
            // Input file
            inputFiles.push_back(argv[i]);
        }
    }
    
    if (inputFiles.empty()) {
        std::cerr << "Error: No input files" << std::endl;
        printHelp();
        return 1;
    }
    
    // Initialize error handler
    astra::ErrorHandler errorHandler;
    
    // Process each input file
    for (const auto& inputFile : inputFiles) {
        std::cout << "Compiling " << inputFile << std::endl;
        
        // Open input file
        std::ifstream file(inputFile);
        if (!file) {
            errorHandler.reportError("Could not open input file: " + inputFile);
            continue;
        }
        
        // Read file content
        std::string sourceCode((std::istreambuf_iterator<char>(file)),
                               std::istreambuf_iterator<char>());
        
        // Create lexer
        auto lexer = std::make_unique<astra::Lexer>(sourceCode, inputFile, errorHandler);
        
        // Tokenize input
        auto tokens = lexer->tokenize();
        if (errorHandler.hasErrors()) {
            continue;
        }
        
        // Create parser
        auto parser = std::make_unique<astra::Parser>(tokens, errorHandler);
        
        // Parse tokens into AST
        auto ast = parser->parse();
        if (errorHandler.hasErrors()) {
            continue;
        }
        
        // Semantic analysis
        auto semanticAnalyzer = std::make_unique<astra::SemanticAnalyzer>(errorHandler);
        semanticAnalyzer->analyze(ast);
        if (errorHandler.hasErrors()) {
            continue;
        }
        
        // Generate IR
        auto irGenerator = std::make_unique<astra::IRGenerator>(errorHandler);
        auto ir = irGenerator->generate(ast);
        if (errorHandler.hasErrors()) {
            continue;
        }
        
        // Optimize IR
        auto optimizer = std::make_unique<astra::Optimizer>(options.optimizationLevel, errorHandler);
        optimizer->optimize(ir);
        
        // Generate code
        auto codeGenerator = std::make_unique<astra::LLVMCodeGenerator>(options.targetArchitecture, errorHandler);
        bool success = codeGenerator->generate(ir, options.outputFile.empty() ? 
                                              inputFile + ".o" : options.outputFile);
        
        if (success) {
            std::cout << "Successfully compiled " << inputFile << std::endl;
        }
    }
    
    return errorHandler.hasErrors() ? 1 : 0;
}