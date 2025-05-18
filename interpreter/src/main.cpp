/**
 * ASTRA Programming Language Interpreter
 * Main entry point
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <chrono>

#include "vm/vm.h"
#include "runtime/runtime.h"

void printVersion() {
    std::cout << "ASTRA Interpreter version 0.1.0" << std::endl;
    std::cout << "Copyright (c) 2025 ASTRA Language Team" << std::endl;
}

void printHelp() {
    std::cout << "Usage: astra [options] [script]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help                Display this help message" << std::endl;
    std::cout << "  -v, --version             Display interpreter version information" << std::endl;
    std::cout << "  -e, --eval <code>         Evaluate ASTRA code" << std::endl;
    std::cout << "  -i, --interactive         Run in interactive mode (REPL)" << std::endl;
    std::cout << "  -c, --check               Check syntax only, don't execute" << std::endl;
    std::cout << "  -d, --debug               Enable debug mode" << std::endl;
    std::cout << "  -t, --trace               Enable execution tracing" << std::endl;
    std::cout << "  -p, --profile             Enable profiling" << std::endl;
    std::cout << "  --verify                  Enable formal verification" << std::endl;
    std::cout << "  --timing-analysis         Perform timing analysis" << std::endl;
    std::cout << "  --resource-analysis       Analyze resource usage" << std::endl;
}

void runInteractive(astra::VM& vm) {
    std::cout << "ASTRA Interactive Mode (REPL)" << std::endl;
    std::cout << "Type 'exit' or 'quit' to exit, 'help' for help" << std::endl;
    
    std::string line;
    while (true) {
        std::cout << "astra> ";
        if (!std::getline(std::cin, line)) {
            break;
        }
        
        if (line == "exit" || line == "quit") {
            break;
        } else if (line == "help") {
            std::cout << "Available commands:" << std::endl;
            std::cout << "  exit, quit - Exit the REPL" << std::endl;
            std::cout << "  help       - Display this help message" << std::endl;
            std::cout << "  clear      - Clear the screen" << std::endl;
            std::cout << "  reset      - Reset the VM state" << std::endl;
            std::cout << "  Any other input will be evaluated as ASTRA code" << std::endl;
            continue;
        } else if (line == "clear") {
            std::cout << "\033[2J\033[1;1H"; // ANSI escape code to clear screen
            continue;
        } else if (line == "reset") {
            vm.reset();
            std::cout << "VM state reset" << std::endl;
            continue;
        }
        
        try {
            auto result = vm.eval(line);
            std::cout << "=> " << result.toString() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
}

void runFile(const std::string& filename, astra::VM& vm, bool checkOnly, bool debug, bool trace, bool profile) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: Could not open file: " << filename << std::endl;
        exit(1);
    }
    
    std::string source((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    
    if (checkOnly) {
        try {
            vm.check(source);
            std::cout << "Syntax OK" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Syntax Error: " << e.what() << std::endl;
            exit(1);
        }
        return;
    }
    
    try {
        auto start = std::chrono::high_resolution_clock::now();
        
        auto result = vm.execute(source, debug, trace);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        if (profile) {
            std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
            vm.printProfileInfo();
        }
    } catch (const std::exception& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
        exit(1);
    }
}

void evalCode(const std::string& code, astra::VM& vm, bool debug, bool trace) {
    try {
        auto result = vm.eval(code, debug, trace);
        std::cout << result.toString() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        exit(1);
    }
}

int main(int argc, char* argv[]) {
    bool interactive = false;
    bool checkOnly = false;
    bool debug = false;
    bool trace = false;
    bool profile = false;
    // Temporarily commented out until these features are implemented
    // bool verify = false;
    // bool timingAnalysis = false;
    // bool resourceAnalysis = false;
    std::string evalCode;
    std::string filename;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printHelp();
            return 0;
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--version") == 0) {
            printVersion();
            return 0;
        } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--interactive") == 0) {
            interactive = true;
        } else if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--check") == 0) {
            checkOnly = true;
        } else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--trace") == 0) {
            trace = true;
        } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--profile") == 0) {
            profile = true;
        } else if (strcmp(argv[i], "--verify") == 0) {
            // verify = true; // Temporarily disabled
            std::cout << "Verification is not implemented yet" << std::endl;
        } else if (strcmp(argv[i], "--timing-analysis") == 0) {
            // timingAnalysis = true; // Temporarily disabled
            std::cout << "Timing analysis is not implemented yet" << std::endl;
        } else if (strcmp(argv[i], "--resource-analysis") == 0) {
            // resourceAnalysis = true; // Temporarily disabled
            std::cout << "Resource analysis is not implemented yet" << std::endl;
        } else if ((strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--eval") == 0) && i + 1 < argc) {
            evalCode = argv[++i];
        } else if (argv[i][0] != '-') {
            filename = argv[i];
        } else {
            std::cerr << "Unknown option: " << argv[i] << std::endl;
            printHelp();
            return 1;
        }
    }
    
    // Initialize VM
    astra::VM vm;
    
    // Load standard library
    // Temporarily commented out until stdlib is fixed
    // astra::stdlib::loadStandardLibrary(vm);
    
    // Run in appropriate mode
    if (!evalCode.empty()) {
        ::evalCode(evalCode, vm, debug, trace);
    } else if (!filename.empty()) {
        runFile(filename, vm, checkOnly, debug, trace, profile);
    } else if (interactive) {
        runInteractive(vm);
    } else {
        // No file or eval code provided, and not in interactive mode
        printHelp();
        return 1;
    }
    
    return 0;
}