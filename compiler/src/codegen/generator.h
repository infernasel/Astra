/**
 * ASTRA Programming Language Compiler
 * Code generator
 */

#ifndef ASTRA_CODE_GENERATOR_H
#define ASTRA_CODE_GENERATOR_H

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <fstream>
#include "../ir/generator.h"
#include "../utils/error_handler.h"

namespace astra {

/**
 * Target architecture
 */
enum class TargetArch {
    X86_64,
    ARM64,
    RISCV,
    WASM
};

/**
 * Target OS
 */
enum class TargetOS {
    Linux,
    MacOS,
    Windows,
    Bare
};

/**
 * Target information
 */
struct TargetInfo {
    TargetArch arch;
    TargetOS os;
    int pointerSize;
    int intSize;
    int longSize;
    int floatSize;
    int doubleSize;
    bool isLittleEndian;
    std::string triple;
    
    // Default constructor
    TargetInfo() 
        : arch(TargetArch::X86_64), os(TargetOS::Linux),
          pointerSize(8), intSize(4), longSize(8), floatSize(4),
          doubleSize(8), isLittleEndian(true), triple("x86_64-unknown-linux-gnu") {}
    
    TargetInfo(TargetArch a, TargetOS o)
        : arch(a), os(o) {
        switch (arch) {
            case TargetArch::X86_64:
                pointerSize = 8;
                intSize = 4;
                longSize = 8;
                floatSize = 4;
                doubleSize = 8;
                isLittleEndian = true;
                triple = "x86_64-";
                break;
            case TargetArch::ARM64:
                pointerSize = 8;
                intSize = 4;
                longSize = 8;
                floatSize = 4;
                doubleSize = 8;
                isLittleEndian = true;
                triple = "aarch64-";
                break;
            case TargetArch::RISCV:
                pointerSize = 8;
                intSize = 4;
                longSize = 8;
                floatSize = 4;
                doubleSize = 8;
                isLittleEndian = true;
                triple = "riscv64-";
                break;
            case TargetArch::WASM:
                pointerSize = 4;
                intSize = 4;
                longSize = 8;
                floatSize = 4;
                doubleSize = 8;
                isLittleEndian = true;
                triple = "wasm32-";
                break;
        }
        
        switch (os) {
            case TargetOS::Linux:
                triple += "linux-gnu";
                break;
            case TargetOS::MacOS:
                triple += "apple-darwin";
                break;
            case TargetOS::Windows:
                triple += "pc-windows-msvc";
                break;
            case TargetOS::Bare:
                triple += "unknown-none";
                break;
        }
    }
};

/**
 * Base class for code generators
 */
class CodeGenerator {
protected:
    std::string targetArchitecture;
    ErrorHandler& errorHandler;
    TargetInfo targetInfo;
    
    /**
     * Parse target architecture string
     */
    std::pair<TargetArch, TargetOS> parseTargetArchitecture(const std::string& target) {
        TargetArch arch = TargetArch::X86_64;
        TargetOS os = TargetOS::Linux;
        
        if (target.find("x86_64") != std::string::npos || target.find("x86-64") != std::string::npos) {
            arch = TargetArch::X86_64;
        } else if (target.find("aarch64") != std::string::npos || target.find("arm64") != std::string::npos) {
            arch = TargetArch::ARM64;
        } else if (target.find("riscv") != std::string::npos || target.find("riscv64") != std::string::npos) {
            arch = TargetArch::RISCV;
        } else if (target.find("wasm") != std::string::npos) {
            arch = TargetArch::WASM;
        }
        
        if (target.find("linux") != std::string::npos) {
            os = TargetOS::Linux;
        } else if (target.find("darwin") != std::string::npos || target.find("macos") != std::string::npos) {
            os = TargetOS::MacOS;
        } else if (target.find("windows") != std::string::npos) {
            os = TargetOS::Windows;
        } else if (target.find("bare") != std::string::npos || target.find("none") != std::string::npos) {
            os = TargetOS::Bare;
        }
        
        return {arch, os};
    }
    
public:
    /**
     * Constructor
     */
    CodeGenerator(const std::string& target, ErrorHandler& errHandler)
        : targetArchitecture(target), errorHandler(errHandler), targetInfo() {
          auto targetPair = parseTargetArchitecture(target);
          targetInfo = TargetInfo(targetPair.first, targetPair.second);
        }
    
    /**
     * Generate code from IR
     */
    virtual bool generate(std::shared_ptr<IRModule> module, const std::string& outputFile) = 0;
};

/**
 * LLVM code generator
 */
class LLVMCodeGenerator : public CodeGenerator {
private:
    // LLVM-specific data structures and methods would go here
    
public:
    /**
     * Constructor
     */
    LLVMCodeGenerator(const std::string& target, ErrorHandler& errHandler)
        : CodeGenerator(target, errHandler) {}
    
    /**
     * Generate LLVM IR and compile to target code
     */
    bool generate(std::shared_ptr<IRModule> module, const std::string& outputFile) override;
};

/**
 * Assembly code generator
 */
class AssemblyCodeGenerator : public CodeGenerator {
private:
    std::ofstream outputStream;
    std::unordered_map<std::string, int> labelCounter;
    
    /**
     * Generate a unique label
     */
    std::string genLabel(const std::string& prefix) {
        return prefix + std::to_string(labelCounter[prefix]++);
    }
    
    /**
     * Generate assembly for a function
     */
    void generateFunction(std::shared_ptr<IRFunction> function);
    
    /**
     * Generate assembly for a basic block
     */
    void generateBasicBlock(std::shared_ptr<IRBasicBlock> block);
    
    /**
     * Generate assembly for an instruction
     */
    void generateInstruction(std::shared_ptr<IRInstruction> instruction);
    
public:
    /**
     * Constructor
     */
    AssemblyCodeGenerator(const std::string& target, ErrorHandler& errHandler)
        : CodeGenerator(target, errHandler) {}
    
    /**
     * Generate assembly code
     */
    bool generate(std::shared_ptr<IRModule> module, const std::string& outputFile) override;
};

/**
 * Factory for creating code generators
 */
class CodeGeneratorFactory {
public:
    /**
     * Create a code generator for the given target
     */
    static std::unique_ptr<CodeGenerator> create(const std::string& target, ErrorHandler& errorHandler) {
        // For now, always use LLVM code generator
        return std::make_unique<LLVMCodeGenerator>(target, errorHandler);
    }
};

} // namespace astra

#endif // ASTRA_CODE_GENERATOR_H