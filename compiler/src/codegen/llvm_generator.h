/**
 * ASTRA Programming Language Compiler
 * LLVM Code generator
 */

#ifndef ASTRA_LLVM_GENERATOR_H
#define ASTRA_LLVM_GENERATOR_H

#include "generator.h"

namespace astra {

/**
 * Output type for code generation
 */
enum class OutputType {
    LLVM_IR,    // Generate LLVM IR (.ll file)
    Object,     // Generate object file (.o file)
    Executable  // Generate executable
};

/**
 * Code generation options
 */
struct CodeGenOptions {
    OutputType outputType = OutputType::LLVM_IR;
    int optimizationLevel = 0;
    bool enableDebugInfo = false;
    std::string targetTriple = "";
    std::string targetCPU = "";
    std::string targetFeatures = "";
};

/**
 * LLVM Code Generator
 */
class LLVMCodeGenerator : public CodeGenerator {
private:
    ErrorHandler& errorHandler;
    CodeGenOptions options;
    
    // LLVM-specific methods
    void initializeLLVM();
    void cleanupLLVM();
    std::string generateLLVMIR(std::shared_ptr<IRModule> module);
    std::string getLLVMType(IRType type);
    std::string generateInstruction(std::shared_ptr<IRInstruction> inst);
    bool compileLLVMIR(const std::string& irFile);

public:
    /**
     * Constructor
     */
    LLVMCodeGenerator(ErrorHandler& errorHandler, const CodeGenOptions& options = CodeGenOptions());
    
    /**
     * Destructor
     */
    ~LLVMCodeGenerator();
    
    /**
     * Generate LLVM IR and compile to target code
     */
    virtual bool generate(std::shared_ptr<IRModule> module, const std::string& outputFile) override;
};

} // namespace astra

#endif // ASTRA_LLVM_GENERATOR_H