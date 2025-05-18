/**
 * ASTRA Programming Language Compiler
 * IR optimizer
 */

#ifndef ASTRA_OPTIMIZER_H
#define ASTRA_OPTIMIZER_H

#include <memory>
#include <vector>
#include <string>
#include <unordered_set>
#include "../ir/generator.h"
#include "../utils/error_handler.h"

namespace astra {

/**
 * Base class for optimization passes
 */
class OptimizationPass {
protected:
    ErrorHandler& errorHandler;
    
public:
    /**
     * Constructor
     */
    OptimizationPass(ErrorHandler& errHandler) : errorHandler(errHandler) {}
    
    /**
     * Virtual destructor
     */
    virtual ~OptimizationPass() = default;
    
    /**
     * Run the optimization pass on a module
     */
    virtual void run(std::shared_ptr<IRModule> module) = 0;
    
    /**
     * Get the name of the pass
     */
    virtual std::string getName() const = 0;
};

/**
 * Dead code elimination pass
 */
class DeadCodeEliminationPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    DeadCodeEliminationPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Dead Code Elimination";
    }
};

/**
 * Constant folding pass
 */
class ConstantFoldingPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    ConstantFoldingPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Constant Folding";
    }
};

/**
 * Common subexpression elimination pass
 */
class CommonSubexpressionEliminationPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    CommonSubexpressionEliminationPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Common Subexpression Elimination";
    }
};

/**
 * Loop optimization pass
 */
class LoopOptimizationPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    LoopOptimizationPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Loop Optimization";
    }
};

/**
 * Function inlining pass
 */
class FunctionInliningPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    FunctionInliningPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Function Inlining";
    }
};

/**
 * Tail call optimization pass
 */
class TailCallOptimizationPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    TailCallOptimizationPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Tail Call Optimization";
    }
};

/**
 * Memory optimization pass
 */
class MemoryOptimizationPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    MemoryOptimizationPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Memory Optimization";
    }
};

/**
 * Vectorization pass
 */
class VectorizationPass : public OptimizationPass {
public:
    /**
     * Constructor
     */
    VectorizationPass(ErrorHandler& errHandler) : OptimizationPass(errHandler) {}
    
    /**
     * Run the pass
     */
    void run(std::shared_ptr<IRModule> module) override;
    
    /**
     * Get the name of the pass
     */
    std::string getName() const override {
        return "Vectorization";
    }
};

/**
 * Optimizer
 */
class Optimizer {
private:
    int optimizationLevel;
    ErrorHandler& errorHandler;
    std::vector<std::unique_ptr<OptimizationPass>> passes;
    
    /**
     * Initialize optimization passes based on optimization level
     */
    void initPasses() {
        // Always add these passes
        passes.push_back(std::make_unique<DeadCodeEliminationPass>(errorHandler));
        passes.push_back(std::make_unique<ConstantFoldingPass>(errorHandler));
        
        if (optimizationLevel >= 1) {
            passes.push_back(std::make_unique<CommonSubexpressionEliminationPass>(errorHandler));
        }
        
        if (optimizationLevel >= 2) {
            passes.push_back(std::make_unique<LoopOptimizationPass>(errorHandler));
            passes.push_back(std::make_unique<FunctionInliningPass>(errorHandler));
        }
        
        if (optimizationLevel >= 3) {
            passes.push_back(std::make_unique<TailCallOptimizationPass>(errorHandler));
            passes.push_back(std::make_unique<MemoryOptimizationPass>(errorHandler));
            passes.push_back(std::make_unique<VectorizationPass>(errorHandler));
        }
    }
    
public:
    /**
     * Constructor
     */
    Optimizer(int level, ErrorHandler& errHandler)
        : optimizationLevel(level), errorHandler(errHandler) {
        initPasses();
    }
    
    /**
     * Optimize IR module
     */
    void optimize(std::shared_ptr<IRModule> module) {
        for (auto& pass : passes) {
            pass->run(module);
        }
    }
    
    /**
     * Get optimization level
     */
    int getOptimizationLevel() const {
        return optimizationLevel;
    }
    
    /**
     * Set optimization level
     */
    void setOptimizationLevel(int level) {
        optimizationLevel = level;
        passes.clear();
        initPasses();
    }
};

} // namespace astra

#endif // ASTRA_OPTIMIZER_H