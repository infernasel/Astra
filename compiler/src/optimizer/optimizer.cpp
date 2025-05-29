#include "optimizer.h"

namespace astra {

// Concrete optimizer implementation
class DefaultOptimizer : public OptimizerPass {
public:
    DefaultOptimizer(const std::string& name) : name(name) {}
    
    void run(std::shared_ptr<IRModule> module) override {
        // Implement optimization pass
        if (name == "constant-folding") {
            // Perform constant folding
            // ...
        } else if (name == "dead-code-elimination") {
            // Perform dead code elimination
            // ...
        } else if (name == "common-subexpression-elimination") {
            // Perform common subexpression elimination
            // ...
        } else if (name == "loop-invariant-code-motion") {
            // Perform loop invariant code motion
            // ...
        } else if (name == "function-inlining") {
            // Perform function inlining
            // ...
        } else if (name == "tail-call-optimization") {
            // Perform tail call optimization
            // ...
        }
    }
    
    std::string getName() const override {
        return name;
    }
    
private:
    std::string name;
};

Optimizer::Optimizer(int optimizationLevel, ErrorHandler& errorHandler)
    : optimizationLevel(optimizationLevel), errorHandler(errorHandler) {}

void Optimizer::optimize(std::unique_ptr<IRModule>& module) {
    if (optimizationLevel <= 0) {
        return;
    }
    
    // Create a shared_ptr from the unique_ptr for the optimizer passes
    std::shared_ptr<IRModule> sharedModule(module.release());
    
    // Apply optimizations based on the optimization level
    if (optimizationLevel >= 1) {
        // Level 1 optimizations
        auto constantFoldingPass = std::make_shared<DefaultOptimizer>("constant-folding");
        constantFoldingPass->run(sharedModule);
        
        auto deadCodeEliminationPass = std::make_shared<DefaultOptimizer>("dead-code-elimination");
        deadCodeEliminationPass->run(sharedModule);
    }
    
    if (optimizationLevel >= 2) {
        // Level 2 optimizations
        auto csePass = std::make_shared<DefaultOptimizer>("common-subexpression-elimination");
        csePass->run(sharedModule);
        
        auto licmPass = std::make_shared<DefaultOptimizer>("loop-invariant-code-motion");
        licmPass->run(sharedModule);
    }
    
    if (optimizationLevel >= 3) {
        // Level 3 optimizations
        auto inliningPass = std::make_shared<DefaultOptimizer>("function-inlining");
        inliningPass->run(sharedModule);
        
        auto tailCallPass = std::make_shared<DefaultOptimizer>("tail-call-optimization");
        tailCallPass->run(sharedModule);
    }
    
    module = std::unique_ptr<IRModule>(std::move(sharedModule));
    module = std::unique_ptr<IRModule>(sharedModule.get());
    sharedModule.release(); // Release ownership from shared_ptr
}

void Optimizer::constantFolding(std::unique_ptr<IRModule>& module) {
    // Implement constant folding optimization
    // This optimization evaluates constant expressions at compile time
    
    // For each function in the module
    for (auto& function : module->functions) {
        // For each block in the function
        for (auto& block : function->blocks) {
            // For each instruction in the block
            for (size_t i = 0; i < block->instructions.size(); ++i) {
                auto& instruction = block->instructions[i];
                
                // Check if the instruction is a binary operation
                auto binaryOp = std::dynamic_pointer_cast<IRBinaryOperation>(instruction);
                if (binaryOp) {
                    // Check if both operands are constants
                    auto leftConst = std::dynamic_pointer_cast<IRConstant>(binaryOp->left);
                    auto rightConst = std::dynamic_pointer_cast<IRConstant>(binaryOp->right);
                    
                    if (leftConst && rightConst) {
                        // Evaluate the constant expression
                        std::shared_ptr<IRConstant> result = evaluateConstantExpression(binaryOp->op, leftConst, rightConst);
                        
                        if (result) {
                            // Replace the binary operation with the constant result
                            block->instructions[i] = result;
                        }
                    }
                }
                
                // Check if the instruction is a unary operation
                auto unaryOp = std::dynamic_pointer_cast<IRUnaryOperation>(instruction);
                if (unaryOp) {
                    // Check if the operand is a constant
                    auto operandConst = std::dynamic_pointer_cast<IRConstant>(unaryOp->operand);
                    
                    if (operandConst) {
                        // Evaluate the constant expression
                        std::shared_ptr<IRConstant> result = evaluateConstantExpression(unaryOp->op, operandConst);
                        
                        if (result) {
                            // Replace the unary operation with the constant result
                            block->instructions[i] = result;
                        }
                    }
                }
            }
        }
    }
    
    // Recursively optimize submodules
    for (auto& submodule : module->submodules) {
        constantFolding(submodule);
    }
}

void Optimizer::deadCodeElimination(std::unique_ptr<IRModule>& module) {
    // Implement dead code elimination optimization
    // This optimization removes code that has no effect on the program's output
    
    // For each function in the module
    for (auto& function : module->functions) {
        // Compute the set of live variables
        std::unordered_set<std::string> liveVariables;
        
        // Backward pass to mark live variables
        for (auto blockIt = function->blocks.rbegin(); blockIt != function->blocks.rend(); ++blockIt) {
            auto& block = *blockIt;
            
            // Backward pass through instructions
            for (auto instIt = block->instructions.rbegin(); instIt != block->instructions.rend(); ++instIt) {
                auto& instruction = *instIt;
                
                // Check if the instruction uses any variables
                std::unordered_set<std::string> usedVariables = getUsedVariables(instruction);
                
                // Mark used variables as live
                liveVariables.insert(usedVariables.begin(), usedVariables.end());
                
                // Check if the instruction defines a variable
                std::string definedVariable = getDefinedVariable(instruction);
                
                if (!definedVariable.empty()) {
                    // If the defined variable is not live, mark the instruction for removal
                    if (liveVariables.find(definedVariable) == liveVariables.end()) {
                        // Mark for removal by setting to nullptr
                        // We'll remove nullptr instructions in a separate pass
                        *instIt = nullptr;
                    } else {
                        // Remove the defined variable from the live set
                        // (unless it's used in its own definition)
                        if (usedVariables.find(definedVariable) == usedVariables.end()) {
                            liveVariables.erase(definedVariable);
                        }
                    }
                }
            }
        }
        
        // Remove marked instructions
        for (auto& block : function->blocks) {
            block->instructions.erase(
                std::remove_if(block->instructions.begin(), block->instructions.end(),
                              [](const std::shared_ptr<IRInstruction>& inst) { return inst == nullptr; }),
                block->instructions.end()
            );
        }
    }
    
    // Recursively optimize submodules
    for (auto& submodule : module->submodules) {
        deadCodeElimination(submodule);
    }
}

void Optimizer::commonSubexpressionElimination(std::unique_ptr<IRModule>& module) {
    // Implement common subexpression elimination optimization
    // This optimization identifies and eliminates repeated computations
    
    // For each function in the module
    for (auto& function : module->functions) {
        // For each block in the function
        for (auto& block : function->blocks) {
            // Map of expressions to variables that hold their values
            std::unordered_map<std::string, std::string> expressionMap;
            
            // For each instruction in the block
            for (size_t i = 0; i < block->instructions.size(); ++i) {
                auto& instruction = block->instructions[i];
                
                // Check if the instruction is an expression
                std::string expressionKey = getExpressionKey(instruction);
                
                if (!expressionKey.empty()) {
                    // Check if we've seen this expression before
                    auto it = expressionMap.find(expressionKey);
                    
                    if (it != expressionMap.end()) {
                        // Replace the expression with a reference to the existing variable
                        std::string varName = it->second;
                        auto varRef = std::make_shared<IRVariable>(varName);
                        
                        // If this is a variable declaration, update its initializer
                        auto varDecl = std::dynamic_pointer_cast<IRVariableDeclaration>(instruction);
                        if (varDecl) {
                            varDecl->initializer = varRef;
                        } else {
                            // Otherwise, replace the instruction with a variable reference
                            block->instructions[i] = varRef;
                        }
                    } else {
                        // Add this expression to the map
                        std::string varName = getDefinedVariable(instruction);
                        if (!varName.empty()) {
                            expressionMap[expressionKey] = varName;
                        }
                    }
                }
            }
        }
    }
    
    // Recursively optimize submodules
    for (auto& submodule : module->submodules) {
        commonSubexpressionElimination(submodule);
    }
}

void Optimizer::loopInvariantCodeMotion(std::unique_ptr<IRModule>& module) {
    // Implement loop invariant code motion optimization
    // This optimization moves code that doesn't change inside a loop to outside the loop
    
    // For each function in the module
    for (auto& function : module->functions) {
        // Identify loops in the function
        std::vector<Loop> loops = identifyLoops(function);
        
        // For each loop
        for (auto& loop : loops) {
            // Identify loop invariant instructions
            std::vector<std::shared_ptr<IRInstruction>> invariantInstructions;
            
            // For each block in the loop
            for (auto& block : loop.blocks) {
                // For each instruction in the block
                for (auto it = block->instructions.begin(); it != block->instructions.end(); ) {
                    auto& instruction = *it;
                    
                    // Check if the instruction is loop invariant
                    if (isLoopInvariant(instruction, loop)) {
                        // Add to the list of invariant instructions
                        invariantInstructions.push_back(instruction);
                        
                        // Remove the instruction from the loop
                        it = block->instructions.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            
            // Insert invariant instructions before the loop
            if (!invariantInstructions.empty() && !loop.preheader->instructions.empty()) {
                // Find the position to insert (before the branch to the loop)
                auto insertPos = loop.preheader->instructions.end() - 1;
                
                // Insert the invariant instructions
                loop.preheader->instructions.insert(insertPos, 
                                                 invariantInstructions.begin(), 
                                                 invariantInstructions.end());
            }
        }
    }
    
    // Recursively optimize submodules
    for (auto& submodule : module->submodules) {
        loopInvariantCodeMotion(submodule);
    }
}

void Optimizer::functionInlining(std::unique_ptr<IRModule>& module) {
    // Implement function inlining optimization
    // This optimization replaces function calls with the body of the called function
    
    // For each function in the module
    for (auto& function : module->functions) {
        // For each block in the function
        for (auto& block : function->blocks) {
            // For each instruction in the block
            for (size_t i = 0; i < block->instructions.size(); ++i) {
                auto& instruction = block->instructions[i];
                
                // Check if the instruction is a function call
                auto call = std::dynamic_pointer_cast<IRCall>(instruction);
                if (call) {
                    // Get the called function
                    auto callee = getCalledFunction(call, module);
                    
                    // Check if the function can be inlined
                    if (callee && canInline(callee)) {
                        // Inline the function call
                        std::vector<std::shared_ptr<IRInstruction>> inlinedInstructions = 
                            inlineFunction(call, callee);
                        
                        // Replace the call with the inlined instructions
                        block->instructions.erase(block->instructions.begin() + i);
                        block->instructions.insert(block->instructions.begin() + i, 
                                                inlinedInstructions.begin(), 
                                                inlinedInstructions.end());
                        
                        // Adjust the index to account for the inserted instructions
                        i += inlinedInstructions.size() - 1;
                    }
                }
            }
        }
    }
    
    // Recursively optimize submodules
    for (auto& submodule : module->submodules) {
        functionInlining(submodule);
    }
}

void Optimizer::tailCallOptimization(std::unique_ptr<IRModule>& module) {
    // Implement tail call optimization
    // This optimization replaces recursive tail calls with loops
    
    // For each function in the module
    for (auto& function : module->functions) {
        // For each block in the function
        for (auto& block : function->blocks) {
            // Check if the last instruction is a return statement
            if (!block->instructions.empty()) {
                auto returnStmt = std::dynamic_pointer_cast<IRReturnStatement>(block->instructions.back());
                
                if (returnStmt && returnStmt->value) {
                    // Check if the return value is a call
                    auto call = std::dynamic_pointer_cast<IRCall>(returnStmt->value);
                    
                    if (call) {
                        // Check if it's a recursive call
                        auto callee = getCalledFunction(call, module);
                        
                        if (callee && callee == function.get()) {
                            // Replace the recursive tail call with a loop
                            optimizeTailCall(block, call, function);
                        }
                    }
                }
            }
        }
    }
    
    // Recursively optimize submodules
    for (auto& submodule : module->submodules) {
        tailCallOptimization(submodule);
    }
}

// Helper methods

std::shared_ptr<IRConstant> Optimizer::evaluateConstantExpression(IRBinaryOp op, 
                                                                std::shared_ptr<IRConstant> left, 
                                                                std::shared_ptr<IRConstant> right) {
    // Implement constant expression evaluation for binary operations
    // This is a simplified implementation
    
    // Check if the operands have the same type
    if (left->type != right->type) {
        return nullptr;
    }
    
    // Evaluate based on the operation and type
    switch (left->type) {
        case IRType::Int:
            {
                int leftValue = std::get<int>(left->value);
                int rightValue = std::get<int>(right->value);
                int result = 0;
                
                switch (op) {
                    case IRBinaryOp::Add:
                        result = leftValue + rightValue;
                        break;
                    case IRBinaryOp::Subtract:
                        result = leftValue - rightValue;
                        break;
                    case IRBinaryOp::Multiply:
                        result = leftValue * rightValue;
                        break;
                    case IRBinaryOp::Divide:
                        if (rightValue == 0) {
                            errorHandler.reportError("Division by zero in constant expression");
                            return nullptr;
                        }
                        result = leftValue / rightValue;
                        break;
                    case IRBinaryOp::Modulo:
                        if (rightValue == 0) {
                            errorHandler.reportError("Modulo by zero in constant expression");
                            return nullptr;
                        }
                        result = leftValue % rightValue;
                        break;
                    default:
                        return nullptr;
                }
                
                return std::make_shared<IRConstant>(result, IRType::Int);
            }
        case IRType::Float:
            {
                float leftValue = std::get<float>(left->value);
                float rightValue = std::get<float>(right->value);
                float result = 0.0f;
                
                switch (op) {
                    case IRBinaryOp::Add:
                        result = leftValue + rightValue;
                        break;
                    case IRBinaryOp::Subtract:
                        result = leftValue - rightValue;
                        break;
                    case IRBinaryOp::Multiply:
                        result = leftValue * rightValue;
                        break;
                    case IRBinaryOp::Divide:
                        if (rightValue == 0.0f) {
                            errorHandler.reportError("Division by zero in constant expression");
                            return nullptr;
                        }
                        result = leftValue / rightValue;
                        break;
                    default:
                        return nullptr;
                }
                
                return std::make_shared<IRConstant>(result, IRType::Float);
            }
        case IRType::Bool:
            {
                bool leftValue = std::get<bool>(left->value);
                bool rightValue = std::get<bool>(right->value);
                bool result = false;
                
                switch (op) {
                    case IRBinaryOp::And:
                        result = leftValue && rightValue;
                        break;
                    case IRBinaryOp::Or:
                        result = leftValue || rightValue;
                        break;
                    case IRBinaryOp::Equal:
                        result = leftValue == rightValue;
                        break;
                    case IRBinaryOp::NotEqual:
                        result = leftValue != rightValue;
                        break;
                    default:
                        return nullptr;
                }
                
                return std::make_shared<IRConstant>(result, IRType::Bool);
            }
        default:
            return nullptr;
    }
}

std::shared_ptr<IRConstant> Optimizer::evaluateConstantExpression(IRUnaryOp op, 
                                                                std::shared_ptr<IRConstant> operand) {
    // Implement constant expression evaluation for unary operations
    // This is a simplified implementation
    
    switch (operand->type) {
        case IRType::Int:
            {
                int value = std::get<int>(operand->value);
                int result = 0;
                
                switch (op) {
                    case IRUnaryOp::Plus:
                        result = value;
                        break;
                    case IRUnaryOp::Minus:
                        result = -value;
                        break;
                    case IRUnaryOp::BitwiseNot:
                        result = ~value;
                        break;
                    default:
                        return nullptr;
                }
                
                return std::make_shared<IRConstant>(result, IRType::Int);
            }
        case IRType::Float:
            {
                float value = std::get<float>(operand->value);
                float result = 0.0f;
                
                switch (op) {
                    case IRUnaryOp::Plus:
                        result = value;
                        break;
                    case IRUnaryOp::Minus:
                        result = -value;
                        break;
                    default:
                        return nullptr;
                }
                
                return std::make_shared<IRConstant>(result, IRType::Float);
            }
        case IRType::Bool:
            {
                bool value = std::get<bool>(operand->value);
                bool result = false;
                
                switch (op) {
                    case IRUnaryOp::Not:
                        result = !value;
                        break;
                    default:
                        return nullptr;
                }
                
                return std::make_shared<IRConstant>(result, IRType::Bool);
            }
        default:
            return nullptr;
    }
}

std::unordered_set<std::string> Optimizer::getUsedVariables(std::shared_ptr<IRInstruction> instruction) {
    // Implement a method to get the variables used by an instruction
    // This is a simplified implementation
    
    std::unordered_set<std::string> usedVariables;
    
    // Check the type of instruction
    if (auto varDecl = std::dynamic_pointer_cast<IRVariableDeclaration>(instruction)) {
        // Variable declaration uses variables in its initializer
        if (varDecl->initializer) {
            auto vars = getUsedVariablesInValue(varDecl->initializer);
            usedVariables.insert(vars.begin(), vars.end());
        }
    } else if (auto exprStmt = std::dynamic_pointer_cast<IRExpressionStatement>(instruction)) {
        // Expression statement uses variables in its expression
        auto vars = getUsedVariablesInValue(exprStmt->expression);
        usedVariables.insert(vars.begin(), vars.end());
    } else if (auto returnStmt = std::dynamic_pointer_cast<IRReturnStatement>(instruction)) {
        // Return statement uses variables in its value
        if (returnStmt->value) {
            auto vars = getUsedVariablesInValue(returnStmt->value);
            usedVariables.insert(vars.begin(), vars.end());
        }
    } else if (auto ifStmt = std::dynamic_pointer_cast<IRIfStatement>(instruction)) {
        // If statement uses variables in its condition
        auto vars = getUsedVariablesInValue(ifStmt->condition);
        usedVariables.insert(vars.begin(), vars.end());
    } else if (auto jump = std::dynamic_pointer_cast<IRConditionalJump>(instruction)) {
        // Conditional jump uses variables in its condition
        auto vars = getUsedVariablesInValue(jump->condition);
        usedVariables.insert(vars.begin(), vars.end());
    }
    
    return usedVariables;
}

std::unordered_set<std::string> Optimizer::getUsedVariablesInValue(std::shared_ptr<IRValue> value) {
    // Implement a method to get the variables used in a value
    // This is a simplified implementation
    
    std::unordered_set<std::string> usedVariables;
    
    // Check the type of value
    if (auto var = std::dynamic_pointer_cast<IRVariable>(value)) {
        // Variable reference uses the variable
        usedVariables.insert(var->name);
    } else if (auto unaryOp = std::dynamic_pointer_cast<IRUnaryOperation>(value)) {
        // Unary operation uses variables in its operand
        auto vars = getUsedVariablesInValue(unaryOp->operand);
        usedVariables.insert(vars.begin(), vars.end());
    } else if (auto binaryOp = std::dynamic_pointer_cast<IRBinaryOperation>(value)) {
        // Binary operation uses variables in its operands
        auto leftVars = getUsedVariablesInValue(binaryOp->left);
        auto rightVars = getUsedVariablesInValue(binaryOp->right);
        usedVariables.insert(leftVars.begin(), leftVars.end());
        usedVariables.insert(rightVars.begin(), rightVars.end());
    } else if (auto call = std::dynamic_pointer_cast<IRCall>(value)) {
        // Function call uses variables in its callee and arguments
        auto calleeVars = getUsedVariablesInValue(call->callee);
        usedVariables.insert(calleeVars.begin(), calleeVars.end());
        
        for (auto& arg : call->arguments) {
            auto argVars = getUsedVariablesInValue(arg);
            usedVariables.insert(argVars.begin(), argVars.end());
        }
    } else if (auto memberAccess = std::dynamic_pointer_cast<IRMemberAccess>(value)) {
        // Member access uses variables in its object
        auto objectVars = getUsedVariablesInValue(memberAccess->object);
        usedVariables.insert(objectVars.begin(), objectVars.end());
    } else if (auto array = std::dynamic_pointer_cast<IRArray>(value)) {
        // Array literal uses variables in its elements
        for (auto& element : array->elements) {
            auto elementVars = getUsedVariablesInValue(element);
            usedVariables.insert(elementVars.begin(), elementVars.end());
        }
    }
    
    return usedVariables;
}

std::string Optimizer::getDefinedVariable(std::shared_ptr<IRInstruction> instruction) {
    // Implement a method to get the variable defined by an instruction
    // This is a simplified implementation
    
    // Check the type of instruction
    if (auto varDecl = std::dynamic_pointer_cast<IRVariableDeclaration>(instruction)) {
        // Variable declaration defines a variable
        return varDecl->name;
    }
    
    // Other instructions don't define variables in this simplified model
    return "";
}

std::string Optimizer::getExpressionKey(std::shared_ptr<IRInstruction> instruction) {
    // Implement a method to get a unique key for an expression
    // This is a simplified implementation
    
    // Check if the instruction is a variable declaration with an initializer
    if (auto varDecl = std::dynamic_pointer_cast<IRVariableDeclaration>(instruction)) {
        if (varDecl->initializer) {
            return getValueKey(varDecl->initializer);
        }
    }
    
    // Check if the instruction is an expression statement
    if (auto exprStmt = std::dynamic_pointer_cast<IRExpressionStatement>(instruction)) {
        return getValueKey(exprStmt->expression);
    }
    
    // Other instructions don't have expression keys in this simplified model
    return "";
}

std::string Optimizer::getValueKey(std::shared_ptr<IRValue> value) {
    // Implement a method to get a unique key for a value
    // This is a simplified implementation
    
    // Check the type of value
    if (auto constant = std::dynamic_pointer_cast<IRConstant>(value)) {
        // Constant key is based on its type and value
        std::string key = "const:";
        
        switch (constant->type) {
            case IRType::Int:
                key += "int:" + std::to_string(std::get<int>(constant->value));
                break;
            case IRType::Float:
                key += "float:" + std::to_string(std::get<float>(constant->value));
                break;
            case IRType::Bool:
                key += "bool:" + (std::get<bool>(constant->value) ? "true" : "false");
                break;
            case IRType::String:
                key += "string:" + std::get<std::string>(constant->value);
                break;
            default:
                return "";
        }
        
        return key;
    } else if (auto var = std::dynamic_pointer_cast<IRVariable>(value)) {
        // Variable key is based on its name
        return "var:" + var->name;
    } else if (auto unaryOp = std::dynamic_pointer_cast<IRUnaryOperation>(value)) {
        // Unary operation key is based on its operator and operand
        std::string key = "unary:";
        
        switch (unaryOp->op) {
            case IRUnaryOp::Plus:
                key += "+:";
                break;
            case IRUnaryOp::Minus:
                key += "-:";
                break;
            case IRUnaryOp::Not:
                key += "!:";
                break;
            case IRUnaryOp::BitwiseNot:
                key += "~:";
                break;
            default:
                return "";
        }
        
        key += getValueKey(unaryOp->operand);
        return key;
    } else if (auto binaryOp = std::dynamic_pointer_cast<IRBinaryOperation>(value)) {
        // Binary operation key is based on its operator and operands
        std::string key = "binary:";
        
        switch (binaryOp->op) {
            case IRBinaryOp::Add:
                key += "+:";
                break;
            case IRBinaryOp::Subtract:
                key += "-:";
                break;
            case IRBinaryOp::Multiply:
                key += "*:";
                break;
            case IRBinaryOp::Divide:
                key += "/:";
                break;
            case IRBinaryOp::Modulo:
                key += "%:";
                break;
            case IRBinaryOp::Equal:
                key += "==:";
                break;
            case IRBinaryOp::NotEqual:
                key += "!=:";
                break;
            case IRBinaryOp::Less:
                key += "<:";
                break;
            case IRBinaryOp::LessEqual:
                key += "<=:";
                break;
            case IRBinaryOp::Greater:
                key += ">:";
                break;
            case IRBinaryOp::GreaterEqual:
                key += ">=:";
                break;
            case IRBinaryOp::And:
                key += "&&:";
                break;
            case IRBinaryOp::Or:
                key += "||:";
                break;
            case IRBinaryOp::BitwiseAnd:
                key += "&:";
                break;
            case IRBinaryOp::BitwiseOr:
                key += "|:";
                break;
            case IRBinaryOp::BitwiseXor:
                key += "^:";
                break;
            default:
                return "";
        }
        
        key += getValueKey(binaryOp->left) + ":" + getValueKey(binaryOp->right);
        return key;
    }
    
    // Other values don't have keys in this simplified model
    return "";
}

std::vector<Optimizer::Loop> Optimizer::identifyLoops(std::shared_ptr<IRFunction>& function) {
    // Implement a method to identify loops in a function
    // This is a simplified implementation
    
    std::vector<Loop> loops;
    
    // Build a control flow graph
    std::unordered_map<IRBlock*, std::vector<IRBlock*>> successors;
    std::unordered_map<IRBlock*, std::vector<IRBlock*>> predecessors;
    
    for (auto& block : function->blocks) {
        // Find the successors of the block
        if (!block->instructions.empty()) {
            auto lastInst = block->instructions.back();
            
            if (auto jump = std::dynamic_pointer_cast<IRJump>(lastInst)) {
                successors[block.get()].push_back(jump->target);
                predecessors[jump->target].push_back(block.get());
            } else if (auto condJump = std::dynamic_pointer_cast<IRConditionalJump>(lastInst)) {
                successors[block.get()].push_back(condJump->thenTarget);
                successors[block.get()].push_back(condJump->elseTarget);
                predecessors[condJump->thenTarget].push_back(block.get());
                predecessors[condJump->elseTarget].push_back(block.get());
            }
        }
    }
    
    // Identify loops using a simple algorithm
    // A loop is a set of blocks where one block (the header) dominates all others
    // and there is a back edge from some block to the header
    
    // For each block
    for (auto& block : function->blocks) {
        // Check if there is a back edge to this block
        for (auto pred : predecessors[block.get()]) {
            // Check if the block dominates the predecessor
            if (dominates(block.get(), pred, predecessors)) {
                // Found a loop with header 'block'
                Loop loop;
                loop.header = block.get();
                
                // Find all blocks in the loop
                findLoopBlocks(loop.blocks, loop.header, pred, predecessors);
                
                // Find the preheader (the predecessor of the header that is not in the loop)
                for (auto p : predecessors[loop.header]) {
                    if (std::find(loop.blocks.begin(), loop.blocks.end(), p) == loop.blocks.end()) {
                        loop.preheader = p;
                        break;
                    }
                }
                
                loops.push_back(loop);
            }
        }
    }
    
    return loops;
}

bool Optimizer::dominates(IRBlock* dominator, IRBlock* block, 
                         const std::unordered_map<IRBlock*, std::vector<IRBlock*>>& predecessors) {
    // Implement a method to check if one block dominates another
    // This is a simplified implementation
    
    // A block dominates itself
    if (dominator == block) {
        return true;
    }
    
    // Check if all paths to 'block' go through 'dominator'
    std::unordered_set<IRBlock*> visited;
    std::queue<IRBlock*> queue;
    
    // Start from the predecessors of 'block'
    for (auto pred : predecessors.at(block)) {
        if (pred != dominator) {
            queue.push(pred);
            visited.insert(pred);
        }
    }
    
    // BFS to find a path to the entry block that doesn't go through 'dominator'
    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();
        
        // If we reached the entry block, then 'dominator' doesn't dominate 'block'
        if (predecessors.find(current) == predecessors.end() || predecessors.at(current).empty()) {
            return false;
        }
        
        // Add unvisited predecessors to the queue
        for (auto pred : predecessors.at(current)) {
            if (pred != dominator && visited.find(pred) == visited.end()) {
                queue.push(pred);
                visited.insert(pred);
            }
        }
    }
    
    // All paths to 'block' go through 'dominator'
    return true;
}

void Optimizer::findLoopBlocks(std::vector<IRBlock*>& blocks, IRBlock* header, IRBlock* current, 
                              const std::unordered_map<IRBlock*, std::vector<IRBlock*>>& predecessors) {
    // Implement a method to find all blocks in a loop
    // This is a simplified implementation
    
    // Add the current block to the loop
    blocks.push_back(current);
    
    // Recursively add predecessors that are not the header
    for (auto pred : predecessors.at(current)) {
        if (pred != header && std::find(blocks.begin(), blocks.end(), pred) == blocks.end()) {
            findLoopBlocks(blocks, header, pred, predecessors);
        }
    }
}

bool Optimizer::isLoopInvariant(std::shared_ptr<IRInstruction> instruction, const Loop& loop) {
    // Implement a method to check if an instruction is loop invariant
    // This is a simplified implementation
    
    // An instruction is loop invariant if it doesn't depend on any values defined in the loop
    std::unordered_set<std::string> loopDefinedVars;
    
    // Find all variables defined in the loop
    for (auto block : loop.blocks) {
        for (auto& inst : block->instructions) {
            std::string definedVar = getDefinedVariable(inst);
            if (!definedVar.empty()) {
                loopDefinedVars.insert(definedVar);
            }
        }
    }
    
    // Check if the instruction uses any loop-defined variables
    std::unordered_set<std::string> usedVars = getUsedVariables(instruction);
    
    for (const auto& var : usedVars) {
        if (loopDefinedVars.find(var) != loopDefinedVars.end()) {
            return false;
        }
    }
    
    // The instruction is loop invariant
    return true;
}

std::shared_ptr<IRFunction> Optimizer::getCalledFunction(std::shared_ptr<IRCall> call, 
                                                       std::unique_ptr<IRModule>& module) {
    // Implement a method to get the function being called
    // This is a simplified implementation
    
    // Check if the callee is a variable reference
    auto varRef = std::dynamic_pointer_cast<IRVariable>(call->callee);
    if (!varRef) {
        return nullptr;
    }
    
    // Find the function with the matching name
    for (auto& function : module->functions) {
        if (function->name == varRef->name) {
            return function;
        }
    }
    
    // Check submodules
    for (auto& submodule : module->submodules) {
        auto function = getCalledFunction(call, submodule);
        if (function) {
            return function;
        }
    }
    
    return nullptr;
}

bool Optimizer::canInline(std::shared_ptr<IRFunction> function) {
    // Implement a method to check if a function can be inlined
    // This is a simplified implementation
    
    // Don't inline functions that are too large
    size_t instructionCount = 0;
    for (auto& block : function->blocks) {
        instructionCount += block->instructions.size();
    }
    
    // Arbitrary threshold for inlining
    return instructionCount <= 20;
}

std::vector<std::shared_ptr<IRInstruction>> Optimizer::inlineFunction(std::shared_ptr<IRCall> call, 
                                                                    std::shared_ptr<IRFunction> function) {
    // Implement a method to inline a function call
    // This is a simplified implementation
    
    std::vector<std::shared_ptr<IRInstruction>> inlinedInstructions;
    
    // Create variable declarations for the parameters
    for (size_t i = 0; i < function->parameters.size() && i < call->arguments.size(); ++i) {
        auto param = function->parameters[i];
        auto arg = call->arguments[i];
        
        // Create a variable declaration for the parameter
        auto varDecl = std::make_shared<IRVariableDeclaration>(param->name, param->type, arg);
        inlinedInstructions.push_back(varDecl);
    }
    
    // Inline the function body
    // This is a simplified implementation that assumes a single block
    if (!function->blocks.empty()) {
        auto& block = function->blocks[0];
        
        // Add all instructions except the return statement
        for (size_t i = 0; i < block->instructions.size(); ++i) {
            auto& instruction = block->instructions[i];
            
            // Check if it's a return statement
            auto returnStmt = std::dynamic_pointer_cast<IRReturnStatement>(instruction);
            if (returnStmt) {
                // Replace the return with an assignment to a temporary variable
                if (returnStmt->value) {
                    // Create a temporary variable for the return value
                    std::string tempVar = "inline_return_" + std::to_string(reinterpret_cast<uintptr_t>(call.get()));
                    
                    // Create a variable declaration for the return value
                    auto varDecl = std::make_shared<IRVariableDeclaration>(tempVar, IRType::Unknown, returnStmt->value);
                    inlinedInstructions.push_back(varDecl);
                }
            } else {
                // Add the instruction to the inlined code
                inlinedInstructions.push_back(instruction);
            }
        }
    }
    
    return inlinedInstructions;
}

void Optimizer::optimizeTailCall(std::shared_ptr<IRBlock>& block, std::shared_ptr<IRCall> call, 
                                std::shared_ptr<IRFunction>& function) {
    // Implement a method to optimize a tail call
    // This is a simplified implementation
    
    // Create assignments for the parameters
    std::vector<std::shared_ptr<IRInstruction>> assignments;
    
    for (size_t i = 0; i < function->parameters.size() && i < call->arguments.size(); ++i) {
        auto param = function->parameters[i];
        auto arg = call->arguments[i];
        
        // Create a temporary variable for the argument
        std::string tempVar = "tail_call_" + param->name + "_" + std::to_string(reinterpret_cast<uintptr_t>(call.get()));
        
        // Create a variable declaration for the argument
        auto varDecl = std::make_shared<IRVariableDeclaration>(tempVar, param->type, arg);
        assignments.push_back(varDecl);
        
        // Create an assignment from the temporary to the parameter
        auto paramVar = std::make_shared<IRVariable>(param->name);
        auto tempVarRef = std::make_shared<IRVariable>(tempVar);
        auto assignment = std::make_shared<IRAssignment>(paramVar, tempVarRef);
        assignments.push_back(assignment);
    }
    
    // Create a jump to the entry block
    auto jump = std::make_shared<IRJump>(function->blocks[0].get());
    
    // Replace the return statement with the assignments and jump
    block->instructions.pop_back(); // Remove the return statement
    block->instructions.insert(block->instructions.end(), assignments.begin(), assignments.end());
    block->instructions.push_back(jump);
}

} // namespace astra