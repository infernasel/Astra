#include "generator.h"

namespace astra {

IRGenerator::IRGenerator(ErrorHandler& errorHandler)
    : errorHandler(errorHandler) {}

std::unique_ptr<IRModule> IRGenerator::generate(const std::shared_ptr<Program>& program) {
    auto irModule = std::make_unique<IRModule>();
    
    // Visit the program to generate IR
    currentModule = irModule.get();
    program->accept(*this);
    
    return irModule;
}

void IRGenerator::visit(Program& node) {
    // Visit all declarations in the program
    for (const auto& decl : node.declarations) {
        decl->accept(*this);
    }
}

void IRGenerator::visit(LiteralExpression& node) {
    // Generate IR for literals
    switch (node.token.type) {
        case TokenType::IntegerLiteral:
            currentValue = std::make_shared<IRConstant>(std::stoi(node.token.lexeme), IRType::Int);
            break;
        case TokenType::FloatLiteral:
            currentValue = std::make_shared<IRConstant>(std::stof(node.token.lexeme), IRType::Float);
            break;
        case TokenType::BooleanLiteral:
            currentValue = std::make_shared<IRConstant>(node.token.lexeme == "true", IRType::Bool);
            break;
        case TokenType::StringLiteral:
            currentValue = std::make_shared<IRConstant>(node.token.lexeme, IRType::String);
            break;
        default:
            errorHandler.reportError("Unsupported literal type", 
                                    node.token.filename, node.token.line, node.token.column);
            currentValue = nullptr;
            break;
    }
}

void IRGenerator::visit(IdentifierExpression& node) {
    // Look up the identifier in the symbol table
    auto symbol = symbolTable.lookup(node.name);
    if (!symbol) {
        errorHandler.reportError("Undefined identifier: " + node.name, 
                                node.token.filename, node.token.line, node.token.column);
        currentValue = nullptr;
        return;
    }
    
    // Create a reference to the variable
    currentValue = std::make_shared<IRVariable>(node.name);
}

void IRGenerator::visit(UnaryExpression& node) {
    // Visit the operand
    node.operand->accept(*this);
    auto operand = currentValue;
    
    if (!operand) {
        currentValue = nullptr;
        return;
    }
    
    // Generate IR for the unary operation
    IRUnaryOp op;
    switch (node.op) {
        case TokenType::Plus:
            op = IRUnaryOp::Plus;
            break;
        case TokenType::Minus:
            op = IRUnaryOp::Minus;
            break;
        case TokenType::Not:
            op = IRUnaryOp::Not;
            break;
        case TokenType::BitwiseNot:
            op = IRUnaryOp::BitwiseNot;
            break;
        default:
            errorHandler.reportError("Unsupported unary operator", 
                                    node.token.filename, node.token.line, node.token.column);
            currentValue = nullptr;
            return;
    }
    
    currentValue = std::make_shared<IRUnaryOperation>(op, operand);
}

void IRGenerator::visit(BinaryExpression& node) {
    // Visit the left operand
    node.left->accept(*this);
    auto left = currentValue;
    
    // Visit the right operand
    node.right->accept(*this);
    auto right = currentValue;
    
    if (!left || !right) {
        currentValue = nullptr;
        return;
    }
    
    // Generate IR for the binary operation
    IRBinaryOp op;
    switch (node.op) {
        case TokenType::Plus:
            op = IRBinaryOp::Add;
            break;
        case TokenType::Minus:
            op = IRBinaryOp::Subtract;
            break;
        case TokenType::Multiply:
            op = IRBinaryOp::Multiply;
            break;
        case TokenType::Divide:
            op = IRBinaryOp::Divide;
            break;
        case TokenType::Modulo:
            op = IRBinaryOp::Modulo;
            break;
        case TokenType::Equal:
            op = IRBinaryOp::Equal;
            break;
        case TokenType::NotEqual:
            op = IRBinaryOp::NotEqual;
            break;
        case TokenType::Less:
            op = IRBinaryOp::Less;
            break;
        case TokenType::LessEqual:
            op = IRBinaryOp::LessEqual;
            break;
        case TokenType::Greater:
            op = IRBinaryOp::Greater;
            break;
        case TokenType::GreaterEqual:
            op = IRBinaryOp::GreaterEqual;
            break;
        case TokenType::And:
            op = IRBinaryOp::And;
            break;
        case TokenType::Or:
            op = IRBinaryOp::Or;
            break;
        case TokenType::BitwiseAnd:
            op = IRBinaryOp::BitwiseAnd;
            break;
        case TokenType::BitwiseOr:
            op = IRBinaryOp::BitwiseOr;
            break;
        case TokenType::BitwiseXor:
            op = IRBinaryOp::BitwiseXor;
            break;
        default:
            errorHandler.reportError("Unsupported binary operator", 
                                    node.token.filename, node.token.line, node.token.column);
            currentValue = nullptr;
            return;
    }
    
    currentValue = std::make_shared<IRBinaryOperation>(op, left, right);
}

void IRGenerator::visit(CallExpression& node) {
    // Visit the callee
    node.callee->accept(*this);
    auto callee = currentValue;
    
    if (!callee) {
        currentValue = nullptr;
        return;
    }
    
    // Visit all arguments
    std::vector<std::shared_ptr<IRValue>> arguments;
    for (const auto& arg : node.arguments) {
        arg->accept(*this);
        if (currentValue) {
            arguments.push_back(currentValue);
        }
    }
    
    // Generate IR for the function call
    currentValue = std::make_shared<IRCall>(callee, arguments);
}

void IRGenerator::visit(MemberExpression& node) {
    // Visit the object
    node.object->accept(*this);
    auto object = currentValue;
    
    if (!object) {
        currentValue = nullptr;
        return;
    }
    
    // Generate IR for the member access
    currentValue = std::make_shared<IRMemberAccess>(object, node.member);
}

void IRGenerator::visit(ArrayExpression& node) {
    // Visit all elements
    std::vector<std::shared_ptr<IRValue>> elements;
    for (const auto& element : node.elements) {
        element->accept(*this);
        if (currentValue) {
            elements.push_back(currentValue);
        }
    }
    
    // Generate IR for the array literal
    currentValue = std::make_shared<IRArray>(elements);
}

void IRGenerator::visit(ExpressionStatement& node) {
    // Visit the expression
    node.expression->accept(*this);
    
    // Add the expression to the current block
    if (currentValue && currentBlock) {
        currentBlock->instructions.push_back(std::make_shared<IRExpressionStatement>(currentValue));
    }
}

void IRGenerator::visit(BlockStatement& node) {
    // Create a new block
    auto block = std::make_shared<IRBlock>();
    auto previousBlock = currentBlock;
    currentBlock = block.get();
    
    // Visit all statements in the block
    for (const auto& stmt : node.statements) {
        stmt->accept(*this);
    }
    
    // Restore the previous block
    currentBlock = previousBlock;
    
    // Add the block to the current function
    if (currentFunction) {
        currentFunction->blocks.push_back(block);
    }
}

void IRGenerator::visit(VariableDeclaration& node) {
    // Visit the initializer if present
    std::shared_ptr<IRValue> initializer = nullptr;
    if (node.initializer) {
        node.initializer->accept(*this);
        initializer = currentValue;
    }
    
    // Determine the variable type
    IRType type = IRType::Unknown;
    if (node.type) {
        // Visit the type to determine the IR type
        node.type->accept(*this);
        // The type information would be stored in a separate field
    }
    
    // Create a variable declaration
    auto varDecl = std::make_shared<IRVariableDeclaration>(node.name, type, initializer);
    
    // Add the variable to the symbol table
    symbolTable.define(node.name, varDecl.get());
    
    // Add the declaration to the current block
    if (currentBlock) {
        currentBlock->instructions.push_back(varDecl);
    }
}

void IRGenerator::visit(FunctionDeclaration& node) {
    // Create a new function
    auto function = std::make_shared<IRFunction>(node.name);
    auto previousFunction = currentFunction;
    currentFunction = function.get();
    
    // Create a new scope for the function
    symbolTable.enterScope();
    
    // Add parameters to the function
    for (const auto& param : node.parameters) {
        IRType paramType = IRType::Unknown;
        if (param.type) {
            // Visit the type to determine the IR type
            param.type->accept(*this);
            // The type information would be stored in a separate field
        }
        
        auto parameter = std::make_shared<IRParameter>(param.name, paramType);
        function->parameters.push_back(parameter);
        
        // Add the parameter to the symbol table
        symbolTable.define(param.name, parameter.get());
    }
    
    // Determine the return type
    IRType returnType = IRType::Void;
    if (node.returnType) {
        // Visit the return type to determine the IR type
        node.returnType->accept(*this);
        // The type information would be stored in a separate field
    }
    function->returnType = returnType;
    
    // Create the entry block
    auto entryBlock = std::make_shared<IRBlock>();
    currentBlock = entryBlock.get();
    function->blocks.push_back(entryBlock);
    
    // Visit the function body
    if (node.body) {
        node.body->accept(*this);
    }
    
    // Exit the function scope
    symbolTable.exitScope();
    
    // Restore the previous function
    currentFunction = previousFunction;
    
    // Add the function to the module
    if (currentModule) {
        currentModule->functions.push_back(function);
    }
}

void IRGenerator::visit(IfStatement& node) {
    // Visit the condition
    node.condition->accept(*this);
    auto condition = currentValue;
    
    if (!condition) {
        return;
    }
    
    // Create blocks for then and else branches
    auto thenBlock = std::make_shared<IRBlock>();
    auto elseBlock = std::make_shared<IRBlock>();
    auto mergeBlock = std::make_shared<IRBlock>();
    
    // Generate IR for the if statement
    auto ifStmt = std::make_shared<IRIfStatement>(condition, thenBlock, elseBlock);
    
    // Add the if statement to the current block
    if (currentBlock) {
        currentBlock->instructions.push_back(ifStmt);
    }
    
    // Visit the then branch
    auto previousBlock = currentBlock;
    currentBlock = thenBlock.get();
    node.thenBranch->accept(*this);
    
    // Add a jump to the merge block
    if (!thenBlock->instructions.empty() && 
        !std::dynamic_pointer_cast<IRReturnStatement>(thenBlock->instructions.back())) {
        thenBlock->instructions.push_back(std::make_shared<IRJump>(mergeBlock));
    }
    
    // Visit the else branch if present
    currentBlock = elseBlock.get();
    if (node.elseBranch) {
        node.elseBranch->accept(*this);
    }
    
    // Add a jump to the merge block
    if (!elseBlock->instructions.empty() && 
        !std::dynamic_pointer_cast<IRReturnStatement>(elseBlock->instructions.back())) {
        elseBlock->instructions.push_back(std::make_shared<IRJump>(mergeBlock));
    }
    
    // Restore the current block
    currentBlock = previousBlock;
    
    // Add the blocks to the current function
    if (currentFunction) {
        currentFunction->blocks.push_back(thenBlock);
        currentFunction->blocks.push_back(elseBlock);
        currentFunction->blocks.push_back(mergeBlock);
    }
}

void IRGenerator::visit(WhileStatement& node) {
    // Create blocks for the loop
    auto conditionBlock = std::make_shared<IRBlock>();
    auto bodyBlock = std::make_shared<IRBlock>();
    auto exitBlock = std::make_shared<IRBlock>();
    
    // Add a jump to the condition block
    if (currentBlock) {
        currentBlock->instructions.push_back(std::make_shared<IRJump>(conditionBlock));
    }
    
    // Visit the condition
    auto previousBlock = currentBlock;
    currentBlock = conditionBlock.get();
    node.condition->accept(*this);
    auto condition = currentValue;
    
    if (!condition) {
        currentBlock = previousBlock;
        return;
    }
    
    // Add the conditional jump
    currentBlock->instructions.push_back(std::make_shared<IRConditionalJump>(condition, bodyBlock, exitBlock));
    
    // Visit the body
    currentBlock = bodyBlock.get();
    
    // Save the current loop information
    auto previousContinueTarget = currentContinueTarget;
    auto previousBreakTarget = currentBreakTarget;
    currentContinueTarget = conditionBlock;
    currentBreakTarget = exitBlock;
    
    node.body->accept(*this);
    
    // Add a jump back to the condition block
    currentBlock->instructions.push_back(std::make_shared<IRJump>(conditionBlock));
    
    // Restore the loop information
    currentContinueTarget = previousContinueTarget;
    currentBreakTarget = previousBreakTarget;
    
    // Set the current block to the exit block
    currentBlock = exitBlock.get();
    
    // Add the blocks to the current function
    if (currentFunction) {
        currentFunction->blocks.push_back(conditionBlock);
        currentFunction->blocks.push_back(bodyBlock);
        currentFunction->blocks.push_back(exitBlock);
    }
}

void IRGenerator::visit(ForStatement& node) {
    // Create a new scope for the for loop
    symbolTable.enterScope();
    
    // Create blocks for the loop
    auto initBlock = std::make_shared<IRBlock>();
    auto conditionBlock = std::make_shared<IRBlock>();
    auto updateBlock = std::make_shared<IRBlock>();
    auto bodyBlock = std::make_shared<IRBlock>();
    auto exitBlock = std::make_shared<IRBlock>();
    
    // Add a jump to the init block
    if (currentBlock) {
        currentBlock->instructions.push_back(std::make_shared<IRJump>(initBlock));
    }
    
    // Visit the initializer
    auto previousBlock = currentBlock;
    currentBlock = initBlock.get();
    if (node.initializer) {
        node.initializer->accept(*this);
    }
    
    // Add a jump to the condition block
    currentBlock->instructions.push_back(std::make_shared<IRJump>(conditionBlock));
    
    // Visit the condition
    currentBlock = conditionBlock.get();
    std::shared_ptr<IRValue> condition = nullptr;
    if (node.condition) {
        node.condition->accept(*this);
        condition = currentValue;
    } else {
        // If no condition is provided, use true
        condition = std::make_shared<IRConstant>(true, IRType::Bool);
    }
    
    // Add the conditional jump
    currentBlock->instructions.push_back(std::make_shared<IRConditionalJump>(condition, bodyBlock, exitBlock));
    
    // Visit the body
    currentBlock = bodyBlock.get();
    
    // Save the current loop information
    auto previousContinueTarget = currentContinueTarget;
    auto previousBreakTarget = currentBreakTarget;
    currentContinueTarget = updateBlock;
    currentBreakTarget = exitBlock;
    
    node.body->accept(*this);
    
    // Add a jump to the update block
    currentBlock->instructions.push_back(std::make_shared<IRJump>(updateBlock));
    
    // Visit the update
    currentBlock = updateBlock.get();
    if (node.update) {
        node.update->accept(*this);
    }
    
    // Add a jump back to the condition block
    currentBlock->instructions.push_back(std::make_shared<IRJump>(conditionBlock));
    
    // Restore the loop information
    currentContinueTarget = previousContinueTarget;
    currentBreakTarget = previousBreakTarget;
    
    // Set the current block to the exit block
    currentBlock = exitBlock.get();
    
    // Add the blocks to the current function
    if (currentFunction) {
        currentFunction->blocks.push_back(initBlock);
        currentFunction->blocks.push_back(conditionBlock);
        currentFunction->blocks.push_back(bodyBlock);
        currentFunction->blocks.push_back(updateBlock);
        currentFunction->blocks.push_back(exitBlock);
    }
    
    // Exit the for loop scope
    symbolTable.exitScope();
}

void IRGenerator::visit(ReturnStatement& node) {
    // Visit the return value if present
    std::shared_ptr<IRValue> returnValue = nullptr;
    if (node.value) {
        node.value->accept(*this);
        returnValue = currentValue;
    }
    
    // Create a return statement
    auto returnStmt = std::make_shared<IRReturnStatement>(returnValue);
    
    // Add the return statement to the current block
    if (currentBlock) {
        currentBlock->instructions.push_back(returnStmt);
    }
}

void IRGenerator::visit(BreakStatement& node) {
    // Check if we're inside a loop
    if (!currentBreakTarget) {
        errorHandler.reportError("Break statement outside of loop", 
                                node.token.filename, node.token.line, node.token.column);
        return;
    }
    
    // Add a jump to the break target
    if (currentBlock) {
        currentBlock->instructions.push_back(std::make_shared<IRJump>(currentBreakTarget));
    }
}

void IRGenerator::visit(ContinueStatement& node) {
    // Check if we're inside a loop
    if (!currentContinueTarget) {
        errorHandler.reportError("Continue statement outside of loop", 
                                node.token.filename, node.token.line, node.token.column);
        return;
    }
    
    // Add a jump to the continue target
    if (currentBlock) {
        currentBlock->instructions.push_back(std::make_shared<IRJump>(currentContinueTarget));
    }
}

void IRGenerator::visit(ImportStatement& node) {
    // Import handling will be done in a separate pass
}

void IRGenerator::visit(ModuleDeclaration& node) {
    // Create a new module
    auto module = std::make_shared<IRModule>();
    auto previousModule = currentModule;
    currentModule = module.get();
    
    // Create a new scope for the module
    symbolTable.enterScope();
    
    // Visit all declarations in the module
    for (const auto& decl : node.declarations) {
        decl->accept(*this);
    }
    
    // Exit the module scope
    symbolTable.exitScope();
    
    // Restore the previous module
    currentModule = previousModule;
    
    // Add the module to the parent module
    if (currentModule) {
        currentModule->submodules.push_back(module);
    }
}

void IRGenerator::visit(TryStatement& node) {
    // Try-catch handling will be implemented in a future version
}

void IRGenerator::visit(ThrowStatement& node) {
    // Exception handling will be implemented in a future version
}

void IRGenerator::visit(TaskDeclaration& node) {
    // Task handling will be implemented in a future version
}

void IRGenerator::visit(AnnotationStatement& node) {
    // Annotation handling will be implemented in a future version
}

void IRGenerator::visit(SimpleType& node) {
    // Type handling will be implemented in a future version
}

void IRGenerator::visit(ArrayType& node) {
    // Type handling will be implemented in a future version
}

void IRGenerator::visit(RangeType& node) {
    // Type handling will be implemented in a future version
}

void IRGenerator::visit(FunctionType& node) {
    // Type handling will be implemented in a future version
}

} // namespace astra