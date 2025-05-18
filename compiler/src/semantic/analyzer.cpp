#include "analyzer.h"

namespace astra {

SemanticAnalyzer::SemanticAnalyzer(ErrorHandler& errorHandler)
    : errorHandler(errorHandler) {}

void SemanticAnalyzer::analyze(const std::shared_ptr<Program>& program) {
    // Visit the program to perform semantic analysis
    program->accept(*this);
}

void SemanticAnalyzer::visit(Program& node) {
    // Visit all declarations in the program
    for (const auto& decl : node.declarations) {
        decl->accept(*this);
    }
}

void SemanticAnalyzer::visit(LiteralExpression& node) {
    // Nothing to do for literals
}

void SemanticAnalyzer::visit(IdentifierExpression& node) {
    // Check if the identifier is defined in the current scope
    if (!symbolTable.lookup(node.name)) {
        errorHandler.reportError("Undefined identifier: " + node.name, 
                                node.token.filename, node.token.line, node.token.column);
    }
}

void SemanticAnalyzer::visit(UnaryExpression& node) {
    // Visit the operand
    node.operand->accept(*this);
    
    // Type checking will be done in a separate pass
}

void SemanticAnalyzer::visit(BinaryExpression& node) {
    // Visit the left and right operands
    node.left->accept(*this);
    node.right->accept(*this);
    
    // Type checking will be done in a separate pass
}

void SemanticAnalyzer::visit(CallExpression& node) {
    // Visit the callee
    node.callee->accept(*this);
    
    // Visit all arguments
    for (const auto& arg : node.arguments) {
        arg->accept(*this);
    }
    
    // Function call validation will be done in a separate pass
}

void SemanticAnalyzer::visit(MemberExpression& node) {
    // Visit the object
    node.object->accept(*this);
    
    // Member access validation will be done in a separate pass
}

void SemanticAnalyzer::visit(ArrayExpression& node) {
    // Visit all elements
    for (const auto& element : node.elements) {
        element->accept(*this);
    }
}

void SemanticAnalyzer::visit(ExpressionStatement& node) {
    // Visit the expression
    node.expression->accept(*this);
}

void SemanticAnalyzer::visit(BlockStatement& node) {
    // Create a new scope
    symbolTable.enterScope();
    
    // Visit all statements in the block
    for (const auto& stmt : node.statements) {
        stmt->accept(*this);
    }
    
    // Exit the scope
    symbolTable.exitScope();
}

void SemanticAnalyzer::visit(VariableDeclaration& node) {
    // Visit the initializer if present
    if (node.initializer) {
        node.initializer->accept(*this);
    }
    
    // Visit the type if present
    if (node.type) {
        node.type->accept(*this);
    }
    
    // Add the variable to the symbol table
    if (!symbolTable.define(node.name, &node)) {
        errorHandler.reportError("Redefinition of variable: " + node.name, 
                                node.token.filename, node.token.line, node.token.column);
    }
}

void SemanticAnalyzer::visit(FunctionDeclaration& node) {
    // Add the function to the symbol table
    if (!symbolTable.define(node.name, &node)) {
        errorHandler.reportError("Redefinition of function: " + node.name, 
                                node.token.filename, node.token.line, node.token.column);
        return;
    }
    
    // Create a new scope for the function body
    symbolTable.enterScope();
    
    // Add parameters to the symbol table
    for (const auto& param : node.parameters) {
        if (param.type) {
            param.type->accept(*this);
        }
        
        if (!symbolTable.define(param.name, &param)) {
            errorHandler.reportError("Redefinition of parameter: " + param.name, 
                                    node.token.filename, node.token.line, node.token.column);
        }
    }
    
    // Visit the return type if present
    if (node.returnType) {
        node.returnType->accept(*this);
    }
    
    // Visit the function body
    if (node.body) {
        node.body->accept(*this);
    }
    
    // Exit the function scope
    symbolTable.exitScope();
}

void SemanticAnalyzer::visit(IfStatement& node) {
    // Visit the condition
    node.condition->accept(*this);
    
    // Visit the then branch
    node.thenBranch->accept(*this);
    
    // Visit the else branch if present
    if (node.elseBranch) {
        node.elseBranch->accept(*this);
    }
}

void SemanticAnalyzer::visit(WhileStatement& node) {
    // Visit the condition
    node.condition->accept(*this);
    
    // Enter a loop scope
    loopDepth++;
    
    // Visit the body
    node.body->accept(*this);
    
    // Exit the loop scope
    loopDepth--;
}

void SemanticAnalyzer::visit(ForStatement& node) {
    // Create a new scope for the for loop
    symbolTable.enterScope();
    
    // Visit the initializer if present
    if (node.initializer) {
        node.initializer->accept(*this);
    }
    
    // Visit the condition if present
    if (node.condition) {
        node.condition->accept(*this);
    }
    
    // Visit the update if present
    if (node.update) {
        node.update->accept(*this);
    }
    
    // Enter a loop scope
    loopDepth++;
    
    // Visit the body
    node.body->accept(*this);
    
    // Exit the loop scope
    loopDepth--;
    
    // Exit the for loop scope
    symbolTable.exitScope();
}

void SemanticAnalyzer::visit(ReturnStatement& node) {
    // Visit the value if present
    if (node.value) {
        node.value->accept(*this);
    }
    
    // Return statement validation will be done in a separate pass
}

void SemanticAnalyzer::visit(BreakStatement& node) {
    // Check if we're inside a loop
    if (loopDepth == 0) {
        errorHandler.reportError("Break statement outside of loop", 
                                node.token.filename, node.token.line, node.token.column);
    }
}

void SemanticAnalyzer::visit(ContinueStatement& node) {
    // Check if we're inside a loop
    if (loopDepth == 0) {
        errorHandler.reportError("Continue statement outside of loop", 
                                node.token.filename, node.token.line, node.token.column);
    }
}

void SemanticAnalyzer::visit(ImportStatement& node) {
    // Import statement handling will be done in a separate pass
}

void SemanticAnalyzer::visit(ModuleDeclaration& node) {
    // Visit the module name
    node.name->accept(*this);
    
    // Create a new scope for the module
    symbolTable.enterScope();
    
    // Visit all declarations in the module
    for (const auto& decl : node.declarations) {
        decl->accept(*this);
    }
    
    // Exit the module scope
    symbolTable.exitScope();
}

void SemanticAnalyzer::visit(TryStatement& node) {
    // Visit the try block
    node.tryBlock->accept(*this);
    
    // Visit all catch clauses
    for (const auto& catchClause : node.catchClauses) {
        // Create a new scope for the catch clause
        symbolTable.enterScope();
        
        // Add the exception variable to the symbol table
        if (!symbolTable.define(catchClause.exceptionVar, nullptr)) {
            errorHandler.reportError("Redefinition of exception variable: " + catchClause.exceptionVar, 
                                    node.token.filename, node.token.line, node.token.column);
        }
        
        // Visit the catch block
        catchClause.catchBlock->accept(*this);
        
        // Exit the catch clause scope
        symbolTable.exitScope();
    }
    
    // Visit the finally block if present
    if (node.finallyBlock) {
        node.finallyBlock->accept(*this);
    }
}

void SemanticAnalyzer::visit(ThrowStatement& node) {
    // Visit the expression
    node.expression->accept(*this);
}

void SemanticAnalyzer::visit(TaskDeclaration& node) {
    // Similar to function declaration
    if (!symbolTable.define(node.name, &node)) {
        errorHandler.reportError("Redefinition of task: " + node.name, 
                                node.token.filename, node.token.line, node.token.column);
        return;
    }
    
    // Create a new scope for the task body
    symbolTable.enterScope();
    
    // Add parameters to the symbol table
    for (const auto& param : node.parameters) {
        if (param.type) {
            param.type->accept(*this);
        }
        
        if (!symbolTable.define(param.name, &param)) {
            errorHandler.reportError("Redefinition of parameter: " + param.name, 
                                    node.token.filename, node.token.line, node.token.column);
        }
    }
    
    // Visit the return type if present
    if (node.returnType) {
        node.returnType->accept(*this);
    }
    
    // Visit the task body
    if (node.body) {
        node.body->accept(*this);
    }
    
    // Exit the task scope
    symbolTable.exitScope();
}

void SemanticAnalyzer::visit(AnnotationStatement& node) {
    // Visit the target
    node.target->accept(*this);
    
    // Visit all arguments
    for (const auto& arg : node.arguments) {
        arg->accept(*this);
    }
}

void SemanticAnalyzer::visit(SimpleType& node) {
    // Nothing to do for simple types
}

void SemanticAnalyzer::visit(ArrayType& node) {
    // Visit the element type
    node.elementType->accept(*this);
    
    // Visit the size expression if present
    if (node.size) {
        node.size->accept(*this);
    }
}

void SemanticAnalyzer::visit(RangeType& node) {
    // Visit the element type
    node.elementType->accept(*this);
    
    // Visit the min and max expressions if present
    if (node.min) {
        node.min->accept(*this);
    }
    
    if (node.max) {
        node.max->accept(*this);
    }
}

void SemanticAnalyzer::visit(FunctionType& node) {
    // Visit the return type
    node.returnType->accept(*this);
    
    // Visit all parameter types
    for (const auto& paramType : node.parameterTypes) {
        paramType->accept(*this);
    }
}

bool SymbolTable::define(const std::string& name, void* symbol) {
    // Check if the symbol already exists in the current scope
    if (scopes.back().find(name) != scopes.back().end()) {
        return false;
    }
    
    // Add the symbol to the current scope
    scopes.back()[name] = symbol;
    return true;
}

void* SymbolTable::lookup(const std::string& name) {
    // Search for the symbol in all scopes, starting from the innermost
    for (auto it = scopes.rbegin(); it != scopes.rend(); ++it) {
        auto symbolIt = it->find(name);
        if (symbolIt != it->end()) {
            return symbolIt->second;
        }
    }
    
    // Symbol not found
    return nullptr;
}

void SymbolTable::enterScope() {
    scopes.push_back({});
}

void SymbolTable::exitScope() {
    if (!scopes.empty()) {
        scopes.pop_back();
    }
}

} // namespace astra