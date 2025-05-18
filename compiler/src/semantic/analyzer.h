/**
 * ASTRA Programming Language Compiler
 * Semantic analyzer
 */

#ifndef ASTRA_SEMANTIC_ANALYZER_H
#define ASTRA_SEMANTIC_ANALYZER_H

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <stack>
#include "../ast/ast.h"
#include "../utils/error_handler.h"

namespace astra {

/**
 * Symbol type
 */
enum class SymbolType {
    Variable,
    Function,
    Module,
    Type,
    Parameter,
    Task
};

/**
 * Symbol information
 */
struct Symbol {
    std::string name;
    SymbolType type;
    std::shared_ptr<Type> dataType;
    bool isConst;
    bool isDefined;
    
    // Default constructor
    Symbol() : name(""), type(SymbolType::Variable), dataType(nullptr), isConst(false), isDefined(false) {}
    
    Symbol(const std::string& n, SymbolType t, std::shared_ptr<Type> dt = nullptr,
          bool constant = false, bool defined = true)
        : name(n), type(t), dataType(dt), isConst(constant), isDefined(defined) {}
};

/**
 * Scope for symbol table
 */
class Scope {
private:
    std::unordered_map<std::string, Symbol> symbols;
    std::shared_ptr<Scope> parent;
    std::string name;
    
public:
    Scope(const std::string& n = "global", std::shared_ptr<Scope> p = nullptr)
        : parent(p), name(n) {}
    
    /**
     * Define a symbol in the current scope
     */
    bool define(const Symbol& symbol) {
        if (symbols.find(symbol.name) != symbols.end()) {
            return false; // Symbol already defined in this scope
        }
        
        symbols[symbol.name] = symbol;
        return true;
    }
    
    /**
     * Look up a symbol in this scope or parent scopes
     */
    std::pair<bool, Symbol> resolve(const std::string& name) {
        auto it = symbols.find(name);
        if (it != symbols.end()) {
            return {true, it->second};
        }
        
        if (parent) {
            return parent->resolve(name);
        }
        
        return {false, Symbol("", SymbolType::Variable)};
    }
    
    /**
     * Get parent scope
     */
    std::shared_ptr<Scope> getParent() const {
        return parent;
    }
    
    /**
     * Get scope name
     */
    const std::string& getName() const {
        return name;
    }
    
    /**
     * Get all symbols in this scope
     */
    const std::unordered_map<std::string, Symbol>& getSymbols() const {
        return symbols;
    }
};

/**
 * Semantic analyzer for ASTRA language
 */
class SemanticAnalyzer : public ASTVisitor {
private:
    ErrorHandler& errorHandler;
    std::shared_ptr<Scope> currentScope;
    std::stack<std::shared_ptr<Type>> typeStack;
    std::stack<bool> inLoopStack;
    std::stack<bool> inFunctionStack;
    std::stack<std::shared_ptr<Type>> expectedReturnTypeStack;
    
    /**
     * Enter a new scope
     */
    void enterScope(const std::string& name) {
        currentScope = std::make_shared<Scope>(name, currentScope);
    }
    
    /**
     * Exit the current scope
     */
    void exitScope() {
        if (currentScope) {
            currentScope = currentScope->getParent();
        }
    }
    
    /**
     * Check if types are compatible
     */
    bool areTypesCompatible(const std::shared_ptr<Type>& left, const std::shared_ptr<Type>& right);
    
    /**
     * Check if a type is a subtype of another
     */
    bool isSubtype(const std::shared_ptr<Type>& sub, const std::shared_ptr<Type>& super);
    
public:
    /**
     * Constructor
     */
    SemanticAnalyzer(ErrorHandler& errHandler)
        : errorHandler(errHandler), currentScope(std::make_shared<Scope>()) {}
    
    /**
     * Analyze AST
     */
    void analyze(std::shared_ptr<Program> ast) {
        ast->accept(*this);
    }
    
    // Visitor methods
    void visit(Program& node) override;
    void visit(LiteralExpression& node) override;
    void visit(IdentifierExpression& node) override;
    void visit(UnaryExpression& node) override;
    void visit(BinaryExpression& node) override;
    void visit(CallExpression& node) override;
    void visit(MemberExpression& node) override;
    void visit(ArrayExpression& node) override;
    void visit(ExpressionStatement& node) override;
    void visit(BlockStatement& node) override;
    void visit(VariableDeclaration& node) override;
    void visit(FunctionDeclaration& node) override;
    void visit(IfStatement& node) override;
    void visit(WhileStatement& node) override;
    void visit(ForStatement& node) override;
    void visit(ReturnStatement& node) override;
    void visit(BreakStatement& node) override;
    void visit(ContinueStatement& node) override;
    void visit(ImportStatement& node) override;
    void visit(ModuleDeclaration& node) override;
    void visit(TryStatement& node) override;
    void visit(ThrowStatement& node) override;
    void visit(TaskDeclaration& node) override;
    void visit(AnnotationStatement& node) override;
    void visit(SimpleType& node) override;
    void visit(ArrayType& node) override;
    void visit(RangeType& node) override;
    void visit(FunctionType& node) override;
};

} // namespace astra

#endif // ASTRA_SEMANTIC_ANALYZER_H