/**
 * ASTRA Programming Language Compiler
 * Abstract Syntax Tree (AST)
 */

#ifndef ASTRA_AST_H
#define ASTRA_AST_H

#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <optional>
#include "../lexer/token.h"

namespace astra {

// Forward declarations
class ASTVisitor;
class Expression;
class Statement;
class Type;

/**
 * Base class for all AST nodes
 */
class ASTNode {
public:
    virtual ~ASTNode() = default;
    virtual void accept(ASTVisitor& visitor) = 0;
};

/**
 * Base class for all expressions
 */
class Expression : public ASTNode {
public:
    virtual ~Expression() = default;
};

/**
 * Literal expression (integer, float, string, boolean)
 */
class LiteralExpression : public Expression {
public:
    Token token;
    
    LiteralExpression(const Token& t) : token(t) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Identifier expression
 */
class IdentifierExpression : public Expression {
public:
    Token token;
    std::string name;
    
    IdentifierExpression(const Token& t, const std::string& n) : token(t), name(n) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Unary expression (e.g., -x, !x)
 */
class UnaryExpression : public Expression {
public:
    Token operatorToken;
    std::shared_ptr<Expression> operand;
    
    UnaryExpression(const Token& op, std::shared_ptr<Expression> expr)
        : operatorToken(op), operand(expr) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Binary expression (e.g., a + b, x == y)
 */
class BinaryExpression : public Expression {
public:
    Token operatorToken;
    std::shared_ptr<Expression> left;
    std::shared_ptr<Expression> right;
    
    BinaryExpression(const Token& op, std::shared_ptr<Expression> l, std::shared_ptr<Expression> r)
        : operatorToken(op), left(l), right(r) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Call expression (e.g., foo(a, b))
 */
class CallExpression : public Expression {
public:
    std::shared_ptr<Expression> callee;
    std::vector<std::shared_ptr<Expression>> arguments;
    Token parenToken;
    
    CallExpression(std::shared_ptr<Expression> c, 
                  std::vector<std::shared_ptr<Expression>> args,
                  const Token& paren)
        : callee(c), arguments(args), parenToken(paren) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Member access expression (e.g., obj.field)
 */
class MemberExpression : public Expression {
public:
    std::shared_ptr<Expression> object;
    Token dotToken;
    std::shared_ptr<IdentifierExpression> property;
    
    MemberExpression(std::shared_ptr<Expression> obj, 
                    const Token& dot,
                    std::shared_ptr<IdentifierExpression> prop)
        : object(obj), dotToken(dot), property(prop) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Array literal expression (e.g., [1, 2, 3])
 */
class ArrayExpression : public Expression {
public:
    Token bracketToken;
    std::vector<std::shared_ptr<Expression>> elements;
    
    ArrayExpression(const Token& bracket, std::vector<std::shared_ptr<Expression>> elems)
        : bracketToken(bracket), elements(elems) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Base class for all statements
 */
class Statement : public ASTNode {
public:
    virtual ~Statement() = default;
};

/**
 * Base class for all declarations
 */
class Declaration : public Statement {
public:
    virtual ~Declaration() = default;
};

/**
 * Expression statement (e.g., foo();)
 */
class ExpressionStatement : public Statement {
public:
    std::shared_ptr<Expression> expression;
    
    ExpressionStatement(std::shared_ptr<Expression> expr) : expression(expr) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Block statement (e.g., { stmt1; stmt2; })
 */
class BlockStatement : public Statement {
public:
    Token braceToken;
    std::vector<std::shared_ptr<Statement>> statements;
    
    BlockStatement(const Token& brace, std::vector<std::shared_ptr<Statement>> stmts)
        : braceToken(brace), statements(stmts) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Variable declaration (e.g., var x: int = 5;)
 */
class VariableDeclaration : public Statement {
public:
    Token varToken;
    std::shared_ptr<IdentifierExpression> identifier;
    std::shared_ptr<Type> type;
    std::shared_ptr<Expression> initializer;
    bool isConst;
    
    VariableDeclaration(const Token& var, 
                       std::shared_ptr<IdentifierExpression> id,
                       std::shared_ptr<Type> t,
                       std::shared_ptr<Expression> init,
                       bool constant = false)
        : varToken(var), identifier(id), type(t), initializer(init), isConst(constant) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Function declaration (e.g., func foo(a: int, b: float) -> int { ... })
 */
class FunctionDeclaration : public Declaration {
public:
    Token funcToken;
    std::shared_ptr<IdentifierExpression> identifier;
    std::vector<std::shared_ptr<VariableDeclaration>> parameters;
    std::shared_ptr<Type> returnType;
    std::shared_ptr<BlockStatement> body;
    std::vector<std::shared_ptr<Statement>> annotations;
    
    FunctionDeclaration(const Token& func,
                       std::shared_ptr<IdentifierExpression> id,
                       std::vector<std::shared_ptr<VariableDeclaration>> params,
                       std::shared_ptr<Type> ret,
                       std::shared_ptr<BlockStatement> b,
                       std::vector<std::shared_ptr<Statement>> annot = {})
        : funcToken(func), identifier(id), parameters(params), 
          returnType(ret), body(b), annotations(annot) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * If statement (e.g., if (cond) { ... } else { ... })
 */
class IfStatement : public Statement {
public:
    Token ifToken;
    std::shared_ptr<Expression> condition;
    std::shared_ptr<Statement> thenBranch;
    std::shared_ptr<Statement> elseBranch;
    
    IfStatement(const Token& if_token,
               std::shared_ptr<Expression> cond,
               std::shared_ptr<Statement> then_branch,
               std::shared_ptr<Statement> else_branch = nullptr)
        : ifToken(if_token), condition(cond), 
          thenBranch(then_branch), elseBranch(else_branch) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * While statement (e.g., while (cond) { ... })
 */
class WhileStatement : public Statement {
public:
    Token whileToken;
    std::shared_ptr<Expression> condition;
    std::shared_ptr<Statement> body;
    
    WhileStatement(const Token& while_token,
                  std::shared_ptr<Expression> cond,
                  std::shared_ptr<Statement> b)
        : whileToken(while_token), condition(cond), body(b) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * For statement (e.g., for i in 0..10 { ... })
 */
class ForStatement : public Statement {
public:
    Token forToken;
    std::shared_ptr<IdentifierExpression> variable;
    std::shared_ptr<Expression> iterable;
    std::shared_ptr<Statement> body;
    
    ForStatement(const Token& for_token,
                std::shared_ptr<IdentifierExpression> var,
                std::shared_ptr<Expression> iter,
                std::shared_ptr<Statement> b)
        : forToken(for_token), variable(var), iterable(iter), body(b) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Return statement (e.g., return expr;)
 */
class ReturnStatement : public Statement {
public:
    Token returnToken;
    std::shared_ptr<Expression> value;
    
    ReturnStatement(const Token& ret, std::shared_ptr<Expression> val = nullptr)
        : returnToken(ret), value(val) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Break statement (e.g., break;)
 */
class BreakStatement : public Statement {
public:
    Token breakToken;
    
    BreakStatement(const Token& brk) : breakToken(brk) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Continue statement (e.g., continue;)
 */
class ContinueStatement : public Statement {
public:
    Token continueToken;
    
    ContinueStatement(const Token& cont) : continueToken(cont) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Import statement (e.g., import module.name;)
 */
class ImportStatement : public Statement {
public:
    Token importToken;
    std::vector<std::string> path;
    
    ImportStatement(const Token& imp, const std::vector<std::string>& p)
        : importToken(imp), path(p) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Module declaration (e.g., module name { ... })
 */
class ModuleDeclaration : public Declaration {
public:
    Token moduleToken;
    std::shared_ptr<IdentifierExpression> identifier;
    std::vector<std::shared_ptr<Declaration>> declarations;
    
    ModuleDeclaration(const Token& mod,
                     std::shared_ptr<IdentifierExpression> id,
                     std::vector<std::shared_ptr<Declaration>> decls)
        : moduleToken(mod), identifier(id), declarations(decls) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Try-catch statement (e.g., try { ... } catch error { ... } finally { ... })
 */
class TryStatement : public Statement {
public:
    Token tryToken;
    std::shared_ptr<BlockStatement> tryBlock;
    std::vector<std::pair<std::shared_ptr<VariableDeclaration>, std::shared_ptr<BlockStatement>>> catchClauses;
    std::shared_ptr<BlockStatement> finallyBlock;
    
    TryStatement(const Token& try_token,
                std::shared_ptr<BlockStatement> try_block,
                std::vector<std::pair<std::shared_ptr<VariableDeclaration>, std::shared_ptr<BlockStatement>>> catch_clauses,
                std::shared_ptr<BlockStatement> finally_block = nullptr)
        : tryToken(try_token), tryBlock(try_block), 
          catchClauses(catch_clauses), finallyBlock(finally_block) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Throw statement (e.g., throw error;)
 */
class ThrowStatement : public Statement {
public:
    Token throwToken;
    std::shared_ptr<Expression> expression;
    
    ThrowStatement(const Token& throw_token, std::shared_ptr<Expression> expr)
        : throwToken(throw_token), expression(expr) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Task declaration (e.g., task name() { ... })
 */
class TaskDeclaration : public Declaration {
public:
    Token taskToken;
    std::shared_ptr<IdentifierExpression> identifier;
    std::vector<std::shared_ptr<VariableDeclaration>> parameters;
    std::shared_ptr<BlockStatement> body;
    std::vector<std::shared_ptr<Statement>> annotations;
    
    TaskDeclaration(const Token& task,
                   std::shared_ptr<IdentifierExpression> id,
                   std::vector<std::shared_ptr<VariableDeclaration>> params,
                   std::shared_ptr<BlockStatement> b,
                   std::vector<std::shared_ptr<Statement>> annot = {})
        : taskToken(task), identifier(id), parameters(params), 
          body(b), annotations(annot) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Annotation statement (e.g., @deadline(5ms))
 */
class AnnotationStatement : public Statement {
public:
    Token atToken;
    std::shared_ptr<IdentifierExpression> identifier;
    std::vector<std::shared_ptr<Expression>> arguments;
    
    AnnotationStatement(const Token& at,
                       std::shared_ptr<IdentifierExpression> id,
                       std::vector<std::shared_ptr<Expression>> args)
        : atToken(at), identifier(id), arguments(args) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Base class for all type nodes
 */
class Type : public ASTNode {
public:
    virtual ~Type() = default;
    virtual std::string toString() const = 0;
};

/**
 * Simple type (e.g., int, float, string)
 */
class SimpleType : public Type {
public:
    Token token;
    std::string name;
    
    SimpleType(const Token& t, const std::string& n) : token(t), name(n) {}
    
    void accept(ASTVisitor& visitor) override;
    std::string toString() const override { return name; }
};

/**
 * Array type (e.g., int[], vector3[])
 */
class ArrayType : public Type {
public:
    std::shared_ptr<Type> elementType;
    Token bracketToken;
    
    ArrayType(std::shared_ptr<Type> elem, const Token& bracket)
        : elementType(elem), bracketToken(bracket) {}
    
    void accept(ASTVisitor& visitor) override;
    std::string toString() const override { return elementType->toString() + "[]"; }
};

/**
 * Range type (e.g., float<0.0..100.0>)
 */
class RangeType : public Type {
public:
    std::shared_ptr<Type> baseType;
    std::shared_ptr<Expression> minValue;
    std::shared_ptr<Expression> maxValue;
    Token lessThanToken;
    
    RangeType(std::shared_ptr<Type> base,
             std::shared_ptr<Expression> min,
             std::shared_ptr<Expression> max,
             const Token& less_than)
        : baseType(base), minValue(min), maxValue(max), lessThanToken(less_than) {}
    
    void accept(ASTVisitor& visitor) override;
    std::string toString() const override { 
        return baseType->toString() + "<range>"; // Simplified for now
    }
};

/**
 * Function type (e.g., (int, float) -> bool)
 */
class FunctionType : public Type {
public:
    std::vector<std::shared_ptr<Type>> parameterTypes;
    std::shared_ptr<Type> returnType;
    Token arrowToken;
    
    FunctionType(std::vector<std::shared_ptr<Type>> params,
                std::shared_ptr<Type> ret,
                const Token& arrow)
        : parameterTypes(params), returnType(ret), arrowToken(arrow) {}
    
    void accept(ASTVisitor& visitor) override;
    std::string toString() const override {
        std::string result = "(";
        for (size_t i = 0; i < parameterTypes.size(); ++i) {
            if (i > 0) result += ", ";
            result += parameterTypes[i]->toString();
        }
        result += ") -> " + returnType->toString();
        return result;
    }
};



/**
 * Program node (root of the AST)
 */
class Program : public ASTNode {
public:
    std::vector<std::shared_ptr<Statement>> statements;
    
    Program(std::vector<std::shared_ptr<Statement>> stmts) : statements(stmts) {}
    
    void accept(ASTVisitor& visitor) override;
};

/**
 * Visitor interface for AST nodes
 */
class ASTVisitor {
public:
    virtual ~ASTVisitor() = default;
    
    virtual void visit(Program& node) = 0;
    virtual void visit(LiteralExpression& node) = 0;
    virtual void visit(IdentifierExpression& node) = 0;
    virtual void visit(UnaryExpression& node) = 0;
    virtual void visit(BinaryExpression& node) = 0;
    virtual void visit(CallExpression& node) = 0;
    virtual void visit(MemberExpression& node) = 0;
    virtual void visit(ArrayExpression& node) = 0;
    virtual void visit(ExpressionStatement& node) = 0;
    virtual void visit(BlockStatement& node) = 0;
    virtual void visit(VariableDeclaration& node) = 0;
    virtual void visit(FunctionDeclaration& node) = 0;
    virtual void visit(IfStatement& node) = 0;
    virtual void visit(WhileStatement& node) = 0;
    virtual void visit(ForStatement& node) = 0;
    virtual void visit(ReturnStatement& node) = 0;
    virtual void visit(BreakStatement& node) = 0;
    virtual void visit(ContinueStatement& node) = 0;
    virtual void visit(ImportStatement& node) = 0;
    virtual void visit(ModuleDeclaration& node) = 0;
    virtual void visit(TryStatement& node) = 0;
    virtual void visit(ThrowStatement& node) = 0;
    virtual void visit(TaskDeclaration& node) = 0;
    virtual void visit(AnnotationStatement& node) = 0;
    virtual void visit(SimpleType& node) = 0;
    virtual void visit(ArrayType& node) = 0;
    virtual void visit(RangeType& node) = 0;
    virtual void visit(FunctionType& node) = 0;
};

// Implementation of accept methods
inline void LiteralExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void IdentifierExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void UnaryExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void BinaryExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void CallExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void MemberExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ArrayExpression::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ExpressionStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void BlockStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void VariableDeclaration::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void FunctionDeclaration::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void IfStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void WhileStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ForStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ReturnStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void BreakStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ContinueStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ImportStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ModuleDeclaration::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void TryStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ThrowStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void TaskDeclaration::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void AnnotationStatement::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void SimpleType::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void ArrayType::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void RangeType::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void FunctionType::accept(ASTVisitor& visitor) { visitor.visit(*this); }
inline void Program::accept(ASTVisitor& visitor) { visitor.visit(*this); }

} // namespace astra

#endif // ASTRA_AST_H