#include "ast.h"
#include <sstream>

namespace astra {

// Force instantiation of virtual destructors
void forceInstantiateDestructors() {
    // This function is never called, but it forces the compiler to generate the code
    // for the virtual destructors
    
    // We can't instantiate ASTNode directly because it's abstract
    // But we can instantiate concrete derived classes
    
    // Test Expression destructor
    {
        Expression* expr = new LiteralExpression(
            Token(TokenType::IntegerLiteral, "42", "test.astra", 1, 1)
        );
        delete expr;
    }
    
    // Test Statement destructor
    {
        Statement* stmt = new ExpressionStatement(
            std::make_shared<LiteralExpression>(
                Token(TokenType::IntegerLiteral, "42", "test.astra", 1, 1)
            )
        );
        delete stmt;
    }
    
    // Test Declaration destructor
    {
        Declaration* decl = new ModuleDeclaration(
            Token(TokenType::Module, "module", "test.astra", 1, 1),
            std::make_shared<IdentifierExpression>(
                Token(TokenType::Identifier, "mymodule", "test.astra", 1, 8),
                "mymodule"
            ),
            std::vector<std::shared_ptr<Declaration>>{}
        );
        delete decl;
    }
    
    // Test Type destructor
    {
        Type* type = new SimpleType(
            Token(TokenType::Identifier, "int", "test.astra", 1, 1),
            "int"
        );
        delete type;
    }
    
    // Test ASTVisitor destructor
    class TestVisitor : public ASTVisitor {
    public:
        void visit(Program& /*node*/) override {}
        void visit(LiteralExpression& /*node*/) override {}
        void visit(IdentifierExpression& /*node*/) override {}
        void visit(UnaryExpression& /*node*/) override {}
        void visit(BinaryExpression& /*node*/) override {}
        void visit(CallExpression& /*node*/) override {}
        void visit(MemberExpression& /*node*/) override {}
        void visit(ArrayExpression& /*node*/) override {}
        void visit(ExpressionStatement& /*node*/) override {}
        void visit(BlockStatement& /*node*/) override {}
        void visit(VariableDeclaration& /*node*/) override {}
        void visit(FunctionDeclaration& /*node*/) override {}
        void visit(IfStatement& /*node*/) override {}
        void visit(WhileStatement& /*node*/) override {}
        void visit(ForStatement& /*node*/) override {}
        void visit(ReturnStatement& /*node*/) override {}
        void visit(BreakStatement& /*node*/) override {}
        void visit(ContinueStatement& /*node*/) override {}
        void visit(ImportStatement& /*node*/) override {}
        void visit(ModuleDeclaration& /*node*/) override {}
        void visit(TryStatement& /*node*/) override {}
        void visit(ThrowStatement& /*node*/) override {}
        void visit(TaskDeclaration& /*node*/) override {}
        void visit(AnnotationStatement& /*node*/) override {}
        void visit(SimpleType& /*node*/) override {}
        void visit(ArrayType& /*node*/) override {}
        void visit(RangeType& /*node*/) override {}
        void visit(FunctionType& /*node*/) override {}
    };
    
    {
        ASTVisitor* visitor = new TestVisitor();
        delete visitor;
    }
}

// Program implementation
void Program::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

// Expression implementations
void LiteralExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void IdentifierExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void UnaryExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void BinaryExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void CallExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void MemberExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ArrayExpression::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

// Statement implementations
void ExpressionStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void BlockStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void IfStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void WhileStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ForStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ReturnStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void BreakStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ContinueStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ImportStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void TryStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ThrowStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void AnnotationStatement::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

// Declaration implementations
void VariableDeclaration::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void FunctionDeclaration::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ModuleDeclaration::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void TaskDeclaration::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

// Type implementations
void SimpleType::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void ArrayType::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void RangeType::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

void FunctionType::accept(ASTVisitor& visitor) {
    visitor.visit(*this);
}

// toString implementations
std::string Program::toString() const {
    std::stringstream ss;
    ss << "Program(";
    for (size_t i = 0; i < declarations.size(); ++i) {
        ss << declarations[i]->toString();
        if (i < declarations.size() - 1) {
            ss << ", ";
        }
    }
    ss << ")";
    return ss.str();
}

std::string LiteralExpression::toString() const {
    std::stringstream ss;
    ss << "Literal(" << token.value << ")";
    return ss.str();
}

std::string IdentifierExpression::toString() const {
    std::stringstream ss;
    ss << "Identifier(" << name << ")";
    return ss.str();
}

std::string UnaryExpression::toString() const {
    std::stringstream ss;
    ss << "UnaryExpr(" << token.value << ", " << operand->toString() << ")";
    return ss.str();
}

std::string BinaryExpression::toString() const {
    std::stringstream ss;
    ss << "BinaryExpr(" << left->toString() << " " << token.value << " " << right->toString() << ")";
    return ss.str();
}

std::string CallExpression::toString() const {
    std::stringstream ss;
    ss << "Call(" << callee->toString() << ", [";
    for (size_t i = 0; i < arguments.size(); ++i) {
        ss << arguments[i]->toString();
        if (i < arguments.size() - 1) {
            ss << ", ";
        }
    }
    ss << "])";
    return ss.str();
}

std::string MemberExpression::toString() const {
    std::stringstream ss;
    ss << "Member(" << object->toString() << "." << property->toString() << ")";
    return ss.str();
}

std::string ArrayExpression::toString() const {
    std::stringstream ss;
    ss << "Array([";
    for (size_t i = 0; i < elements.size(); ++i) {
        ss << elements[i]->toString();
        if (i < elements.size() - 1) {
            ss << ", ";
        }
    }
    ss << "])";
    return ss.str();
}

std::string ExpressionStatement::toString() const {
    std::stringstream ss;
    ss << "ExprStmt(" << expression->toString() << ")";
    return ss.str();
}

std::string BlockStatement::toString() const {
    std::stringstream ss;
    ss << "Block([";
    for (size_t i = 0; i < statements.size(); ++i) {
        ss << statements[i]->toString();
        if (i < statements.size() - 1) {
            ss << ", ";
        }
    }
    ss << "])";
    return ss.str();
}

std::string VariableDeclaration::toString() const {
    std::stringstream ss;
    ss << "VarDecl(" << name << ", ";
    if (type) {
        ss << type->toString() << ", ";
    } else {
        ss << "null, ";
    }
    if (initializer) {
        ss << initializer->toString();
    } else {
        ss << "null";
    }
    ss << ")";
    return ss.str();
}

std::string FunctionDeclaration::toString() const {
    std::stringstream ss;
    ss << "FuncDecl(" << name << ", [";
    for (size_t i = 0; i < parameters.size(); ++i) {
        ss << parameters[i]->toString();
        if (i < parameters.size() - 1) {
            ss << ", ";
        }
    }
    ss << "], ";
    if (returnType) {
        ss << returnType->toString() << ", ";
    } else {
        ss << "null, ";
    }
    ss << body->toString() << ")";
    return ss.str();
}

std::string IfStatement::toString() const {
    std::stringstream ss;
    ss << "If(" << condition->toString() << ", " << thenBranch->toString();
    if (elseBranch) {
        ss << ", " << elseBranch->toString();
    }
    ss << ")";
    return ss.str();
}

std::string WhileStatement::toString() const {
    std::stringstream ss;
    ss << "While(" << condition->toString() << ", " << body->toString() << ")";
    return ss.str();
}

std::string ForStatement::toString() const {
    std::stringstream ss;
    ss << "For(";
    if (initializer) {
        ss << initializer->toString();
    } else {
        ss << "null";
    }
    ss << ", ";
    if (condition) {
        ss << condition->toString();
    } else {
        ss << "null";
    }
    ss << ", ";
    if (increment) {
        ss << increment->toString();
    } else {
        ss << "null";
    }
    ss << ", " << body->toString() << ")";
    return ss.str();
}

std::string ReturnStatement::toString() const {
    std::stringstream ss;
    ss << "Return(";
    if (value) {
        ss << value->toString();
    }
    ss << ")";
    return ss.str();
}

std::string BreakStatement::toString() const {
    return "Break()";
}

std::string ContinueStatement::toString() const {
    return "Continue()";
}

std::string ImportStatement::toString() const {
    std::stringstream ss;
    ss << "Import(" << path << ")";
    return ss.str();
}

std::string ModuleDeclaration::toString() const {
    std::stringstream ss;
    ss << "Module(" << name->toString() << ", [";
    for (size_t i = 0; i < declarations.size(); ++i) {
        ss << declarations[i]->toString();
        if (i < declarations.size() - 1) {
            ss << ", ";
        }
    }
    ss << "])";
    return ss.str();
}

std::string TryStatement::toString() const {
    std::stringstream ss;
    ss << "Try(" << tryBlock->toString() << ", [";
    for (size_t i = 0; i < catchBlocks.size(); ++i) {
        ss << "Catch(" << catchParameters[i]->toString() << ", " << catchBlocks[i]->toString() << ")";
        if (i < catchBlocks.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    if (finallyBlock) {
        ss << ", " << finallyBlock->toString();
    }
    ss << ")";
    return ss.str();
}

std::string ThrowStatement::toString() const {
    std::stringstream ss;
    ss << "Throw(" << expression->toString() << ")";
    return ss.str();
}

std::string TaskDeclaration::toString() const {
    std::stringstream ss;
    ss << "Task(" << name << ", [";
    for (size_t i = 0; i < parameters.size(); ++i) {
        ss << parameters[i]->toString();
        if (i < parameters.size() - 1) {
            ss << ", ";
        }
    }
    ss << "], ";
    if (returnType) {
        ss << returnType->toString() << ", ";
    } else {
        ss << "null, ";
    }
    ss << body->toString() << ")";
    return ss.str();
}

std::string AnnotationStatement::toString() const {
    std::stringstream ss;
    ss << "Annotation(" << name << ", [";
    for (size_t i = 0; i < arguments.size(); ++i) {
        ss << arguments[i]->toString();
        if (i < arguments.size() - 1) {
            ss << ", ";
        }
    }
    ss << "])";
    return ss.str();
}

std::string SimpleType::toString() const {
    std::stringstream ss;
    ss << "Type(" << name << ")";
    return ss.str();
}

std::string ArrayType::toString() const {
    std::stringstream ss;
    ss << "ArrayType(" << elementType->toString() << ")";
    return ss.str();
}

std::string RangeType::toString() const {
    std::stringstream ss;
    ss << "RangeType(" << elementType->toString() << ")";
    return ss.str();
}

std::string FunctionType::toString() const {
    std::stringstream ss;
    ss << "FunctionType([";
    for (size_t i = 0; i < parameterTypes.size(); ++i) {
        ss << parameterTypes[i]->toString();
        if (i < parameterTypes.size() - 1) {
            ss << ", ";
        }
    }
    ss << "], " << returnType->toString() << ")";
    return ss.str();
}

} // namespace astra