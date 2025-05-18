#include <gtest/gtest.h>
#include "../src/ast/ast.h"
#include <memory>

// Forward declaration of the function that forces instantiation of virtual destructors
namespace astra {
    void forceInstantiateDestructors();
}

// Test to cover virtual destructors of base classes
TEST(DestructorsTest, BaseClassDestructors) {
    // Force instantiation of virtual destructors for delete[]
    astra::forceInstantiateDestructors();
    // Test ASTNode destructor with delete
    {
        astra::ASTNode* node = new astra::Program(
            std::vector<std::shared_ptr<astra::Statement>>{}
        );
        delete node; // This will call the virtual destructor
    }
    
    // Test Expression destructor with delete
    {
        astra::Expression* expr = new astra::LiteralExpression(
            astra::Token(astra::TokenType::IntegerLiteral, "42", "test.astra", 1, 1)
        );
        delete expr; // This will call the virtual destructor
    }
    
    // Test Statement destructor with delete
    {
        astra::Statement* stmt = new astra::ExpressionStatement(
            std::make_shared<astra::LiteralExpression>(
                astra::Token(astra::TokenType::IntegerLiteral, "42", "test.astra", 1, 1)
            )
        );
        delete stmt; // This will call the virtual destructor
    }
    
    // Test Declaration destructor with delete
    {
        astra::Declaration* decl = new astra::ModuleDeclaration(
            astra::Token(astra::TokenType::Module, "module", "test.astra", 1, 1),
            std::make_shared<astra::IdentifierExpression>(
                astra::Token(astra::TokenType::Identifier, "mymodule", "test.astra", 1, 8),
                "mymodule"
            ),
            std::vector<std::shared_ptr<astra::Declaration>>{}
        );
        delete decl; // This will call the virtual destructor
    }
    
    // Test Type destructor with delete
    {
        astra::Type* type = new astra::SimpleType(
            astra::Token(astra::TokenType::Identifier, "int", "test.astra", 1, 1),
            "int"
        );
        delete type; // This will call the virtual destructor
    }
    
    // Test ASTVisitor destructor with delete
    class TestVisitor : public astra::ASTVisitor {
    public:
        void visit(astra::Program& /*node*/) override {}
        void visit(astra::LiteralExpression& /*node*/) override {}
        void visit(astra::IdentifierExpression& /*node*/) override {}
        void visit(astra::UnaryExpression& /*node*/) override {}
        void visit(astra::BinaryExpression& /*node*/) override {}
        void visit(astra::CallExpression& /*node*/) override {}
        void visit(astra::MemberExpression& /*node*/) override {}
        void visit(astra::ArrayExpression& /*node*/) override {}
        void visit(astra::ExpressionStatement& /*node*/) override {}
        void visit(astra::BlockStatement& /*node*/) override {}
        void visit(astra::VariableDeclaration& /*node*/) override {}
        void visit(astra::FunctionDeclaration& /*node*/) override {}
        void visit(astra::IfStatement& /*node*/) override {}
        void visit(astra::WhileStatement& /*node*/) override {}
        void visit(astra::ForStatement& /*node*/) override {}
        void visit(astra::ReturnStatement& /*node*/) override {}
        void visit(astra::BreakStatement& /*node*/) override {}
        void visit(astra::ContinueStatement& /*node*/) override {}
        void visit(astra::ImportStatement& /*node*/) override {}
        void visit(astra::ModuleDeclaration& /*node*/) override {}
        void visit(astra::TryStatement& /*node*/) override {}
        void visit(astra::ThrowStatement& /*node*/) override {}
        void visit(astra::TaskDeclaration& /*node*/) override {}
        void visit(astra::AnnotationStatement& /*node*/) override {}
        void visit(astra::SimpleType& /*node*/) override {}
        void visit(astra::ArrayType& /*node*/) override {}
        void visit(astra::RangeType& /*node*/) override {}
        void visit(astra::FunctionType& /*node*/) override {}
    };
    
    {
        astra::ASTVisitor* visitor = new TestVisitor();
        delete visitor; // This will call the virtual destructor
    }
    
    // This test doesn't have any assertions - it just ensures the destructors are called
    SUCCEED();
}