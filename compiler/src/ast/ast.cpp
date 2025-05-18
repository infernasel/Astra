#include "ast.h"

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

} // namespace astra