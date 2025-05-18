#include <gtest/gtest.h>
#include "../src/ast/ast.h"
#include "../src/lexer/token.h"
#include <memory>
#include <string>

// Mock implementation of ASTVisitor for testing
class MockASTVisitor : public astra::ASTVisitor {
public:
    std::string lastVisited;

    void visit(astra::Program& /*node*/) override { lastVisited = "Program"; }
    void visit(astra::LiteralExpression& /*node*/) override { lastVisited = "LiteralExpression"; }
    void visit(astra::IdentifierExpression& /*node*/) override { lastVisited = "IdentifierExpression"; }
    void visit(astra::UnaryExpression& /*node*/) override { lastVisited = "UnaryExpression"; }
    void visit(astra::BinaryExpression& /*node*/) override { lastVisited = "BinaryExpression"; }
    void visit(astra::CallExpression& /*node*/) override { lastVisited = "CallExpression"; }
    void visit(astra::MemberExpression& /*node*/) override { lastVisited = "MemberExpression"; }
    void visit(astra::ArrayExpression& /*node*/) override { lastVisited = "ArrayExpression"; }
    void visit(astra::ExpressionStatement& /*node*/) override { lastVisited = "ExpressionStatement"; }
    void visit(astra::BlockStatement& /*node*/) override { lastVisited = "BlockStatement"; }
    void visit(astra::VariableDeclaration& /*node*/) override { lastVisited = "VariableDeclaration"; }
    void visit(astra::FunctionDeclaration& /*node*/) override { lastVisited = "FunctionDeclaration"; }
    void visit(astra::IfStatement& /*node*/) override { lastVisited = "IfStatement"; }
    void visit(astra::WhileStatement& /*node*/) override { lastVisited = "WhileStatement"; }
    void visit(astra::ForStatement& /*node*/) override { lastVisited = "ForStatement"; }
    void visit(astra::ReturnStatement& /*node*/) override { lastVisited = "ReturnStatement"; }
    void visit(astra::BreakStatement& /*node*/) override { lastVisited = "BreakStatement"; }
    void visit(astra::ContinueStatement& /*node*/) override { lastVisited = "ContinueStatement"; }
    void visit(astra::ImportStatement& /*node*/) override { lastVisited = "ImportStatement"; }
    void visit(astra::ModuleDeclaration& /*node*/) override { lastVisited = "ModuleDeclaration"; }
    void visit(astra::TryStatement& /*node*/) override { lastVisited = "TryStatement"; }
    void visit(astra::ThrowStatement& /*node*/) override { lastVisited = "ThrowStatement"; }
    void visit(astra::TaskDeclaration& /*node*/) override { lastVisited = "TaskDeclaration"; }
    void visit(astra::AnnotationStatement& /*node*/) override { lastVisited = "AnnotationStatement"; }
    void visit(astra::SimpleType& /*node*/) override { lastVisited = "SimpleType"; }
    void visit(astra::ArrayType& /*node*/) override { lastVisited = "ArrayType"; }
    void visit(astra::RangeType& /*node*/) override { lastVisited = "RangeType"; }
    void visit(astra::FunctionType& /*node*/) override { lastVisited = "FunctionType"; }
};

// Helper function to create a token for testing
astra::Token createToken(astra::TokenType type, const std::string& lexeme = "") {
    return astra::Token(type, lexeme, "test.astra", 1, 1);
}

TEST(ASTTest, LiteralExpression) {
    astra::Token token = createToken(astra::TokenType::IntegerLiteral, "42");
    astra::LiteralExpression expr(token);
    
    EXPECT_EQ(token.lexeme, expr.token.lexeme);
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("LiteralExpression", visitor.lastVisited);
}

TEST(ASTTest, IdentifierExpression) {
    astra::Token token = createToken(astra::TokenType::Identifier, "variable");
    astra::IdentifierExpression expr(token, "variable");
    
    EXPECT_EQ("variable", expr.name);
    EXPECT_EQ(token.lexeme, expr.token.lexeme);
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("IdentifierExpression", visitor.lastVisited);
}

TEST(ASTTest, UnaryExpression) {
    astra::Token opToken = createToken(astra::TokenType::Minus, "-");
    auto operand = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "42"));
    
    astra::UnaryExpression expr(opToken, operand);
    
    EXPECT_EQ(opToken.lexeme, expr.operatorToken.lexeme);
    EXPECT_EQ(operand, expr.operand);
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("UnaryExpression", visitor.lastVisited);
}

TEST(ASTTest, BinaryExpression) {
    astra::Token opToken = createToken(astra::TokenType::Plus, "+");
    auto left = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "5"));
    auto right = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "10"));
    
    astra::BinaryExpression expr(opToken, left, right);
    
    EXPECT_EQ(opToken.lexeme, expr.operatorToken.lexeme);
    EXPECT_EQ(left, expr.left);
    EXPECT_EQ(right, expr.right);
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("BinaryExpression", visitor.lastVisited);
}

TEST(ASTTest, CallExpression) {
    auto callee = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "func"), "func");
    
    std::vector<std::shared_ptr<astra::Expression>> args;
    args.push_back(std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "42")));
    
    astra::Token parenToken = createToken(astra::TokenType::LeftParen, "(");
    
    astra::CallExpression expr(callee, args, parenToken);
    
    EXPECT_EQ(callee, expr.callee);
    EXPECT_EQ(1, expr.arguments.size());
    EXPECT_EQ(parenToken.lexeme, expr.parenToken.lexeme);
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("CallExpression", visitor.lastVisited);
}

TEST(ASTTest, MemberExpression) {
    auto object = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "obj"), "obj");
    
    astra::Token dotToken = createToken(astra::TokenType::Dot, ".");
    
    auto property = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "field"), "field");
    
    astra::MemberExpression expr(object, dotToken, property);
    
    EXPECT_EQ(object, expr.object);
    EXPECT_EQ(dotToken.lexeme, expr.dotToken.lexeme);
    EXPECT_EQ(property, expr.property);
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("MemberExpression", visitor.lastVisited);
}

TEST(ASTTest, ArrayExpression) {
    astra::Token bracketToken = createToken(astra::TokenType::LeftBracket, "[");
    
    std::vector<std::shared_ptr<astra::Expression>> elements;
    elements.push_back(std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1")));
    elements.push_back(std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "2")));
    
    astra::ArrayExpression expr(bracketToken, elements);
    
    EXPECT_EQ(bracketToken.lexeme, expr.bracketToken.lexeme);
    EXPECT_EQ(2, expr.elements.size());
    
    MockASTVisitor visitor;
    expr.accept(visitor);
    EXPECT_EQ("ArrayExpression", visitor.lastVisited);
}

TEST(ASTTest, ExpressionStatement) {
    auto expression = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "42"));
    
    astra::ExpressionStatement stmt(expression);
    
    EXPECT_EQ(expression, stmt.expression);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("ExpressionStatement", visitor.lastVisited);
}

TEST(ASTTest, BlockStatement) {
    astra::Token braceToken = createToken(astra::TokenType::LeftBrace, "{");
    
    std::vector<std::shared_ptr<astra::Statement>> statements;
    auto expr1 = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto expr2 = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "2"));
    
    statements.push_back(std::make_shared<astra::ExpressionStatement>(expr1));
    statements.push_back(std::make_shared<astra::ExpressionStatement>(expr2));
    
    astra::BlockStatement stmt(braceToken, statements);
    
    EXPECT_EQ(braceToken.lexeme, stmt.braceToken.lexeme);
    EXPECT_EQ(2, stmt.statements.size());
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("BlockStatement", visitor.lastVisited);
}

TEST(ASTTest, VariableDeclaration) {
    astra::Token varToken = createToken(astra::TokenType::Var, "var");
    auto identifier = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "x"), "x");
    
    auto type = std::make_shared<astra::SimpleType>(
        createToken(astra::TokenType::Identifier, "int"), "int");
    
    auto initializer = std::make_shared<astra::LiteralExpression>(
        createToken(astra::TokenType::IntegerLiteral, "42"));
    
    astra::VariableDeclaration decl(varToken, identifier, type, initializer, false);
    
    EXPECT_EQ(varToken.lexeme, decl.varToken.lexeme);
    EXPECT_EQ(identifier, decl.identifier);
    EXPECT_EQ(type, decl.type);
    EXPECT_EQ(initializer, decl.initializer);
    EXPECT_FALSE(decl.isConst);
    
    MockASTVisitor visitor;
    decl.accept(visitor);
    EXPECT_EQ("VariableDeclaration", visitor.lastVisited);
}

TEST(ASTTest, FunctionDeclaration) {
    astra::Token funcToken = createToken(astra::TokenType::Func, "func");
    auto identifier = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "add"), "add");
    
    std::vector<std::shared_ptr<astra::VariableDeclaration>> parameters;
    
    auto param1Id = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "a"), "a");
    auto param1Type = std::make_shared<astra::SimpleType>(
        createToken(astra::TokenType::Identifier, "int"), "int");
    auto param1 = std::make_shared<astra::VariableDeclaration>(
        createToken(astra::TokenType::Var, "var"),
        param1Id,
        param1Type,
        nullptr);
    
    auto param2Id = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "b"), "b");
    auto param2Type = std::make_shared<astra::SimpleType>(
        createToken(astra::TokenType::Identifier, "int"), "int");
    auto param2 = std::make_shared<astra::VariableDeclaration>(
        createToken(astra::TokenType::Var, "var"),
        param2Id,
        param2Type,
        nullptr);
    
    parameters.push_back(param1);
    parameters.push_back(param2);
    
    auto returnType = std::make_shared<astra::SimpleType>(
        createToken(astra::TokenType::Identifier, "int"), "int");
    
    auto bodyExpr = std::make_shared<astra::BinaryExpression>(
        createToken(astra::TokenType::Plus, "+"),
        std::make_shared<astra::IdentifierExpression>(createToken(astra::TokenType::Identifier, "a"), "a"),
        std::make_shared<astra::IdentifierExpression>(createToken(astra::TokenType::Identifier, "b"), "b"));
    
    auto returnStmt = std::make_shared<astra::ReturnStatement>(
        createToken(astra::TokenType::Return, "return"),
        bodyExpr);
    
    auto body = std::make_shared<astra::BlockStatement>(
        createToken(astra::TokenType::LeftBrace, "{"),
        std::vector<std::shared_ptr<astra::Statement>>{returnStmt});
    
    std::vector<std::shared_ptr<astra::Statement>> annotations;
    
    astra::FunctionDeclaration decl(funcToken, identifier, parameters, returnType, body, annotations);
    
    EXPECT_EQ(funcToken.lexeme, decl.funcToken.lexeme);
    EXPECT_EQ(identifier, decl.identifier);
    EXPECT_EQ(2, decl.parameters.size());
    EXPECT_EQ(returnType, decl.returnType);
    EXPECT_EQ(body, decl.body);
    EXPECT_EQ(0, decl.annotations.size());
    
    MockASTVisitor visitor;
    decl.accept(visitor);
    EXPECT_EQ("FunctionDeclaration", visitor.lastVisited);
}

TEST(ASTTest, SimpleType) {
    astra::Token token = createToken(astra::TokenType::Identifier, "int");
    astra::SimpleType type(token, "int");
    
    EXPECT_EQ("int", type.name);
    EXPECT_EQ("int", type.toString());
    
    MockASTVisitor visitor;
    type.accept(visitor);
    EXPECT_EQ("SimpleType", visitor.lastVisited);
}

TEST(ASTTest, ArrayType) {
    astra::Token token = createToken(astra::TokenType::Identifier, "int");
    auto elementType = std::make_shared<astra::SimpleType>(token, "int");
    
    astra::Token bracketToken = createToken(astra::TokenType::LeftBracket, "[");
    
    astra::ArrayType type(elementType, bracketToken);
    
    EXPECT_EQ(elementType, type.elementType);
    EXPECT_EQ("int[]", type.toString());
    
    MockASTVisitor visitor;
    type.accept(visitor);
    EXPECT_EQ("ArrayType", visitor.lastVisited);
}

TEST(ASTTest, FunctionType) {
    std::vector<std::shared_ptr<astra::Type>> paramTypes;
    
    auto intType = std::make_shared<astra::SimpleType>(createToken(astra::TokenType::Identifier, "int"), "int");
    auto floatType = std::make_shared<astra::SimpleType>(createToken(astra::TokenType::Identifier, "float"), "float");
    auto boolType = std::make_shared<astra::SimpleType>(createToken(astra::TokenType::Identifier, "bool"), "bool");
    
    paramTypes.push_back(intType);
    paramTypes.push_back(floatType);
    
    astra::Token arrowToken = createToken(astra::TokenType::Arrow, "->");
    
    astra::FunctionType type(paramTypes, boolType, arrowToken);
    
    EXPECT_EQ(2, type.parameterTypes.size());
    EXPECT_EQ(boolType, type.returnType);
    EXPECT_EQ("(int, float) -> bool", type.toString());
    
    MockASTVisitor visitor;
    type.accept(visitor);
    EXPECT_EQ("FunctionType", visitor.lastVisited);
}

TEST(ASTTest, IfStatement) {
    astra::Token ifToken = createToken(astra::TokenType::If, "if");
    auto condition = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::BooleanLiteral, "true"));
    
    auto thenExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto thenStmt = std::make_shared<astra::ExpressionStatement>(thenExpr);
    
    auto elseExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "2"));
    auto elseStmt = std::make_shared<astra::ExpressionStatement>(elseExpr);
    
    astra::IfStatement stmt(ifToken, condition, thenStmt, elseStmt);
    
    EXPECT_EQ(ifToken.lexeme, stmt.ifToken.lexeme);
    EXPECT_EQ(condition, stmt.condition);
    EXPECT_EQ(thenStmt, stmt.thenBranch);
    EXPECT_EQ(elseStmt, stmt.elseBranch);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("IfStatement", visitor.lastVisited);
}

TEST(ASTTest, WhileStatement) {
    astra::Token whileToken = createToken(astra::TokenType::While, "while");
    auto condition = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::BooleanLiteral, "true"));
    
    auto bodyExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto bodyStmt = std::make_shared<astra::ExpressionStatement>(bodyExpr);
    
    astra::WhileStatement stmt(whileToken, condition, bodyStmt);
    
    EXPECT_EQ(whileToken.lexeme, stmt.whileToken.lexeme);
    EXPECT_EQ(condition, stmt.condition);
    EXPECT_EQ(bodyStmt, stmt.body);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("WhileStatement", visitor.lastVisited);
}

TEST(ASTTest, ForStatement) {
    astra::Token forToken = createToken(astra::TokenType::For, "for");
    auto variable = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "i"), "i");
    
    auto iterable = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "10"));
    
    auto bodyExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto bodyStmt = std::make_shared<astra::ExpressionStatement>(bodyExpr);
    
    astra::ForStatement stmt(forToken, variable, iterable, bodyStmt);
    
    EXPECT_EQ(forToken.lexeme, stmt.forToken.lexeme);
    EXPECT_EQ(variable, stmt.variable);
    EXPECT_EQ(iterable, stmt.iterable);
    EXPECT_EQ(bodyStmt, stmt.body);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("ForStatement", visitor.lastVisited);
}

TEST(ASTTest, ReturnStatement) {
    astra::Token returnToken = createToken(astra::TokenType::Return, "return");
    auto value = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "42"));
    
    astra::ReturnStatement stmt(returnToken, value);
    
    EXPECT_EQ(returnToken.lexeme, stmt.returnToken.lexeme);
    EXPECT_EQ(value, stmt.value);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("ReturnStatement", visitor.lastVisited);
}

TEST(ASTTest, BreakStatement) {
    astra::Token breakToken = createToken(astra::TokenType::Break, "break");
    
    astra::BreakStatement stmt(breakToken);
    
    EXPECT_EQ(breakToken.lexeme, stmt.breakToken.lexeme);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("BreakStatement", visitor.lastVisited);
}

TEST(ASTTest, ContinueStatement) {
    astra::Token continueToken = createToken(astra::TokenType::Continue, "continue");
    
    astra::ContinueStatement stmt(continueToken);
    
    EXPECT_EQ(continueToken.lexeme, stmt.continueToken.lexeme);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("ContinueStatement", visitor.lastVisited);
}

TEST(ASTTest, ImportStatement) {
    astra::Token importToken = createToken(astra::TokenType::Import, "import");
    std::vector<std::string> path = {"module", "submodule"};
    
    astra::ImportStatement stmt(importToken, path);
    
    EXPECT_EQ(importToken.lexeme, stmt.importToken.lexeme);
    EXPECT_EQ(2, stmt.path.size());
    EXPECT_EQ("module", stmt.path[0]);
    EXPECT_EQ("submodule", stmt.path[1]);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("ImportStatement", visitor.lastVisited);
}

TEST(ASTTest, ModuleDeclaration) {
    astra::Token moduleToken = createToken(astra::TokenType::Module, "module");
    auto identifier = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "mymodule"), "mymodule");
    
    std::vector<std::shared_ptr<astra::Declaration>> declarations;
    
    astra::ModuleDeclaration decl(moduleToken, identifier, declarations);
    
    EXPECT_EQ(moduleToken.lexeme, decl.moduleToken.lexeme);
    EXPECT_EQ(identifier, decl.identifier);
    EXPECT_EQ(0, decl.declarations.size());
    
    MockASTVisitor visitor;
    decl.accept(visitor);
    EXPECT_EQ("ModuleDeclaration", visitor.lastVisited);
}

TEST(ASTTest, TryStatement) {
    astra::Token tryToken = createToken(astra::TokenType::Try, "try");
    
    auto tryExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto tryBlock = std::make_shared<astra::BlockStatement>(
        createToken(astra::TokenType::LeftBrace, "{"),
        std::vector<std::shared_ptr<astra::Statement>>{std::make_shared<astra::ExpressionStatement>(tryExpr)});
    
    std::vector<std::pair<std::shared_ptr<astra::VariableDeclaration>, std::shared_ptr<astra::BlockStatement>>> catchClauses;
    
    auto errorId = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "error"), "error");
    auto errorType = std::make_shared<astra::SimpleType>(
        createToken(astra::TokenType::Identifier, "Exception"), "Exception");
    
    auto errorVar = std::make_shared<astra::VariableDeclaration>(
        createToken(astra::TokenType::Var, "var"),
        errorId,
        errorType,
        nullptr);
    
    auto catchExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "2"));
    auto catchBlock = std::make_shared<astra::BlockStatement>(
        createToken(astra::TokenType::LeftBrace, "{"),
        std::vector<std::shared_ptr<astra::Statement>>{std::make_shared<astra::ExpressionStatement>(catchExpr)});
    
    catchClauses.push_back(std::make_pair(errorVar, catchBlock));
    
    auto finallyExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "3"));
    auto finallyBlock = std::make_shared<astra::BlockStatement>(
        createToken(astra::TokenType::LeftBrace, "{"),
        std::vector<std::shared_ptr<astra::Statement>>{std::make_shared<astra::ExpressionStatement>(finallyExpr)});
    
    astra::TryStatement stmt(tryToken, tryBlock, catchClauses, finallyBlock);
    
    EXPECT_EQ(tryToken.lexeme, stmt.tryToken.lexeme);
    EXPECT_EQ(tryBlock, stmt.tryBlock);
    EXPECT_EQ(1, stmt.catchClauses.size());
    EXPECT_EQ(finallyBlock, stmt.finallyBlock);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("TryStatement", visitor.lastVisited);
}

TEST(ASTTest, ThrowStatement) {
    astra::Token throwToken = createToken(astra::TokenType::Throw, "throw");
    auto expression = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "error"), "error");
    
    astra::ThrowStatement stmt(throwToken, expression);
    
    EXPECT_EQ(throwToken.lexeme, stmt.throwToken.lexeme);
    EXPECT_EQ(expression, stmt.expression);
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("ThrowStatement", visitor.lastVisited);
}

TEST(ASTTest, TaskDeclaration) {
    astra::Token taskToken = createToken(astra::TokenType::Task, "task");
    auto identifier = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "mytask"), "mytask");
    
    std::vector<std::shared_ptr<astra::VariableDeclaration>> parameters;
    
    auto bodyExpr = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto body = std::make_shared<astra::BlockStatement>(
        createToken(astra::TokenType::LeftBrace, "{"),
        std::vector<std::shared_ptr<astra::Statement>>{std::make_shared<astra::ExpressionStatement>(bodyExpr)});
    
    std::vector<std::shared_ptr<astra::Statement>> annotations;
    
    astra::TaskDeclaration decl(taskToken, identifier, parameters, body, annotations);
    
    EXPECT_EQ(taskToken.lexeme, decl.taskToken.lexeme);
    EXPECT_EQ(identifier, decl.identifier);
    EXPECT_EQ(0, decl.parameters.size());
    EXPECT_EQ(body, decl.body);
    EXPECT_EQ(0, decl.annotations.size());
    
    MockASTVisitor visitor;
    decl.accept(visitor);
    EXPECT_EQ("TaskDeclaration", visitor.lastVisited);
}

TEST(ASTTest, AnnotationStatement) {
    astra::Token atToken = createToken(astra::TokenType::At, "@");
    auto identifier = std::make_shared<astra::IdentifierExpression>(
        createToken(astra::TokenType::Identifier, "deadline"), "deadline");
    
    std::vector<std::shared_ptr<astra::Expression>> arguments;
    arguments.push_back(std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "100")));
    
    astra::AnnotationStatement stmt(atToken, identifier, arguments);
    
    EXPECT_EQ(atToken.lexeme, stmt.atToken.lexeme);
    EXPECT_EQ(identifier, stmt.identifier);
    EXPECT_EQ(1, stmt.arguments.size());
    
    MockASTVisitor visitor;
    stmt.accept(visitor);
    EXPECT_EQ("AnnotationStatement", visitor.lastVisited);
}

TEST(ASTTest, RangeType) {
    auto baseType = std::make_shared<astra::SimpleType>(
        createToken(astra::TokenType::Identifier, "int"), "int");
    
    auto minValue = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "0"));
    auto maxValue = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "100"));
    
    astra::Token lessThanToken = createToken(astra::TokenType::Less, "<");
    
    astra::RangeType type(baseType, minValue, maxValue, lessThanToken);
    
    EXPECT_EQ(baseType, type.baseType);
    EXPECT_EQ(minValue, type.minValue);
    EXPECT_EQ(maxValue, type.maxValue);
    EXPECT_EQ(lessThanToken.lexeme, type.lessThanToken.lexeme);
    EXPECT_EQ("int<range>", type.toString());
    
    MockASTVisitor visitor;
    type.accept(visitor);
    EXPECT_EQ("RangeType", visitor.lastVisited);
}

TEST(ASTTest, Program) {
    std::vector<std::shared_ptr<astra::Statement>> statements;
    
    auto expr1 = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "1"));
    auto expr2 = std::make_shared<astra::LiteralExpression>(createToken(astra::TokenType::IntegerLiteral, "2"));
    
    statements.push_back(std::make_shared<astra::ExpressionStatement>(expr1));
    statements.push_back(std::make_shared<astra::ExpressionStatement>(expr2));
    
    astra::Program program(statements);
    
    EXPECT_EQ(2, program.statements.size());
    
    MockASTVisitor visitor;
    program.accept(visitor);
    EXPECT_EQ("Program", visitor.lastVisited);
}