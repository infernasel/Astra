#include "parser.h"
#include <stdexcept>
#include <sstream>

namespace astra {

Parser::Parser(const std::vector<Token>& tokens, ErrorHandler& errorHandler)
    : tokens(tokens), errorHandler(errorHandler), current(0) {}

std::unique_ptr<AST::Program> Parser::parse() {
    auto program = std::make_unique<AST::Program>();
    
    try {
        while (!isAtEnd() && !check(TokenType::EndOfFile)) {
            program->declarations.push_back(parseDeclaration());
        }
    } catch (const ParseError& error) {
        // Error already reported by errorHandler
        synchronize();
    }
    
    return program;
}

std::unique_ptr<AST::Declaration> Parser::parseDeclaration() {
    try {
        if (match(TokenType::Var)) {
            return parseVariableDeclaration();
        } else if (match(TokenType::Const)) {
            return parseConstantDeclaration();
        } else if (match(TokenType::Func)) {
            return parseFunctionDeclaration();
        } else if (match(TokenType::Class)) {
            return parseClassDeclaration();
        } else if (match(TokenType::Interface)) {
            return parseInterfaceDeclaration();
        } else if (match(TokenType::Import)) {
            return parseImportDeclaration();
        } else if (match(TokenType::Export)) {
            return parseExportDeclaration();
        } else {
            return parseStatement();
        }
    } catch (const ParseError& error) {
        synchronize();
        return nullptr;
    }
}

std::unique_ptr<AST::VariableDeclaration> Parser::parseVariableDeclaration() {
    Token name = consume(TokenType::Identifier, "Expected variable name.");
    
    std::unique_ptr<AST::TypeAnnotation> typeAnnotation = nullptr;
    if (match(TokenType::Colon)) {
        typeAnnotation = parseTypeAnnotation();
    }
    
    std::unique_ptr<AST::Expression> initializer = nullptr;
    if (match(TokenType::Assign)) {
        initializer = parseExpression();
    }
    
    consume(TokenType::Semicolon, "Expected ';' after variable declaration.");
    
    return std::make_unique<AST::VariableDeclaration>(name, std::move(typeAnnotation), std::move(initializer));
}

std::unique_ptr<AST::ConstantDeclaration> Parser::parseConstantDeclaration() {
    Token name = consume(TokenType::Identifier, "Expected constant name.");
    
    std::unique_ptr<AST::TypeAnnotation> typeAnnotation = nullptr;
    if (match(TokenType::Colon)) {
        typeAnnotation = parseTypeAnnotation();
    }
    
    consume(TokenType::Assign, "Expected '=' after constant name.");
    std::unique_ptr<AST::Expression> initializer = parseExpression();
    
    consume(TokenType::Semicolon, "Expected ';' after constant declaration.");
    
    return std::make_unique<AST::ConstantDeclaration>(name, std::move(typeAnnotation), std::move(initializer));
}

std::unique_ptr<AST::FunctionDeclaration> Parser::parseFunctionDeclaration() {
    Token name = consume(TokenType::Identifier, "Expected function name.");
    
    consume(TokenType::LeftParen, "Expected '(' after function name.");
    std::vector<AST::Parameter> parameters = parseParameters();
    consume(TokenType::RightParen, "Expected ')' after parameters.");
    
    std::unique_ptr<AST::TypeAnnotation> returnType = nullptr;
    if (match(TokenType::Arrow)) {
        returnType = parseTypeAnnotation();
    }
    
    std::unique_ptr<AST::BlockStatement> body = parseBlockStatement();
    
    return std::make_unique<AST::FunctionDeclaration>(name, std::move(parameters), std::move(returnType), std::move(body));
}

std::vector<AST::Parameter> Parser::parseParameters() {
    std::vector<AST::Parameter> parameters;
    
    if (!check(TokenType::RightParen)) {
        do {
            Token name = consume(TokenType::Identifier, "Expected parameter name.");
            
            std::unique_ptr<AST::TypeAnnotation> type = nullptr;
            if (match(TokenType::Colon)) {
                type = parseTypeAnnotation();
            }
            
            parameters.push_back(AST::Parameter{name, std::move(type)});
        } while (match(TokenType::Comma));
    }
    
    return parameters;
}

std::unique_ptr<AST::ClassDeclaration> Parser::parseClassDeclaration() {
    Token name = consume(TokenType::Identifier, "Expected class name.");
    
    std::unique_ptr<AST::TypeAnnotation> superClass = nullptr;
    if (match(TokenType::Extends)) {
        superClass = parseTypeAnnotation();
    }
    
    std::vector<std::unique_ptr<AST::TypeAnnotation>> interfaces;
    if (match(TokenType::Implements)) {
        do {
            interfaces.push_back(parseTypeAnnotation());
        } while (match(TokenType::Comma));
    }
    
    consume(TokenType::LeftBrace, "Expected '{' before class body.");
    
    std::vector<std::unique_ptr<AST::ClassMember>> members;
    while (!check(TokenType::RightBrace) && !isAtEnd()) {
        members.push_back(parseClassMember());
    }
    
    consume(TokenType::RightBrace, "Expected '}' after class body.");
    
    return std::make_unique<AST::ClassDeclaration>(name, std::move(superClass), std::move(interfaces), std::move(members));
}

std::unique_ptr<AST::ClassMember> Parser::parseClassMember() {
    AST::AccessModifier accessModifier = AST::AccessModifier::Public;
    bool isStatic = false;
    
    // Parse access modifier
    if (match(TokenType::Public)) {
        accessModifier = AST::AccessModifier::Public;
    } else if (match(TokenType::Private)) {
        accessModifier = AST::AccessModifier::Private;
    } else if (match(TokenType::Protected)) {
        accessModifier = AST::AccessModifier::Protected;
    }
    
    // Parse static modifier
    if (match(TokenType::Static)) {
        isStatic = true;
    }
    
    if (match(TokenType::Var)) {
        // Field declaration
        Token name = consume(TokenType::Identifier, "Expected field name.");
        
        std::unique_ptr<AST::TypeAnnotation> typeAnnotation = nullptr;
        if (match(TokenType::Colon)) {
            typeAnnotation = parseTypeAnnotation();
        }
        
        std::unique_ptr<AST::Expression> initializer = nullptr;
        if (match(TokenType::Assign)) {
            initializer = parseExpression();
        }
        
        consume(TokenType::Semicolon, "Expected ';' after field declaration.");
        
        return std::make_unique<AST::FieldDeclaration>(name, std::move(typeAnnotation), std::move(initializer), accessModifier, isStatic);
    } else if (match(TokenType::Func)) {
        // Method declaration
        Token name = consume(TokenType::Identifier, "Expected method name.");
        
        consume(TokenType::LeftParen, "Expected '(' after method name.");
        std::vector<AST::Parameter> parameters = parseParameters();
        consume(TokenType::RightParen, "Expected ')' after parameters.");
        
        std::unique_ptr<AST::TypeAnnotation> returnType = nullptr;
        if (match(TokenType::Arrow)) {
            returnType = parseTypeAnnotation();
        }
        
        std::unique_ptr<AST::BlockStatement> body = parseBlockStatement();
        
        return std::make_unique<AST::MethodDeclaration>(name, std::move(parameters), std::move(returnType), std::move(body), accessModifier, isStatic);
    } else {
        throw error(peek(), "Expected field or method declaration.");
    }
}

std::unique_ptr<AST::InterfaceDeclaration> Parser::parseInterfaceDeclaration() {
    Token name = consume(TokenType::Identifier, "Expected interface name.");
    
    std::vector<std::unique_ptr<AST::TypeAnnotation>> extends;
    if (match(TokenType::Extends)) {
        do {
            extends.push_back(parseTypeAnnotation());
        } while (match(TokenType::Comma));
    }
    
    consume(TokenType::LeftBrace, "Expected '{' before interface body.");
    
    std::vector<std::unique_ptr<AST::MethodSignature>> methods;
    while (!check(TokenType::RightBrace) && !isAtEnd()) {
        methods.push_back(parseMethodSignature());
    }
    
    consume(TokenType::RightBrace, "Expected '}' after interface body.");
    
    return std::make_unique<AST::InterfaceDeclaration>(name, std::move(extends), std::move(methods));
}

std::unique_ptr<AST::MethodSignature> Parser::parseMethodSignature() {
    Token name = consume(TokenType::Identifier, "Expected method name.");
    
    consume(TokenType::LeftParen, "Expected '(' after method name.");
    std::vector<AST::Parameter> parameters = parseParameters();
    consume(TokenType::RightParen, "Expected ')' after parameters.");
    
    std::unique_ptr<AST::TypeAnnotation> returnType = nullptr;
    if (match(TokenType::Arrow)) {
        returnType = parseTypeAnnotation();
    }
    
    consume(TokenType::Semicolon, "Expected ';' after method signature.");
    
    return std::make_unique<AST::MethodSignature>(name, std::move(parameters), std::move(returnType));
}

std::unique_ptr<AST::ImportDeclaration> Parser::parseImportDeclaration() {
    std::vector<std::string> path;
    
    do {
        Token name = consume(TokenType::Identifier, "Expected module name.");
        path.push_back(name.lexeme);
    } while (match(TokenType::Dot));
    
    consume(TokenType::Semicolon, "Expected ';' after import declaration.");
    
    return std::make_unique<AST::ImportDeclaration>(path);
}

std::unique_ptr<AST::ExportDeclaration> Parser::parseExportDeclaration() {
    std::unique_ptr<AST::Declaration> declaration = parseDeclaration();
    return std::make_unique<AST::ExportDeclaration>(std::move(declaration));
}

std::unique_ptr<AST::TypeAnnotation> Parser::parseTypeAnnotation() {
    Token name = consume(TokenType::Identifier, "Expected type name.");
    
    std::vector<std::unique_ptr<AST::TypeAnnotation>> typeArguments;
    if (match(TokenType::Less)) {
        do {
            typeArguments.push_back(parseTypeAnnotation());
        } while (match(TokenType::Comma));
        
        consume(TokenType::Greater, "Expected '>' after type arguments.");
    }
    
    return std::make_unique<AST::TypeAnnotation>(name, std::move(typeArguments));
}

std::unique_ptr<AST::Statement> Parser::parseStatement() {
    if (match(TokenType::If)) {
        return parseIfStatement();
    } else if (match(TokenType::While)) {
        return parseWhileStatement();
    } else if (match(TokenType::For)) {
        return parseForStatement();
    } else if (match(TokenType::Return)) {
        return parseReturnStatement();
    } else if (match(TokenType::Break)) {
        return parseBreakStatement();
    } else if (match(TokenType::Continue)) {
        return parseContinueStatement();
    } else if (match(TokenType::LeftBrace)) {
        return parseBlockStatement();
    } else {
        return parseExpressionStatement();
    }
}

std::unique_ptr<AST::IfStatement> Parser::parseIfStatement() {
    consume(TokenType::LeftParen, "Expected '(' after 'if'.");
    std::unique_ptr<AST::Expression> condition = parseExpression();
    consume(TokenType::RightParen, "Expected ')' after if condition.");
    
    std::unique_ptr<AST::Statement> thenBranch = parseStatement();
    std::unique_ptr<AST::Statement> elseBranch = nullptr;
    
    if (match(TokenType::Else)) {
        elseBranch = parseStatement();
    }
    
    return std::make_unique<AST::IfStatement>(std::move(condition), std::move(thenBranch), std::move(elseBranch));
}

std::unique_ptr<AST::WhileStatement> Parser::parseWhileStatement() {
    consume(TokenType::LeftParen, "Expected '(' after 'while'.");
    std::unique_ptr<AST::Expression> condition = parseExpression();
    consume(TokenType::RightParen, "Expected ')' after while condition.");
    
    std::unique_ptr<AST::Statement> body = parseStatement();
    
    return std::make_unique<AST::WhileStatement>(std::move(condition), std::move(body));
}

std::unique_ptr<AST::ForStatement> Parser::parseForStatement() {
    consume(TokenType::LeftParen, "Expected '(' after 'for'.");
    
    std::unique_ptr<AST::Statement> initializer = nullptr;
    if (!check(TokenType::Semicolon)) {
        if (match(TokenType::Var)) {
            initializer = parseVariableDeclaration();
        } else {
            initializer = parseExpressionStatement();
        }
    } else {
        consume(TokenType::Semicolon, "Expected ';'.");
    }
    
    std::unique_ptr<AST::Expression> condition = nullptr;
    if (!check(TokenType::Semicolon)) {
        condition = parseExpression();
    }
    consume(TokenType::Semicolon, "Expected ';' after for condition.");
    
    std::unique_ptr<AST::Expression> increment = nullptr;
    if (!check(TokenType::RightParen)) {
        increment = parseExpression();
    }
    consume(TokenType::RightParen, "Expected ')' after for clauses.");
    
    std::unique_ptr<AST::Statement> body = parseStatement();
    
    return std::make_unique<AST::ForStatement>(std::move(initializer), std::move(condition), std::move(increment), std::move(body));
}

std::unique_ptr<AST::ReturnStatement> Parser::parseReturnStatement() {
    Token keyword = previous();
    std::unique_ptr<AST::Expression> value = nullptr;
    
    if (!check(TokenType::Semicolon)) {
        value = parseExpression();
    }
    
    consume(TokenType::Semicolon, "Expected ';' after return value.");
    
    return std::make_unique<AST::ReturnStatement>(keyword, std::move(value));
}

std::unique_ptr<AST::BreakStatement> Parser::parseBreakStatement() {
    Token keyword = previous();
    consume(TokenType::Semicolon, "Expected ';' after 'break'.");
    
    return std::make_unique<AST::BreakStatement>(keyword);
}

std::unique_ptr<AST::ContinueStatement> Parser::parseContinueStatement() {
    Token keyword = previous();
    consume(TokenType::Semicolon, "Expected ';' after 'continue'.");
    
    return std::make_unique<AST::ContinueStatement>(keyword);
}

std::unique_ptr<AST::BlockStatement> Parser::parseBlockStatement() {
    std::vector<std::unique_ptr<AST::Statement>> statements;
    
    while (!check(TokenType::RightBrace) && !isAtEnd()) {
        statements.push_back(parseDeclaration());
    }
    
    consume(TokenType::RightBrace, "Expected '}' after block.");
    
    return std::make_unique<AST::BlockStatement>(std::move(statements));
}

std::unique_ptr<AST::ExpressionStatement> Parser::parseExpressionStatement() {
    std::unique_ptr<AST::Expression> expression = parseExpression();
    consume(TokenType::Semicolon, "Expected ';' after expression.");
    
    return std::make_unique<AST::ExpressionStatement>(std::move(expression));
}

std::unique_ptr<AST::Expression> Parser::parseExpression() {
    return parseAssignment();
}

std::unique_ptr<AST::Expression> Parser::parseAssignment() {
    std::unique_ptr<AST::Expression> expr = parseLogicalOr();
    
    if (match(TokenType::Assign)) {
        Token equals = previous();
        std::unique_ptr<AST::Expression> value = parseAssignment();
        
        if (auto* variable = dynamic_cast<AST::VariableExpression*>(expr.get())) {
            Token name = variable->name;
            return std::make_unique<AST::AssignmentExpression>(name, std::move(value));
        } else if (auto* get = dynamic_cast<AST::GetExpression*>(expr.get())) {
            return std::make_unique<AST::SetExpression>(std::move(get->object), get->name, std::move(value));
        } else if (auto* index = dynamic_cast<AST::IndexExpression*>(expr.get())) {
            return std::make_unique<AST::IndexAssignmentExpression>(std::move(index->object), std::move(index->index), std::move(value));
        }
        
        error(equals, "Invalid assignment target.");
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseLogicalOr() {
    std::unique_ptr<AST::Expression> expr = parseLogicalAnd();
    
    while (match(TokenType::Or)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseLogicalAnd();
        expr = std::make_unique<AST::LogicalExpression>(std::move(expr), op, std::move(right));
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseLogicalAnd() {
    std::unique_ptr<AST::Expression> expr = parseEquality();
    
    while (match(TokenType::And)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseEquality();
        expr = std::make_unique<AST::LogicalExpression>(std::move(expr), op, std::move(right));
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseEquality() {
    std::unique_ptr<AST::Expression> expr = parseComparison();
    
    while (match(TokenType::Equal) || match(TokenType::NotEqual)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseComparison();
        expr = std::make_unique<AST::BinaryExpression>(std::move(expr), op, std::move(right));
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseComparison() {
    std::unique_ptr<AST::Expression> expr = parseTerm();
    
    while (match(TokenType::Less) || match(TokenType::LessEqual) ||
           match(TokenType::Greater) || match(TokenType::GreaterEqual)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseTerm();
        expr = std::make_unique<AST::BinaryExpression>(std::move(expr), op, std::move(right));
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseTerm() {
    std::unique_ptr<AST::Expression> expr = parseFactor();
    
    while (match(TokenType::Plus) || match(TokenType::Minus)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseFactor();
        expr = std::make_unique<AST::BinaryExpression>(std::move(expr), op, std::move(right));
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseFactor() {
    std::unique_ptr<AST::Expression> expr = parseUnary();
    
    while (match(TokenType::Multiply) || match(TokenType::Divide) || match(TokenType::Modulo)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseUnary();
        expr = std::make_unique<AST::BinaryExpression>(std::move(expr), op, std::move(right));
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::parseUnary() {
    if (match(TokenType::Not) || match(TokenType::Minus) || match(TokenType::BitwiseNot)) {
        Token op = previous();
        std::unique_ptr<AST::Expression> right = parseUnary();
        return std::make_unique<AST::UnaryExpression>(op, std::move(right));
    }
    
    return parseCall();
}

std::unique_ptr<AST::Expression> Parser::parseCall() {
    std::unique_ptr<AST::Expression> expr = parsePrimary();
    
    while (true) {
        if (match(TokenType::LeftParen)) {
            expr = finishCall(std::move(expr));
        } else if (match(TokenType::Dot)) {
            Token name = consume(TokenType::Identifier, "Expected property name after '.'.");
            expr = std::make_unique<AST::GetExpression>(std::move(expr), name);
        } else if (match(TokenType::LeftBracket)) {
            std::unique_ptr<AST::Expression> index = parseExpression();
            consume(TokenType::RightBracket, "Expected ']' after index.");
            expr = std::make_unique<AST::IndexExpression>(std::move(expr), std::move(index));
        } else {
            break;
        }
    }
    
    return expr;
}

std::unique_ptr<AST::Expression> Parser::finishCall(std::unique_ptr<AST::Expression> callee) {
    std::vector<std::unique_ptr<AST::Expression>> arguments;
    
    if (!check(TokenType::RightParen)) {
        do {
            arguments.push_back(parseExpression());
        } while (match(TokenType::Comma));
    }
    
    Token paren = consume(TokenType::RightParen, "Expected ')' after arguments.");
    
    return std::make_unique<AST::CallExpression>(std::move(callee), paren, std::move(arguments));
}

std::unique_ptr<AST::Expression> Parser::parsePrimary() {
    if (match(TokenType::False)) {
        return std::make_unique<AST::LiteralExpression>(false);
    }
    
    if (match(TokenType::True)) {
        return std::make_unique<AST::LiteralExpression>(true);
    }
    
    if (match(TokenType::Null)) {
        return std::make_unique<AST::LiteralExpression>(nullptr);
    }
    
    if (match(TokenType::IntegerLiteral)) {
        return std::make_unique<AST::LiteralExpression>(previous().value);
    }
    
    if (match(TokenType::FloatLiteral)) {
        return std::make_unique<AST::LiteralExpression>(previous().value);
    }
    
    if (match(TokenType::StringLiteral)) {
        return std::make_unique<AST::LiteralExpression>(previous().value);
    }
    
    if (match(TokenType::Identifier)) {
        return std::make_unique<AST::VariableExpression>(previous());
    }
    
    if (match(TokenType::LeftParen)) {
        std::unique_ptr<AST::Expression> expr = parseExpression();
        consume(TokenType::RightParen, "Expected ')' after expression.");
        return std::make_unique<AST::GroupingExpression>(std::move(expr));
    }
    
    if (match(TokenType::LeftBracket)) {
        return parseArrayLiteral();
    }
    
    if (match(TokenType::LeftBrace)) {
        return parseObjectLiteral();
    }
    
    if (match(TokenType::New)) {
        return parseNewExpression();
    }
    
    throw error(peek(), "Expected expression.");
}

std::unique_ptr<AST::Expression> Parser::parseArrayLiteral() {
    std::vector<std::unique_ptr<AST::Expression>> elements;
    
    if (!check(TokenType::RightBracket)) {
        do {
            elements.push_back(parseExpression());
        } while (match(TokenType::Comma));
    }
    
    consume(TokenType::RightBracket, "Expected ']' after array elements.");
    
    return std::make_unique<AST::ArrayLiteralExpression>(std::move(elements));
}

std::unique_ptr<AST::Expression> Parser::parseObjectLiteral() {
    std::vector<AST::ObjectProperty> properties;
    
    if (!check(TokenType::RightBrace)) {
        do {
            Token key = consume(TokenType::Identifier, "Expected property name.");
            consume(TokenType::Colon, "Expected ':' after property name.");
            std::unique_ptr<AST::Expression> value = parseExpression();
            
            properties.push_back(AST::ObjectProperty{key, std::move(value)});
        } while (match(TokenType::Comma));
    }
    
    consume(TokenType::RightBrace, "Expected '}' after object properties.");
    
    return std::make_unique<AST::ObjectLiteralExpression>(std::move(properties));
}

std::unique_ptr<AST::Expression> Parser::parseNewExpression() {
    Token className = consume(TokenType::Identifier, "Expected class name after 'new'.");
    
    consume(TokenType::LeftParen, "Expected '(' after class name.");
    std::vector<std::unique_ptr<AST::Expression>> arguments;
    
    if (!check(TokenType::RightParen)) {
        do {
            arguments.push_back(parseExpression());
        } while (match(TokenType::Comma));
    }
    
    consume(TokenType::RightParen, "Expected ')' after arguments.");
    
    return std::make_unique<AST::NewExpression>(className, std::move(arguments));
}

bool Parser::match(TokenType type) {
    if (check(type)) {
        advance();
        return true;
    }
    return false;
}

bool Parser::check(TokenType type) const {
    if (isAtEnd()) {
        return false;
    }
    return peek().type == type;
}

Token Parser::advance() {
    if (!isAtEnd()) {
        current++;
    }
    return previous();
}

bool Parser::isAtEnd() const {
    return peek().type == TokenType::EndOfFile;
}

Token Parser::peek() const {
    return tokens[current];
}

Token Parser::previous() const {
    return tokens[current - 1];
}

Token Parser::consume(TokenType type, const std::string& message) {
    if (check(type)) {
        return advance();
    }
    
    throw error(peek(), message);
}

ParseError Parser::error(const Token& token, const std::string& message) {
    errorHandler.reportError(message, token.filename, token.line, token.column);
    return ParseError(message);
}

void Parser::synchronize() {
    advance();
    
    while (!isAtEnd()) {
        if (previous().type == TokenType::Semicolon) {
            return;
        }
        
        switch (peek().type) {
            case TokenType::Class:
            case TokenType::Func:
            case TokenType::Var:
            case TokenType::Const:
            case TokenType::For:
            case TokenType::If:
            case TokenType::While:
            case TokenType::Return:
            case TokenType::Import:
            case TokenType::Export:
                return;
            default:
                break;
        }
        
        advance();
    }
}

} // namespace astra