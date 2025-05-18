/**
 * ASTRA Programming Language Compiler
 * Parser
 */

#ifndef ASTRA_PARSER_H
#define ASTRA_PARSER_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>
#include "../lexer/token.h"
#include "../ast/ast.h"
#include "../utils/error_handler.h"

namespace astra {

/**
 * Precedence levels for operators
 */
enum class Precedence {
    NONE,
    ASSIGNMENT,  // =, +=, -=, etc.
    LOGICAL_OR,  // ||
    LOGICAL_AND, // &&
    BITWISE_OR,  // |
    BITWISE_XOR, // ^
    BITWISE_AND, // &
    EQUALITY,    // ==, !=
    COMPARISON,  // <, >, <=, >=
    SHIFT,       // <<, >>
    TERM,        // +, -
    FACTOR,      // *, /, %
    UNARY,       // !, -, ~
    CALL,        // (), []
    PRIMARY
};

/**
 * Parser for ASTRA language
 */
class Parser {
private:
    std::vector<Token> tokens;
    ErrorHandler& errorHandler;
    size_t current = 0;
    
    // Operator precedence map
    std::unordered_map<TokenType, Precedence> precedenceMap = {
        {TokenType::Assign, Precedence::ASSIGNMENT},
        {TokenType::PlusAssign, Precedence::ASSIGNMENT},
        {TokenType::MinusAssign, Precedence::ASSIGNMENT},
        {TokenType::MultiplyAssign, Precedence::ASSIGNMENT},
        {TokenType::DivideAssign, Precedence::ASSIGNMENT},
        {TokenType::ModuloAssign, Precedence::ASSIGNMENT},
        {TokenType::AndAssign, Precedence::ASSIGNMENT},
        {TokenType::OrAssign, Precedence::ASSIGNMENT},
        {TokenType::XorAssign, Precedence::ASSIGNMENT},
        {TokenType::LeftShiftAssign, Precedence::ASSIGNMENT},
        {TokenType::RightShiftAssign, Precedence::ASSIGNMENT},
        {TokenType::Or, Precedence::LOGICAL_OR},
        {TokenType::And, Precedence::LOGICAL_AND},
        {TokenType::BitwiseOr, Precedence::BITWISE_OR},
        {TokenType::BitwiseXor, Precedence::BITWISE_XOR},
        {TokenType::BitwiseAnd, Precedence::BITWISE_AND},
        {TokenType::Equal, Precedence::EQUALITY},
        {TokenType::NotEqual, Precedence::EQUALITY},
        {TokenType::Less, Precedence::COMPARISON},
        {TokenType::LessEqual, Precedence::COMPARISON},
        {TokenType::Greater, Precedence::COMPARISON},
        {TokenType::GreaterEqual, Precedence::COMPARISON},
        {TokenType::LeftShift, Precedence::SHIFT},
        {TokenType::RightShift, Precedence::SHIFT},
        {TokenType::Plus, Precedence::TERM},
        {TokenType::Minus, Precedence::TERM},
        {TokenType::Multiply, Precedence::FACTOR},
        {TokenType::Divide, Precedence::FACTOR},
        {TokenType::Modulo, Precedence::FACTOR},
        {TokenType::LeftParen, Precedence::CALL},
        {TokenType::LeftBracket, Precedence::CALL},
        {TokenType::Dot, Precedence::CALL}
    };
    
    // Prefix parse functions
    using PrefixParseFn = std::function<std::shared_ptr<Expression>()>;
    std::unordered_map<TokenType, PrefixParseFn> prefixParseFns;
    
    // Infix parse functions
    using InfixParseFn = std::function<std::shared_ptr<Expression>(std::shared_ptr<Expression>)>;
    std::unordered_map<TokenType, InfixParseFn> infixParseFns;
    
    /**
     * Register prefix parse functions
     */
    void registerPrefixParseFns();
    
    /**
     * Register infix parse functions
     */
    void registerInfixParseFns();
    
    /**
     * Get current token
     */
    Token& peek() {
        return tokens[current];
    }
    
    /**
     * Get next token
     */
    Token& peekNext() {
        if (current + 1 >= tokens.size()) {
            return tokens[current];
        }
        return tokens[current + 1];
    }
    
    /**
     * Advance to next token
     */
    Token advance() {
        if (!isAtEnd()) {
            current++;
        }
        return previous();
    }
    
    /**
     * Get previous token
     */
    Token& previous() {
        return tokens[current - 1];
    }
    
    /**
     * Check if at end of tokens
     */
    bool isAtEnd() {
        return peek().type == TokenType::EndOfFile;
    }
    
    /**
     * Check if current token matches expected type
     */
    bool check(TokenType type) {
        if (isAtEnd()) return false;
        return peek().type == type;
    }
    
    /**
     * Consume token if it matches expected type
     */
    bool match(TokenType type) {
        if (check(type)) {
            advance();
            return true;
        }
        return false;
    }
    
    /**
     * Consume token if it matches any of the expected types
     */
    bool match(std::initializer_list<TokenType> types) {
        for (auto type : types) {
            if (check(type)) {
                advance();
                return true;
            }
        }
        return false;
    }
    
    /**
     * Consume token if it matches expected type, otherwise report error
     */
    Token consume(TokenType type, const std::string& message) {
        if (check(type)) {
            return advance();
        }
        
        Token errorToken = peek();
        errorHandler.reportError(message, errorToken.filename, errorToken.line, errorToken.column);
        return errorToken;
    }
    
    /**
     * Get precedence of current token
     */
    Precedence getCurrentPrecedence() {
        auto it = precedenceMap.find(peek().type);
        if (it != precedenceMap.end()) {
            return it->second;
        }
        return Precedence::NONE;
    }
    
    /**
     * Get precedence of next token
     */
    Precedence getNextPrecedence() {
        auto it = precedenceMap.find(peekNext().type);
        if (it != precedenceMap.end()) {
            return it->second;
        }
        return Precedence::NONE;
    }
    
    /**
     * Parse expression with given precedence
     */
    std::shared_ptr<Expression> parseExpression(Precedence precedence);
    
    /**
     * Parse prefix expression
     */
    std::shared_ptr<Expression> parsePrefixExpression();
    
    /**
     * Parse infix expression
     */
    std::shared_ptr<Expression> parseInfixExpression(std::shared_ptr<Expression> left);
    
    /**
     * Parse grouped expression
     */
    std::shared_ptr<Expression> parseGroupedExpression();
    
    /**
     * Parse identifier
     */
    std::shared_ptr<Expression> parseIdentifier();
    
    /**
     * Parse integer literal
     */
    std::shared_ptr<Expression> parseIntegerLiteral();
    
    /**
     * Parse float literal
     */
    std::shared_ptr<Expression> parseFloatLiteral();
    
    /**
     * Parse string literal
     */
    std::shared_ptr<Expression> parseStringLiteral();
    
    /**
     * Parse boolean literal
     */
    std::shared_ptr<Expression> parseBooleanLiteral();
    
    /**
     * Parse array literal
     */
    std::shared_ptr<Expression> parseArrayLiteral();
    
    /**
     * Parse call expression
     */
    std::shared_ptr<Expression> parseCallExpression(std::shared_ptr<Expression> callee);
    
    /**
     * Parse member expression
     */
    std::shared_ptr<Expression> parseMemberExpression(std::shared_ptr<Expression> object);
    
    /**
     * Parse statement
     */
    std::shared_ptr<Statement> parseStatement();
    
    /**
     * Parse expression statement
     */
    std::shared_ptr<Statement> parseExpressionStatement();
    
    /**
     * Parse block statement
     */
    std::shared_ptr<BlockStatement> parseBlockStatement();
    
    /**
     * Parse variable declaration
     */
    std::shared_ptr<VariableDeclaration> parseVariableDeclaration();
    
    /**
     * Parse function declaration
     */
    std::shared_ptr<FunctionDeclaration> parseFunctionDeclaration();
    
    /**
     * Parse if statement
     */
    std::shared_ptr<Statement> parseIfStatement();
    
    /**
     * Parse while statement
     */
    std::shared_ptr<Statement> parseWhileStatement();
    
    /**
     * Parse for statement
     */
    std::shared_ptr<Statement> parseForStatement();
    
    /**
     * Parse return statement
     */
    std::shared_ptr<Statement> parseReturnStatement();
    
    /**
     * Parse break statement
     */
    std::shared_ptr<Statement> parseBreakStatement();
    
    /**
     * Parse continue statement
     */
    std::shared_ptr<Statement> parseContinueStatement();
    
    /**
     * Parse import statement
     */
    std::shared_ptr<Statement> parseImportStatement();
    
    /**
     * Parse module declaration
     */
    std::shared_ptr<ModuleDeclaration> parseModuleDeclaration();
    
    /**
     * Parse try statement
     */
    std::shared_ptr<Statement> parseTryStatement();
    
    /**
     * Parse throw statement
     */
    std::shared_ptr<Statement> parseThrowStatement();
    
    /**
     * Parse task declaration
     */
    std::shared_ptr<TaskDeclaration> parseTaskDeclaration();
    
    /**
     * Parse annotation statement
     */
    std::shared_ptr<Statement> parseAnnotationStatement();
    
    /**
     * Parse type
     */
    std::shared_ptr<Type> parseType();
    
    /**
     * Parse simple type
     */
    std::shared_ptr<Type> parseSimpleType();
    
    /**
     * Parse array type
     */
    std::shared_ptr<Type> parseArrayType(std::shared_ptr<Type> elementType);
    
    /**
     * Parse range type
     */
    std::shared_ptr<Type> parseRangeType(std::shared_ptr<Type> baseType);
    
    /**
     * Parse function type
     */
    std::shared_ptr<Type> parseFunctionType();
    
    /**
     * Synchronize parser after error
     */
    void synchronize();
    
public:
    /**
     * Constructor
     */
    Parser(const std::vector<Token>& toks, ErrorHandler& errHandler)
        : tokens(toks), errorHandler(errHandler) {
        registerPrefixParseFns();
        registerInfixParseFns();
    }
    
    /**
     * Parse tokens into AST
     */
    std::shared_ptr<Program> parse();
};

} // namespace astra

#endif // ASTRA_PARSER_H