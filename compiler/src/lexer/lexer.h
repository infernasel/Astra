/**
 * ASTRA Programming Language Compiler
 * Lexical analyzer
 */

#ifndef ASTRA_LEXER_H
#define ASTRA_LEXER_H

#include <string>
#include <vector>
#include <cctype>
#include <stdexcept>
#include "token.h"
#include "../utils/error_handler.h"

namespace astra {

/**
 * Lexical analyzer for ASTRA language
 */
class Lexer {
private:
    std::string source;
    std::string filename;
    ErrorHandler& errorHandler;
    
    size_t position = 0;
    int line = 1;
    int column = 1;
    
    /**
     * Get current character
     */
    char current() const {
        if (position >= source.length()) {
            return '\0';
        }
        return source[position];
    }
    
    /**
     * Peek at the next character
     */
    char peek() const {
        if (position + 1 >= source.length()) {
            return '\0';
        }
        return source[position + 1];
    }
    
    /**
     * Advance to the next character
     */
    void advance() {
        if (current() == '\n') {
            line++;
            column = 1;
        } else {
            column++;
        }
        position++;
    }
    
    /**
     * Match the current character and advance if it matches
     */
    bool match(char expected) {
        if (current() != expected) {
            return false;
        }
        advance();
        return true;
    }
    
    /**
     * Skip whitespace
     */
    void skipWhitespace() {
        while (isspace(current())) {
            advance();
        }
    }
    
    /**
     * Skip comments
     */
    void skipComments() {
        if (current() == '/' && peek() == '/') {
            // Single-line comment
            while (current() != '\n' && current() != '\0') {
                advance();
            }
        } else if (current() == '/' && peek() == '*') {
            // Multi-line comment
            advance(); // Skip '/'
            advance(); // Skip '*'
            
            while (!(current() == '*' && peek() == '/') && current() != '\0') {
                advance();
            }
            
            if (current() == '\0') {
                errorHandler.reportError("Unterminated multi-line comment", filename, line, column);
            } else {
                advance(); // Skip '*'
                advance(); // Skip '/'
            }
        }
    }
    
    /**
     * Scan an identifier or keyword
     */
    Token scanIdentifier() {
        int startColumn = column;
        std::string lexeme;
        
        while (isalnum(current()) || current() == '_') {
            lexeme += current();
            advance();
        }
        
        // Check if it's a keyword
        auto it = KEYWORDS.find(lexeme);
        if (it != KEYWORDS.end()) {
            TokenType type = it->second;
            
            // Handle boolean literals
            if (type == TokenType::BooleanLiteral) {
                bool value = (lexeme == "true");
                return Token(type, value, lexeme, filename, line, startColumn);
            }
            
            return Token(type, lexeme, filename, line, startColumn);
        }
        
        // It's an identifier
        return Token(TokenType::Identifier, lexeme, filename, line, startColumn);
    }
    
    /**
     * Scan a number (integer or float)
     */
    Token scanNumber() {
        int startColumn = column;
        std::string lexeme;
        bool isFloat = false;
        
        // Scan integer part
        while (isdigit(current())) {
            lexeme += current();
            advance();
        }
        
        // Check for decimal point
        if (current() == '.' && isdigit(peek())) {
            isFloat = true;
            lexeme += current();
            advance();
            
            // Scan fractional part
            while (isdigit(current())) {
                lexeme += current();
                advance();
            }
        }
        
        // Check for exponent
        if (current() == 'e' || current() == 'E') {
            isFloat = true;
            lexeme += current();
            advance();
            
            // Check for sign
            if (current() == '+' || current() == '-') {
                lexeme += current();
                advance();
            }
            
            // Scan exponent
            if (!isdigit(current())) {
                errorHandler.reportError("Expected digits after exponent", filename, line, column);
                return Token(TokenType::Error, lexeme, filename, line, startColumn);
            }
            
            while (isdigit(current())) {
                lexeme += current();
                advance();
            }
        }
        
        // Create token
        if (isFloat) {
            try {
                double value = std::stod(lexeme);
                return Token(TokenType::FloatLiteral, value, lexeme, filename, line, startColumn);
            } catch (const std::invalid_argument&) {
                errorHandler.reportError("Invalid float literal", filename, line, startColumn);
                return Token(TokenType::Error, lexeme, filename, line, startColumn);
            } catch (const std::out_of_range&) {
                errorHandler.reportError("Float literal out of range", filename, line, startColumn);
                return Token(TokenType::Error, lexeme, filename, line, startColumn);
            }
        } else {
            try {
                int64_t value = std::stoll(lexeme);
                return Token(TokenType::IntegerLiteral, value, lexeme, filename, line, startColumn);
            } catch (const std::invalid_argument&) {
                errorHandler.reportError("Invalid integer literal", filename, line, startColumn);
                return Token(TokenType::Error, lexeme, filename, line, startColumn);
            } catch (const std::out_of_range&) {
                errorHandler.reportError("Integer literal out of range", filename, line, startColumn);
                return Token(TokenType::Error, lexeme, filename, line, startColumn);
            }
        }
    }
    
    /**
     * Scan a string literal
     */
    Token scanString() {
        int startColumn = column;
        std::string lexeme = "\"";
        std::string value;
        
        advance(); // Skip opening quote
        
        while (current() != '"' && current() != '\0') {
            if (current() == '\\') {
                lexeme += current();
                advance();
                
                switch (current()) {
                    case '"': value += '"'; break;
                    case '\\': value += '\\'; break;
                    case 'n': value += '\n'; break;
                    case 't': value += '\t'; break;
                    case 'r': value += '\r'; break;
                    case '0': value += '\0'; break;
                    default:
                        errorHandler.reportError("Invalid escape sequence", filename, line, column);
                        return Token(TokenType::Error, lexeme, filename, line, startColumn);
                }
            } else {
                value += current();
            }
            
            lexeme += current();
            advance();
        }
        
        if (current() == '\0') {
            errorHandler.reportError("Unterminated string literal", filename, line, startColumn);
            return Token(TokenType::Error, lexeme, filename, line, startColumn);
        }
        
        lexeme += '"';
        advance(); // Skip closing quote
        
        return Token(TokenType::StringLiteral, value, lexeme, filename, line, startColumn);
    }
    
    /**
     * Scan a single token
     */
    Token scanToken() {
        skipWhitespace();
        
        if (current() == '\0') {
            return Token(TokenType::EndOfFile, "", filename, line, column);
        }
        
        // Check for comments
        if (current() == '/' && (peek() == '/' || peek() == '*')) {
            skipComments();
            return scanToken();
        }
        
        int startColumn = column;
        char c = current();
        
        // Scan based on the first character
        if (isalpha(c) || c == '_') {
            return scanIdentifier();
        } else if (isdigit(c)) {
            return scanNumber();
        } else if (c == '"') {
            return scanString();
        }
        
        // Single-character tokens
        advance();
        switch (c) {
            case '(': return Token(TokenType::LeftParen, "(", filename, line, startColumn);
            case ')': return Token(TokenType::RightParen, ")", filename, line, startColumn);
            case '{': return Token(TokenType::LeftBrace, "{", filename, line, startColumn);
            case '}': return Token(TokenType::RightBrace, "}", filename, line, startColumn);
            case '[': return Token(TokenType::LeftBracket, "[", filename, line, startColumn);
            case ']': return Token(TokenType::RightBracket, "]", filename, line, startColumn);
            case ';': return Token(TokenType::Semicolon, ";", filename, line, startColumn);
            case ':': return Token(TokenType::Colon, ":", filename, line, startColumn);
            case ',': return Token(TokenType::Comma, ",", filename, line, startColumn);
            case '@': return Token(TokenType::At, "@", filename, line, startColumn);
            
            // Operators that could be part of multi-character operators
            case '+':
                if (match('=')) {
                    return Token(TokenType::PlusAssign, "+=", filename, line, startColumn);
                }
                return Token(TokenType::Plus, "+", filename, line, startColumn);
                
            case '-':
                if (match('>')) {
                    return Token(TokenType::Arrow, "->", filename, line, startColumn);
                } else if (match('=')) {
                    return Token(TokenType::MinusAssign, "-=", filename, line, startColumn);
                }
                return Token(TokenType::Minus, "-", filename, line, startColumn);
                
            case '*':
                if (match('=')) {
                    return Token(TokenType::MultiplyAssign, "*=", filename, line, startColumn);
                }
                return Token(TokenType::Multiply, "*", filename, line, startColumn);
                
            case '/':
                if (match('=')) {
                    return Token(TokenType::DivideAssign, "/=", filename, line, startColumn);
                }
                return Token(TokenType::Divide, "/", filename, line, startColumn);
                
            case '%':
                if (match('=')) {
                    return Token(TokenType::ModuloAssign, "%=", filename, line, startColumn);
                }
                return Token(TokenType::Modulo, "%", filename, line, startColumn);
                
            case '=':
                if (match('=')) {
                    return Token(TokenType::Equal, "==", filename, line, startColumn);
                }
                return Token(TokenType::Assign, "=", filename, line, startColumn);
                
            case '!':
                if (match('=')) {
                    return Token(TokenType::NotEqual, "!=", filename, line, startColumn);
                }
                return Token(TokenType::Not, "!", filename, line, startColumn);
                
            case '<':
                if (match('=')) {
                    return Token(TokenType::LessEqual, "<=", filename, line, startColumn);
                } else if (match('<')) {
                    if (match('=')) {
                        return Token(TokenType::LeftShiftAssign, "<<=", filename, line, startColumn);
                    }
                    return Token(TokenType::LeftShift, "<<", filename, line, startColumn);
                }
                return Token(TokenType::Less, "<", filename, line, startColumn);
                
            case '>':
                if (match('=')) {
                    return Token(TokenType::GreaterEqual, ">=", filename, line, startColumn);
                } else if (match('>')) {
                    if (match('=')) {
                        return Token(TokenType::RightShiftAssign, ">>=", filename, line, startColumn);
                    }
                    return Token(TokenType::RightShift, ">>", filename, line, startColumn);
                }
                return Token(TokenType::Greater, ">", filename, line, startColumn);
                
            case '&':
                if (match('&')) {
                    return Token(TokenType::And, "&&", filename, line, startColumn);
                } else if (match('=')) {
                    return Token(TokenType::AndAssign, "&=", filename, line, startColumn);
                }
                return Token(TokenType::BitwiseAnd, "&", filename, line, startColumn);
                
            case '|':
                if (match('|')) {
                    return Token(TokenType::Or, "||", filename, line, startColumn);
                } else if (match('=')) {
                    return Token(TokenType::OrAssign, "|=", filename, line, startColumn);
                }
                return Token(TokenType::BitwiseOr, "|", filename, line, startColumn);
                
            case '^':
                if (match('=')) {
                    return Token(TokenType::XorAssign, "^=", filename, line, startColumn);
                }
                return Token(TokenType::BitwiseXor, "^", filename, line, startColumn);
                
            case '~':
                return Token(TokenType::BitwiseNot, "~", filename, line, startColumn);
                
            case '.':
                if (match('.')) {
                    return Token(TokenType::Range, "..", filename, line, startColumn);
                }
                return Token(TokenType::Dot, ".", filename, line, startColumn);
                
            default:
                errorHandler.reportError("Unexpected character: " + std::string(1, c), 
                                        filename, line, startColumn);
                return Token(TokenType::Error, std::string(1, c), filename, line, startColumn);
        }
    }
    
public:
    /**
     * Constructor
     */
    Lexer(const std::string& src, const std::string& fname, ErrorHandler& errHandler)
        : source(src), filename(fname), errorHandler(errHandler) {}
    
    /**
     * Tokenize the entire source code
     */
    std::vector<Token> tokenize() {
        std::vector<Token> tokens;
        
        while (true) {
            Token token = scanToken();
            tokens.push_back(token);
            
            if (token.type == TokenType::EndOfFile) {
                break;
            }
        }
        
        return tokens;
    }
};

} // namespace astra

#endif // ASTRA_LEXER_H