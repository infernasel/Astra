#include "lexer.h"
#include <cctype>
#include <unordered_map>

namespace astra {

Lexer::Lexer(const std::string& source, const std::string& filename, ErrorHandler& errorHandler)
    : source(source), filename(filename), errorHandler(errorHandler), 
      currentPosition(0), currentLine(1), currentColumn(1) {}

std::vector<Token> Lexer::tokenize() {
    std::vector<Token> tokens;
    
    while (currentPosition < source.length()) {
        char c = source[currentPosition];
        
        if (std::isspace(c)) {
            // Handle whitespace
            if (c == '\n') {
                currentLine++;
                currentColumn = 1;
            } else {
                currentColumn++;
            }
            currentPosition++;
        } else if (std::isalpha(c) || c == '_') {
            // Handle identifiers and keywords
            tokens.push_back(scanIdentifier());
        } else if (std::isdigit(c)) {
            // Handle numeric literals
            tokens.push_back(scanNumber());
        } else if (c == '"') {
            // Handle string literals
            tokens.push_back(scanString());
        } else if (c == '/' && currentPosition + 1 < source.length() && 
                  (source[currentPosition + 1] == '/' || source[currentPosition + 1] == '*')) {
            // Handle comments
            scanComment();
        } else {
            // Handle operators and punctuation
            tokens.push_back(scanOperator());
        }
    }
    
    // Add EOF token
    tokens.push_back(Token(TokenType::EndOfFile, "", filename, currentLine, currentColumn));
    
    return tokens;
}

Token Lexer::scanIdentifier() {
    size_t start = currentPosition;
    size_t startColumn = currentColumn;
    
    while (currentPosition < source.length() && 
           (std::isalnum(source[currentPosition]) || source[currentPosition] == '_')) {
        currentPosition++;
        currentColumn++;
    }
    
    std::string lexeme = source.substr(start, currentPosition - start);
    
    // Check if it's a keyword
    static const std::unordered_map<std::string, TokenType> keywords = {
        {"module", TokenType::Module},
        {"import", TokenType::Import},
        {"function", TokenType::Function},
        {"task", TokenType::Task},
        {"var", TokenType::Var},
        {"const", TokenType::Const},
        {"if", TokenType::If},
        {"else", TokenType::Else},
        {"while", TokenType::While},
        {"for", TokenType::For},
        {"return", TokenType::Return},
        {"break", TokenType::Break},
        {"continue", TokenType::Continue},
        {"true", TokenType::BooleanLiteral},
        {"false", TokenType::BooleanLiteral},
        {"try", TokenType::Try},
        {"catch", TokenType::Catch},
        {"throw", TokenType::Throw},
        {"int", TokenType::TypeInt},
        {"float", TokenType::TypeFloat},
        {"bool", TokenType::TypeBool},
        {"string", TokenType::TypeString},
        {"void", TokenType::TypeVoid},
        {"array", TokenType::TypeArray},
        {"range", TokenType::TypeRange}
    };
    
    auto it = keywords.find(lexeme);
    if (it != keywords.end()) {
        return Token(it->second, lexeme, filename, currentLine, startColumn);
    }
    
    return Token(TokenType::Identifier, lexeme, filename, currentLine, startColumn);
}

Token Lexer::scanNumber() {
    size_t start = currentPosition;
    size_t startColumn = currentColumn;
    bool isFloat = false;
    
    while (currentPosition < source.length() && std::isdigit(source[currentPosition])) {
        currentPosition++;
        currentColumn++;
    }
    
    // Check for decimal point
    if (currentPosition < source.length() && source[currentPosition] == '.') {
        isFloat = true;
        currentPosition++;
        currentColumn++;
        
        while (currentPosition < source.length() && std::isdigit(source[currentPosition])) {
            currentPosition++;
            currentColumn++;
        }
    }
    
    // Check for exponent
    if (currentPosition < source.length() && (source[currentPosition] == 'e' || source[currentPosition] == 'E')) {
        isFloat = true;
        currentPosition++;
        currentColumn++;
        
        // Optional sign
        if (currentPosition < source.length() && (source[currentPosition] == '+' || source[currentPosition] == '-')) {
            currentPosition++;
            currentColumn++;
        }
        
        // Must have at least one digit after exponent
        if (currentPosition < source.length() && std::isdigit(source[currentPosition])) {
            while (currentPosition < source.length() && std::isdigit(source[currentPosition])) {
                currentPosition++;
                currentColumn++;
            }
        } else {
            errorHandler.reportError("Invalid floating point literal: expected digits after exponent", 
                                    filename, currentLine, currentColumn);
        }
    }
    
    std::string lexeme = source.substr(start, currentPosition - start);
    
    if (isFloat) {
        return Token(TokenType::FloatLiteral, lexeme, filename, currentLine, startColumn);
    } else {
        return Token(TokenType::IntegerLiteral, lexeme, filename, currentLine, startColumn);
    }
}

Token Lexer::scanString() {
    size_t start = currentPosition;
    size_t startColumn = currentColumn;
    
    // Skip opening quote
    currentPosition++;
    currentColumn++;
    
    while (currentPosition < source.length() && source[currentPosition] != '"') {
        // Handle escape sequences
        if (source[currentPosition] == '\\' && currentPosition + 1 < source.length()) {
            currentPosition += 2;
            currentColumn += 2;
        } else {
            if (source[currentPosition] == '\n') {
                currentLine++;
                currentColumn = 1;
            } else {
                currentColumn++;
            }
            currentPosition++;
        }
    }
    
    if (currentPosition >= source.length()) {
        errorHandler.reportError("Unterminated string literal", filename, currentLine, startColumn);
        return Token(TokenType::StringLiteral, source.substr(start + 1, currentPosition - start - 1), 
                    filename, currentLine, startColumn);
    }
    
    // Skip closing quote
    currentPosition++;
    currentColumn++;
    
    // Extract the string without the quotes
    std::string lexeme = source.substr(start + 1, currentPosition - start - 2);
    
    return Token(TokenType::StringLiteral, lexeme, filename, currentLine, startColumn);
}

void Lexer::scanComment() {
    if (source[currentPosition + 1] == '/') {
        // Single-line comment
        currentPosition += 2;
        currentColumn += 2;
        
        while (currentPosition < source.length() && source[currentPosition] != '\n') {
            currentPosition++;
            currentColumn++;
        }
    } else {
        // Multi-line comment
        currentPosition += 2;
        currentColumn += 2;
        
        while (currentPosition < source.length() && 
              !(source[currentPosition] == '*' && 
                currentPosition + 1 < source.length() && 
                source[currentPosition + 1] == '/')) {
            
            if (source[currentPosition] == '\n') {
                currentLine++;
                currentColumn = 1;
            } else {
                currentColumn++;
            }
            currentPosition++;
        }
        
        if (currentPosition >= source.length()) {
            errorHandler.reportError("Unterminated multi-line comment", filename, currentLine, currentColumn);
            return;
        }
        
        // Skip closing */
        currentPosition += 2;
        currentColumn += 2;
    }
}

Token Lexer::scanOperator() {
    size_t startColumn = currentColumn;
    char c = source[currentPosition];
    currentPosition++;
    currentColumn++;
    
    TokenType type;
    std::string lexeme(1, c);
    
    switch (c) {
        case '+': type = TokenType::Plus; break;
        case '-': type = TokenType::Minus; break;
        case '*': type = TokenType::Multiply; break;
        case '/': type = TokenType::Divide; break;
        case '%': type = TokenType::Modulo; break;
        case '=': 
            if (currentPosition < source.length() && source[currentPosition] == '=') {
                type = TokenType::Equal;
                lexeme += '=';
                currentPosition++;
                currentColumn++;
            } else {
                type = TokenType::Assign;
            }
            break;
        case '!': 
            if (currentPosition < source.length() && source[currentPosition] == '=') {
                type = TokenType::NotEqual;
                lexeme += '=';
                currentPosition++;
                currentColumn++;
            } else {
                type = TokenType::Not;
            }
            break;
        case '<': 
            if (currentPosition < source.length() && source[currentPosition] == '=') {
                type = TokenType::LessEqual;
                lexeme += '=';
                currentPosition++;
                currentColumn++;
            } else {
                type = TokenType::Less;
            }
            break;
        case '>': 
            if (currentPosition < source.length() && source[currentPosition] == '=') {
                type = TokenType::GreaterEqual;
                lexeme += '=';
                currentPosition++;
                currentColumn++;
            } else {
                type = TokenType::Greater;
            }
            break;
        case '&': 
            if (currentPosition < source.length() && source[currentPosition] == '&') {
                type = TokenType::And;
                lexeme += '&';
                currentPosition++;
                currentColumn++;
            } else {
                type = TokenType::BitwiseAnd;
            }
            break;
        case '|': 
            if (currentPosition < source.length() && source[currentPosition] == '|') {
                type = TokenType::Or;
                lexeme += '|';
                currentPosition++;
                currentColumn++;
            } else {
                type = TokenType::BitwiseOr;
            }
            break;
        case '^': type = TokenType::BitwiseXor; break;
        case '~': type = TokenType::BitwiseNot; break;
        case '(': type = TokenType::LeftParen; break;
        case ')': type = TokenType::RightParen; break;
        case '[': type = TokenType::LeftBracket; break;
        case ']': type = TokenType::RightBracket; break;
        case '{': type = TokenType::LeftBrace; break;
        case '}': type = TokenType::RightBrace; break;
        case ',': type = TokenType::Comma; break;
        case '.': type = TokenType::Dot; break;
        case ':': type = TokenType::Colon; break;
        case ';': type = TokenType::Semicolon; break;
        case '@': type = TokenType::At; break;
        default:
            errorHandler.reportError("Unknown character: " + lexeme, filename, currentLine, startColumn);
            type = TokenType::Unknown;
            break;
    }
    
    return Token(type, lexeme, filename, currentLine, startColumn);
}

} // namespace astra