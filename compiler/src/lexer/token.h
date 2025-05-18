/**
 * ASTRA Programming Language Compiler
 * Token definitions
 */

#ifndef ASTRA_TOKEN_H
#define ASTRA_TOKEN_H

#include <string>
#include <variant>
#include <vector>
#include <unordered_map>

namespace astra {

/**
 * Token types
 */
enum class TokenType {
    // End of file
    EndOfFile,
    
    // Literals
    IntegerLiteral,
    FloatLiteral,
    StringLiteral,
    BooleanLiteral,
    
    // Identifiers
    Identifier,
    
    // Keywords
    Var,
    Const,
    Func,
    Return,
    If,
    Else,
    For,
    While,
    Loop,
    Break,
    Continue,
    Import,
    Module,
    Try,
    Catch,
    Finally,
    Throw,
    Task,
    With,
    Sleep,
    
    // Operators
    Plus,           // +
    Minus,          // -
    Multiply,       // *
    Divide,         // /
    Modulo,         // %
    Assign,         // =
    Equal,          // ==
    NotEqual,       // !=
    Less,           // <
    LessEqual,      // <=
    Greater,        // >
    GreaterEqual,   // >=
    And,            // &&
    Or,             // ||
    Not,            // !
    BitwiseAnd,     // &
    BitwiseOr,      // |
    BitwiseXor,     // ^
    BitwiseNot,     // ~
    LeftShift,      // <<
    RightShift,     // >>
    PlusAssign,     // +=
    MinusAssign,    // -=
    MultiplyAssign, // *=
    DivideAssign,   // /=
    ModuloAssign,   // %=
    AndAssign,      // &=
    OrAssign,       // |=
    XorAssign,      // ^=
    LeftShiftAssign,// <<=
    RightShiftAssign,// >>=
    
    // Punctuation
    LeftParen,      // (
    RightParen,     // )
    LeftBrace,      // {
    RightBrace,     // }
    LeftBracket,    // [
    RightBracket,   // ]
    Semicolon,      // ;
    Colon,          // :
    Comma,          // ,
    Dot,            // .
    Arrow,          // ->
    Range,          // ..
    At,             // @
    
    // Special
    Error
};

/**
 * Token value types
 */
using TokenValue = std::variant<
    std::monostate,  // No value
    int64_t,         // Integer
    double,          // Float
    std::string,     // String or identifier
    bool             // Boolean
>;

/**
 * Token structure
 */
struct Token {
    TokenType type;
    TokenValue value;
    std::string lexeme;
    std::string filename;
    int line;
    int column;
    
    Token(TokenType t, const std::string& l, const std::string& f, int ln, int col)
        : type(t), lexeme(l), filename(f), line(ln), column(col) {}
    
    Token(TokenType t, TokenValue v, const std::string& l, const std::string& f, int ln, int col)
        : type(t), value(v), lexeme(l), filename(f), line(ln), column(col) {}
};

/**
 * Keyword map
 */
const std::unordered_map<std::string, TokenType> KEYWORDS = {
    {"var", TokenType::Var},
    {"const", TokenType::Const},
    {"func", TokenType::Func},
    {"return", TokenType::Return},
    {"if", TokenType::If},
    {"else", TokenType::Else},
    {"for", TokenType::For},
    {"while", TokenType::While},
    {"loop", TokenType::Loop},
    {"break", TokenType::Break},
    {"continue", TokenType::Continue},
    {"import", TokenType::Import},
    {"module", TokenType::Module},
    {"try", TokenType::Try},
    {"catch", TokenType::Catch},
    {"finally", TokenType::Finally},
    {"throw", TokenType::Throw},
    {"task", TokenType::Task},
    {"with", TokenType::With},
    {"sleep", TokenType::Sleep},
    {"true", TokenType::BooleanLiteral},
    {"false", TokenType::BooleanLiteral}
};

/**
 * Get string representation of token type
 */
inline std::string tokenTypeToString(TokenType type) {
    switch (type) {
        case TokenType::EndOfFile: return "EOF";
        case TokenType::IntegerLiteral: return "INTEGER";
        case TokenType::FloatLiteral: return "FLOAT";
        case TokenType::StringLiteral: return "STRING";
        case TokenType::BooleanLiteral: return "BOOLEAN";
        case TokenType::Identifier: return "IDENTIFIER";
        case TokenType::Var: return "VAR";
        case TokenType::Const: return "CONST";
        case TokenType::Func: return "FUNC";
        case TokenType::Return: return "RETURN";
        case TokenType::If: return "IF";
        case TokenType::Else: return "ELSE";
        case TokenType::For: return "FOR";
        case TokenType::While: return "WHILE";
        case TokenType::Loop: return "LOOP";
        case TokenType::Break: return "BREAK";
        case TokenType::Continue: return "CONTINUE";
        case TokenType::Import: return "IMPORT";
        case TokenType::Module: return "MODULE";
        case TokenType::Try: return "TRY";
        case TokenType::Catch: return "CATCH";
        case TokenType::Finally: return "FINALLY";
        case TokenType::Throw: return "THROW";
        case TokenType::Task: return "TASK";
        case TokenType::With: return "WITH";
        case TokenType::Sleep: return "SLEEP";
        case TokenType::Plus: return "+";
        case TokenType::Minus: return "-";
        case TokenType::Multiply: return "*";
        case TokenType::Divide: return "/";
        case TokenType::Modulo: return "%";
        case TokenType::Assign: return "=";
        case TokenType::Equal: return "==";
        case TokenType::NotEqual: return "!=";
        case TokenType::Less: return "<";
        case TokenType::LessEqual: return "<=";
        case TokenType::Greater: return ">";
        case TokenType::GreaterEqual: return ">=";
        case TokenType::And: return "&&";
        case TokenType::Or: return "||";
        case TokenType::Not: return "!";
        case TokenType::BitwiseAnd: return "&";
        case TokenType::BitwiseOr: return "|";
        case TokenType::BitwiseXor: return "^";
        case TokenType::BitwiseNot: return "~";
        case TokenType::LeftShift: return "<<";
        case TokenType::RightShift: return ">>";
        case TokenType::PlusAssign: return "+=";
        case TokenType::MinusAssign: return "-=";
        case TokenType::MultiplyAssign: return "*=";
        case TokenType::DivideAssign: return "/=";
        case TokenType::ModuloAssign: return "%=";
        case TokenType::AndAssign: return "&=";
        case TokenType::OrAssign: return "|=";
        case TokenType::XorAssign: return "^=";
        case TokenType::LeftShiftAssign: return "<<=";
        case TokenType::RightShiftAssign: return ">>=";
        case TokenType::LeftParen: return "(";
        case TokenType::RightParen: return ")";
        case TokenType::LeftBrace: return "{";
        case TokenType::RightBrace: return "}";
        case TokenType::LeftBracket: return "[";
        case TokenType::RightBracket: return "]";
        case TokenType::Semicolon: return ";";
        case TokenType::Colon: return ":";
        case TokenType::Comma: return ",";
        case TokenType::Dot: return ".";
        case TokenType::Arrow: return "->";
        case TokenType::Range: return "..";
        case TokenType::At: return "@";
        case TokenType::Error: return "ERROR";
        default: return "UNKNOWN";
    }
}

} // namespace astra

#endif // ASTRA_TOKEN_H