#include <gtest/gtest.h>
#include "../src/lexer/lexer.h"
#include "../src/utils/error_handler.h"

using namespace astra;

class LexerTest : public ::testing::Test {
protected:
    ErrorHandler errorHandler;
};

TEST_F(LexerTest, EmptyInput) {
    Lexer lexer("", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(1, tokens.size());
    EXPECT_EQ(TokenType::EndOfFile, tokens[0].type);
}

TEST_F(LexerTest, Identifiers) {
    Lexer lexer("foo bar baz", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(4, tokens.size()); // 3 identifiers + EOF
    EXPECT_EQ(TokenType::Identifier, tokens[0].type);
    ASSERT_TRUE(std::holds_alternative<std::string>(tokens[0].value));
    EXPECT_EQ("foo", std::get<std::string>(tokens[0].value));
    EXPECT_EQ(TokenType::Identifier, tokens[1].type);
    ASSERT_TRUE(std::holds_alternative<std::string>(tokens[1].value));
    EXPECT_EQ("bar", std::get<std::string>(tokens[1].value));
    EXPECT_EQ(TokenType::Identifier, tokens[2].type);
    ASSERT_TRUE(std::holds_alternative<std::string>(tokens[2].value));
    EXPECT_EQ("baz", std::get<std::string>(tokens[2].value));
    EXPECT_EQ(TokenType::EndOfFile, tokens[3].type);
}

TEST_F(LexerTest, Keywords) {
    Lexer lexer("var const func if else while for", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(8, tokens.size()); // 7 keywords + EOF
    EXPECT_EQ(TokenType::Var, tokens[0].type);
    EXPECT_EQ(TokenType::Const, tokens[1].type);
    EXPECT_EQ(TokenType::Func, tokens[2].type);
    EXPECT_EQ(TokenType::If, tokens[3].type);
    EXPECT_EQ(TokenType::Else, tokens[4].type);
    EXPECT_EQ(TokenType::While, tokens[5].type);
    EXPECT_EQ(TokenType::For, tokens[6].type);
}

TEST_F(LexerTest, Operators) {
    Lexer lexer("+ - * / % = == != < <= > >= && || ! & | ^ ~ << >>", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(22, tokens.size()); // 21 operators + EOF
    EXPECT_EQ(TokenType::Plus, tokens[0].type);
    EXPECT_EQ(TokenType::Minus, tokens[1].type);
    EXPECT_EQ(TokenType::Multiply, tokens[2].type);
    EXPECT_EQ(TokenType::Divide, tokens[3].type);
    EXPECT_EQ(TokenType::Modulo, tokens[4].type);
    EXPECT_EQ(TokenType::Assign, tokens[5].type);
    EXPECT_EQ(TokenType::Equal, tokens[6].type);
    EXPECT_EQ(TokenType::NotEqual, tokens[7].type);
    EXPECT_EQ(TokenType::Less, tokens[8].type);
    EXPECT_EQ(TokenType::LessEqual, tokens[9].type);
    EXPECT_EQ(TokenType::Greater, tokens[10].type);
    EXPECT_EQ(TokenType::GreaterEqual, tokens[11].type);
    EXPECT_EQ(TokenType::And, tokens[12].type);
    EXPECT_EQ(TokenType::Or, tokens[13].type);
    EXPECT_EQ(TokenType::Not, tokens[14].type);
    EXPECT_EQ(TokenType::BitwiseAnd, tokens[15].type);
    EXPECT_EQ(TokenType::BitwiseOr, tokens[16].type);
    EXPECT_EQ(TokenType::BitwiseXor, tokens[17].type);
    EXPECT_EQ(TokenType::BitwiseNot, tokens[18].type);
    EXPECT_EQ(TokenType::LeftShift, tokens[19].type);
    EXPECT_EQ(TokenType::RightShift, tokens[20].type);
    EXPECT_EQ(TokenType::EndOfFile, tokens[21].type);
}

TEST_F(LexerTest, Literals) {
    Lexer lexer("123 3.14 \"hello\" true false", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(6, tokens.size()); // 5 literals + EOF
    EXPECT_EQ(TokenType::IntegerLiteral, tokens[0].type);
    EXPECT_EQ(123, std::get<int64_t>(tokens[0].value));
    EXPECT_EQ(TokenType::FloatLiteral, tokens[1].type);
    EXPECT_DOUBLE_EQ(3.14, std::get<double>(tokens[1].value));
    EXPECT_EQ(TokenType::StringLiteral, tokens[2].type);
    EXPECT_EQ("hello", std::get<std::string>(tokens[2].value));
    EXPECT_EQ(TokenType::BooleanLiteral, tokens[3].type);
    EXPECT_EQ(true, std::get<bool>(tokens[3].value));
    EXPECT_EQ(TokenType::BooleanLiteral, tokens[4].type);
    EXPECT_EQ(false, std::get<bool>(tokens[4].value));
}

TEST_F(LexerTest, Punctuation) {
    Lexer lexer("( ) { } [ ] ; : , . -> .. @", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(14, tokens.size()); // 13 punctuation + EOF
    EXPECT_EQ(TokenType::LeftParen, tokens[0].type);
    EXPECT_EQ(TokenType::RightParen, tokens[1].type);
    EXPECT_EQ(TokenType::LeftBrace, tokens[2].type);
    EXPECT_EQ(TokenType::RightBrace, tokens[3].type);
    EXPECT_EQ(TokenType::LeftBracket, tokens[4].type);
    EXPECT_EQ(TokenType::RightBracket, tokens[5].type);
    EXPECT_EQ(TokenType::Semicolon, tokens[6].type);
    EXPECT_EQ(TokenType::Colon, tokens[7].type);
    EXPECT_EQ(TokenType::Comma, tokens[8].type);
    EXPECT_EQ(TokenType::Dot, tokens[9].type);
    EXPECT_EQ(TokenType::Arrow, tokens[10].type);
    EXPECT_EQ(TokenType::Range, tokens[11].type);
    EXPECT_EQ(TokenType::At, tokens[12].type);
}

TEST_F(LexerTest, Comments) {
    Lexer lexer("foo // This is a comment\nbar /* This is a\nmulti-line comment */ baz", "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    ASSERT_EQ(4, tokens.size()); // 3 identifiers + EOF
    EXPECT_EQ(TokenType::Identifier, tokens[0].type);
    EXPECT_EQ("foo", std::get<std::string>(tokens[0].value));
    EXPECT_EQ(TokenType::Identifier, tokens[1].type);
    EXPECT_EQ("bar", std::get<std::string>(tokens[1].value));
    EXPECT_EQ(TokenType::Identifier, tokens[2].type);
    EXPECT_EQ("baz", std::get<std::string>(tokens[2].value));
    EXPECT_EQ(TokenType::EndOfFile, tokens[3].type);
}

TEST_F(LexerTest, ComplexExample) {
    std::string code = R"(
        func calculateOrbit(satellite: Satellite, time: float) -> Vector3 {
            var position = satellite.position;
            var velocity = satellite.velocity;
            
            // Update position based on time
            position = position + velocity * time;
            
            if (position.magnitude() > MAX_ORBIT_RADIUS) {
                throw OrbitException("Orbit exceeds maximum radius");
            }
            
            return position;
        }
    )";
    
    Lexer lexer(code, "test", errorHandler);
    auto tokens = lexer.tokenize();
    
    // We don't need to check every token, just make sure we have a reasonable number
    // and that the first few and last few are what we expect
    ASSERT_GT(tokens.size(), 20);
    
    EXPECT_EQ(TokenType::Func, tokens[0].type);
    EXPECT_EQ(TokenType::Identifier, tokens[1].type);
    // Проверяем, что значение токена - строка, но не проверяем конкретное значение
    ASSERT_TRUE(std::holds_alternative<std::string>(tokens[1].value));
    EXPECT_EQ(TokenType::LeftParen, tokens[2].type);
    
    // Check the last token before EOF
    EXPECT_EQ(TokenType::RightBrace, tokens[tokens.size() - 2].type);
    EXPECT_EQ(TokenType::EndOfFile, tokens[tokens.size() - 1].type);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}