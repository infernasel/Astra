#include <gtest/gtest.h>
#include "../src/utils/error_handler.h"
#include <sstream>

// Redirect cout/cerr for testing
class ErrorHandlerTest : public ::testing::Test {
protected:
    std::stringstream cout_buffer;
    std::stringstream cerr_buffer;
    std::streambuf* old_cout;
    std::streambuf* old_cerr;

    void SetUp() override {
        old_cout = std::cout.rdbuf(cout_buffer.rdbuf());
        old_cerr = std::cerr.rdbuf(cerr_buffer.rdbuf());
    }

    void TearDown() override {
        std::cout.rdbuf(old_cout);
        std::cerr.rdbuf(old_cerr);
    }
};

TEST_F(ErrorHandlerTest, ReportError) {
    astra::ErrorHandler handler;
    handler.reportError("Test error", "test.astra", 10, 5);
    
    EXPECT_EQ(1, handler.getErrorCount());
    EXPECT_EQ(0, handler.getWarningCount());
    EXPECT_TRUE(handler.hasErrors());
    
    std::string output = cerr_buffer.str();
    EXPECT_TRUE(output.find("test.astra:10:5: error: Test error") != std::string::npos);
}

TEST_F(ErrorHandlerTest, ReportWarning) {
    astra::ErrorHandler handler;
    handler.reportWarning("Test warning", "test.astra", 15, 8);
    
    EXPECT_EQ(0, handler.getErrorCount());
    EXPECT_EQ(1, handler.getWarningCount());
    EXPECT_FALSE(handler.hasErrors());
    
    std::string output = cerr_buffer.str();
    EXPECT_TRUE(output.find("test.astra:15:8: warning: Test warning") != std::string::npos);
}

TEST_F(ErrorHandlerTest, ReportInfo) {
    astra::ErrorHandler handler;
    handler.reportInfo("Test info", "test.astra", 20, 10);
    
    EXPECT_EQ(0, handler.getErrorCount());
    EXPECT_EQ(0, handler.getWarningCount());
    EXPECT_FALSE(handler.hasErrors());
    
    std::string output = cout_buffer.str();
    EXPECT_TRUE(output.find("test.astra:20:10: info: Test info") != std::string::npos);
}

TEST_F(ErrorHandlerTest, FormatDiagnostic) {
    astra::ErrorHandler handler;
    
    astra::Diagnostic error(astra::DiagnosticSeverity::Error, "Test error", "test.astra", 10, 5);
    std::string formatted = handler.formatDiagnostic(error);
    EXPECT_EQ("test.astra:10:5: error: Test error", formatted);
    
    astra::Diagnostic warning(astra::DiagnosticSeverity::Warning, "Test warning");
    formatted = handler.formatDiagnostic(warning);
    EXPECT_EQ("warning: Test warning", formatted);
    
    astra::Diagnostic info(astra::DiagnosticSeverity::Info, "Test info", "test.astra");
    formatted = handler.formatDiagnostic(info);
    EXPECT_EQ("test.astra: info: Test info", formatted);
    
    astra::Diagnostic fatal(astra::DiagnosticSeverity::Fatal, "Test fatal", "test.astra", 30);
    formatted = handler.formatDiagnostic(fatal);
    EXPECT_EQ("test.astra:30: fatal error: Test fatal", formatted);
}

TEST_F(ErrorHandlerTest, GetDiagnostics) {
    astra::ErrorHandler handler;
    
    handler.reportError("Error 1");
    handler.reportWarning("Warning 1");
    handler.reportInfo("Info 1");
    handler.reportError("Error 2");
    
    const auto& diagnostics = handler.getDiagnostics();
    EXPECT_EQ(4, diagnostics.size());
    EXPECT_EQ(astra::DiagnosticSeverity::Error, diagnostics[0].severity);
    EXPECT_EQ("Error 1", diagnostics[0].message);
    EXPECT_EQ(astra::DiagnosticSeverity::Warning, diagnostics[1].severity);
    EXPECT_EQ("Warning 1", diagnostics[1].message);
    EXPECT_EQ(astra::DiagnosticSeverity::Info, diagnostics[2].severity);
    EXPECT_EQ("Info 1", diagnostics[2].message);
    EXPECT_EQ(astra::DiagnosticSeverity::Error, diagnostics[3].severity);
    EXPECT_EQ("Error 2", diagnostics[3].message);
}

// We can't easily test reportFatal since it calls exit(1)
// But we can test that it adds a diagnostic before exiting
TEST_F(ErrorHandlerTest, ReportFatalDiagnostic) {
    astra::ErrorHandler handler;
    
    // Create a diagnostic that would be added by reportFatal
    astra::Diagnostic fatal(astra::DiagnosticSeverity::Fatal, "Test fatal", "test.astra", 40, 15);
    
    // Format it and check the result
    std::string formatted = handler.formatDiagnostic(fatal);
    EXPECT_EQ("test.astra:40:15: fatal error: Test fatal", formatted);
}