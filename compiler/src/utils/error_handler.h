/**
 * ASTRA Programming Language Compiler
 * Error handling
 */

#ifndef ASTRA_ERROR_HANDLER_H
#define ASTRA_ERROR_HANDLER_H

#include <string>
#include <vector>
#include <iostream>

namespace astra {

/**
 * Severity level for diagnostics
 */
enum class DiagnosticSeverity {
    Info,
    Warning,
    Error,
    Fatal
};

/**
 * Diagnostic message
 */
struct Diagnostic {
    DiagnosticSeverity severity;
    std::string message;
    std::string filename;
    int line;
    int column;
    
    Diagnostic(DiagnosticSeverity sev, const std::string& msg, 
               const std::string& file = "", int ln = 0, int col = 0)
        : severity(sev), message(msg), filename(file), line(ln), column(col) {}
};

/**
 * Error handler for compiler diagnostics
 */
class ErrorHandler {
private:
    std::vector<Diagnostic> diagnostics;
    int errorCount = 0;
    int warningCount = 0;
    
public:
    ErrorHandler() = default;
    
    /**
     * Report an error
     */
    void reportError(const std::string& message, 
                    const std::string& filename = "", 
                    int line = 0, 
                    int column = 0) {
        Diagnostic diag(DiagnosticSeverity::Error, message, filename, line, column);
        diagnostics.push_back(diag);
        errorCount++;
        
        // Print error immediately
        std::cerr << formatDiagnostic(diag) << std::endl;
    }
    
    /**
     * Report a warning
     */
    void reportWarning(const std::string& message, 
                      const std::string& filename = "", 
                      int line = 0, 
                      int column = 0) {
        Diagnostic diag(DiagnosticSeverity::Warning, message, filename, line, column);
        diagnostics.push_back(diag);
        warningCount++;
        
        // Print warning immediately
        std::cerr << formatDiagnostic(diag) << std::endl;
    }
    
    /**
     * Report an informational message
     */
    void reportInfo(const std::string& message, 
                   const std::string& filename = "", 
                   int line = 0, 
                   int column = 0) {
        Diagnostic diag(DiagnosticSeverity::Info, message, filename, line, column);
        diagnostics.push_back(diag);
        
        // Print info immediately
        std::cout << formatDiagnostic(diag) << std::endl;
    }
    
    /**
     * Report a fatal error and exit
     */
    void reportFatal(const std::string& message, 
                    const std::string& filename = "", 
                    int line = 0, 
                    int column = 0) {
        Diagnostic diag(DiagnosticSeverity::Fatal, message, filename, line, column);
        diagnostics.push_back(diag);
        errorCount++;
        
        // Print fatal error immediately
        std::cerr << formatDiagnostic(diag) << std::endl;
        
        // Exit with error code
        exit(1);
    }
    
    /**
     * Check if there are any errors
     */
    bool hasErrors() const {
        return errorCount > 0;
    }
    
    /**
     * Get the number of errors
     */
    int getErrorCount() const {
        return errorCount;
    }
    
    /**
     * Get the number of warnings
     */
    int getWarningCount() const {
        return warningCount;
    }
    
    /**
     * Get all diagnostics
     */
    const std::vector<Diagnostic>& getDiagnostics() const {
        return diagnostics;
    }
    
    /**
     * Format a diagnostic message
     */
    std::string formatDiagnostic(const Diagnostic& diag) const {
        std::string result;
        
        // Add file location if available
        if (!diag.filename.empty()) {
            result += diag.filename;
            if (diag.line > 0) {
                result += ":" + std::to_string(diag.line);
                if (diag.column > 0) {
                    result += ":" + std::to_string(diag.column);
                }
            }
            result += ": ";
        }
        
        // Add severity
        switch (diag.severity) {
            case DiagnosticSeverity::Info:
                result += "info: ";
                break;
            case DiagnosticSeverity::Warning:
                result += "warning: ";
                break;
            case DiagnosticSeverity::Error:
                result += "error: ";
                break;
            case DiagnosticSeverity::Fatal:
                result += "fatal error: ";
                break;
        }
        
        // Add message
        result += diag.message;
        
        return result;
    }
};

} // namespace astra

#endif // ASTRA_ERROR_HANDLER_H