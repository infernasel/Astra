#ifndef ASTRA_ERROR_HANDLER_MINIMAL_H
#define ASTRA_ERROR_HANDLER_MINIMAL_H

#include <string>
#include <vector>

namespace astra {

/**
 * Error handler class for reporting errors, warnings, and notes
 */
class ErrorHandler {
public:
    /**
     * Report an error
     */
    void reportError(const std::string& message, const std::string& filename, int line, int column);
    
    /**
     * Report a warning
     */
    void reportWarning(const std::string& message, const std::string& filename, int line, int column);
    
    /**
     * Check if there are any errors
     */
    bool hasErrors() const;
    
private:
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

} // namespace astra

#endif // ASTRA_ERROR_HANDLER_MINIMAL_H