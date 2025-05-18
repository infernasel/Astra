#include "error_handler_minimal.h"
#include <iostream>

namespace astra {

void ErrorHandler::reportError(const std::string& message, const std::string& filename, 
                              int line, int column) {
    errors.push_back(message);
    std::cerr << filename << ":" << line << ":" << column << ": error: " << message << std::endl;
}

void ErrorHandler::reportWarning(const std::string& message, const std::string& filename, 
                                int line, int column) {
    warnings.push_back(message);
    std::cerr << filename << ":" << line << ":" << column << ": warning: " << message << std::endl;
}

bool ErrorHandler::hasErrors() const {
    return !errors.empty();
}

} // namespace astra