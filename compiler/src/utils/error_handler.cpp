#include "error_handler.h"
#include <iostream>

namespace astra {

void ErrorHandler::reportError(const std::string& message, const std::string& filename, 
                              int line, int column) {
    errors.push_back({message, filename, line, column});
    std::cerr << filename << ":" << line << ":" << column << ": error: " << message << std::endl;
}

void ErrorHandler::reportWarning(const std::string& message, const std::string& filename, 
                                int line, int column) {
    warnings.push_back({message, filename, line, column});
    std::cerr << filename << ":" << line << ":" << column << ": warning: " << message << std::endl;
}

void ErrorHandler::reportNote(const std::string& message, const std::string& filename, 
                             int line, int column) {
    notes.push_back({message, filename, line, column});
    std::cerr << filename << ":" << line << ":" << column << ": note: " << message << std::endl;
}

bool ErrorHandler::hasErrors() const {
    return !errors.empty();
}

bool ErrorHandler::hasWarnings() const {
    return !warnings.empty();
}

const std::vector<ErrorHandler::Diagnostic>& ErrorHandler::getErrors() const {
    return errors;
}

const std::vector<ErrorHandler::Diagnostic>& ErrorHandler::getWarnings() const {
    return warnings;
}

const std::vector<ErrorHandler::Diagnostic>& ErrorHandler::getNotes() const {
    return notes;
}

void ErrorHandler::clear() {
    errors.clear();
    warnings.clear();
    notes.clear();
}

} // namespace astra