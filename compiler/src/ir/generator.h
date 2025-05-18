/**
 * ASTRA Programming Language Compiler
 * Intermediate Representation (IR) generator
 */

#ifndef ASTRA_IR_GENERATOR_H
#define ASTRA_IR_GENERATOR_H

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include "../ast/ast.h"
#include "../utils/error_handler.h"

namespace astra {

/**
 * IR operation types
 */
enum class IROpType {
    // Constants
    ConstInt,
    ConstFloat,
    ConstString,
    ConstBool,
    
    // Memory operations
    Alloca,
    Load,
    Store,
    GetElementPtr,
    
    // Arithmetic operations
    Add,
    Sub,
    Mul,
    Div,
    Mod,
    Neg,
    
    // Bitwise operations
    And,
    Or,
    Xor,
    Not,
    Shl,
    Shr,
    
    // Comparison operations
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
    
    // Control flow
    Jump,
    Branch,
    Call,
    Return,
    Phi,
    
    // Other
    Cast,
    Nop
};

/**
 * IR type
 */
class IRType {
public:
    enum class Kind {
        Void,
        Int,
        Float,
        Bool,
        String,
        Array,
        Struct,
        Function,
        Pointer
    };
    
    Kind kind;
    std::shared_ptr<IRType> elementType; // For arrays and pointers
    std::vector<std::shared_ptr<IRType>> fieldTypes; // For structs
    std::vector<std::shared_ptr<IRType>> paramTypes; // For functions
    std::shared_ptr<IRType> returnType; // For functions
    size_t size; // Size in bytes
    
    IRType(Kind k, size_t s = 0) : kind(k), size(s) {}
    
    static std::shared_ptr<IRType> getVoidType() {
        return std::make_shared<IRType>(Kind::Void, 0);
    }
    
    static std::shared_ptr<IRType> getIntType(size_t bits = 32) {
        return std::make_shared<IRType>(Kind::Int, bits / 8);
    }
    
    static std::shared_ptr<IRType> getFloatType(size_t bits = 32) {
        return std::make_shared<IRType>(Kind::Float, bits / 8);
    }
    
    static std::shared_ptr<IRType> getBoolType() {
        return std::make_shared<IRType>(Kind::Bool, 1);
    }
    
    static std::shared_ptr<IRType> getStringType() {
        return std::make_shared<IRType>(Kind::String, 8); // Pointer size
    }
    
    static std::shared_ptr<IRType> getArrayType(std::shared_ptr<IRType> elemType, size_t count = 0) {
        auto type = std::make_shared<IRType>(Kind::Array, elemType->size * count);
        type->elementType = elemType;
        return type;
    }
    
    static std::shared_ptr<IRType> getStructType(const std::vector<std::shared_ptr<IRType>>& fields) {
        auto type = std::make_shared<IRType>(Kind::Struct);
        type->fieldTypes = fields;
        
        // Calculate size
        size_t totalSize = 0;
        for (const auto& field : fields) {
            totalSize += field->size;
        }
        type->size = totalSize;
        
        return type;
    }
    
    static std::shared_ptr<IRType> getFunctionType(
        std::shared_ptr<IRType> ret,
        const std::vector<std::shared_ptr<IRType>>& params) {
        auto type = std::make_shared<IRType>(Kind::Function);
        type->returnType = ret;
        type->paramTypes = params;
        return type;
    }
    
    static std::shared_ptr<IRType> getPointerType(std::shared_ptr<IRType> pointee) {
        auto type = std::make_shared<IRType>(Kind::Pointer, 8); // Pointer size
        type->elementType = pointee;
        return type;
    }
    
    bool isVoid() const { return kind == Kind::Void; }
    bool isInt() const { return kind == Kind::Int; }
    bool isFloat() const { return kind == Kind::Float; }
    bool isBool() const { return kind == Kind::Bool; }
    bool isString() const { return kind == Kind::String; }
    bool isArray() const { return kind == Kind::Array; }
    bool isStruct() const { return kind == Kind::Struct; }
    bool isFunction() const { return kind == Kind::Function; }
    bool isPointer() const { return kind == Kind::Pointer; }
    
    std::string toString() const {
        switch (kind) {
            case Kind::Void: return "void";
            case Kind::Int: return "i" + std::to_string(size * 8);
            case Kind::Float: return (size == 4) ? "float" : "double";
            case Kind::Bool: return "bool";
            case Kind::String: return "string";
            case Kind::Array: return "[" + std::to_string(size / elementType->size) + " x " + elementType->toString() + "]";
            case Kind::Pointer: return elementType->toString() + "*";
            case Kind::Function: {
                std::string result = returnType->toString() + "(";
                for (size_t i = 0; i < paramTypes.size(); ++i) {
                    if (i > 0) result += ", ";
                    result += paramTypes[i]->toString();
                }
                result += ")";
                return result;
            }
            case Kind::Struct: {
                std::string result = "{ ";
                for (size_t i = 0; i < fieldTypes.size(); ++i) {
                    if (i > 0) result += ", ";
                    result += fieldTypes[i]->toString();
                }
                result += " }";
                return result;
            }
            default: return "unknown";
        }
    }
};

/**
 * IR value
 */
class IRValue {
public:
    enum class Kind {
        Constant,
        Register,
        Global,
        Parameter,
        BasicBlock
    };
    
    Kind kind;
    std::shared_ptr<IRType> type;
    std::string name;
    
    IRValue(Kind k, std::shared_ptr<IRType> t, const std::string& n = "")
        : kind(k), type(t), name(n) {}
    
    virtual ~IRValue() = default;
    
    bool isConstant() const { return kind == Kind::Constant; }
    bool isRegister() const { return kind == Kind::Register; }
    bool isGlobal() const { return kind == Kind::Global; }
    bool isParameter() const { return kind == Kind::Parameter; }
    bool isBasicBlock() const { return kind == Kind::BasicBlock; }
    
    virtual std::string toString() const {
        return name.empty() ? "%" + std::to_string(reinterpret_cast<uintptr_t>(this)) : name;
    }
};

/**
 * IR constant value
 */
class IRConstant : public IRValue {
public:
    enum class ConstantKind {
        Int,
        Float,
        String,
        Bool,
        Null,
        Undef
    };
    
    ConstantKind constantKind;
    
    union {
        int64_t intValue;
        double floatValue;
        bool boolValue;
    };
    
    std::string stringValue; // For string constants
    
    IRConstant(int64_t value)
        : IRValue(Kind::Constant, IRType::getIntType(64)), constantKind(ConstantKind::Int) {
        intValue = value;
    }
    
    IRConstant(double value)
        : IRValue(Kind::Constant, IRType::getFloatType(64)), constantKind(ConstantKind::Float) {
        floatValue = value;
    }
    
    IRConstant(bool value)
        : IRValue(Kind::Constant, IRType::getBoolType()), constantKind(ConstantKind::Bool) {
        boolValue = value;
    }
    
    IRConstant(const std::string& value)
        : IRValue(Kind::Constant, IRType::getStringType()), constantKind(ConstantKind::String), stringValue(value) {}
    
    static IRConstant* getNull(std::shared_ptr<IRType> type) {
        auto constant = new IRConstant(static_cast<int64_t>(0));
        constant->constantKind = ConstantKind::Null;
        constant->type = IRType::getPointerType(type);
        return constant;
    }
    
    static IRConstant* getUndef(std::shared_ptr<IRType> type) {
        auto constant = new IRConstant(static_cast<int64_t>(0));
        constant->constantKind = ConstantKind::Undef;
        constant->type = type;
        return constant;
    }
    
    std::string toString() const override {
        switch (constantKind) {
            case ConstantKind::Int: return std::to_string(intValue);
            case ConstantKind::Float: return std::to_string(floatValue);
            case ConstantKind::Bool: return boolValue ? "true" : "false";
            case ConstantKind::String: return "\"" + stringValue + "\"";
            case ConstantKind::Null: return "null";
            case ConstantKind::Undef: return "undef";
            default: return "unknown";
        }
    }
};

/**
 * IR instruction
 */
class IRInstruction {
public:
    IROpType opType;
    std::shared_ptr<IRValue> result;
    std::vector<std::shared_ptr<IRValue>> operands;
    
    IRInstruction(IROpType op, std::shared_ptr<IRValue> res = nullptr)
        : opType(op), result(res) {}
    
    void addOperand(std::shared_ptr<IRValue> operand) {
        operands.push_back(operand);
    }
    
    std::string toString() const {
        std::string opStr;
        switch (opType) {
            case IROpType::ConstInt: opStr = "constint"; break;
            case IROpType::ConstFloat: opStr = "constfloat"; break;
            case IROpType::ConstString: opStr = "conststring"; break;
            case IROpType::ConstBool: opStr = "constbool"; break;
            case IROpType::Alloca: opStr = "alloca"; break;
            case IROpType::Load: opStr = "load"; break;
            case IROpType::Store: opStr = "store"; break;
            case IROpType::GetElementPtr: opStr = "getelementptr"; break;
            case IROpType::Add: opStr = "add"; break;
            case IROpType::Sub: opStr = "sub"; break;
            case IROpType::Mul: opStr = "mul"; break;
            case IROpType::Div: opStr = "div"; break;
            case IROpType::Mod: opStr = "mod"; break;
            case IROpType::Neg: opStr = "neg"; break;
            case IROpType::And: opStr = "and"; break;
            case IROpType::Or: opStr = "or"; break;
            case IROpType::Xor: opStr = "xor"; break;
            case IROpType::Not: opStr = "not"; break;
            case IROpType::Shl: opStr = "shl"; break;
            case IROpType::Shr: opStr = "shr"; break;
            case IROpType::Eq: opStr = "eq"; break;
            case IROpType::Ne: opStr = "ne"; break;
            case IROpType::Lt: opStr = "lt"; break;
            case IROpType::Le: opStr = "le"; break;
            case IROpType::Gt: opStr = "gt"; break;
            case IROpType::Ge: opStr = "ge"; break;
            case IROpType::Jump: opStr = "jump"; break;
            case IROpType::Branch: opStr = "branch"; break;
            case IROpType::Call: opStr = "call"; break;
            case IROpType::Return: opStr = "return"; break;
            case IROpType::Phi: opStr = "phi"; break;
            case IROpType::Cast: opStr = "cast"; break;
            case IROpType::Nop: opStr = "nop"; break;
            default: opStr = "unknown"; break;
        }
        
        std::string resultStr = result ? result->toString() + " = " : "";
        std::string operandsStr;
        
        for (size_t i = 0; i < operands.size(); ++i) {
            if (i > 0) operandsStr += ", ";
            operandsStr += operands[i]->toString();
        }
        
        return resultStr + opStr + " " + operandsStr;
    }
};

/**
 * IR basic block
 */
class IRBasicBlock : public IRValue {
public:
    std::string label;
    std::vector<std::shared_ptr<IRInstruction>> instructions;
    std::vector<std::shared_ptr<IRBasicBlock>> predecessors;
    std::vector<std::shared_ptr<IRBasicBlock>> successors;
    
    IRBasicBlock(const std::string& l = "")
        : IRValue(Kind::BasicBlock, nullptr, l), label(l) {}
    
    void addInstruction(std::shared_ptr<IRInstruction> instruction) {
        instructions.push_back(instruction);
    }
    
    void addPredecessor(std::shared_ptr<IRBasicBlock> block) {
        predecessors.push_back(block);
    }
    
    void addSuccessor(std::shared_ptr<IRBasicBlock> block) {
        successors.push_back(block);
    }
    
    std::string toString() const override {
        return label.empty() ? "bb" + std::to_string(reinterpret_cast<uintptr_t>(this)) : label;
    }
};

/**
 * IR function
 */
class IRFunction {
public:
    std::string name;
    std::shared_ptr<IRType> type;
    std::vector<std::shared_ptr<IRValue>> parameters;
    std::vector<std::shared_ptr<IRBasicBlock>> blocks;
    std::shared_ptr<IRBasicBlock> entryBlock;
    
    IRFunction(const std::string& n, std::shared_ptr<IRType> t)
        : name(n), type(t) {}
    
    void addParameter(std::shared_ptr<IRValue> param) {
        parameters.push_back(param);
    }
    
    void addBlock(std::shared_ptr<IRBasicBlock> block) {
        blocks.push_back(block);
        if (!entryBlock) {
            entryBlock = block;
        }
    }
    
    std::string toString() const {
        std::string result = "function " + name + "(";
        
        for (size_t i = 0; i < parameters.size(); ++i) {
            if (i > 0) result += ", ";
            result += parameters[i]->type->toString() + " " + parameters[i]->toString();
        }
        
        result += ") -> " + type->returnType->toString() + " {\n";
        
        for (const auto& block : blocks) {
            result += block->toString() + ":\n";
            for (const auto& inst : block->instructions) {
                result += "  " + inst->toString() + "\n";
            }
        }
        
        result += "}\n";
        return result;
    }
};

/**
 * IR module
 */
class IRModule {
public:
    std::string name;
    std::vector<std::shared_ptr<IRFunction>> functions;
    std::vector<std::shared_ptr<IRValue>> globals;
    
    IRModule(const std::string& n) : name(n) {}
    
    void addFunction(std::shared_ptr<IRFunction> function) {
        functions.push_back(function);
    }
    
    void addGlobal(std::shared_ptr<IRValue> global) {
        globals.push_back(global);
    }
    
    std::string toString() const {
        std::string result = "module " + name + " {\n";
        
        for (const auto& global : globals) {
            result += "  global " + global->toString() + " : " + global->type->toString() + "\n";
        }
        
        for (const auto& function : functions) {
            result += function->toString() + "\n";
        }
        
        result += "}\n";
        return result;
    }
};

/**
 * IR generator
 */
class IRGenerator : public ASTVisitor {
private:
    ErrorHandler& errorHandler;
    std::shared_ptr<IRModule> currentModule;
    std::shared_ptr<IRFunction> currentFunction;
    std::shared_ptr<IRBasicBlock> currentBlock;
    std::unordered_map<std::string, std::shared_ptr<IRValue>> symbolTable;
    int tempCounter = 0;
    
    /**
     * Generate a temporary name
     */
    std::string genTemp() {
        return "t" + std::to_string(tempCounter++);
    }
    
    /**
     * Create a new basic block
     */
    std::shared_ptr<IRBasicBlock> createBlock(const std::string& label = "") {
        auto block = std::make_shared<IRBasicBlock>(label);
        if (currentFunction) {
            currentFunction->addBlock(block);
        }
        return block;
    }
    
    /**
     * Add an instruction to the current block
     */
    std::shared_ptr<IRInstruction> addInstruction(IROpType opType, std::shared_ptr<IRValue> result = nullptr) {
        auto instruction = std::make_shared<IRInstruction>(opType, result);
        if (currentBlock) {
            currentBlock->addInstruction(instruction);
        }
        return instruction;
    }
    
    /**
     * Convert AST type to IR type
     */
    std::shared_ptr<IRType> convertType(std::shared_ptr<Type> type);
    
public:
    /**
     * Constructor
     */
    IRGenerator(ErrorHandler& errHandler)
        : errorHandler(errHandler) {}
    
    /**
     * Generate IR from AST
     */
    std::shared_ptr<IRModule> generate(std::shared_ptr<Program> ast) {
        currentModule = std::make_shared<IRModule>("main");
        ast->accept(*this);
        return currentModule;
    }
    
    // Visitor methods
    void visit(Program& node) override;
    void visit(LiteralExpression& node) override;
    void visit(IdentifierExpression& node) override;
    void visit(UnaryExpression& node) override;
    void visit(BinaryExpression& node) override;
    void visit(CallExpression& node) override;
    void visit(MemberExpression& node) override;
    void visit(ArrayExpression& node) override;
    void visit(ExpressionStatement& node) override;
    void visit(BlockStatement& node) override;
    void visit(VariableDeclaration& node) override;
    void visit(FunctionDeclaration& node) override;
    void visit(IfStatement& node) override;
    void visit(WhileStatement& node) override;
    void visit(ForStatement& node) override;
    void visit(ReturnStatement& node) override;
    void visit(BreakStatement& node) override;
    void visit(ContinueStatement& node) override;
    void visit(ImportStatement& node) override;
    void visit(ModuleDeclaration& node) override;
    void visit(TryStatement& node) override;
    void visit(ThrowStatement& node) override;
    void visit(TaskDeclaration& node) override;
    void visit(AnnotationStatement& node) override;
    void visit(SimpleType& node) override;
    void visit(ArrayType& node) override;
    void visit(RangeType& node) override;
    void visit(FunctionType& node) override;
};

} // namespace astra

#endif // ASTRA_IR_GENERATOR_H