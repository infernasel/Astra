#include "stdlib/network_module.h"
#include "runtime/runtime.h"
#include "runtime/value.h"
#include "runtime/object.h"
#include "runtime/function.h"
#include "runtime/array.h"
#include "runtime/exception.h"

namespace astra {
namespace stdlib {

using namespace runtime;

// Helper functions to convert between ASTRA and C++ types
static Value ipAddressToValue(Runtime& runtime, const network::IPAddress& address) {
    ObjectPtr obj = runtime.createObject("IPAddress");
    
    // Set properties
    obj->setProperty("type", Value(static_cast<int>(address.getType())));
    obj->setProperty("valid", Value(address.isValid()));
    
    // Convert bytes to array
    std::vector<uint8_t> bytes = address.getBytes();
    ArrayPtr bytesArray = runtime.createArray();
    for (size_t i = 0; i < bytes.size(); ++i) {
        bytesArray->push(Value(static_cast<int>(bytes[i])));
    }
    obj->setProperty("bytes", Value(bytesArray));
    
    // Set string representation
    obj->setProperty("address", Value(address.toString()));
    
    // Set flags
    obj->setProperty("isLoopback", Value(address.isLoopback()));
    obj->setProperty("isPrivate", Value(address.isPrivate()));
    obj->setProperty("isMulticast", Value(address.isMulticast()));
    
    return Value(obj);
}

static network::IPAddress valueToIPAddress(const Value& value) {
    if (!value.isObject()) {
        throw Exception("Expected IPAddress object");
    }
    
    ObjectPtr obj = value.asObject();
    
    // Check if it's a string
    if (obj->hasProperty("address") && obj->getProperty("address").isString()) {
        return network::IPAddress(obj->getProperty("address").asString());
    }
    
    // Check if it has bytes and type
    if (obj->hasProperty("bytes") && obj->hasProperty("type")) {
        Value bytesValue = obj->getProperty("bytes");
        Value typeValue = obj->getProperty("type");
        
        if (!bytesValue.isArray() || !typeValue.isNumber()) {
            throw Exception("Invalid IPAddress object");
        }
        
        ArrayPtr bytesArray = bytesValue.asArray();
        std::vector<uint8_t> bytes;
        
        for (size_t i = 0; i < bytesArray->size(); ++i) {
            Value byteValue = bytesArray->get(i);
            if (!byteValue.isNumber()) {
                throw Exception("Invalid IPAddress bytes");
            }
            bytes.push_back(static_cast<uint8_t>(byteValue.asNumber()));
        }
        
        network::IPAddress::Type type = static_cast<network::IPAddress::Type>(typeValue.asNumber());
        return network::IPAddress(bytes, type);
    }
    
    throw Exception("Invalid IPAddress object");
}

static Value socketAddressToValue(Runtime& runtime, const network::SocketAddress& address) {
    ObjectPtr obj = runtime.createObject("SocketAddress");
    
    // Set properties
    obj->setProperty("address", ipAddressToValue(runtime, address.getAddress()));
    obj->setProperty("port", Value(static_cast<int>(address.getPort())));
    obj->setProperty("valid", Value(address.isValid()));
    obj->setProperty("string", Value(address.toString()));
    
    return Value(obj);
}

static network::SocketAddress valueToSocketAddress(Runtime& runtime, const Value& value) {
    if (!value.isObject()) {
        throw Exception("Expected SocketAddress object");
    }
    
    ObjectPtr obj = value.asObject();
    
    // Check if it has address and port
    if (obj->hasProperty("address") && obj->hasProperty("port")) {
        Value addressValue = obj->getProperty("address");
        Value portValue = obj->getProperty("port");
        
        if (!portValue.isNumber()) {
            throw Exception("Invalid SocketAddress port");
        }
        
        network::IPAddress address = valueToIPAddress(addressValue);
        uint16_t port = static_cast<uint16_t>(portValue.asNumber());
        
        return network::SocketAddress(address, port);
    }
    
    // Check if it's a string with format "address:port"
    if (obj->hasProperty("string") && obj->getProperty("string").isString()) {
        std::string str = obj->getProperty("string").asString();
        size_t colonPos = str.find_last_of(':');
        
        if (colonPos != std::string::npos) {
            std::string addressStr = str.substr(0, colonPos);
            std::string portStr = str.substr(colonPos + 1);
            
            // Remove brackets from IPv6 address
            if (addressStr.front() == '[' && addressStr.back() == ']') {
                addressStr = addressStr.substr(1, addressStr.size() - 2);
            }
            
            network::IPAddress address(addressStr);
            uint16_t port = static_cast<uint16_t>(std::stoi(portStr));
            
            return network::SocketAddress(address, port);
        }
    }
    
    throw Exception("Invalid SocketAddress object");
}

static Value socketToValue(Runtime& runtime, std::shared_ptr<network::Socket> socket) {
    ObjectPtr obj = runtime.createObject("Socket");
    
    // Store the socket pointer in a native object
    obj->setNativeObject(socket);
    
    // Set properties
    obj->setProperty("type", Value(static_cast<int>(socket->getType())));
    obj->setProperty("state", Value(static_cast<int>(socket->getState())));
    obj->setProperty("localAddress", socketAddressToValue(runtime, socket->getLocalAddress()));
    obj->setProperty("remoteAddress", socketAddressToValue(runtime, socket->getRemoteAddress()));
    obj->setProperty("lastError", Value(socket->getLastError()));
    
    return Value(obj);
}

static std::shared_ptr<network::Socket> valueToSocket(const Value& value) {
    if (!value.isObject()) {
        throw Exception("Expected Socket object");
    }
    
    ObjectPtr obj = value.asObject();
    
    // Get the socket pointer from the native object
    auto socket = std::static_pointer_cast<network::Socket>(obj->getNativeObject());
    if (!socket) {
        throw Exception("Invalid Socket object");
    }
    
    return socket;
}

static Value httpRequestToValue(Runtime& runtime, const network::HttpRequest& request) {
    ObjectPtr obj = runtime.createObject("HttpRequest");
    
    // Set properties
    obj->setProperty("method", Value(static_cast<int>(request.getMethod())));
    obj->setProperty("url", Value(request.getUrl()));
    obj->setProperty("timeout", Value(request.getTimeout()));
    
    // Convert headers to object
    ObjectPtr headersObj = runtime.createObject();
    auto headers = request.getHeaders();
    for (const auto& header : headers) {
        headersObj->setProperty(header.first, Value(header.second));
    }
    obj->setProperty("headers", Value(headersObj));
    
    // Convert body to array
    std::vector<uint8_t> body = request.getBody();
    ArrayPtr bodyArray = runtime.createArray();
    for (size_t i = 0; i < body.size(); ++i) {
        bodyArray->push(Value(static_cast<int>(body[i])));
    }
    obj->setProperty("body", Value(bodyArray));
    
    return Value(obj);
}

static network::HttpRequest valueToHttpRequest(Runtime& runtime, const Value& value) {
    if (!value.isObject()) {
        throw Exception("Expected HttpRequest object");
    }
    
    ObjectPtr obj = value.asObject();
    
    // Create request
    network::HttpRequest request;
    
    // Set method
    if (obj->hasProperty("method")) {
        Value methodValue = obj->getProperty("method");
        if (methodValue.isNumber()) {
            request.setMethod(static_cast<network::HttpRequest::Method>(methodValue.asNumber()));
        }
    }
    
    // Set URL
    if (obj->hasProperty("url")) {
        Value urlValue = obj->getProperty("url");
        if (urlValue.isString()) {
            request.setUrl(urlValue.asString());
        }
    }
    
    // Set timeout
    if (obj->hasProperty("timeout")) {
        Value timeoutValue = obj->getProperty("timeout");
        if (timeoutValue.isNumber()) {
            request.setTimeout(static_cast<int>(timeoutValue.asNumber()));
        }
    }
    
    // Set headers
    if (obj->hasProperty("headers")) {
        Value headersValue = obj->getProperty("headers");
        if (headersValue.isObject()) {
            ObjectPtr headersObj = headersValue.asObject();
            auto properties = headersObj->getProperties();
            for (const auto& property : properties) {
                if (property.second.isString()) {
                    request.setHeader(property.first, property.second.asString());
                }
            }
        }
    }
    
    // Set body
    if (obj->hasProperty("body")) {
        Value bodyValue = obj->getProperty("body");
        if (bodyValue.isArray()) {
            ArrayPtr bodyArray = bodyValue.asArray();
            std::vector<uint8_t> body;
            for (size_t i = 0; i < bodyArray->size(); ++i) {
                Value byteValue = bodyArray->get(i);
                if (byteValue.isNumber()) {
                    body.push_back(static_cast<uint8_t>(byteValue.asNumber()));
                }
            }
            request.setBody(body);
        } else if (bodyValue.isString()) {
            std::string bodyStr = bodyValue.asString();
            std::vector<uint8_t> body(bodyStr.begin(), bodyStr.end());
            request.setBody(body);
        }
    }
    
    return request;
}

static Value httpResponseToValue(Runtime& runtime, const network::HttpResponse& response) {
    ObjectPtr obj = runtime.createObject("HttpResponse");
    
    // Set properties
    obj->setProperty("statusCode", Value(response.getStatusCode()));
    obj->setProperty("statusMessage", Value(response.getStatusMessage()));
    
    // Convert headers to object
    ObjectPtr headersObj = runtime.createObject();
    auto headers = response.getHeaders();
    for (const auto& header : headers) {
        headersObj->setProperty(header.first, Value(header.second));
    }
    obj->setProperty("headers", Value(headersObj));
    
    // Convert body to array
    std::vector<uint8_t> body = response.getBody();
    ArrayPtr bodyArray = runtime.createArray();
    for (size_t i = 0; i < body.size(); ++i) {
        bodyArray->push(Value(static_cast<int>(body[i])));
    }
    obj->setProperty("body", Value(bodyArray));
    
    // Set flags
    obj->setProperty("isSuccess", Value(response.isSuccess()));
    obj->setProperty("isRedirect", Value(response.isRedirect()));
    obj->setProperty("isClientError", Value(response.isClientError()));
    obj->setProperty("isServerError", Value(response.isServerError()));
    
    return Value(obj);
}

static network::HttpResponse valueToHttpResponse(Runtime& runtime, const Value& value) {
    if (!value.isObject()) {
        throw Exception("Expected HttpResponse object");
    }
    
    ObjectPtr obj = value.asObject();
    
    // Create response
    network::HttpResponse response;
    
    // Set status code
    if (obj->hasProperty("statusCode")) {
        Value statusCodeValue = obj->getProperty("statusCode");
        if (statusCodeValue.isNumber()) {
            response.setStatusCode(static_cast<int>(statusCodeValue.asNumber()));
        }
    }
    
    // Set status message
    if (obj->hasProperty("statusMessage")) {
        Value statusMessageValue = obj->getProperty("statusMessage");
        if (statusMessageValue.isString()) {
            response.setStatusMessage(statusMessageValue.asString());
        }
    }
    
    // Set headers
    if (obj->hasProperty("headers")) {
        Value headersValue = obj->getProperty("headers");
        if (headersValue.isObject()) {
            ObjectPtr headersObj = headersValue.asObject();
            auto properties = headersObj->getProperties();
            for (const auto& property : properties) {
                if (property.second.isString()) {
                    response.setHeader(property.first, property.second.asString());
                }
            }
        }
    }
    
    // Set body
    if (obj->hasProperty("body")) {
        Value bodyValue = obj->getProperty("body");
        if (bodyValue.isArray()) {
            ArrayPtr bodyArray = bodyValue.asArray();
            std::vector<uint8_t> body;
            for (size_t i = 0; i < bodyArray->size(); ++i) {
                Value byteValue = bodyArray->get(i);
                if (byteValue.isNumber()) {
                    body.push_back(static_cast<uint8_t>(byteValue.asNumber()));
                }
            }
            response.setBody(body);
        }
    }
    
    return response;
}

static Value httpClientToValue(Runtime& runtime, std::shared_ptr<network::HttpClient> client) {
    ObjectPtr obj = runtime.createObject("HttpClient");
    
    // Store the client pointer in a native object
    obj->setNativeObject(client);
    
    return Value(obj);
}

static std::shared_ptr<network::HttpClient> valueToHttpClient(const Value& value) {
    if (!value.isObject()) {
        throw Exception("Expected HttpClient object");
    }
    
    ObjectPtr obj = value.asObject();
    
    // Get the client pointer from the native object
    auto client = std::static_pointer_cast<network::HttpClient>(obj->getNativeObject());
    if (!client) {
        throw Exception("Invalid HttpClient object");
    }
    
    return client;
}

static Value networkInterfaceToValue(Runtime& runtime, const network::NetworkInterface& interface) {
    ObjectPtr obj = runtime.createObject("NetworkInterface");
    
    // Set properties
    obj->setProperty("name", Value(interface.getName()));
    obj->setProperty("address", ipAddressToValue(runtime, interface.getAddress()));
    obj->setProperty("netmask", ipAddressToValue(runtime, interface.getNetmask()));
    obj->setProperty("broadcast", ipAddressToValue(runtime, interface.getBroadcast()));
    obj->setProperty("up", Value(interface.isUp()));
    obj->setProperty("loopback", Value(interface.isLoopback()));
    
    return Value(obj);
}

// IPAddress class methods
static Value IPAddress_constructor(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1) {
        return ipAddressToValue(runtime, network::IPAddress());
    }
    
    if (args[0].isString()) {
        return ipAddressToValue(runtime, network::IPAddress(args[0].asString()));
    }
    
    if (args[0].isArray() && args.size() > 1 && args[1].isNumber()) {
        ArrayPtr bytesArray = args[0].asArray();
        std::vector<uint8_t> bytes;
        
        for (size_t i = 0; i < bytesArray->size(); ++i) {
            Value byteValue = bytesArray->get(i);
            if (!byteValue.isNumber()) {
                throw Exception("Invalid IPAddress bytes");
            }
            bytes.push_back(static_cast<uint8_t>(byteValue.asNumber()));
        }
        
        network::IPAddress::Type type = static_cast<network::IPAddress::Type>(args[1].asNumber());
        return ipAddressToValue(runtime, network::IPAddress(bytes, type));
    }
    
    throw Exception("Invalid arguments for IPAddress constructor");
}

static Value IPAddress_toString(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for IPAddress.toString");
    }
    
    network::IPAddress address = valueToIPAddress(args[0]);
    return Value(address.toString());
}

static Value IPAddress_isLoopback(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for IPAddress.isLoopback");
    }
    
    network::IPAddress address = valueToIPAddress(args[0]);
    return Value(address.isLoopback());
}

static Value IPAddress_isPrivate(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for IPAddress.isPrivate");
    }
    
    network::IPAddress address = valueToIPAddress(args[0]);
    return Value(address.isPrivate());
}

static Value IPAddress_isMulticast(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for IPAddress.isMulticast");
    }
    
    network::IPAddress address = valueToIPAddress(args[0]);
    return Value(address.isMulticast());
}

static Value IPAddress_isValid(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for IPAddress.isValid");
    }
    
    network::IPAddress address = valueToIPAddress(args[0]);
    return Value(address.isValid());
}

// SocketAddress class methods
static Value SocketAddress_constructor(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1) {
        return socketAddressToValue(runtime, network::SocketAddress());
    }
    
    if (args[0].isString() && args.size() > 1 && args[1].isNumber()) {
        return socketAddressToValue(runtime, network::SocketAddress(args[0].asString(), static_cast<uint16_t>(args[1].asNumber())));
    }
    
    if (args[0].isObject() && args.size() > 1 && args[1].isNumber()) {
        network::IPAddress address = valueToIPAddress(args[0]);
        return socketAddressToValue(runtime, network::SocketAddress(address, static_cast<uint16_t>(args[1].asNumber())));
    }
    
    throw Exception("Invalid arguments for SocketAddress constructor");
}

static Value SocketAddress_toString(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for SocketAddress.toString");
    }
    
    network::SocketAddress address = valueToSocketAddress(runtime, args[0]);
    return Value(address.toString());
}

static Value SocketAddress_isValid(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid this object for SocketAddress.isValid");
    }
    
    network::SocketAddress address = valueToSocketAddress(runtime, args[0]);
    return Value(address.isValid());
}

// Socket class methods
static Value Socket_constructor(Runtime& runtime, const std::vector<Value>& args) {
    network::Socket::Type type = network::Socket::Type::TCP;
    
    if (args.size() > 0 && args[0].isNumber()) {
        type = static_cast<network::Socket::Type>(args[0].asNumber());
    }
    
    return socketToValue(runtime, std::make_shared<network::Socket>(type));
}

static Value Socket_bind(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isObject()) {
        throw Exception("Invalid arguments for Socket.bind");
    }
    
    auto socket = valueToSocket(args[0]);
    network::SocketAddress address = valueToSocketAddress(runtime, args[1]);
    
    return Value(socket->bind(address));
}

static Value Socket_connect(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isObject()) {
        throw Exception("Invalid arguments for Socket.connect");
    }
    
    auto socket = valueToSocket(args[0]);
    network::SocketAddress address = valueToSocketAddress(runtime, args[1]);
    
    return Value(socket->connect(address));
}

static Value Socket_listen(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for Socket.listen");
    }
    
    auto socket = valueToSocket(args[0]);
    int backlog = static_cast<int>(args[1].asNumber());
    
    return Value(socket->listen(backlog));
}

static Value Socket_accept(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.accept");
    }
    
    auto socket = valueToSocket(args[0]);
    auto clientSocket = socket->accept();
    
    if (!clientSocket) {
        return Value::null();
    }
    
    return socketToValue(runtime, clientSocket);
}

static Value Socket_send(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isArray()) {
        throw Exception("Invalid arguments for Socket.send");
    }
    
    auto socket = valueToSocket(args[0]);
    ArrayPtr dataArray = args[1].asArray();
    std::vector<uint8_t> data;
    
    for (size_t i = 0; i < dataArray->size(); ++i) {
        Value byteValue = dataArray->get(i);
        if (!byteValue.isNumber()) {
            throw Exception("Invalid data bytes");
        }
        data.push_back(static_cast<uint8_t>(byteValue.asNumber()));
    }
    
    return Value(socket->send(data));
}

static Value Socket_receive(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for Socket.receive");
    }
    
    auto socket = valueToSocket(args[0]);
    size_t maxSize = static_cast<size_t>(args[1].asNumber());
    
    std::vector<uint8_t> data;
    int bytesRead = socket->receive(data, maxSize);
    
    if (bytesRead < 0) {
        return Value::null();
    }
    
    ArrayPtr resultArray = runtime.createArray();
    for (size_t i = 0; i < data.size(); ++i) {
        resultArray->push(Value(static_cast<int>(data[i])));
    }
    
    return Value(resultArray);
}

static Value Socket_sendTo(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 3 || !args[0].isObject() || !args[1].isArray() || !args[2].isObject()) {
        throw Exception("Invalid arguments for Socket.sendTo");
    }
    
    auto socket = valueToSocket(args[0]);
    ArrayPtr dataArray = args[1].asArray();
    std::vector<uint8_t> data;
    
    for (size_t i = 0; i < dataArray->size(); ++i) {
        Value byteValue = dataArray->get(i);
        if (!byteValue.isNumber()) {
            throw Exception("Invalid data bytes");
        }
        data.push_back(static_cast<uint8_t>(byteValue.asNumber()));
    }
    
    network::SocketAddress address = valueToSocketAddress(runtime, args[2]);
    
    return Value(socket->sendTo(data, address));
}

static Value Socket_receiveFrom(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for Socket.receiveFrom");
    }
    
    auto socket = valueToSocket(args[0]);
    size_t maxSize = static_cast<size_t>(args[1].asNumber());
    
    std::vector<uint8_t> data;
    network::SocketAddress address;
    int bytesRead = socket->receiveFrom(data, maxSize, address);
    
    if (bytesRead < 0) {
        return Value::null();
    }
    
    ObjectPtr resultObj = runtime.createObject();
    
    ArrayPtr dataArray = runtime.createArray();
    for (size_t i = 0; i < data.size(); ++i) {
        dataArray->push(Value(static_cast<int>(data[i])));
    }
    
    resultObj->setProperty("data", Value(dataArray));
    resultObj->setProperty("address", socketAddressToValue(runtime, address));
    
    return Value(resultObj);
}

static Value Socket_close(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.close");
    }
    
    auto socket = valueToSocket(args[0]);
    return Value(socket->close());
}

static Value Socket_setBlocking(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isBoolean()) {
        throw Exception("Invalid arguments for Socket.setBlocking");
    }
    
    auto socket = valueToSocket(args[0]);
    bool blocking = args[1].asBoolean();
    
    return Value(socket->setBlocking(blocking));
}

static Value Socket_setReuseAddress(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isBoolean()) {
        throw Exception("Invalid arguments for Socket.setReuseAddress");
    }
    
    auto socket = valueToSocket(args[0]);
    bool reuse = args[1].asBoolean();
    
    return Value(socket->setReuseAddress(reuse));
}

static Value Socket_setReceiveTimeout(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for Socket.setReceiveTimeout");
    }
    
    auto socket = valueToSocket(args[0]);
    int timeout = static_cast<int>(args[1].asNumber());
    
    return Value(socket->setReceiveTimeout(timeout));
}

static Value Socket_setSendTimeout(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for Socket.setSendTimeout");
    }
    
    auto socket = valueToSocket(args[0]);
    int timeout = static_cast<int>(args[1].asNumber());
    
    return Value(socket->setSendTimeout(timeout));
}

static Value Socket_getState(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.getState");
    }
    
    auto socket = valueToSocket(args[0]);
    return Value(static_cast<int>(socket->getState()));
}

static Value Socket_getType(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.getType");
    }
    
    auto socket = valueToSocket(args[0]);
    return Value(static_cast<int>(socket->getType()));
}

static Value Socket_getLocalAddress(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.getLocalAddress");
    }
    
    auto socket = valueToSocket(args[0]);
    return socketAddressToValue(runtime, socket->getLocalAddress());
}

static Value Socket_getRemoteAddress(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.getRemoteAddress");
    }
    
    auto socket = valueToSocket(args[0]);
    return socketAddressToValue(runtime, socket->getRemoteAddress());
}

static Value Socket_getLastError(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for Socket.getLastError");
    }
    
    auto socket = valueToSocket(args[0]);
    return Value(socket->getLastError());
}

// HttpRequest class methods
static Value HttpRequest_constructor(Runtime& runtime, const std::vector<Value>& args) {
    network::HttpRequest request;
    
    if (args.size() > 0 && args[0].isNumber()) {
        request.setMethod(static_cast<network::HttpRequest::Method>(args[0].asNumber()));
    }
    
    if (args.size() > 1 && args[1].isString()) {
        request.setUrl(args[1].asString());
    }
    
    return httpRequestToValue(runtime, request);
}

static Value HttpRequest_setMethod(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for HttpRequest.setMethod");
    }
    
    network::HttpRequest request = valueToHttpRequest(runtime, args[0]);
    request.setMethod(static_cast<network::HttpRequest::Method>(args[1].asNumber()));
    
    return httpRequestToValue(runtime, request);
}

static Value HttpRequest_setUrl(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpRequest.setUrl");
    }
    
    network::HttpRequest request = valueToHttpRequest(runtime, args[0]);
    request.setUrl(args[1].asString());
    
    return httpRequestToValue(runtime, request);
}

static Value HttpRequest_setHeader(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 3 || !args[0].isObject() || !args[1].isString() || !args[2].isString()) {
        throw Exception("Invalid arguments for HttpRequest.setHeader");
    }
    
    network::HttpRequest request = valueToHttpRequest(runtime, args[0]);
    request.setHeader(args[1].asString(), args[2].asString());
    
    return httpRequestToValue(runtime, request);
}

static Value HttpRequest_setBody(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject()) {
        throw Exception("Invalid arguments for HttpRequest.setBody");
    }
    
    network::HttpRequest request = valueToHttpRequest(runtime, args[0]);
    
    if (args[1].isArray()) {
        ArrayPtr bodyArray = args[1].asArray();
        std::vector<uint8_t> body;
        
        for (size_t i = 0; i < bodyArray->size(); ++i) {
            Value byteValue = bodyArray->get(i);
            if (!byteValue.isNumber()) {
                throw Exception("Invalid body bytes");
            }
            body.push_back(static_cast<uint8_t>(byteValue.asNumber()));
        }
        
        request.setBody(body);
    } else if (args[1].isString()) {
        std::string bodyStr = args[1].asString();
        std::vector<uint8_t> body(bodyStr.begin(), bodyStr.end());
        request.setBody(body);
    } else {
        throw Exception("Invalid body type");
    }
    
    return httpRequestToValue(runtime, request);
}

static Value HttpRequest_setContentType(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpRequest.setContentType");
    }
    
    network::HttpRequest request = valueToHttpRequest(runtime, args[0]);
    request.setContentType(args[1].asString());
    
    return httpRequestToValue(runtime, request);
}

static Value HttpRequest_setTimeout(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for HttpRequest.setTimeout");
    }
    
    network::HttpRequest request = valueToHttpRequest(runtime, args[0]);
    request.setTimeout(static_cast<int>(args[1].asNumber()));
    
    return httpRequestToValue(runtime, request);
}

// HttpClient class methods
static Value HttpClient_constructor(Runtime& runtime, const std::vector<Value>& args) {
    return httpClientToValue(runtime, std::make_shared<network::HttpClient>());
}

static Value HttpClient_send(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isObject()) {
        throw Exception("Invalid arguments for HttpClient.send");
    }
    
    auto client = valueToHttpClient(args[0]);
    network::HttpRequest request = valueToHttpRequest(runtime, args[1]);
    
    network::HttpResponse response = client->send(request);
    return httpResponseToValue(runtime, response);
}

static Value HttpClient_get(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpClient.get");
    }
    
    auto client = valueToHttpClient(args[0]);
    std::string url = args[1].asString();
    
    network::HttpResponse response = client->get(url);
    return httpResponseToValue(runtime, response);
}

static Value HttpClient_post(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 3 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpClient.post");
    }
    
    auto client = valueToHttpClient(args[0]);
    std::string url = args[1].asString();
    std::vector<uint8_t> body;
    std::string contentType = "application/octet-stream";
    
    if (args[2].isArray()) {
        ArrayPtr bodyArray = args[2].asArray();
        
        for (size_t i = 0; i < bodyArray->size(); ++i) {
            Value byteValue = bodyArray->get(i);
            if (!byteValue.isNumber()) {
                throw Exception("Invalid body bytes");
            }
            body.push_back(static_cast<uint8_t>(byteValue.asNumber()));
        }
    } else if (args[2].isString()) {
        std::string bodyStr = args[2].asString();
        body.assign(bodyStr.begin(), bodyStr.end());
        contentType = "text/plain";
    } else {
        throw Exception("Invalid body type");
    }
    
    if (args.size() > 3 && args[3].isString()) {
        contentType = args[3].asString();
    }
    
    network::HttpResponse response = client->post(url, body, contentType);
    return httpResponseToValue(runtime, response);
}

static Value HttpClient_put(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 3 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpClient.put");
    }
    
    auto client = valueToHttpClient(args[0]);
    std::string url = args[1].asString();
    std::vector<uint8_t> body;
    std::string contentType = "application/octet-stream";
    
    if (args[2].isArray()) {
        ArrayPtr bodyArray = args[2].asArray();
        
        for (size_t i = 0; i < bodyArray->size(); ++i) {
            Value byteValue = bodyArray->get(i);
            if (!byteValue.isNumber()) {
                throw Exception("Invalid body bytes");
            }
            body.push_back(static_cast<uint8_t>(byteValue.asNumber()));
        }
    } else if (args[2].isString()) {
        std::string bodyStr = args[2].asString();
        body.assign(bodyStr.begin(), bodyStr.end());
        contentType = "text/plain";
    } else {
        throw Exception("Invalid body type");
    }
    
    if (args.size() > 3 && args[3].isString()) {
        contentType = args[3].asString();
    }
    
    network::HttpResponse response = client->put(url, body, contentType);
    return httpResponseToValue(runtime, response);
}

static Value HttpClient_del(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpClient.del");
    }
    
    auto client = valueToHttpClient(args[0]);
    std::string url = args[1].asString();
    
    network::HttpResponse response = client->del(url);
    return httpResponseToValue(runtime, response);
}

static Value HttpClient_setDefaultHeader(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 3 || !args[0].isObject() || !args[1].isString() || !args[2].isString()) {
        throw Exception("Invalid arguments for HttpClient.setDefaultHeader");
    }
    
    auto client = valueToHttpClient(args[0]);
    std::string name = args[1].asString();
    std::string value = args[2].asString();
    
    client->setDefaultHeader(name, value);
    return Value::null();
}

static Value HttpClient_setDefaultTimeout(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for HttpClient.setDefaultTimeout");
    }
    
    auto client = valueToHttpClient(args[0]);
    int timeout = static_cast<int>(args[1].asNumber());
    
    client->setDefaultTimeout(timeout);
    return Value::null();
}

static Value HttpClient_setFollowRedirects(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isBoolean()) {
        throw Exception("Invalid arguments for HttpClient.setFollowRedirects");
    }
    
    auto client = valueToHttpClient(args[0]);
    bool follow = args[1].asBoolean();
    
    client->setFollowRedirects(follow);
    return Value::null();
}

static Value HttpClient_setMaxRedirects(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isNumber()) {
        throw Exception("Invalid arguments for HttpClient.setMaxRedirects");
    }
    
    auto client = valueToHttpClient(args[0]);
    int maxRedirects = static_cast<int>(args[1].asNumber());
    
    client->setMaxRedirects(maxRedirects);
    return Value::null();
}

static Value HttpClient_setUserAgent(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[0].isObject() || !args[1].isString()) {
        throw Exception("Invalid arguments for HttpClient.setUserAgent");
    }
    
    auto client = valueToHttpClient(args[0]);
    std::string userAgent = args[1].asString();
    
    client->setUserAgent(userAgent);
    return Value::null();
}

// DnsResolver class methods
static Value DnsResolver_resolve(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isString()) {
        throw Exception("Invalid arguments for DnsResolver.resolve");
    }
    
    network::DnsResolver resolver;
    std::string hostname = args[0].asString();
    
    std::vector<network::IPAddress> addresses = resolver.resolve(hostname);
    
    ArrayPtr resultArray = runtime.createArray();
    for (const auto& address : addresses) {
        resultArray->push(ipAddressToValue(runtime, address));
    }
    
    return Value(resultArray);
}

static Value DnsResolver_resolveFirst(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isString()) {
        throw Exception("Invalid arguments for DnsResolver.resolveFirst");
    }
    
    network::DnsResolver resolver;
    std::string hostname = args[0].asString();
    
    network::IPAddress address = resolver.resolveFirst(hostname);
    return ipAddressToValue(runtime, address);
}

static Value DnsResolver_reverseLookup(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1 || !args[0].isObject()) {
        throw Exception("Invalid arguments for DnsResolver.reverseLookup");
    }
    
    network::DnsResolver resolver;
    network::IPAddress address = valueToIPAddress(args[0]);
    
    std::string hostname = resolver.reverseLookup(address);
    return Value(hostname);
}

// NetworkUtils class methods
static Value NetworkUtils_getNetworkInterfaces(Runtime& runtime, const std::vector<Value>& args) {
    std::vector<network::NetworkInterface> interfaces = network::NetworkUtils::getNetworkInterfaces();
    
    ArrayPtr resultArray = runtime.createArray();
    for (const auto& interface : interfaces) {
        resultArray->push(networkInterfaceToValue(runtime, interface));
    }
    
    return Value(resultArray);
}

static Value NetworkUtils_ping(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 1) {
        throw Exception("Invalid arguments for NetworkUtils.ping");
    }
    
    int timeout = 1000;
    if (args.size() > 1 && args[1].isNumber()) {
        timeout = static_cast<int>(args[1].asNumber());
    }
    
    if (args[0].isString()) {
        std::string hostname = args[0].asString();
        return Value(network::NetworkUtils::ping(hostname, timeout));
    } else if (args[0].isObject()) {
        network::IPAddress address = valueToIPAddress(args[0]);
        return Value(network::NetworkUtils::ping(address, timeout));
    }
    
    throw Exception("Invalid arguments for NetworkUtils.ping");
}

static Value NetworkUtils_getHostname(Runtime& runtime, const std::vector<Value>& args) {
    std::string hostname;
    int result = network::NetworkUtils::getHostname(hostname);
    
    if (result != 0) {
        return Value::null();
    }
    
    return Value(hostname);
}

static Value NetworkUtils_isPortOpen(Runtime& runtime, const std::vector<Value>& args) {
    if (args.size() < 2 || !args[1].isNumber()) {
        throw Exception("Invalid arguments for NetworkUtils.isPortOpen");
    }
    
    uint16_t port = static_cast<uint16_t>(args[1].asNumber());
    int timeout = 1000;
    
    if (args.size() > 2 && args[2].isNumber()) {
        timeout = static_cast<int>(args[2].asNumber());
    }
    
    if (args[0].isString()) {
        std::string hostname = args[0].asString();
        return Value(network::NetworkUtils::isPortOpen(hostname, port, timeout));
    } else if (args[0].isObject()) {
        network::IPAddress address = valueToIPAddress(args[0]);
        return Value(network::NetworkUtils::isPortOpen(address, port, timeout));
    }
    
    throw Exception("Invalid arguments for NetworkUtils.isPortOpen");
}

void registerNetworkModule(runtime::Runtime& runtime) {
    // Register IPAddress class
    ClassPtr ipAddressClass = runtime.createClass("IPAddress");
    ipAddressClass->setConstructor(IPAddress_constructor);
    ipAddressClass->addMethod("toString", IPAddress_toString);
    ipAddressClass->addMethod("isLoopback", IPAddress_isLoopback);
    ipAddressClass->addMethod("isPrivate", IPAddress_isPrivate);
    ipAddressClass->addMethod("isMulticast", IPAddress_isMulticast);
    ipAddressClass->addMethod("isValid", IPAddress_isValid);
    
    // Register IPAddress type constants
    ObjectPtr ipAddressType = runtime.createObject();
    ipAddressType->setProperty("IPv4", Value(static_cast<int>(network::IPAddress::Type::IPv4)));
    ipAddressType->setProperty("IPv6", Value(static_cast<int>(network::IPAddress::Type::IPv6)));
    runtime.setGlobal("IPAddressType", Value(ipAddressType));
    
    // Register SocketAddress class
    ClassPtr socketAddressClass = runtime.createClass("SocketAddress");
    socketAddressClass->setConstructor(SocketAddress_constructor);
    socketAddressClass->addMethod("toString", SocketAddress_toString);
    socketAddressClass->addMethod("isValid", SocketAddress_isValid);
    
    // Register Socket class
    ClassPtr socketClass = runtime.createClass("Socket");
    socketClass->setConstructor(Socket_constructor);
    socketClass->addMethod("bind", Socket_bind);
    socketClass->addMethod("connect", Socket_connect);
    socketClass->addMethod("listen", Socket_listen);
    socketClass->addMethod("accept", Socket_accept);
    socketClass->addMethod("send", Socket_send);
    socketClass->addMethod("receive", Socket_receive);
    socketClass->addMethod("sendTo", Socket_sendTo);
    socketClass->addMethod("receiveFrom", Socket_receiveFrom);
    socketClass->addMethod("close", Socket_close);
    socketClass->addMethod("setBlocking", Socket_setBlocking);
    socketClass->addMethod("setReuseAddress", Socket_setReuseAddress);
    socketClass->addMethod("setReceiveTimeout", Socket_setReceiveTimeout);
    socketClass->addMethod("setSendTimeout", Socket_setSendTimeout);
    socketClass->addMethod("getState", Socket_getState);
    socketClass->addMethod("getType", Socket_getType);
    socketClass->addMethod("getLocalAddress", Socket_getLocalAddress);
    socketClass->addMethod("getRemoteAddress", Socket_getRemoteAddress);
    socketClass->addMethod("getLastError", Socket_getLastError);
    
    // Register Socket type constants
    ObjectPtr socketType = runtime.createObject();
    socketType->setProperty("TCP", Value(static_cast<int>(network::Socket::Type::TCP)));
    socketType->setProperty("UDP", Value(static_cast<int>(network::Socket::Type::UDP)));
    runtime.setGlobal("SocketType", Value(socketType));
    
    // Register Socket state constants
    ObjectPtr socketState = runtime.createObject();
    socketState->setProperty("Closed", Value(static_cast<int>(network::Socket::State::Closed)));
    socketState->setProperty("Listening", Value(static_cast<int>(network::Socket::State::Listening)));
    socketState->setProperty("Connected", Value(static_cast<int>(network::Socket::State::Connected)));
    socketState->setProperty("Error", Value(static_cast<int>(network::Socket::State::Error)));
    runtime.setGlobal("SocketState", Value(socketState));
    
    // Register HttpRequest class
    ClassPtr httpRequestClass = runtime.createClass("HttpRequest");
    httpRequestClass->setConstructor(HttpRequest_constructor);
    httpRequestClass->addMethod("setMethod", HttpRequest_setMethod);
    httpRequestClass->addMethod("setUrl", HttpRequest_setUrl);
    httpRequestClass->addMethod("setHeader", HttpRequest_setHeader);
    httpRequestClass->addMethod("setBody", HttpRequest_setBody);
    httpRequestClass->addMethod("setContentType", HttpRequest_setContentType);
    httpRequestClass->addMethod("setTimeout", HttpRequest_setTimeout);
    
    // Register HttpRequest method constants
    ObjectPtr httpMethod = runtime.createObject();
    httpMethod->setProperty("GET", Value(static_cast<int>(network::HttpRequest::Method::GET)));
    httpMethod->setProperty("POST", Value(static_cast<int>(network::HttpRequest::Method::POST)));
    httpMethod->setProperty("PUT", Value(static_cast<int>(network::HttpRequest::Method::PUT)));
    httpMethod->setProperty("DELETE", Value(static_cast<int>(network::HttpRequest::Method::DELETE)));
    httpMethod->setProperty("HEAD", Value(static_cast<int>(network::HttpRequest::Method::HEAD)));
    httpMethod->setProperty("OPTIONS", Value(static_cast<int>(network::HttpRequest::Method::OPTIONS)));
    httpMethod->setProperty("PATCH", Value(static_cast<int>(network::HttpRequest::Method::PATCH)));
    runtime.setGlobal("HttpMethod", Value(httpMethod));
    
    // Register HttpClient class
    ClassPtr httpClientClass = runtime.createClass("HttpClient");
    httpClientClass->setConstructor(HttpClient_constructor);
    httpClientClass->addMethod("send", HttpClient_send);
    httpClientClass->addMethod("get", HttpClient_get);
    httpClientClass->addMethod("post", HttpClient_post);
    httpClientClass->addMethod("put", HttpClient_put);
    httpClientClass->addMethod("del", HttpClient_del);
    httpClientClass->addMethod("setDefaultHeader", HttpClient_setDefaultHeader);
    httpClientClass->addMethod("setDefaultTimeout", HttpClient_setDefaultTimeout);
    httpClientClass->addMethod("setFollowRedirects", HttpClient_setFollowRedirects);
    httpClientClass->addMethod("setMaxRedirects", HttpClient_setMaxRedirects);
    httpClientClass->addMethod("setUserAgent", HttpClient_setUserAgent);
    
    // Register DnsResolver functions
    ObjectPtr dnsResolver = runtime.createObject();
    dnsResolver->setProperty("resolve", Value(runtime.createFunction(DnsResolver_resolve)));
    dnsResolver->setProperty("resolveFirst", Value(runtime.createFunction(DnsResolver_resolveFirst)));
    dnsResolver->setProperty("reverseLookup", Value(runtime.createFunction(DnsResolver_reverseLookup)));
    runtime.setGlobal("DnsResolver", Value(dnsResolver));
    
    // Register NetworkUtils functions
    ObjectPtr networkUtils = runtime.createObject();
    networkUtils->setProperty("getNetworkInterfaces", Value(runtime.createFunction(NetworkUtils_getNetworkInterfaces)));
    networkUtils->setProperty("ping", Value(runtime.createFunction(NetworkUtils_ping)));
    networkUtils->setProperty("getHostname", Value(runtime.createFunction(NetworkUtils_getHostname)));
    networkUtils->setProperty("isPortOpen", Value(runtime.createFunction(NetworkUtils_isPortOpen)));
    runtime.setGlobal("NetworkUtils", Value(networkUtils));
}

} // namespace stdlib
} // namespace astra