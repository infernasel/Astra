#include "network_module.h"

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/types.h>
    #include <ifaddrs.h>
    #include <net/if.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
#endif

namespace astra {
namespace stdlib {
namespace network {

// Initialize Winsock on Windows
#ifdef _WIN32
class WinsockInitializer {
public:
    WinsockInitializer() {
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            std::cerr << "WSAStartup failed: " << result << std::endl;
        }
    }

    ~WinsockInitializer() {
        WSACleanup();
    }
};

static WinsockInitializer winsockInitializer;
#endif

//-----------------------------------------------------------------------------
// IPAddress implementation
//-----------------------------------------------------------------------------

IPAddress::IPAddress() : m_type(Type::IPv4), m_valid(false) {
    m_bytes.resize(4, 0);
}

IPAddress::IPAddress(const std::string& address) : m_valid(false) {
    // Try IPv4 first
    struct in_addr addr4;
    if (inet_pton(AF_INET, address.c_str(), &addr4) == 1) {
        m_type = Type::IPv4;
        m_bytes.resize(4);
        memcpy(m_bytes.data(), &addr4.s_addr, 4);
        m_valid = true;
        return;
    }

    // Try IPv6
    struct in6_addr addr6;
    if (inet_pton(AF_INET6, address.c_str(), &addr6) == 1) {
        m_type = Type::IPv6;
        m_bytes.resize(16);
        memcpy(m_bytes.data(), &addr6.s6_addr, 16);
        m_valid = true;
        return;
    }
}

IPAddress::IPAddress(const std::vector<uint8_t>& bytes, Type type) : m_bytes(bytes), m_type(type) {
    m_valid = (type == Type::IPv4 && bytes.size() == 4) || (type == Type::IPv6 && bytes.size() == 16);
}

IPAddress::Type IPAddress::getType() const {
    return m_type;
}

std::string IPAddress::toString() const {
    if (!m_valid) {
        return "";
    }

    char buffer[INET6_ADDRSTRLEN];
    
    if (m_type == Type::IPv4) {
        struct in_addr addr;
        memcpy(&addr.s_addr, m_bytes.data(), 4);
        inet_ntop(AF_INET, &addr, buffer, INET_ADDRSTRLEN);
    } else {
        struct in6_addr addr;
        memcpy(&addr.s6_addr, m_bytes.data(), 16);
        inet_ntop(AF_INET6, &addr, buffer, INET6_ADDRSTRLEN);
    }
    
    return std::string(buffer);
}

std::vector<uint8_t> IPAddress::getBytes() const {
    return m_bytes;
}

bool IPAddress::isLoopback() const {
    if (!m_valid) {
        return false;
    }

    if (m_type == Type::IPv4) {
        // 127.0.0.0/8
        return m_bytes[0] == 127;
    } else {
        // ::1
        for (size_t i = 0; i < 15; ++i) {
            if (m_bytes[i] != 0) {
                return false;
            }
        }
        return m_bytes[15] == 1;
    }
}

bool IPAddress::isPrivate() const {
    if (!m_valid) {
        return false;
    }

    if (m_type == Type::IPv4) {
        // 10.0.0.0/8
        if (m_bytes[0] == 10) {
            return true;
        }
        
        // 172.16.0.0/12
        if (m_bytes[0] == 172 && (m_bytes[1] >= 16 && m_bytes[1] <= 31)) {
            return true;
        }
        
        // 192.168.0.0/16
        if (m_bytes[0] == 192 && m_bytes[1] == 168) {
            return true;
        }
        
        return false;
    } else {
        // fc00::/7
        return (m_bytes[0] & 0xfe) == 0xfc;
    }
}

bool IPAddress::isMulticast() const {
    if (!m_valid) {
        return false;
    }

    if (m_type == Type::IPv4) {
        // 224.0.0.0/4
        return (m_bytes[0] & 0xf0) == 0xe0;
    } else {
        // ff00::/8
        return m_bytes[0] == 0xff;
    }
}

bool IPAddress::isValid() const {
    return m_valid;
}

//-----------------------------------------------------------------------------
// SocketAddress implementation
//-----------------------------------------------------------------------------

SocketAddress::SocketAddress() : m_port(0) {
}

SocketAddress::SocketAddress(const IPAddress& address, uint16_t port) : m_address(address), m_port(port) {
}

SocketAddress::SocketAddress(const std::string& address, uint16_t port) : m_address(address), m_port(port) {
}

IPAddress SocketAddress::getAddress() const {
    return m_address;
}

uint16_t SocketAddress::getPort() const {
    return m_port;
}

std::string SocketAddress::toString() const {
    std::ostringstream oss;
    if (m_address.getType() == IPAddress::Type::IPv6) {
        oss << "[" << m_address.toString() << "]:" << m_port;
    } else {
        oss << m_address.toString() << ":" << m_port;
    }
    return oss.str();
}

bool SocketAddress::isValid() const {
    return m_address.isValid();
}

//-----------------------------------------------------------------------------
// Socket implementation
//-----------------------------------------------------------------------------

Socket::Socket(Type type) : m_socket(INVALID_SOCKET), m_type(type), m_state(State::Closed) {
}

Socket::~Socket() {
    close();
}

bool Socket::bind(const SocketAddress& address) {
    if (m_state != State::Closed) {
        m_lastError = "Socket is not in closed state";
        return false;
    }

    if (!address.isValid()) {
        m_lastError = "Invalid address";
        return false;
    }

    // Create socket
    int domain = (address.getAddress().getType() == IPAddress::Type::IPv6) ? AF_INET6 : AF_INET;
    int type = (m_type == Type::TCP) ? SOCK_STREAM : SOCK_DGRAM;
    int protocol = (m_type == Type::TCP) ? IPPROTO_TCP : IPPROTO_UDP;
    
    m_socket = socket(domain, type, protocol);
    if (m_socket == INVALID_SOCKET) {
        m_lastError = "Failed to create socket";
        return false;
    }

    // Bind socket
    if (domain == AF_INET) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(address.getPort());
        memcpy(&addr.sin_addr.s_addr, address.getAddress().getBytes().data(), 4);
        
        if (::bind(m_socket, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            m_lastError = "Failed to bind socket";
            closesocket(m_socket);
            m_socket = INVALID_SOCKET;
            return false;
        }
    } else {
        struct sockaddr_in6 addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin6_family = AF_INET6;
        addr.sin6_port = htons(address.getPort());
        memcpy(&addr.sin6_addr.s6_addr, address.getAddress().getBytes().data(), 16);
        
        if (::bind(m_socket, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            m_lastError = "Failed to bind socket";
            closesocket(m_socket);
            m_socket = INVALID_SOCKET;
            return false;
        }
    }

    m_localAddress = address;
    m_state = State::Closed;
    return true;
}

bool Socket::connect(const SocketAddress& address) {
    if (m_state != State::Closed) {
        m_lastError = "Socket is not in closed state";
        return false;
    }

    if (!address.isValid()) {
        m_lastError = "Invalid address";
        return false;
    }

    // Create socket if not already created
    if (m_socket == INVALID_SOCKET) {
        int domain = (address.getAddress().getType() == IPAddress::Type::IPv6) ? AF_INET6 : AF_INET;
        int type = (m_type == Type::TCP) ? SOCK_STREAM : SOCK_DGRAM;
        int protocol = (m_type == Type::TCP) ? IPPROTO_TCP : IPPROTO_UDP;
        
        m_socket = socket(domain, type, protocol);
        if (m_socket == INVALID_SOCKET) {
            m_lastError = "Failed to create socket";
            return false;
        }
    }

    // Connect socket
    if (address.getAddress().getType() == IPAddress::Type::IPv4) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(address.getPort());
        memcpy(&addr.sin_addr.s_addr, address.getAddress().getBytes().data(), 4);
        
        if (::connect(m_socket, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            m_lastError = "Failed to connect socket";
            closesocket(m_socket);
            m_socket = INVALID_SOCKET;
            return false;
        }
    } else {
        struct sockaddr_in6 addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin6_family = AF_INET6;
        addr.sin6_port = htons(address.getPort());
        memcpy(&addr.sin6_addr.s6_addr, address.getAddress().getBytes().data(), 16);
        
        if (::connect(m_socket, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            m_lastError = "Failed to connect socket";
            closesocket(m_socket);
            m_socket = INVALID_SOCKET;
            return false;
        }
    }

    m_remoteAddress = address;
    m_state = State::Connected;
    return true;
}

bool Socket::listen(int backlog) {
    if (m_state != State::Closed) {
        m_lastError = "Socket is not in closed state";
        return false;
    }

    if (m_socket == INVALID_SOCKET) {
        m_lastError = "Socket is not created";
        return false;
    }

    if (m_type != Type::TCP) {
        m_lastError = "Only TCP sockets can listen";
        return false;
    }

    if (::listen(m_socket, backlog) == SOCKET_ERROR) {
        m_lastError = "Failed to listen on socket";
        return false;
    }

    m_state = State::Listening;
    return true;
}

std::shared_ptr<Socket> Socket::accept() {
    if (m_state != State::Listening) {
        m_lastError = "Socket is not in listening state";
        return nullptr;
    }

    struct sockaddr_storage addr;
    socklen_t addrLen = sizeof(addr);
    
    SOCKET clientSocket = ::accept(m_socket, (struct sockaddr*)&addr, &addrLen);
    if (clientSocket == INVALID_SOCKET) {
        m_lastError = "Failed to accept connection";
        return nullptr;
    }

    std::shared_ptr<Socket> client = std::make_shared<Socket>(Type::TCP);
    client->m_socket = clientSocket;
    client->m_state = State::Connected;

    // Set remote address
    if (addr.ss_family == AF_INET) {
        struct sockaddr_in* addr4 = (struct sockaddr_in*)&addr;
        std::vector<uint8_t> bytes(4);
        memcpy(bytes.data(), &addr4->sin_addr.s_addr, 4);
        IPAddress ipAddress(bytes, IPAddress::Type::IPv4);
        client->m_remoteAddress = SocketAddress(ipAddress, ntohs(addr4->sin_port));
    } else {
        struct sockaddr_in6* addr6 = (struct sockaddr_in6*)&addr;
        std::vector<uint8_t> bytes(16);
        memcpy(bytes.data(), &addr6->sin6_addr.s6_addr, 16);
        IPAddress ipAddress(bytes, IPAddress::Type::IPv6);
        client->m_remoteAddress = SocketAddress(ipAddress, ntohs(addr6->sin6_port));
    }

    return client;
}

int Socket::send(const std::vector<uint8_t>& data) {
    if (m_state != State::Connected) {
        m_lastError = "Socket is not connected";
        return -1;
    }

    if (m_type != Type::TCP) {
        m_lastError = "Use sendTo for UDP sockets";
        return -1;
    }

    int result = ::send(m_socket, (const char*)data.data(), data.size(), 0);
    if (result == SOCKET_ERROR) {
        m_lastError = "Failed to send data";
        return -1;
    }

    return result;
}

int Socket::receive(std::vector<uint8_t>& data, size_t maxSize) {
    if (m_state != State::Connected) {
        m_lastError = "Socket is not connected";
        return -1;
    }

    if (m_type != Type::TCP) {
        m_lastError = "Use receiveFrom for UDP sockets";
        return -1;
    }

    data.resize(maxSize);
    int result = ::recv(m_socket, (char*)data.data(), maxSize, 0);
    if (result == SOCKET_ERROR) {
        m_lastError = "Failed to receive data";
        return -1;
    }

    data.resize(result);
    return result;
}

int Socket::sendTo(const std::vector<uint8_t>& data, const SocketAddress& address) {
    if (m_state == State::Error) {
        m_lastError = "Socket is in error state";
        return -1;
    }

    if (m_type != Type::UDP) {
        m_lastError = "Use send for TCP sockets";
        return -1;
    }

    // Create socket if not already created
    if (m_socket == INVALID_SOCKET) {
        int domain = (address.getAddress().getType() == IPAddress::Type::IPv6) ? AF_INET6 : AF_INET;
        m_socket = socket(domain, SOCK_DGRAM, IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET) {
            m_lastError = "Failed to create socket";
            return -1;
        }
    }

    int result;
    if (address.getAddress().getType() == IPAddress::Type::IPv4) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(address.getPort());
        memcpy(&addr.sin_addr.s_addr, address.getAddress().getBytes().data(), 4);
        
        result = ::sendto(m_socket, (const char*)data.data(), data.size(), 0, (struct sockaddr*)&addr, sizeof(addr));
    } else {
        struct sockaddr_in6 addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin6_family = AF_INET6;
        addr.sin6_port = htons(address.getPort());
        memcpy(&addr.sin6_addr.s6_addr, address.getAddress().getBytes().data(), 16);
        
        result = ::sendto(m_socket, (const char*)data.data(), data.size(), 0, (struct sockaddr*)&addr, sizeof(addr));
    }

    if (result == SOCKET_ERROR) {
        m_lastError = "Failed to send data";
        return -1;
    }

    return result;
}

int Socket::receiveFrom(std::vector<uint8_t>& data, size_t maxSize, SocketAddress& address) {
    if (m_state == State::Error) {
        m_lastError = "Socket is in error state";
        return -1;
    }

    if (m_type != Type::UDP) {
        m_lastError = "Use receive for TCP sockets";
        return -1;
    }

    // Create socket if not already created
    if (m_socket == INVALID_SOCKET) {
        m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET) {
            m_lastError = "Failed to create socket";
            return -1;
        }
    }

    data.resize(maxSize);
    struct sockaddr_storage addr;
    socklen_t addrLen = sizeof(addr);
    
    int result = ::recvfrom(m_socket, (char*)data.data(), maxSize, 0, (struct sockaddr*)&addr, &addrLen);
    if (result == SOCKET_ERROR) {
        m_lastError = "Failed to receive data";
        return -1;
    }

    data.resize(result);

    // Set sender address
    if (addr.ss_family == AF_INET) {
        struct sockaddr_in* addr4 = (struct sockaddr_in*)&addr;
        std::vector<uint8_t> bytes(4);
        memcpy(bytes.data(), &addr4->sin_addr.s_addr, 4);
        IPAddress ipAddress(bytes, IPAddress::Type::IPv4);
        address = SocketAddress(ipAddress, ntohs(addr4->sin_port));
    } else {
        struct sockaddr_in6* addr6 = (struct sockaddr_in6*)&addr;
        std::vector<uint8_t> bytes(16);
        memcpy(bytes.data(), &addr6->sin6_addr.s6_addr, 16);
        IPAddress ipAddress(bytes, IPAddress::Type::IPv6);
        address = SocketAddress(ipAddress, ntohs(addr6->sin6_port));
    }

    return result;
}

bool Socket::close() {
    if (m_socket != INVALID_SOCKET) {
        closesocket(m_socket);
        m_socket = INVALID_SOCKET;
    }
    
    m_state = State::Closed;
    return true;
}

bool Socket::setBlocking(bool blocking) {
    if (m_socket == INVALID_SOCKET) {
        m_lastError = "Socket is not created";
        return false;
    }

#ifdef _WIN32
    u_long mode = blocking ? 0 : 1;
    if (ioctlsocket(m_socket, FIONBIO, &mode) != 0) {
        m_lastError = "Failed to set blocking mode";
        return false;
    }
#else
    int flags = fcntl(m_socket, F_GETFL, 0);
    if (flags == -1) {
        m_lastError = "Failed to get socket flags";
        return false;
    }
    
    if (blocking) {
        flags &= ~O_NONBLOCK;
    } else {
        flags |= O_NONBLOCK;
    }
    
    if (fcntl(m_socket, F_SETFL, flags) == -1) {
        m_lastError = "Failed to set socket flags";
        return false;
    }
#endif

    return true;
}

bool Socket::setReuseAddress(bool reuse) {
    if (m_socket == INVALID_SOCKET) {
        m_lastError = "Socket is not created";
        return false;
    }

    int value = reuse ? 1 : 0;
    if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (const char*)&value, sizeof(value)) == SOCKET_ERROR) {
        m_lastError = "Failed to set SO_REUSEADDR";
        return false;
    }

    return true;
}

bool Socket::setReceiveTimeout(int milliseconds) {
    if (m_socket == INVALID_SOCKET) {
        m_lastError = "Socket is not created";
        return false;
    }

#ifdef _WIN32
    DWORD timeout = milliseconds;
#else
    struct timeval timeout;
    timeout.tv_sec = milliseconds / 1000;
    timeout.tv_usec = (milliseconds % 1000) * 1000;
#endif

    if (setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) == SOCKET_ERROR) {
        m_lastError = "Failed to set SO_RCVTIMEO";
        return false;
    }

    return true;
}

bool Socket::setSendTimeout(int milliseconds) {
    if (m_socket == INVALID_SOCKET) {
        m_lastError = "Socket is not created";
        return false;
    }

#ifdef _WIN32
    DWORD timeout = milliseconds;
#else
    struct timeval timeout;
    timeout.tv_sec = milliseconds / 1000;
    timeout.tv_usec = (milliseconds % 1000) * 1000;
#endif

    if (setsockopt(m_socket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout)) == SOCKET_ERROR) {
        m_lastError = "Failed to set SO_SNDTIMEO";
        return false;
    }

    return true;
}

Socket::State Socket::getState() const {
    return m_state;
}

Socket::Type Socket::getType() const {
    return m_type;
}

SocketAddress Socket::getLocalAddress() const {
    return m_localAddress;
}

SocketAddress Socket::getRemoteAddress() const {
    return m_remoteAddress;
}

std::string Socket::getLastError() const {
    return m_lastError;
}

//-----------------------------------------------------------------------------
// HttpRequest implementation
//-----------------------------------------------------------------------------

HttpRequest::HttpRequest() : m_method(Method::GET), m_timeout(30000) {
}

HttpRequest::HttpRequest(Method method, const std::string& url) : m_method(method), m_url(url), m_timeout(30000) {
}

void HttpRequest::setMethod(Method method) {
    m_method = method;
}

HttpRequest::Method HttpRequest::getMethod() const {
    return m_method;
}

void HttpRequest::setUrl(const std::string& url) {
    m_url = url;
}

std::string HttpRequest::getUrl() const {
    return m_url;
}

void HttpRequest::setHeader(const std::string& name, const std::string& value) {
    m_headers[name] = value;
}

std::string HttpRequest::getHeader(const std::string& name) const {
    auto it = m_headers.find(name);
    if (it != m_headers.end()) {
        return it->second;
    }
    return "";
}

std::map<std::string, std::string> HttpRequest::getHeaders() const {
    return m_headers;
}

void HttpRequest::setBody(const std::vector<uint8_t>& body) {
    m_body = body;
}

std::vector<uint8_t> HttpRequest::getBody() const {
    return m_body;
}

void HttpRequest::setContentType(const std::string& contentType) {
    setHeader("Content-Type", contentType);
}

std::string HttpRequest::getContentType() const {
    return getHeader("Content-Type");
}

void HttpRequest::setTimeout(int milliseconds) {
    m_timeout = milliseconds;
}

int HttpRequest::getTimeout() const {
    return m_timeout;
}

//-----------------------------------------------------------------------------
// HttpResponse implementation
//-----------------------------------------------------------------------------

HttpResponse::HttpResponse() : m_statusCode(0) {
}

void HttpResponse::setStatusCode(int statusCode) {
    m_statusCode = statusCode;
}

int HttpResponse::getStatusCode() const {
    return m_statusCode;
}

void HttpResponse::setStatusMessage(const std::string& statusMessage) {
    m_statusMessage = statusMessage;
}

std::string HttpResponse::getStatusMessage() const {
    return m_statusMessage;
}

void HttpResponse::setHeader(const std::string& name, const std::string& value) {
    m_headers[name] = value;
}

std::string HttpResponse::getHeader(const std::string& name) const {
    auto it = m_headers.find(name);
    if (it != m_headers.end()) {
        return it->second;
    }
    return "";
}

std::map<std::string, std::string> HttpResponse::getHeaders() const {
    return m_headers;
}

void HttpResponse::setBody(const std::vector<uint8_t>& body) {
    m_body = body;
}

std::vector<uint8_t> HttpResponse::getBody() const {
    return m_body;
}

void HttpResponse::setContentType(const std::string& contentType) {
    setHeader("Content-Type", contentType);
}

std::string HttpResponse::getContentType() const {
    return getHeader("Content-Type");
}

bool HttpResponse::isSuccess() const {
    return m_statusCode >= 200 && m_statusCode < 300;
}

bool HttpResponse::isRedirect() const {
    return m_statusCode >= 300 && m_statusCode < 400;
}

bool HttpResponse::isClientError() const {
    return m_statusCode >= 400 && m_statusCode < 500;
}

bool HttpResponse::isServerError() const {
    return m_statusCode >= 500 && m_statusCode < 600;
}

//-----------------------------------------------------------------------------
// HttpClient implementation
//-----------------------------------------------------------------------------

HttpClient::HttpClient() : m_defaultTimeout(30000), m_followRedirects(true), m_maxRedirects(5) {
    m_defaultHeaders["User-Agent"] = "ASTRA/1.0";
}

HttpClient::~HttpClient() {
}

HttpResponse HttpClient::send(const HttpRequest& request) {
    HttpResponse response;
    
    // Parse URL
    std::string url = request.getUrl();
    std::string protocol;
    std::string host;
    std::string path;
    int port = 80;
    
    // Extract protocol
    size_t protocolEnd = url.find("://");
    if (protocolEnd != std::string::npos) {
        protocol = url.substr(0, protocolEnd);
        url = url.substr(protocolEnd + 3);
    }
    
    // Extract host and port
    size_t hostEnd = url.find('/');
    if (hostEnd != std::string::npos) {
        host = url.substr(0, hostEnd);
        path = url.substr(hostEnd);
    } else {
        host = url;
        path = "/";
    }
    
    // Extract port from host
    size_t portStart = host.find(':');
    if (portStart != std::string::npos) {
        port = std::stoi(host.substr(portStart + 1));
        host = host.substr(0, portStart);
    } else if (protocol == "https") {
        port = 443;
    }
    
    // Create socket
    Socket socket(Socket::Type::TCP);
    
    // Resolve host
    DnsResolver resolver;
    IPAddress address = resolver.resolveFirst(host);
    if (!address.isValid()) {
        response.setStatusCode(0);
        response.setStatusMessage("Failed to resolve host");
        return response;
    }
    
    // Connect to server
    if (!socket.connect(SocketAddress(address, port))) {
        response.setStatusCode(0);
        response.setStatusMessage("Failed to connect to server");
        return response;
    }
    
    // Set timeout
    socket.setReceiveTimeout(request.getTimeout());
    socket.setSendTimeout(request.getTimeout());
    
    // Build request
    std::string methodStr;
    switch (request.getMethod()) {
        case HttpRequest::Method::GET:
            methodStr = "GET";
            break;
        case HttpRequest::Method::POST:
            methodStr = "POST";
            break;
        case HttpRequest::Method::PUT:
            methodStr = "PUT";
            break;
        case HttpRequest::Method::DELETE:
            methodStr = "DELETE";
            break;
        case HttpRequest::Method::HEAD:
            methodStr = "HEAD";
            break;
        case HttpRequest::Method::OPTIONS:
            methodStr = "OPTIONS";
            break;
        case HttpRequest::Method::PATCH:
            methodStr = "PATCH";
            break;
    }
    
    std::ostringstream requestStream;
    requestStream << methodStr << " " << path << " HTTP/1.1\r\n";
    requestStream << "Host: " << host << "\r\n";
    
    // Add default headers
    for (const auto& header : m_defaultHeaders) {
        // Skip headers that are overridden by the request
        if (request.getHeader(header.first).empty()) {
            requestStream << header.first << ": " << header.second << "\r\n";
        }
    }
    
    // Add request headers
    for (const auto& header : request.getHeaders()) {
        requestStream << header.first << ": " << header.second << "\r\n";
    }
    
    // Add content length if body is not empty
    if (!request.getBody().empty()) {
        requestStream << "Content-Length: " << request.getBody().size() << "\r\n";
    }
    
    // End headers
    requestStream << "\r\n";
    
    // Convert request to bytes
    std::string requestStr = requestStream.str();
    std::vector<uint8_t> requestData(requestStr.begin(), requestStr.end());
    
    // Add body if not empty
    if (!request.getBody().empty()) {
        requestData.insert(requestData.end(), request.getBody().begin(), request.getBody().end());
    }
    
    // Send request
    if (socket.send(requestData) < 0) {
        response.setStatusCode(0);
        response.setStatusMessage("Failed to send request");
        return response;
    }
    
    // Receive response
    std::vector<uint8_t> responseData;
    std::vector<uint8_t> buffer(4096);
    int bytesRead;
    
    while ((bytesRead = socket.receive(buffer, buffer.size())) > 0) {
        responseData.insert(responseData.end(), buffer.begin(), buffer.begin() + bytesRead);
    }
    
    // Parse response
    if (responseData.empty()) {
        response.setStatusCode(0);
        response.setStatusMessage("Empty response");
        return response;
    }
    
    // Convert response to string for parsing
    std::string responseStr(responseData.begin(), responseData.end());
    
    // Find end of headers
    size_t headersEnd = responseStr.find("\r\n\r\n");
    if (headersEnd == std::string::npos) {
        response.setStatusCode(0);
        response.setStatusMessage("Invalid response format");
        return response;
    }
    
    // Extract headers
    std::string headersStr = responseStr.substr(0, headersEnd);
    
    // Extract body
    std::vector<uint8_t> body(responseData.begin() + headersEnd + 4, responseData.end());
    response.setBody(body);
    
    // Parse status line
    size_t statusLineEnd = headersStr.find("\r\n");
    if (statusLineEnd == std::string::npos) {
        response.setStatusCode(0);
        response.setStatusMessage("Invalid response format");
        return response;
    }
    
    std::string statusLine = headersStr.substr(0, statusLineEnd);
    
    // Parse HTTP version and status code
    size_t httpVersionEnd = statusLine.find(' ');
    if (httpVersionEnd == std::string::npos) {
        response.setStatusCode(0);
        response.setStatusMessage("Invalid response format");
        return response;
    }
    
    size_t statusCodeEnd = statusLine.find(' ', httpVersionEnd + 1);
    if (statusCodeEnd == std::string::npos) {
        response.setStatusCode(0);
        response.setStatusMessage("Invalid response format");
        return response;
    }
    
    std::string statusCodeStr = statusLine.substr(httpVersionEnd + 1, statusCodeEnd - httpVersionEnd - 1);
    std::string statusMessage = statusLine.substr(statusCodeEnd + 1);
    
    response.setStatusCode(std::stoi(statusCodeStr));
    response.setStatusMessage(statusMessage);
    
    // Parse headers
    std::string headerLines = headersStr.substr(statusLineEnd + 2);
    size_t pos = 0;
    std::string line;
    
    while ((pos = headerLines.find("\r\n")) != std::string::npos) {
        line = headerLines.substr(0, pos);
        headerLines = headerLines.substr(pos + 2);
        
        size_t colonPos = line.find(':');
        if (colonPos != std::string::npos) {
            std::string name = line.substr(0, colonPos);
            std::string value = line.substr(colonPos + 1);
            
            // Trim leading and trailing whitespace from value
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            response.setHeader(name, value);
        }
    }
    
    // Handle last header line
    if (!headerLines.empty()) {
        size_t colonPos = headerLines.find(':');
        if (colonPos != std::string::npos) {
            std::string name = headerLines.substr(0, colonPos);
            std::string value = headerLines.substr(colonPos + 1);
            
            // Trim leading and trailing whitespace from value
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            response.setHeader(name, value);
        }
    }
    
    // Handle redirects
    if (m_followRedirects && response.isRedirect() && response.getHeader("Location") != "") {
        static int redirectCount = 0;
        
        if (redirectCount < m_maxRedirects) {
            redirectCount++;
            
            HttpRequest redirectRequest = request;
            redirectRequest.setUrl(response.getHeader("Location"));
            
            HttpResponse redirectResponse = send(redirectRequest);
            
            redirectCount--;
            return redirectResponse;
        }
    }
    
    return response;
}

HttpResponse HttpClient::get(const std::string& url) {
    HttpRequest request(HttpRequest::Method::GET, url);
    return send(request);
}

HttpResponse HttpClient::post(const std::string& url, const std::vector<uint8_t>& body, const std::string& contentType) {
    HttpRequest request(HttpRequest::Method::POST, url);
    request.setBody(body);
    request.setContentType(contentType);
    return send(request);
}

HttpResponse HttpClient::put(const std::string& url, const std::vector<uint8_t>& body, const std::string& contentType) {
    HttpRequest request(HttpRequest::Method::PUT, url);
    request.setBody(body);
    request.setContentType(contentType);
    return send(request);
}

HttpResponse HttpClient::del(const std::string& url) {
    HttpRequest request(HttpRequest::Method::DELETE, url);
    return send(request);
}

void HttpClient::setDefaultHeader(const std::string& name, const std::string& value) {
    m_defaultHeaders[name] = value;
}

void HttpClient::setDefaultTimeout(int milliseconds) {
    m_defaultTimeout = milliseconds;
}

void HttpClient::setFollowRedirects(bool follow) {
    m_followRedirects = follow;
}

void HttpClient::setMaxRedirects(int maxRedirects) {
    m_maxRedirects = maxRedirects;
}

void HttpClient::setUserAgent(const std::string& userAgent) {
    m_defaultHeaders["User-Agent"] = userAgent;
}

//-----------------------------------------------------------------------------
// HttpServer implementation
//-----------------------------------------------------------------------------

HttpServer::HttpServer() : m_running(false) {
    m_defaultHandler = [](const HttpRequest&) {
        HttpResponse response;
        response.setStatusCode(404);
        response.setStatusMessage("Not Found");
        return response;
    };
}

HttpServer::~HttpServer() {
    stop();
}

bool HttpServer::start(uint16_t port, const std::string& host) {
    if (m_running) {
        return false;
    }

    // Create socket
    m_socket = std::make_shared<Socket>(Socket::Type::TCP);
    
    // Bind socket
    if (!m_socket->bind(SocketAddress(host, port))) {
        return false;
    }
    
    // Set socket options
    m_socket->setReuseAddress(true);
    
    // Listen for connections
    if (!m_socket->listen(10)) {
        return false;
    }
    
    m_running = true;
    
    // Start accept thread
    m_threads.push_back(std::thread([this]() {
        while (m_running) {
            // Accept connection
            std::shared_ptr<Socket> client = m_socket->accept();
            if (client) {
                // Handle client in a new thread
                m_threads.push_back(std::thread([this, client]() {
                    handleClient(client);
                }));
            }
        }
    }));
    
    return true;
}

void HttpServer::stop() {
    if (!m_running) {
        return;
    }
    
    m_running = false;
    
    // Close socket
    if (m_socket) {
        m_socket->close();
    }
    
    // Wait for threads to finish
    for (auto& thread : m_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    m_threads.clear();
}

bool HttpServer::isRunning() const {
    return m_running;
}

void HttpServer::setHandler(const std::string& path, HttpRequest::Method method, RequestHandler handler) {
    m_handlers[path][method] = handler;
}

void HttpServer::setDefaultHandler(RequestHandler handler) {
    m_defaultHandler = handler;
}

void HttpServer::handleClient(std::shared_ptr<Socket> client) {
    // Receive request
    std::vector<uint8_t> requestData;
    std::vector<uint8_t> buffer(4096);
    int bytesRead;
    
    while ((bytesRead = client->receive(buffer, buffer.size())) > 0) {
        requestData.insert(requestData.end(), buffer.begin(), buffer.begin() + bytesRead);
        
        // Check if we have received the complete request
        std::string requestStr(requestData.begin(), requestData.end());
        if (requestStr.find("\r\n\r\n") != std::string::npos) {
            break;
        }
    }
    
    // Parse request
    HttpRequest request;
    
    // Convert request to string for parsing
    std::string requestStr(requestData.begin(), requestData.end());
    
    // Find end of headers
    size_t headersEnd = requestStr.find("\r\n\r\n");
    if (headersEnd == std::string::npos) {
        // Invalid request format
        HttpResponse response;
        response.setStatusCode(400);
        response.setStatusMessage("Bad Request");
        sendResponse(client, response);
        return;
    }
    
    // Extract headers
    std::string headersStr = requestStr.substr(0, headersEnd);
    
    // Extract body
    std::vector<uint8_t> body(requestData.begin() + headersEnd + 4, requestData.end());
    request.setBody(body);
    
    // Parse request line
    size_t requestLineEnd = headersStr.find("\r\n");
    if (requestLineEnd == std::string::npos) {
        // Invalid request format
        HttpResponse response;
        response.setStatusCode(400);
        response.setStatusMessage("Bad Request");
        sendResponse(client, response);
        return;
    }
    
    std::string requestLine = headersStr.substr(0, requestLineEnd);
    
    // Parse method, path, and HTTP version
    size_t methodEnd = requestLine.find(' ');
    if (methodEnd == std::string::npos) {
        // Invalid request format
        HttpResponse response;
        response.setStatusCode(400);
        response.setStatusMessage("Bad Request");
        sendResponse(client, response);
        return;
    }
    
    size_t pathEnd = requestLine.find(' ', methodEnd + 1);
    if (pathEnd == std::string::npos) {
        // Invalid request format
        HttpResponse response;
        response.setStatusCode(400);
        response.setStatusMessage("Bad Request");
        sendResponse(client, response);
        return;
    }
    
    std::string methodStr = requestLine.substr(0, methodEnd);
    std::string path = requestLine.substr(methodEnd + 1, pathEnd - methodEnd - 1);
    
    // Parse method
    HttpRequest::Method method;
    if (methodStr == "GET") {
        method = HttpRequest::Method::GET;
    } else if (methodStr == "POST") {
        method = HttpRequest::Method::POST;
    } else if (methodStr == "PUT") {
        method = HttpRequest::Method::PUT;
    } else if (methodStr == "DELETE") {
        method = HttpRequest::Method::DELETE;
    } else if (methodStr == "HEAD") {
        method = HttpRequest::Method::HEAD;
    } else if (methodStr == "OPTIONS") {
        method = HttpRequest::Method::OPTIONS;
    } else if (methodStr == "PATCH") {
        method = HttpRequest::Method::PATCH;
    } else {
        // Unsupported method
        HttpResponse response;
        response.setStatusCode(405);
        response.setStatusMessage("Method Not Allowed");
        sendResponse(client, response);
        return;
    }
    
    request.setMethod(method);
    request.setUrl(path);
    
    // Parse headers
    std::string headerLines = headersStr.substr(requestLineEnd + 2);
    size_t pos = 0;
    std::string line;
    
    while ((pos = headerLines.find("\r\n")) != std::string::npos) {
        line = headerLines.substr(0, pos);
        headerLines = headerLines.substr(pos + 2);
        
        size_t colonPos = line.find(':');
        if (colonPos != std::string::npos) {
            std::string name = line.substr(0, colonPos);
            std::string value = line.substr(colonPos + 1);
            
            // Trim leading and trailing whitespace from value
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            request.setHeader(name, value);
        }
    }
    
    // Handle last header line
    if (!headerLines.empty()) {
        size_t colonPos = headerLines.find(':');
        if (colonPos != std::string::npos) {
            std::string name = headerLines.substr(0, colonPos);
            std::string value = headerLines.substr(colonPos + 1);
            
            // Trim leading and trailing whitespace from value
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            request.setHeader(name, value);
        }
    }
    
    // Find handler for path and method
    auto pathIt = m_handlers.find(path);
    if (pathIt != m_handlers.end()) {
        auto methodIt = pathIt->second.find(method);
        if (methodIt != pathIt->second.end()) {
            // Call handler
            HttpResponse response = methodIt->second(request);
            sendResponse(client, response);
            return;
        }
    }
    
    // No handler found, call default handler
    HttpResponse response = m_defaultHandler(request);
    sendResponse(client, response);
}

void HttpServer::sendResponse(std::shared_ptr<Socket> client, const HttpResponse& response) {
    // Build response
    std::ostringstream responseStream;
    responseStream << "HTTP/1.1 " << response.getStatusCode() << " " << response.getStatusMessage() << "\r\n";
    
    // Add headers
    for (const auto& header : response.getHeaders()) {
        responseStream << header.first << ": " << header.second << "\r\n";
    }
    
    // Add content length if body is not empty
    if (!response.getBody().empty()) {
        responseStream << "Content-Length: " << response.getBody().size() << "\r\n";
    }
    
    // End headers
    responseStream << "\r\n";
    
    // Convert response to bytes
    std::string responseStr = responseStream.str();
    std::vector<uint8_t> responseData(responseStr.begin(), responseStr.end());
    
    // Add body if not empty
    if (!response.getBody().empty()) {
        responseData.insert(responseData.end(), response.getBody().begin(), response.getBody().end());
    }
    
    // Send response
    client->send(responseData);
}

//-----------------------------------------------------------------------------
// WebSocketClient implementation
//-----------------------------------------------------------------------------

WebSocketClient::WebSocketClient() : m_connected(false) {
    m_messageHandler = [](const std::vector<uint8_t>&, bool) {};
    m_errorHandler = [](const std::string&) {};
    m_connectHandler = []() {};
    m_closeHandler = [](int, const std::string&) {};
}

WebSocketClient::~WebSocketClient() {
    close();
}

bool WebSocketClient::connect(const std::string& url) {
    if (m_connected) {
        return false;
    }

    // Parse URL
    std::string protocol;
    std::string host;
    std::string path;
    int port = 80;
    
    // Extract protocol
    size_t protocolEnd = url.find("://");
    if (protocolEnd != std::string::npos) {
        protocol = url.substr(0, protocolEnd);
        std::string remaining = url.substr(protocolEnd + 3);
        
        // Extract host and port
        size_t hostEnd = remaining.find('/');
        if (hostEnd != std::string::npos) {
            host = remaining.substr(0, hostEnd);
            path = remaining.substr(hostEnd);
        } else {
            host = remaining;
            path = "/";
        }
        
        // Extract port from host
        size_t portStart = host.find(':');
        if (portStart != std::string::npos) {
            port = std::stoi(host.substr(portStart + 1));
            host = host.substr(0, portStart);
        } else if (protocol == "wss") {
            port = 443;
        }
    } else {
        // Invalid URL
        m_errorHandler("Invalid WebSocket URL");
        return false;
    }
    
    // Create socket
    m_socket = std::make_shared<Socket>(Socket::Type::TCP);
    
    // Resolve host
    DnsResolver resolver;
    IPAddress address = resolver.resolveFirst(host);
    if (!address.isValid()) {
        m_errorHandler("Failed to resolve host");
        return false;
    }
    
    // Connect to server
    if (!m_socket->connect(SocketAddress(address, port))) {
        m_errorHandler("Failed to connect to server");
        return false;
    }
    
    // Perform WebSocket handshake
    // TODO: Implement WebSocket handshake
    
    m_connected = true;
    
    // Call connect handler
    m_connectHandler();
    
    // Start receive thread
    m_receiveThread = std::thread([this]() {
        // TODO: Implement WebSocket message receiving
    });
    
    return true;
}

bool WebSocketClient::send(const std::vector<uint8_t>& data, bool binary) {
    if (!m_connected) {
        return false;
    }

    // TODO: Implement WebSocket message sending
    
    return true;
}

bool WebSocketClient::close(int code, const std::string& reason) {
    if (!m_connected) {
        return false;
    }

    // TODO: Implement WebSocket close
    
    m_connected = false;
    
    // Wait for receive thread to finish
    if (m_receiveThread.joinable()) {
        m_receiveThread.join();
    }
    
    // Close socket
    if (m_socket) {
        m_socket->close();
    }
    
    // Call close handler
    m_closeHandler(code, reason);
    
    return true;
}

bool WebSocketClient::isConnected() const {
    return m_connected;
}

void WebSocketClient::setMessageHandler(MessageHandler handler) {
    m_messageHandler = handler;
}

void WebSocketClient::setErrorHandler(ErrorHandler handler) {
    m_errorHandler = handler;
}

void WebSocketClient::setConnectHandler(ConnectHandler handler) {
    m_connectHandler = handler;
}

void WebSocketClient::setCloseHandler(CloseHandler handler) {
    m_closeHandler = handler;
}

//-----------------------------------------------------------------------------
// WebSocketServer implementation
//-----------------------------------------------------------------------------

WebSocketServer::WebSocketServer() : m_running(false) {
    m_messageHandler = [](int, const std::vector<uint8_t>&, bool) {};
    m_errorHandler = [](int, const std::string&) {};
    m_connectHandler = [](int) {};
    m_closeHandler = [](int, int, const std::string&) {};
}

WebSocketServer::~WebSocketServer() {
    stop();
}

bool WebSocketServer::start(uint16_t port, const std::string& host) {
    if (m_running) {
        return false;
    }

    // Create socket
    m_socket = std::make_shared<Socket>(Socket::Type::TCP);
    
    // Bind socket
    if (!m_socket->bind(SocketAddress(host, port))) {
        return false;
    }
    
    // Set socket options
    m_socket->setReuseAddress(true);
    
    // Listen for connections
    if (!m_socket->listen(10)) {
        return false;
    }
    
    m_running = true;
    
    // Start accept thread
    m_acceptThread = std::thread([this]() {
        while (m_running) {
            // Accept connection
            std::shared_ptr<Socket> client = m_socket->accept();
            if (client) {
                // Generate client ID
                int clientId = generateClientId();
                
                // Store client socket
                m_clients[clientId] = client;
                
                // Call connect handler
                m_connectHandler(clientId);
                
                // Handle client in a new thread
                m_receiveThreads.push_back(std::thread([this, clientId, client]() {
                    handleClient(clientId, client);
                }));
            }
        }
    });
    
    return true;
}

void WebSocketServer::stop() {
    if (!m_running) {
        return;
    }
    
    m_running = false;
    
    // Close all client connections
    for (auto& client : m_clients) {
        client.second->close();
    }
    
    m_clients.clear();
    
    // Close server socket
    if (m_socket) {
        m_socket->close();
    }
    
    // Wait for accept thread to finish
    if (m_acceptThread.joinable()) {
        m_acceptThread.join();
    }
    
    // Wait for receive threads to finish
    for (auto& thread : m_receiveThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    m_receiveThreads.clear();
}

bool WebSocketServer::isRunning() const {
    return m_running;
}

bool WebSocketServer::send(int clientId, const std::vector<uint8_t>& data, bool binary) {
    auto it = m_clients.find(clientId);
    if (it == m_clients.end()) {
        return false;
    }

    // TODO: Implement WebSocket message sending
    
    return true;
}

bool WebSocketServer::close(int clientId, int code, const std::string& reason) {
    auto it = m_clients.find(clientId);
    if (it == m_clients.end()) {
        return false;
    }

    // TODO: Implement WebSocket close
    
    // Remove client
    m_clients.erase(it);
    
    // Call close handler
    m_closeHandler(clientId, code, reason);
    
    return true;
}

bool WebSocketServer::broadcast(const std::vector<uint8_t>& data, bool binary) {
    bool success = true;
    
    for (auto& client : m_clients) {
        if (!send(client.first, data, binary)) {
            success = false;
        }
    }
    
    return success;
}

void WebSocketServer::setMessageHandler(MessageHandler handler) {
    m_messageHandler = handler;
}

void WebSocketServer::setErrorHandler(ErrorHandler handler) {
    m_errorHandler = handler;
}

void WebSocketServer::setConnectHandler(ConnectHandler handler) {
    m_connectHandler = handler;
}

void WebSocketServer::setCloseHandler(CloseHandler handler) {
    m_closeHandler = handler;
}

int WebSocketServer::generateClientId() {
    static int nextId = 1;
    return nextId++;
}

void WebSocketServer::handleClient(int clientId, std::shared_ptr<Socket> client) {
    // Perform WebSocket handshake
    // TODO: Implement WebSocket handshake
    
    // Receive messages
    // TODO: Implement WebSocket message receiving
}

//-----------------------------------------------------------------------------
// DnsResolver implementation
//-----------------------------------------------------------------------------

DnsResolver::DnsResolver() {
}

DnsResolver::~DnsResolver() {
}

std::vector<IPAddress> DnsResolver::resolve(const std::string& hostname) {
    std::vector<IPAddress> addresses;
    
    struct addrinfo hints, *result, *rp;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;    // Allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM;
    
    int status = getaddrinfo(hostname.c_str(), NULL, &hints, &result);
    if (status != 0) {
        m_lastError = gai_strerror(status);
        return addresses;
    }
    
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        if (rp->ai_family == AF_INET) {
            struct sockaddr_in* addr = (struct sockaddr_in*)rp->ai_addr;
            std::vector<uint8_t> bytes(4);
            memcpy(bytes.data(), &addr->sin_addr.s_addr, 4);
            addresses.push_back(IPAddress(bytes, IPAddress::Type::IPv4));
        } else if (rp->ai_family == AF_INET6) {
            struct sockaddr_in6* addr = (struct sockaddr_in6*)rp->ai_addr;
            std::vector<uint8_t> bytes(16);
            memcpy(bytes.data(), &addr->sin6_addr.s6_addr, 16);
            addresses.push_back(IPAddress(bytes, IPAddress::Type::IPv6));
        }
    }
    
    freeaddrinfo(result);
    return addresses;
}

IPAddress DnsResolver::resolveFirst(const std::string& hostname) {
    std::vector<IPAddress> addresses = resolve(hostname);
    if (!addresses.empty()) {
        return addresses[0];
    }
    return IPAddress();
}

std::string DnsResolver::reverseLookup(const IPAddress& address) {
    if (!address.isValid()) {
        m_lastError = "Invalid IP address";
        return "";
    }
    
    char hostname[NI_MAXHOST];
    
    if (address.getType() == IPAddress::Type::IPv4) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        memcpy(&addr.sin_addr.s_addr, address.getBytes().data(), 4);
        
        int status = getnameinfo((struct sockaddr*)&addr, sizeof(addr), hostname, NI_MAXHOST, NULL, 0, NI_NAMEREQD);
        if (status != 0) {
            m_lastError = gai_strerror(status);
            return "";
        }
    } else {
        struct sockaddr_in6 addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin6_family = AF_INET6;
        memcpy(&addr.sin6_addr.s6_addr, address.getBytes().data(), 16);
        
        int status = getnameinfo((struct sockaddr*)&addr, sizeof(addr), hostname, NI_MAXHOST, NULL, 0, NI_NAMEREQD);
        if (status != 0) {
            m_lastError = gai_strerror(status);
            return "";
        }
    }
    
    return std::string(hostname);
}

//-----------------------------------------------------------------------------
// NetworkInterface implementation
//-----------------------------------------------------------------------------

NetworkInterface::NetworkInterface() : m_up(false), m_loopback(false) {
}

NetworkInterface::NetworkInterface(const std::string& name, const IPAddress& address, const IPAddress& netmask, const IPAddress& broadcast)
    : m_name(name), m_address(address), m_netmask(netmask), m_broadcast(broadcast), m_up(true), m_loopback(address.isLoopback()) {
}

std::string NetworkInterface::getName() const {
    return m_name;
}

IPAddress NetworkInterface::getAddress() const {
    return m_address;
}

IPAddress NetworkInterface::getNetmask() const {
    return m_netmask;
}

IPAddress NetworkInterface::getBroadcast() const {
    return m_broadcast;
}

bool NetworkInterface::isUp() const {
    return m_up;
}

bool NetworkInterface::isLoopback() const {
    return m_loopback;
}

//-----------------------------------------------------------------------------
// NetworkUtils implementation
//-----------------------------------------------------------------------------

std::vector<NetworkInterface> NetworkUtils::getNetworkInterfaces() {
    std::vector<NetworkInterface> interfaces;
    
#ifdef _WIN32
    // TODO: Implement for Windows
#else
    struct ifaddrs *ifaddr, *ifa;
    
    if (getifaddrs(&ifaddr) == -1) {
        return interfaces;
    }
    
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) {
            continue;
        }
        
        std::string name = ifa->ifa_name;
        bool up = (ifa->ifa_flags & IFF_UP) != 0;
        bool loopback = (ifa->ifa_flags & IFF_LOOPBACK) != 0;
        
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in* addr = (struct sockaddr_in*)ifa->ifa_addr;
            struct sockaddr_in* netmask = (struct sockaddr_in*)ifa->ifa_netmask;
            struct sockaddr_in* broadcast = (struct sockaddr_in*)ifa->ifa_broadaddr;
            
            std::vector<uint8_t> addrBytes(4);
            std::vector<uint8_t> netmaskBytes(4);
            std::vector<uint8_t> broadcastBytes(4);
            
            memcpy(addrBytes.data(), &addr->sin_addr.s_addr, 4);
            memcpy(netmaskBytes.data(), &netmask->sin_addr.s_addr, 4);
            
            if (broadcast != NULL) {
                memcpy(broadcastBytes.data(), &broadcast->sin_addr.s_addr, 4);
            }
            
            IPAddress ipAddress(addrBytes, IPAddress::Type::IPv4);
            IPAddress ipNetmask(netmaskBytes, IPAddress::Type::IPv4);
            IPAddress ipBroadcast(broadcastBytes, IPAddress::Type::IPv4);
            
            NetworkInterface interface(name, ipAddress, ipNetmask, ipBroadcast);
            interfaces.push_back(interface);
        } else if (ifa->ifa_addr->sa_family == AF_INET6) {
            struct sockaddr_in6* addr = (struct sockaddr_in6*)ifa->ifa_addr;
            struct sockaddr_in6* netmask = (struct sockaddr_in6*)ifa->ifa_netmask;
            
            std::vector<uint8_t> addrBytes(16);
            std::vector<uint8_t> netmaskBytes(16);
            
            memcpy(addrBytes.data(), &addr->sin6_addr.s6_addr, 16);
            memcpy(netmaskBytes.data(), &netmask->sin6_addr.s6_addr, 16);
            
            IPAddress ipAddress(addrBytes, IPAddress::Type::IPv6);
            IPAddress ipNetmask(netmaskBytes, IPAddress::Type::IPv6);
            
            NetworkInterface interface(name, ipAddress, ipNetmask, IPAddress());
            interfaces.push_back(interface);
        }
    }
    
    freeifaddrs(ifaddr);
#endif
    
    return interfaces;
}

bool NetworkUtils::ping(const IPAddress& address, int timeout) {
    // TODO: Implement ping
    return false;
}

bool NetworkUtils::ping(const std::string& hostname, int timeout) {
    DnsResolver resolver;
    IPAddress address = resolver.resolveFirst(hostname);
    if (!address.isValid()) {
        return false;
    }
    
    return ping(address, timeout);
}

int NetworkUtils::getHostname(std::string& hostname) {
    char buffer[256];
    
    if (gethostname(buffer, sizeof(buffer)) != 0) {
        return -1;
    }
    
    hostname = std::string(buffer);
    return 0;
}

bool NetworkUtils::isPortOpen(const IPAddress& address, uint16_t port, int timeout) {
    Socket socket(Socket::Type::TCP);
    
    // Set timeout
    socket.setConnectTimeout(timeout);
    
    // Try to connect
    return socket.connect(SocketAddress(address, port));
}

bool NetworkUtils::isPortOpen(const std::string& hostname, uint16_t port, int timeout) {
    DnsResolver resolver;
    IPAddress address = resolver.resolveFirst(hostname);
    if (!address.isValid()) {
        return false;
    }
    
    return isPortOpen(address, port, timeout);
}

} // namespace network
} // namespace stdlib
} // namespace astra