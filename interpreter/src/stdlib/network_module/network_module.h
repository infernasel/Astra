#ifndef ASTRA_NETWORK_MODULE_H
#define ASTRA_NETWORK_MODULE_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <iostream>

namespace astra {
namespace stdlib {
namespace network {

/**
 * @brief Represents an IP address (IPv4 or IPv6)
 */
class IPAddress {
public:
    enum class Type {
        IPv4,
        IPv6
    };

    IPAddress();
    IPAddress(const std::string& address);
    IPAddress(const std::vector<uint8_t>& bytes, Type type);

    Type getType() const;
    std::string toString() const;
    std::vector<uint8_t> getBytes() const;
    bool isLoopback() const;
    bool isPrivate() const;
    bool isMulticast() const;
    bool isValid() const;

private:
    std::vector<uint8_t> m_bytes;
    Type m_type;
    bool m_valid;
};

/**
 * @brief Represents a socket address (IP address and port)
 */
class SocketAddress {
public:
    SocketAddress();
    SocketAddress(const IPAddress& address, uint16_t port);
    SocketAddress(const std::string& address, uint16_t port);

    IPAddress getAddress() const;
    uint16_t getPort() const;
    std::string toString() const;
    bool isValid() const;

private:
    IPAddress m_address;
    uint16_t m_port;
};

/**
 * @brief Represents a socket
 */
class Socket {
public:
    enum class Type {
        TCP,
        UDP
    };

    enum class State {
        Closed,
        Listening,
        Connected,
        Error
    };

    Socket(Type type);
    ~Socket();

    bool bind(const SocketAddress& address);
    bool connect(const SocketAddress& address);
    bool listen(int backlog);
    std::shared_ptr<Socket> accept();
    
    int send(const std::vector<uint8_t>& data);
    int receive(std::vector<uint8_t>& data, size_t maxSize);
    
    int sendTo(const std::vector<uint8_t>& data, const SocketAddress& address);
    int receiveFrom(std::vector<uint8_t>& data, size_t maxSize, SocketAddress& address);
    
    bool close();
    
    bool setBlocking(bool blocking);
    bool setReuseAddress(bool reuse);
    bool setReceiveTimeout(int milliseconds);
    bool setSendTimeout(int milliseconds);
    
    State getState() const;
    Type getType() const;
    SocketAddress getLocalAddress() const;
    SocketAddress getRemoteAddress() const;
    std::string getLastError() const;

private:
    int m_socket;
    Type m_type;
    State m_state;
    SocketAddress m_localAddress;
    SocketAddress m_remoteAddress;
    std::string m_lastError;
};

/**
 * @brief Represents an HTTP request
 */
class HttpRequest {
public:
    enum class Method {
        GET,
        POST,
        PUT,
        DELETE,
        HEAD,
        OPTIONS,
        PATCH
    };

    HttpRequest();
    HttpRequest(Method method, const std::string& url);

    void setMethod(Method method);
    Method getMethod() const;

    void setUrl(const std::string& url);
    std::string getUrl() const;

    void setHeader(const std::string& name, const std::string& value);
    std::string getHeader(const std::string& name) const;
    std::map<std::string, std::string> getHeaders() const;

    void setBody(const std::vector<uint8_t>& body);
    std::vector<uint8_t> getBody() const;

    void setContentType(const std::string& contentType);
    std::string getContentType() const;

    void setTimeout(int milliseconds);
    int getTimeout() const;

private:
    Method m_method;
    std::string m_url;
    std::map<std::string, std::string> m_headers;
    std::vector<uint8_t> m_body;
    int m_timeout;
};

/**
 * @brief Represents an HTTP response
 */
class HttpResponse {
public:
    HttpResponse();

    void setStatusCode(int statusCode);
    int getStatusCode() const;

    void setStatusMessage(const std::string& statusMessage);
    std::string getStatusMessage() const;

    void setHeader(const std::string& name, const std::string& value);
    std::string getHeader(const std::string& name) const;
    std::map<std::string, std::string> getHeaders() const;

    void setBody(const std::vector<uint8_t>& body);
    std::vector<uint8_t> getBody() const;

    void setContentType(const std::string& contentType);
    std::string getContentType() const;

    bool isSuccess() const;
    bool isRedirect() const;
    bool isClientError() const;
    bool isServerError() const;

private:
    int m_statusCode;
    std::string m_statusMessage;
    std::map<std::string, std::string> m_headers;
    std::vector<uint8_t> m_body;
};

/**
 * @brief HTTP client for making HTTP requests
 */
class HttpClient {
public:
    HttpClient();
    ~HttpClient();

    HttpResponse send(const HttpRequest& request);
    HttpResponse get(const std::string& url);
    HttpResponse post(const std::string& url, const std::vector<uint8_t>& body, const std::string& contentType = "application/octet-stream");
    HttpResponse put(const std::string& url, const std::vector<uint8_t>& body, const std::string& contentType = "application/octet-stream");
    HttpResponse del(const std::string& url);

    void setDefaultHeader(const std::string& name, const std::string& value);
    void setDefaultTimeout(int milliseconds);
    void setFollowRedirects(bool follow);
    void setMaxRedirects(int maxRedirects);
    void setUserAgent(const std::string& userAgent);

private:
    std::map<std::string, std::string> m_defaultHeaders;
    int m_defaultTimeout;
    bool m_followRedirects;
    int m_maxRedirects;
};

/**
 * @brief HTTP server for handling HTTP requests
 */
class HttpServer {
public:
    using RequestHandler = std::function<HttpResponse(const HttpRequest&)>;

    HttpServer();
    ~HttpServer();

    bool start(uint16_t port, const std::string& host = "0.0.0.0");
    void stop();
    bool isRunning() const;

    void setHandler(const std::string& path, HttpRequest::Method method, RequestHandler handler);
    void setDefaultHandler(RequestHandler handler);

private:
    bool m_running;
    std::map<std::string, std::map<HttpRequest::Method, RequestHandler>> m_handlers;
    RequestHandler m_defaultHandler;
    std::shared_ptr<Socket> m_socket;
    std::vector<std::thread> m_threads;
};

/**
 * @brief WebSocket client for WebSocket communication
 */
class WebSocketClient {
public:
    using MessageHandler = std::function<void(const std::vector<uint8_t>&, bool)>;
    using ErrorHandler = std::function<void(const std::string&)>;
    using ConnectHandler = std::function<void()>;
    using CloseHandler = std::function<void(int, const std::string&)>;

    WebSocketClient();
    ~WebSocketClient();

    bool connect(const std::string& url);
    bool send(const std::vector<uint8_t>& data, bool binary = false);
    bool close(int code = 1000, const std::string& reason = "");
    bool isConnected() const;

    void setMessageHandler(MessageHandler handler);
    void setErrorHandler(ErrorHandler handler);
    void setConnectHandler(ConnectHandler handler);
    void setCloseHandler(CloseHandler handler);

private:
    bool m_connected;
    std::shared_ptr<Socket> m_socket;
    MessageHandler m_messageHandler;
    ErrorHandler m_errorHandler;
    ConnectHandler m_connectHandler;
    CloseHandler m_closeHandler;
    std::thread m_receiveThread;
};

/**
 * @brief WebSocket server for WebSocket communication
 */
class WebSocketServer {
public:
    using MessageHandler = std::function<void(int, const std::vector<uint8_t>&, bool)>;
    using ErrorHandler = std::function<void(int, const std::string&)>;
    using ConnectHandler = std::function<void(int)>;
    using CloseHandler = std::function<void(int, int, const std::string&)>;

    WebSocketServer();
    ~WebSocketServer();

    bool start(uint16_t port, const std::string& host = "0.0.0.0");
    void stop();
    bool isRunning() const;

    bool send(int clientId, const std::vector<uint8_t>& data, bool binary = false);
    bool close(int clientId, int code = 1000, const std::string& reason = "");
    bool broadcast(const std::vector<uint8_t>& data, bool binary = false);

    void setMessageHandler(MessageHandler handler);
    void setErrorHandler(ErrorHandler handler);
    void setConnectHandler(ConnectHandler handler);
    void setCloseHandler(CloseHandler handler);

private:
    bool m_running;
    std::shared_ptr<Socket> m_socket;
    std::map<int, std::shared_ptr<Socket>> m_clients;
    MessageHandler m_messageHandler;
    ErrorHandler m_errorHandler;
    ConnectHandler m_connectHandler;
    CloseHandler m_closeHandler;
    std::thread m_acceptThread;
    std::vector<std::thread> m_receiveThreads;
};

/**
 * @brief DNS resolver for resolving domain names to IP addresses
 */
class DnsResolver {
public:
    DnsResolver();
    ~DnsResolver();

    std::vector<IPAddress> resolve(const std::string& hostname);
    IPAddress resolveFirst(const std::string& hostname);
    std::string reverseLookup(const IPAddress& address);

private:
    std::string m_lastError;
};

/**
 * @brief Network interface information
 */
class NetworkInterface {
public:
    NetworkInterface();
    NetworkInterface(const std::string& name, const IPAddress& address, const IPAddress& netmask, const IPAddress& broadcast);

    std::string getName() const;
    IPAddress getAddress() const;
    IPAddress getNetmask() const;
    IPAddress getBroadcast() const;
    bool isUp() const;
    bool isLoopback() const;

private:
    std::string m_name;
    IPAddress m_address;
    IPAddress m_netmask;
    IPAddress m_broadcast;
    bool m_up;
    bool m_loopback;
};

/**
 * @brief Network utilities
 */
class NetworkUtils {
public:
    static std::vector<NetworkInterface> getNetworkInterfaces();
    static bool ping(const IPAddress& address, int timeout = 1000);
    static bool ping(const std::string& hostname, int timeout = 1000);
    static int getHostname(std::string& hostname);
    static bool isPortOpen(const IPAddress& address, uint16_t port, int timeout = 1000);
    static bool isPortOpen(const std::string& hostname, uint16_t port, int timeout = 1000);
};

} // namespace network
} // namespace stdlib
} // namespace astra

#endif // ASTRA_NETWORK_MODULE_H