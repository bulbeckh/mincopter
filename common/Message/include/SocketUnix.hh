
#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <sys/un.h>

#include <filesystem>
#include <optional>

/* @brief UNIX Socket class used by both the simulation plugin (as server) and the MinCopter hal.sim object (as
 * client). Both use the same class but initialise differently. After initialisation/connection, communication
 * is bi-directional. */
class SocketUnix {
public:

    SocketUnix();

    ~SocketUnix();

    // TODO This is poorly architected. We should have the message inherit from a base class that
    // implements a virtual serialize method, which is called here by send_message to construct
    // the stream of bytes to be sent.

    /* @brief Send a message through the unix socket */
    template <typename T>
    bool send_message(const T& message);

    /* @brief Receive a message through the unix socket */
    template <typename T>
    const std::optional<T> receive_message(void);

    // TODO This interface is kind of stupid - should be virtualised and new classes for each of mincopter/simulator

    /* @brief Initialise this socket on the MinCopter side */
    bool init_client(void);

    /* @brief Initialise this socket on the Simulator side */
    bool init_server(void);

    /* @brief Check if we are still connected to server (simulator) */
    bool connected(void) { return _connected; }

    const char* get_socket_path(void) {
	    return socket_path.c_str();
    }

private:

    std::filesystem::path socket_path{};

    bool _socket_initialised{false};

    bool _connected{false};

    struct sockaddr_un addr{};

    /* @brief Socket file descritpro. Used for sending & receiving */
    int sockfd = -1;

    /* @brief Server file descriptor, used only for initial connection server */
    int serverfd = -1;

};

