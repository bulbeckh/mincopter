
#include "SocketUnix.hh"

#include "SimulationMessage.h"

#include <thread>

#include <iostream>
#include <filesystem>

#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <chrono>

using namespace std::chrono_literals;

SocketUnix::SocketUnix() {
	// TODO
}

SocketUnix::~SocketUnix() {
	// TODO
}

bool SocketUnix::init_server(void)
{
	// We are using UNIX sockets here, resolving under $XDG_RUNTIME_DIR/mincopter/0
	
	const char* runtime_dir = std::getenv("XDG_RUNTIME_DIR");

	if (!runtime_dir) {
		return false;
	}

	std::filesystem::path socket_dir = std::filesystem::path(runtime_dir) / "mincopter";
	std::filesystem::create_directories(socket_dir);

	// TODO This hardcodes this socket file and prevents multiple instances of mincopter/gazebo running at
	// once. This is fine for now but needs to be updated if we support parallel testing.
	
	socket_path = socket_dir / "0";

	// Create a unix socket
	serverfd = socket(AF_UNIX, SOCK_SEQPACKET, 0);

	if (serverfd < 0 ) {
		return false;
	}

	addr.sun_family = AF_UNIX;
	std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path)-1);

	// Remove any existing socket file
	unlink(socket_path.c_str());

	int bind_success = bind(serverfd,
			reinterpret_cast<sockaddr*>(&addr),
			sizeof(addr));

	if (bind_success<0) {
		close(serverfd);
		return false;
    }

	// TODO Check that this is still correct for UNIX sockets
	// Setup a 5s timeout for the receive function
	struct timeval _tv_timeout;
	_tv_timeout.tv_sec=5;
	_tv_timeout.tv_usec=0;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &_tv_timeout, sizeof(_tv_timeout));

	// Mark socket as initialised
	_socket_initialised = true;
	
	// TODO Up to here, the init_server is the exact same as init_client
	
	// TODO No more than 1 connection allowed - should the backlog arg be 1 or 0?
	listen(serverfd, 1);

	sockfd = accept(serverfd, nullptr, nullptr);

	if (sockfd < 0 ) {
		return false;
	}

	_connected = true;

	return true;
}

bool SocketUnix::init_client(void)
{
	// We are using UNIX sockets here, resolving under $XDG_RUNTIME_DIR/mincopter/0
	
	const char* runtime_dir = std::getenv("XDG_RUNTIME_DIR");

	if (!runtime_dir) {
		return false;
	}

	std::filesystem::path socket_dir = std::filesystem::path(runtime_dir) / "mincopter";
	std::filesystem::create_directories(socket_dir);

	// TODO This hardcodes this socket file and prevents multiple instances of mincopter/gazebo running at
	// once. This is fine for now but needs to be updated if we support parallel testing.
	
	socket_path = socket_dir / "0";

	// Create a unix socket
	sockfd = socket(AF_UNIX, SOCK_SEQPACKET, 0);

	if (sockfd < 0 ) {
		return false;
	}

	addr.sun_family = AF_UNIX;
	std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path)-1);

	// Remove any existing socket file
	unlink(socket_path.c_str());

	int bind_success = bind(sockfd,
			reinterpret_cast<sockaddr*>(&addr),
			sizeof(addr));

	if (bind_success<0) {
		close(sockfd);
		return false;
    	}

	// TODO Check that this is still correct for UNIX sockets
	// Setup a 1s timeout for the receive function
	struct timeval _tv_timeout;
	_tv_timeout.tv_sec=1;
	_tv_timeout.tv_usec=0;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &_tv_timeout, sizeof(_tv_timeout));

	// Mark socket as initialised
	_socket_initialised = true;

	// Attempt to connect for 5 seconds

	auto deadline = std::chrono::steady_clock::now() + 5s;
	int connect_success{-1};

	while (std::chrono::steady_clock::now() < deadline) {
		connect_success = connect(sockfd,
			reinterpret_cast<sockaddr*>(&addr),
			sizeof(addr));

		if (connect_success==0) break;

		std::this_thread::sleep_for(100ms);
	}

	if (connect_success < 0) {
		close(sockfd);
		std::cout << "SocketUnix failed to connect to server (simulator) \n";
		return false;
	}

	_connected = true;

	return true;
}

template <typename T>
bool SocketUnix::send_message(const T& message)
{
	if (!_socket_initialised) {
		std::cout << "Socket needs to be initialised before attempting to send message\n";
		return false;
	}

	ByteWriter writer;
	
	// Serialize message
	serialize(writer, message);

	// std::vector<std::byte> object
	auto tx_buffer = writer.data();

	// Send message
	ssize_t n = send(sockfd,
			tx_buffer.data(),
			tx_buffer.size(),
			0);

	if (n<0) {
		std::cout << "Failed to send message via SocketUnix\n";
		return false;
	}

	return true;
}

// TODO We need a template specialisation here because we compile mc-common seperately to the mc-arch (hal) libraries
// This is also because of our bad design (discussed above) where we serialise the message ourselves in the send_message
// function.
template bool SocketUnix::send_message<mc::ControlMessage>(const mc::ControlMessage&);
template bool SocketUnix::send_message<mc::StateMessage>(const mc::StateMessage&);
template bool SocketUnix::send_message<mc::StateUpdateMessage>(const mc::StateUpdateMessage&);
template bool SocketUnix::send_message<mc::ForceUpdateMessage>(const mc::ForceUpdateMessage&);

template <typename T>
const std::optional<T> SocketUnix::receive_message(void)
{
	std::vector<std::byte> rx_buffer(1024);

	ssize_t n = ::recv(sockfd,
			rx_buffer.data(),
			rx_buffer.size(),
			0);

	if (n<0) {
		std::cout << "Failed to receive message via SocketUnix\n";
		return std::nullopt;
	}

	if (n==0) {
		std::cout << "Received 0 byte message via SocketUnix\n";
		return std::nullopt;
	}

	std::cout << "Socket received " << static_cast<int32_t>(n) << " bytes\n";

	rx_buffer.resize(static_cast<std::size_t>(n));

	// TODO Add efficient move representations
	// Return deserialized message
	return mc::deserialize<T>(rx_buffer);
}

template const std::optional<mc::ControlMessage> SocketUnix::receive_message<mc::ControlMessage>();
template const std::optional<mc::StateMessage> SocketUnix::receive_message<mc::StateMessage>();
template const std::optional<mc::StateUpdateMessage> SocketUnix::receive_message<mc::StateUpdateMessage>();
template const std::optional<mc::ForceUpdateMessage> SocketUnix::receive_message<mc::ForceUpdateMessage>();


