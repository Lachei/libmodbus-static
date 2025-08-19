#include "fronius-meter-sunspec-layout.h"
#include <modbus-register.h>
#include <ranges>
#include <print>
#include <atomic>

#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace libmodbus_static;

void print_usage() {
	std::println(R"(
Simple modbus tcp server demonstrating the usage of the libmodbus-static with linux tcp sockets

modbus-tcp-linux-server [--port,-p PORT=502]
)");
}

static std::atomic<bool>& RunningSingleton() { static std::atomic<bool> is_running{true}; return is_running; }
static int& TcpSocketSingleton() { static int socket{}; return socket; }

void sig_handler(int) {
	RunningSingleton() = false;
	close(TcpSocketSingleton());
}

int create_tcp_socket(int port) {
	struct sockaddr_in addr;
	int &fd = TcpSocketSingleton();

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if(fd == -1)
	{
		std::println("Error opening socket");
		return -1;
	}

	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = 0;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_family = AF_INET;

	if(bind(fd, (struct sockaddr *)&addr,sizeof(struct sockaddr_in) ) == -1)
	{
		std::println("Error binding socket on port {}", port);
		return -1;
	}

	if (0 > listen(fd, 10)) {
		std::println("Listen on the socket failed");
		return -1;
	}

	std::println("Successfully bound tcp socket to port {}", port);
	return fd;
}

int main(int argc, char **argv) {
	signal(SIGINT, sig_handler);
	if (argc > 3) {
		std::println("Error too many arguments: {}", argc);
		print_usage();
		return EXIT_FAILURE;
	}

	int port = 502;
	for (int i: std::ranges::iota_view{0, argc}) {
		if ((argv[i] == std::string_view("--port") ||
      			argv[i] == std::string_view("-p")) && i + 1 < argc)
			port = std::strtol(argv[i + 1], nullptr, 0);
	}
	
	int tcp_socket = create_tcp_socket(port);
	if (tcp_socket == -1)
		return EXIT_FAILURE;

	auto modbus_server = modbus_register<fronius_meter::layout, 1>::Default();
	std::println("Created modbus server on port {} with addr {}", port, modbus_server.addr);

	while (RunningSingleton()) {
		// get buffer data
		struct sockaddr_in s_addr;
		struct sockaddr_in c_addr;
		socklen_t s = sizeof(s_addr);
		int c = accept(tcp_socket, reinterpret_cast<struct sockaddr*>(&c_addr), &s);
		if (c <= 0) {
			std::println("Accept error");
			close(c);
			continue;
		}
		std::array<uint8_t, 1024> buf{};
		int len{};
		if (0 > (len = recv(c, buf.data(), buf.size() -1, 0))) {
			close(c);
		}
		std::println("Got frame: {}", std::span(buf.data(), len));
		// execute modbus response
		for(int i: std::ranges::iota_view{0, len}) {
			std::string_view r = modbus_server.process_tcp(buf[i]).err;
			if (r == IN_PROGRESS)
				continue;
			if (r != OK) {
				std::println("Modbus parsing failed with {}", r);
				continue;
			}
			auto [res, err] = modbus_server.get_frame_response();
			if (err != OK) {
				std::println("Modbus generating response failed with {}", err);
				r_tie{res, err} = modbus_server.get_frame_error_response(err);
				if (err != OK) {
					std::println("Modbus generating error response failed with {}", err);
					continue;
				}
			}
			std::println("Sending response: {}", res);
			send(c, res.data(), res.size(), 0);
			modbus_server.switch_to_request();
		}
		// close connection for next request
		close(c);
	}

	std::println("Modbus server done, shutting down");

	close(tcp_socket);

	return EXIT_SUCCESS;
}

