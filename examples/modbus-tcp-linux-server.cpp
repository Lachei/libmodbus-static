#include <tcp-server.h>
#include <ranges>
#include <print>
#include <atomic>

#include <sys/socket.h>
#include <netinet/in.h>

void print_usage() {
	std::println(R"(
Simple modbus tcp server demonstrating the usage of the libmodbus-static with linux tcp sockets

modbus-tcp-linux-server [--port,-p PORT=502]
)");
}

static std::atomic<bool>& RunningSingleton() {
	static std::atomic<bool> is_running{true};
	return is_running;
}

void sig_handler(int) {
	RunningSingleton() = false;
}

int create_tcp_socket(int port) {
	struct sockaddr_in addr;
	int fd;

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
		std::println("Error binding socket");
		return -1;
	}

	std::println("Successfully bound to port {:x}", port);
}

int main(int argc, char **argv) {
	signal(SIGINT, sig_handler);
	if (argc > 2) {
		print_usage();
		return EXIT_FAILURE;
	}

	int port = 502;
	for (int i: std::ranges::iota_view{0, argc}) {
		if ((argv[i] == std::string_view("--port") ||
      			argv[i] == std::string_view("-p")) && i + 1 < argc)
			port = std::strtol(argv[i + 1]);
	}
	
	int tcp_socket = create_tcp_socket(port);
	if (tcp_socket == -1)
		return EXIT_FAILURE;



	while (RunningSingleton()) {
		
	}

	return EXIT_SUCCESS;
}

