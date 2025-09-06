#include "fronius-meter-sunspec-layout.h"
#include <modbus-actor.h>
#include <print>
#include <array>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace libmodbus_static;

struct tcp_io {
	const char *ip{};
	int fd{};
	std::array<uint8_t, 1024> receive_buffer{};

	static constexpr transport_t TRANSPORT_TYPE{transport_t::TCP};
	void init() {
		struct sockaddr_in local_addr;
		fd = socket(AF_INET, SOCK_STREAM, 0);
		local_addr.sin_port = htons(0);
		local_addr.sin_addr.s_addr = INADDR_ANY;
		local_addr.sin_family = AF_INET;
		bind(fd, (struct sockaddr *)&local_addr, sizeof(local_addr));

		struct sockaddr_in server_addr;
		fd = socket(AF_INET, SOCK_STREAM, 0);
		server_addr.sin_port = htons(1502);
		server_addr.sin_addr.s_addr = inet_addr(ip);
		server_addr.sin_family = AF_INET;
		if (0 != connect(fd, (struct sockaddr *)&server_addr, sizeof(server_addr))) {
			close(fd);
			fd = 0;
		}
	}
	void deinit() {
		if (fd <= 0)
			return;
		close(fd);
	}
	std::span<uint8_t> read_bytes(std::chrono::milliseconds max_timeout) {
		if (fd <= 0)
			return {};
		size_t len = recv(fd, receive_buffer.data(), receive_buffer.size() - 1, 0);
		return {receive_buffer.data(), std::max(size_t(0), len)};
	}
	void write_bytes(std::span<uint8_t> data) {
		if (fd <= 0)
			return;
		write(fd, data.data(), data.size());
	}
	result get_status() const {
		if (fd <= 0)
			return "CONNECTION_ERROR";
		return OK;
	}
};

int main() {
	modbus_actor<fronius_meter::layout, tcp_io> modbus_client{0, tcp_io{"127.0.0.1"}};
	modbus_client.write(1.0f, &fronius_meter::halfs_layout::pf);
	
	result r = modbus_client.read_remote(1, &fronius_meter::halfs_layout::pfpha, &fronius_meter::halfs_layout::pfphc);

	std::println("Reading the remote returned with status: {}, connection status: {}", r, modbus_client.io.get_status());

	std::println("Power Factor: {}", modbus_client.read(&fronius_meter::halfs_layout::pf));

	return 0;
}
