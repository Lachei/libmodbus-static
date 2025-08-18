#pragma once
#include "modbus-register.h"

//TODO: implement

namespace libmodbus_static::modbus_tcp_server {

constexpr void noop(auto& cur_request){};
constexpr void nolog(std::string_view){};

/**
 * @brief Is an easier abstraction for creating a modbus tcp server with the underlying modbus_register class,
 * however uses only functions from modbus_register for all implementations
 *
 * This wrapper class is still independent of the underlying transport adapter.
 * See the `examples` folder for examples of how to add such an adapter for your platform (example for linux given)
 */
template<typename Layout, uint8_t Address, int MaxSize, typename Socket, typename ReadFn, typename WriteFn, typename PreResponseFn = decltype(noop), typename LogFn = decltype(nolog)>
concept ValidWriteFn = requires(WriteFn f) {f(std::declval<Socket>(), uint8_t);}
concept ValidReadFn = requires(ReadFn f) {uint8_t c = f(std::declval<Socket>());}
concept ValidPostResponseFn = requires(PostResponseFn f) {f(std::declval<modbus_frame<MaxSize>>());}
struct tcp_server: public modbus_register<Layout, Address, MaxSize> {
	tcp_server(Socket s, ReadFn read_fn, WriteFn write_fn, PreResponseFn pre_response_fn = noop, LogFn log_fn = nolog):
		s{s}, read_fn{read_fn}, write_fn{write_fn}, pre_response_fn{pre_response_fn}, log_fn{log_fn} {}
	Socket s{};
	ReadFn read_fn{};
	WriteFn write_fn{};
	PostResponseFn pre_response_fn{};
	LogFn log_fn{};

	/*
	* @brief main server function which does 
	* 1. process incoming bytes with read_fn
	* 2. calls pre_response_fn after a frame has received and the repsonse not yet has been sent
	* 3. sends out the response via write_fn
	*
	* All errors etc. are automatically handled and do not interrupt the control flow, for logging proved log_fn
	*/
	void step() {
		std::string_view r = process_tcp(read_fn(s));
		if (r == IN_PROGRESS)
			return;
		if (r != OK) {
			switch_to_request();
			log_fn(res);
			return;
		}
		pre_response_fn(buffer); // buffer is defined in modbus_register
		auto [res, err] = get_frame_response();
		if (err != OK) {
			switch_to_request();
			log_fn(err);
			return;
		}
		for (uint8_t b: res)
			write_fn(s, b);
		switch_to_request();
		return
	}
};

}

