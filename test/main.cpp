#include <cassert>
#include <modbus-register.h>
#include <iostream>

using namespace libmodbus_static;

#pragma pack(push, 1)
struct example_layout {
	struct bits_layout {
		constexpr static int OFFSET{0};
		bool enabled: 1{};
		bool active: 1{};
		bool visible: 1{};
		bool trusted: 1{};
		bool uknown: 1{};
	} bits_registers;
	// if eg. no discrete inputs are used, just leave them out else
	// struct bits_write_layout {
	// 	constexpr static int OFFSET{256};
	//	bool setable: 1{};
	// } bits_write_registers;
	struct halfs_layout {
		constexpr static int OFFSET{40000};
		float a{};
		float b{};
		uint16_t another{};
		mod_string<32> string_field{"Default"}; // careful with default inited strings, have to be byte swapped
		uint32_t events{};
	} halfs_registers;
	struct halfs_write_layout {
		constexpr static int OFFSET{60000};
		float a{};
		float b{};
		uint16_t others{};
	} halfs_write_registers;
};
#pragma pack(pop)
using el = example_layout;

int main() {
	static_assert(sizeof(example_layout::bits_registers) == 1);
	static_assert(sizeof(example_layout::halfs_registers) == 46);
	static_assert(sizeof(example_layout::halfs_write_registers) == 10);
	static_assert(sizeof(example_layout) == 57);

	modbus_register<example_layout>& modbus{modbus_register<example_layout>::Default(20)};

	mod_string<32> el::halfs_layout:: *a = &el::halfs_layout::string_field;
	mod_string<32> t = el::halfs_layout{}.*a;


	// ---------------------------------------------------------------------------------------
	std::cout << "Read/write tests\n";
	// ---------------------------------------------------------------------------------------
	assert(modbus.addr == 20);
	
	std::string_view sv = to_string_view(modbus.read(MEMBER_CHAIN(example_layout, halfs, string_field)));
	assert(sv == "Default");
	modbus.write(mod_string<32>{"Another"}, MEMBER_CHAIN(example_layout, halfs, string_field));
	sv = to_string_view(modbus.read(MEMBER_CHAIN(example_layout, halfs, string_field)));
	assert(sv == "Another");
	assert(modbus.read(MEMBER_CHAIN(example_layout, halfs, a)) == 0);
	modbus.write(20.0f, MEMBER_CHAIN(el, halfs, a));
	assert(modbus.read(MEMBER_CHAIN(example_layout, halfs, a)) == 20.0f);
	assert(modbus.storage.halfs_registers.a != 20.0f);
	assert(modbus.read(MEMBER_CHAIN(el, halfs_write, others)) == 0);
	modbus.write(uint16_t(0x00ff), MEMBER_CHAIN(el, halfs_write, others));
	assert(modbus.storage.halfs_write_registers.others == 0xff00);

	// bits structs should be written directly
	modbus.storage.bits_registers.enabled = true;

	std::cout << "Done.\n\n";

	// ---------------------------------------------------------------------------------------
	std::cout << "Client tests\n";
	// ---------------------------------------------------------------------------------------
	
	
	std::cout << "Done.\n\n";

	// ---------------------------------------------------------------------------------------
	std::cout << "Server tests\n";
	// ---------------------------------------------------------------------------------------
	//
	std::cout << "Done.\n\n";
}

