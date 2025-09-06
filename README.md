# libmodbus-static

A c++ template based modbus library which is independent of the underlying transport (eg. Modbus-RTU, Modbus-TCP), and mmodbus-register-layout definition via c++ structs which does not require **any** dynamic memory allocations.

This enables high portability as no transport specific code is included, albeit at the cost of having to write a bit of additional code for working transport adoption.

The main approach to using the library is to create a modbus actor which can then be run as server or client.

All registers can the be addressed in a named manner, so modbus addresses mapping has to be kept in code.

# Standard usage

A standard modbus client (master) with unix sockets can be created as follows:

```cpp
// ---------------------------------------------------------------
//               modbus register layout defintion
// ---------------------------------------------------------------
struct register_layout {
    // NOTE: struct naming for the layouts is irrelevant, external structs can be used as well
    struct bits_layout {
        constexpr int OFFSET{20}; // register address of the first member
        bool bit_a: 1{};
        bool bit_b: 1{};
        bool bit_c: 1{};
        bool bit_d: 1{};
    };
    struct bits_write_layout {
        constexpr int OFFSET{34}; // register address of the first member
        bool bit_w: 1{};
        bool bit_x: 1{};
        bool bit_y: 1{};
        bool bit_z: 1{};
    };
    #pragma pack(push, 2) // halfs layout has to be 2 byte aligned
    struct halfs_layout {
        constexpr int OFFSET{40000}; // register address of the first member
        std::array<char, 12> a_string{"Hello"};
        uint16_t half{};
        float float_prec{};
        double double_prec{};
        int another{};
    };
    struct halfs_write_layout {
        constexpr int OFFSET{50000}; // register address of the first member
        float whatever{};
        float you{};
        float like{};
        std::array<uint8_t, 32> update_data{};
    }:
    #pragma pack (pop)

    // NOTE: naming of the storage variables is important!
    bits_layout bits_registers{};
    bits_write_layout bits_write_registers{};
    halfs_layout halfs_registers{};
    halfs_write_layout halfs_write_layout{};
};

// ---------------------------------------------------------------
//               tcp io definition for unix sockets
// ---------------------------------------------------------------
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

// ---------------------------------------------------------------
//               main modbus defintion and usage
// ---------------------------------------------------------------
int main() {
    using namespace libmodbus_static;

    modbus_actor<register_layout, tcp_io> modbus_client{/*modbus addr for client*/ 0, tcp_io{.ip = "127.0.0.1"}};

    // set bit registers (are used when calling write_remote) -----------------------------------
    // bit registers have to be written directly
    modbus_client.storage.bits_registers.bit_a = true;
    modbus_client.storage.bits_write_registers.bit_w = true;
    // half registers should be written with the `write()` function for correct byte ordering
    modbus_client.write(10, &register_layout::halfs_layout::another);
    modbus_client.write(10.f, &register_layout::halfs_write_layout::whatever);

    // make calls to a server -------------------------------------------------------------------
    result r;
    // read single remote bit, eg. read bit_a
    r = modbus_client.read_remote(/*server address*/ 1, register_layout::bits_layout{.bit_a = true});
    r = modbus_client.read_remote(/*server address*/ 10, register_layout::bits_write_layout{.bit_w = true});
    // read single remote register, eg. another
    r = modbus_client.read_remote(/*server address*/ 1, &register_layout::halfs_layout::another);
    r = modbus_client.read_remote(/*server address*/ 13, &register_layout::halfs_write_layout::whatever);

    // read range of remote bits, eg. read bit_a to bit_c
    r = modbus_client.read_remote(/*server address*/ 1, 
                                  register_layout::bits_layout{.bit_a = true, .bit_c = true});
    r = modbus_client.read_remote(/*server address*/ 10, 
                                  register_layout::bits_write_layout{.bit_w = true, .bit_z = true});
    // read range of remote register, eg. half to another
    r = modbus_client.read_remote(/*server address*/ 6, 
                                  &register_layout::halfs_layout::half, 
                                  &register_layout::halfs_layout::another);
    r = modbus_client.read_remote(/*server address*/ 13, 
                                  &register_layout::halfs_write_layout::whatever, 
                                  &register_layout::halfs_write_layout::like);

    // NOTE: write can only be done with the write layouts
    // write single remote bit. Uses the data stored in the modbus_client.storage
    r = modbus_client.write_remote(/*server address*/ 10, register_layout::bits_write_layout{.bit_w = true});
    // write single remote register, eg. another
    r = modbus_client.write_remote(/*server address*/ 13, &register_layout::halfs_write_layout::whatever);

    // write range of remote bits, eg. read bit_w to bit_z
    r = modbus_client.write_remote(/*server address*/ 10, 
                                  register_layout::bits_write_layout{.bit_w = true, .bit_z = true});
    // write range of remote register, eg. whatever to like
    r = modbus_client.write_remote(/*server address*/ 13, 
                                  &register_layout::halfs_write_layout::whatever, 
                                  &register_layout::halfs_write_layout::like);

    if (r != OK) {
        std::cout << "Failed to read_remote with error: " << r << std::cout;
        return 1;
    }

    // use updated internal storage -------------------------------------------------------------
    bool bit;
    // bit registers should be read directly
    bit = modbus_client.storage.bits_registers.bit_a;
    bit = modbus_client.storage.bits_write_registers.bit_w;
    // halfs registers should be read with read
    int another = modbus_client(&register_layout::halfs_layout::another);
    float whatever = modbus_client(&register_layout::halfs_write_layout::whatever);

    return 0;
}
```

