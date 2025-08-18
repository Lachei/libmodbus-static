#pragma once

namespace fronius_meter {

/**
* Definition of the fronius sunspec-meter model with all registers needed for operation.
*/
struct halfs_layout {
	constexpr static int OFFSET = 40000;
};

struct layout {
	halfs_layout halfs_registers{};
}

}

