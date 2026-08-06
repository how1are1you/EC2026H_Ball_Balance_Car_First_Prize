#include "startup_motion_gate.h"

#include <assert.h>

int main(void)
{
    startup_motion_gate_t gate;

    startup_motion_gate_reset(&gate);
    assert(startup_motion_gate_update(&gate, 0.00f, 0.00f) == 0U);
    assert(startup_motion_gate_update(&gate, 0.03f, 0.03f) == 0U);
    assert(startup_motion_gate_update(&gate, 0.03f, 0.03f) == 1U);
    assert(startup_motion_gate_update(&gate, 0.00f, 0.00f) == 1U);

    startup_motion_gate_reset(&gate);
    assert(startup_motion_gate_update(&gate, 0.03f, 0.03f) == 0U);
    assert(startup_motion_gate_update(&gate, 0.00f, 0.00f) == 0U);
    assert(startup_motion_gate_update(&gate, 0.03f, 0.03f) == 0U);

    return 0;
}
