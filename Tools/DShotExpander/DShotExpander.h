#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "Parameters.h"


class DShotExpander : public AP_HAL::HAL::Callbacks {
public:
    DShotExpander(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    static const AP_Param::Info var_info[];
    Parameters g;
    
    void load_parameters();
};

extern const AP_HAL::HAL& hal;
extern DShotExpander dshotexpander;
