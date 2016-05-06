/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Plane transmitter tuning
 */
class Tuning
{
public:
    friend class Plane;

    Tuning(void);
    
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void check_input(void);

    enum tuning_func {
        TUNING_NONE =                          0,

        // quadplane tuning
        TUNING_Q_RATE_ROLL_KPI =               1,
        TUNING_Q_RATE_ROLL_KP =                2,
        TUNING_Q_RATE_ROLL_KI =                3,
        TUNING_Q_RATE_ROLL_KD =                4,

        TUNING_Q_RATE_PITCH_KPI =              5,
        TUNING_Q_RATE_PITCH_KP =               6,
        TUNING_Q_RATE_PITCH_KI =               7,
        TUNING_Q_RATE_PITCH_KD =               8,

        TUNING_Q_RATE_YAW_KPI =                9,
        TUNING_Q_RATE_YAW_KP =                10,
        TUNING_Q_RATE_YAW_KI =                11,
        TUNING_Q_RATE_YAW_KD =                12,

        TUNING_Q_ANG_ROLL_KP =                13,
        TUNING_Q_ANG_PITCH_KP =               14,
        TUNING_Q_ANG_YAW_KP =                 15,

        TUNING_Q_PXY_P =                      16,
        TUNING_Q_PZ_P  =                      17,

        TUNING_Q_VXY_P =                      18,
        TUNING_Q_VXY_I =                      19,
        TUNING_Q_VZ_P  =                      20,

        TUNING_Q_AZ_P =                       21,
        TUNING_Q_AZ_I =                       22,
        TUNING_Q_AZ_D  =                      23,

        // fixed wing tuning
        TUNING_RLL_P =                        24,
        TUNING_RLL_I =                        25,
        TUNING_RLL_D =                        26,
        TUNING_RLL_FF =                       27,

        TUNING_PIT_P =                        28,
        TUNING_PIT_I =                        29,
        TUNING_PIT_D =                        30,
        TUNING_PIT_FF =                       31,
    };
    
private:
    AP_Int8 channel;
    AP_Int8 selector;
    AP_Int8 parmset;
    AP_Float range;

    // when selector was triggered
    uint32_t selector_start_ms;

    // are we waiting for channel mid-point?
    bool mid_point_wait;

    // last input from tuning channel
    float last_channel_value;
    
    // mid-value for current parameter
    float center_value;

    uint32_t last_check_ms;

    struct PACKED log_ParameterTuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
        float    tuning_value;  // normalized value used inside tuning() function
        float    tuning_low;    // tuning low end value
        float    tuning_high;   // tuning high end value
    };

    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_low, float tune_high);
    
    enum tuning_sets {
        TUNING_SET_NONE =                      0,

        TUNING_SET_Q_RATE_ROLL_PITCH =         1,
        TUNING_SET_Q_RATE_ROLL =               2,
        TUNING_SET_Q_RATE_PITCH =              3,
    };

    // the parameter we are tuning
    enum tuning_func current_parm;

    // pointer to current parameter
    AP_Float *current_parm_ptr;

    // current index into the parameter set
    uint8_t current_parm_index;

    // current parameter set
    enum tuning_sets current_set;

    struct tuning_set {
        enum tuning_sets set;
        uint8_t num_parms;
        const enum tuning_func *parms;
    };

    // table of tuning sets
    static const tuning_set tuning_sets[];

    // table of tuning parameter names for reporting
    static const struct tuning_name {
        enum tuning_func parm;
        const char *name;
    } tuning_names[];
    
    void check_selector_switch(void);
    void re_center(void);
    void next_parameter(void);
    void save_parameters(void);
    AP_Float *get_param_pointer(enum tuning_func parm);
    const char *get_tuning_name(enum tuning_func parm);
};
