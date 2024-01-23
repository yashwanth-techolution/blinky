/*
 * SYNTIANT CONFIDENTIAL
 * _____________________
 *
 *   Copyright (c) 2019 Syntiant Corporation
 *   All Rights Reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the property of
 *  Syntiant Corporation and its suppliers, if any.  The intellectual and
 *  technical concepts contained herein are proprietary to Syntiant Corporation
 *  and its suppliers and may be covered by U.S. and Foreign Patents, patents
 *  in process, and are protected by trade secret or copyright law.
 *  Dissemination of this information or reproduction of this material is
 *  strictly forbidden unless prior written permission is obtained from
 *  Syntiant Corporation.
 */
#ifndef SYNTIANT_NDP120_H
#define SYNTIANT_NDP120_H

#include <syntiant_packager/syntiant_package_consts.h>
#include <syntiant_ilib/ndp120_regs.h>
#include <syntiant-firmware/ndp120_firmware.h>
#include <syntiant-dsp-firmware/ndp120_dsp_fw_state.h>
#include <syntiant_ilib/syntiant_ndp_driver.h>

#define NDP120_DEBUG 0
#define NDP120_DEBUG_MB 0
#define NDP120_DEBUG_POLL 0
#define NDP120_DEBUG_POLL_MATCH 0
#define NDP120_DEBUG_SEND 0

#define NDP120_A0_DEVICE_ID 0XC

int DEBUG_PRINTF(const char * fmt, ...);
#define ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))

/**
 * @file syntiant_ndp120.h
 * @date 2019-10-11
 * @brief Interface to Syntiant NDP120 chip-specific interface library functions
 */

/**
 * @brief NDP120 firmware posterior handler set flags
 */
enum syntiant_ndp120_posterior_config_action_type_e {
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_ACTION_TYPE_MATCH = 0,
    /**< report a match in the summary and go to state 0 */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_ACTION_TYPE_STATE = 1,
    /**< go to state @c action_state */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_ACTION_TYPE_STAY = 2,
    /**< stay in the current state and leave timer running */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_ACTION_TYPE_MAX
    = ((NDP120_CONFIG_SET_POSTERIOR_CONFIG_ACTION_TYPE_STAY << 1) - 1)
};

/**
 * @brief NDP120 configuration data
 */

enum {
    /* clk.src */
    NDP120_CONFIG_SET_CLK_SRC_REFSEL            = 0x1,
    NDP120_CONFIG_SET_CLK_SRC_CLKSEL            = 0x2,
    NDP120_CONFIG_SET_CLK_SRC_EXTSEL            = 0x4,
    NDP120_CONFIG_SET_CLK_SRC_FORCE_EXT         = 0x8,
    NDP120_CONFIG_SET_CLK_SRC_FORCE_INT         = 0x10,
    NDP120_CONFIG_SET_CLK_SRC_FORCE_DNN_CLK     = 0x20,
    NDP120_CONFIG_SET_CLK_SRC_FORCE_AESOTP_CLK  = 0x40,

    /* clk.div */
    NDP120_CONFIG_SET_CLK_DIV_MCUCLKDIV         = 0x1,

    /* clk.pll */
    NDP120_CONFIG_SET_CLK_PLL_PRESET            = 0x01,

    /* clk.ext_freq */
    NDP120_CONFIG_SET_CLK_EXT_FREQ_HZ           = 0x01

};

enum {
    NDP120_MAIN_CLK_SRC_EXT = 1,
    NDP120_MAIN_CLK_SRC_PLL = 2,
    NDP120_MAIN_CLK_SRC_FLL = 3
};





enum {
    /* gpio */
    NDP120_CONFIG_SET_GPIO_DIR                  = 0x01,
    NDP120_CONFIG_SET_GPIO_VALUE                = 0x02,
    NDP120_CONFIG_VALUE_GPIO_DIR_OUT            = 0x00,
    NDP120_CONFIG_VALUE_GPIO_DIR_IN             = 0x01
};

enum {
    /* PDM setters */
    NDP120_CONFIG_SET_PDM_SAMPLE_RATE            = 0x0001,
    NDP120_CONFIG_SET_PDM_PDM_RATE               = 0x0002,
    NDP120_CONFIG_SET_PDM_CLK_MODE               = 0x0004,
    NDP120_CONFIG_SET_PDM_MODE                   = 0x0008,
    NDP120_CONFIG_SET_PDM_MAX_SPL                = 0x0010
}; 

enum {
    /* PDM mode */
    NDP120_CONFIG_VALUE_PDM_MODE_OFF,
    NDP120_CONFIG_VALUE_PDM_MODE_LEFT,
    NDP120_CONFIG_VALUE_PDM_MODE_RIGHT,
    NDP120_CONFIG_VALUE_PDM_MODE_STEREO
};

enum {
    /* PDM clock mode */
    NDP120_CONFIG_VALUE_PDM_CLK_MODE_EXTERNAL,
    NDP120_CONFIG_VALUE_PDM_CLK_MODE_INTERNAL,
    NDP120_CONFIG_VALUE_PDM_CLK_MODE_DUAL_INTERNAL
};

typedef struct syntiant_ndp120_config_pdm_s {
        uint32_t get;
        uint32_t set;
        uint32_t interface;
        uint32_t sample_rate;
        uint32_t pdm_rate;
        uint32_t clk_mode;
        uint32_t mode;
        uint32_t max_spl;
} syntiant_ndp120_config_pdm_t;

/* decimation setters */
enum {
    NDP120_CONFIG_SET_DECIMATION_INSHIFT = 1,
    NDP120_CONFIG_SET_DECIMATION_OUTSHIFT = 2
};

typedef struct syntiant_ndp120_config_decimation_s {
        uint32_t get;
        uint32_t set;
        uint32_t mic;
        uint32_t inshift;
        uint32_t outshift;
} syntiant_ndp120_config_decimation_t;

/* gain setters */
enum {
    NDP120_CONFIG_SET_GAIN_DCREMOVALMODE = 1,
    NDP120_CONFIG_SET_GAIN_AGCSHIFTDIR = 2,
    NDP120_CONFIG_SET_GAIN_AGCSHIFTCNT = 4,
    NDP120_CONFIG_SET_GAIN_AGCFINEGRAINMUL = 8
};
/* gain enums */
enum {
    NDP120_CONFIG_VALUE_GAIN_DCREMOVALMODE_OFF = NDP120_DSP_CONFIG_PDMCFG_B_DCREMOVALMODE_OFF,
    NDP120_CONFIG_VALUE_GAIN_DCREMOVALMODE_STATIC = NDP120_DSP_CONFIG_PDMCFG_B_DCREMOVALMODE_STATIC,
    NDP120_CONFIG_VALUE_GAIN_DCREMOVALMODE_ON = NDP120_DSP_CONFIG_PDMCFG_B_DCREMOVALMODE_ON,
    NDP120_CONFIG_VALUE_GAIN_AGCSHIFTDIR_LEFT = 0,
    NDP120_CONFIG_VALUE_GAIN_AGCSHIFTDIR_RIGHT = 1
};

typedef struct syntiant_ndp120_config_gain_s {
        uint32_t get;
        uint32_t set;
        uint32_t mic;
        uint32_t dcremovalmode;
        uint32_t agcshiftdir;
        uint32_t agcshiftcnt;
        uint32_t agcfinegrainmul;
} syntiant_ndp120_config_gain_t;

/* farrow setters */
enum {
    NDP120_CONFIG_SET_FARROW_BYPASS = 1
};

typedef struct syntiant_ndp120_config_farrow_s {
        uint32_t get;
        uint32_t set;
        uint32_t interface;
        uint32_t bypass;
} syntiant_ndp120_config_farrow_t;


enum syntiant_ndp120_debug_extract_type_e {
    SYNTIANT_NDP120_DEBUG_EXTRACT_TYPE_FW_STATE = 0x00,
    SYNTIANT_NDP120_DEBUG_EXTRACT_TYPE_PH_STATE = 0x02,
    SYNTIANT_NDP120_DEBUG_EXTRACT_NN_NUM = 0x04,
    SYNTIANT_NDP120_DEBUG_EXTRACT_TYPE_LAST
        = SYNTIANT_NDP120_DEBUG_EXTRACT_NN_NUM
};

typedef struct syntiant_ndp120_dsp_flow_rule_pkg_fmt_s {
    uint8_t src_type;
    uint8_t src_param;
    uint8_t dst_type;
    uint8_t dst_param;
} syntiant_ndp120_dsp_flow_rule_pkg_fmt_t;

typedef struct syntiant_ndp120_dsp_flow_collection_pkg_fmt_s {
    uint8_t set_id;
    uint8_t src_type;
    uint8_t src_param;
    uint8_t dst_type;
    uint8_t dst_param;
} syntiant_ndp120_dsp_flow_collection_pkg_fmt_t;

typedef struct syntiant_ndp120_mcu_orch_pkg_fmt_s {
    uint8_t id;
    uint8_t flow_id;
    uint8_t status;
    uint8_t type;
    uint8_t action;
    uint8_t num_inputs;
    uint8_t num_outputs;
    uint8_t input_edges[4];
    uint8_t next_ids[4];
} syntiant_ndp120_mcu_orch_pkg_fmt_t;

/* UNDER REVIEW
 * 
 * The below was added for completeness 
 * in parity with 10x code, and may or may 
 * not be appropriate here.  It's here 
 * as a reminder of what _might_ need 
 * to be implemented.
 */
/*TODO create a struct abstracting out the common config between 120 and 10x
 * and embed that in 120 config
 */

/**
 * @brief NDP120 configuration data
 */
typedef struct syntiant_ndp120_config_misc_s {
    unsigned int set;              /**< configuration variable set flags */
    unsigned int set1;             /**< configuration variable set flags */
    int get; /**< read all configuration state (touching the chip) */
    /* fields returning valid info even when set == 0 && get == 0 */

    /* samples per freq domain frame (read only) */
    unsigned int audio_frame_size; 

    /* samples per freq domain step (read only) */
    unsigned int audio_frame_step; 

    /* bytes per sample (read only) */
    unsigned int audio_sample_size_bytes; 

    /* input source for dnn processing */
    unsigned int input;

    /* streaming features per DNN frame */
    unsigned int dnn_frame_size;   

    /* audio input buffer used frames (read only) */
    unsigned int audio_buffer_used; 

    /* match interrupt for every frame on/off */
    unsigned int match_per_frame_on;

#if 0
    /* fields returning valid info when set != 0 || get != 0 */
    unsigned int input_clock_rate; /**< input clock rate in hz */
    unsigned int core_clock_rate;  /**< core clock rate in hz (read only) */
    unsigned int mcu_clock_rate;   /**< MCU clock rate in hz */
    unsigned int spi_max_pcm_input_rate;
    /**< maximum SPI speed for PCM data input in hz (read only) */
    unsigned int spi_word_bits;    /**< SPI input word bit width */
    unsigned int i2s_frame_size;   /**< I2S frame size in bits */
    unsigned int i2s_sample_size;  /**< I2S sample size in bits */
    unsigned int i2s_sample_msbit; /**< I2S sample most significant bit index */
    unsigned int freq_clock_rate;  /**< freq domain clock rate in hz */
#endif

#if 0
    unsigned int water_mark_on;
    /**< enable audio input buffer low water mark notification */
    unsigned int dnn_clock_rate;   /**< DNN clock rate in hz */
    uint32_t fw_pointers_addr; /**< firmware state pointers addr (read only) */
    /* fields returning valid info when get != 0 */
#endif
} syntiant_ndp120_config_misc_t;


#if 1

/* UNDER REVIEW
 * 
 * The below was added for completeness 
 * in parity with 10x code, and may or may 
 * not be appropriate here.  It's here 
 * as a reminder of what _might_ need 
 * to be implemented.
 */
/**
 * @brief DNN input sources
 */
enum syntiant_ndp120_config_dnn_input_e {
    NDP120_CONFIG_SET_DNN_INPUT_NONE = 0,
    /**< DNN input disabled */
    NDP120_CONFIG_SET_DNN_INPUT_PDM0 = 1,
    /**< PDM microphone, falling clock */
    NDP120_CONFIG_SET_DNN_INPUT_PDM1 = 2,
    /**< PDM microphone, rising clock */
    NDP120_CONFIG_SET_DNN_INPUT_PDM_SUM = 3,
    /**< Sum of both PDM microphones */
    NDP120_CONFIG_SET_DNN_INPUT_I2S_LEFT = 4,
    /**< I2S left input */
    NDP120_CONFIG_SET_DNN_INPUT_I2S_RIGHT = 5,
    /**< I2S right input */
    NDP120_CONFIG_SET_DNN_INPUT_I2S_SUM = 6,
    /**< I2S left + right inputs */
    NDP120_CONFIG_SET_DNN_INPUT_I2S_MONO = 7,
    /**< I2S 1 channel input */
    NDP120_CONFIG_SET_DNN_INPUT_I2S_DIRECT = 8,
    /**< I2S interface direct feature input (bypass frequency block) */
    NDP120_CONFIG_SET_DNN_INPUT_SPI = 9,
    /**< SPI interface PCM */
    NDP120_CONFIG_SET_DNN_INPUT_SPI_DIRECT = 10,
    /**< SPI interface direct feature input (bypass frequency block) */
    NDP120_CONFIG_SET_DNN_INPUT_MAX
    = NDP120_CONFIG_SET_DNN_INPUT_SPI_DIRECT
};
#endif

/* UNDER REVIEW
 * 
 * The below was added for completeness 
 * in parity with 10x code, and may or may 
 * not be appropriate here.  It's here 
 * as a reminder of what _might_ need 
 * to be implemented.
 */
/**
 * @brief configuration variable set flags
 * @note this struct is sefined the same way in packager file board_TLVs.
 */
enum syntiant_ndp120_config_set_e {
    NDP120_CONFIG_SET_MISC_INPUT = 0x1,
    NDP120_CONFIG_SET_MISC_MATCH_PER_FRAME_ON = 0x2,
    NDP120_CONFIG_SET_MISC_ALL_M = 0x03
};

/*

Mohammedreza:

The comment in the struct shows the CLI options it's supposed to cover.  However
structure this in a way that works for C configuration, and don't worry about The
CLI too much.

-- EP

See this doc:
https://docs.google.com/document/d/1autU3UJELTtHqzormwvVYPYPKA0ffQeu6I493j1T4B4/edit#bookmark=id.rwwtxlttjb8b

*/


typedef struct syntiant_ndp120_config_filter_s {
        uint32_t get;
        uint32_t set;
        /* fixme 

            --high-pass-cutoff
            --high-pass-coef
            --remove-dc
            --remove-dc
        */
} syntiant_ndp120_config_filter_t;

typedef struct syntiant_ndp120_config_clk_src_s {
            uint32_t get;
            uint32_t set;

            uint32_t refsel;    /* chip_config.clkctl1.refsel */
            uint32_t clksel;    /* chip_config.clkctl1.clksel */
            uint32_t extsel;    /* chip_config.clkctl1.extsel */
            uint32_t force_ext; /* spi.ctl.extsel */
            uint32_t force_int; /* spi.ctl.intsel */

            /* "clock gate for dnn" */
            uint32_t force_dnn_clk; /* chip_config.clkctl0.force_dnn_clk */
            /* "clock gate for aesotp" */
            uint32_t
                force_aesotp_clk; /* chip_config.clkctl0.force_aesotp_clk */

} syntiant_ndp120_config_clk_src_t;

typedef struct syntiant_ndp120_config_clk_pll_s {
            uint32_t get;
            uint32_t set;
            uint32_t preset;
            uint32_t locked;
            uint32_t no_switch;
} syntiant_ndp120_config_clk_pll_t;

typedef struct syntiant_ndp120_config_clk_fll_s {
            uint32_t get;
            uint32_t set;
} syntiant_ndp120_config_clk_fll_t;

typedef struct syntiant_ndp120_config_clk_div_s {
            uint32_t get;
            uint32_t set;
            uint32_t mcuclkdiv;         /* chip_config.clkctl0 */
} syntiant_ndp120_config_clk_div_t;


typedef struct {
    uint32_t set;
    uint32_t gpio_num;
    uint32_t dir;
    uint32_t value;
} syntiant_ndp120_config_gpio_t;

/**
 * @brief NDP10x firmware posterior handler set flags
 */
enum syntiant_ndp120_posterior_config_set_e {
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_STATES = 0x0001,
    /**< set the number of states */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_CLASSES = 0x0002,
    /**< set the number of classes */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_SM_QUEUE_SIZE = 0x0004,
    /**< set the number of frames to smooth probabilities over */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_TIMEOUT = 0x0008,
    /**< set the state timeout */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_TIMEOUT_ACTION = 0x0010,
    /**< set the state timeout action */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_THRESHOLD = 0x0020,
    /**< set the class active threshold */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_WINDOW = 0x0040,
    /**< set the class match window */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_BACKOFF = 0x0080,
    /**< set the class match backoff timer */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_ENABLE = 0x0100,
    /**< set whether the posterior is enabled or not **/
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_ACTION = 0x0200,
    /**< set the class action*/
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_PH_TYPE = 0x0400,
    /**< set the posterior handler type */
    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_NUM_PH = 0x0800,
    /**< set the number of posterior handlers */

    NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_ALL_M
    = ((NDP120_CONFIG_SET_POSTERIOR_CONFIG_SET_NUM_PH << 1) - 1)
};

enum syntiant_ndp120_config_tank_input_e {
    NDP120_CONFIG_SET_MISC_TANK_INPUT_NONE = 0
    /**< Tank input disabled */
};

int syntiant_ndp120_scratch_get_valid(struct syntiant_ndp_device_s *ndp, uint32_t *valid);
int syntiant_ndp120_scratch_get_valid_skip_crc(struct syntiant_ndp_device_s *ndp, uint32_t *valid);
int syntiant_ndp120_scratch_set_valid(struct syntiant_ndp_device_s *ndp, uint32_t valid);

/*TODO Remove all these macros */
#define SCRATCH_VARIABLE_ADX(x) (uint32_t)(NDP120_ILIB_SCRATCH_ORIGIN + offsetof(syntiant_ndp120_scratch_t, x))

#define SCRATCH_VALID_ADX (SCRATCH_VARIABLE_ADX(valid))
#define SCRATCH_CHECKSUM_ADX (SCRATCH_VARIABLE_ADX(checksum))
#define SCRATCH_PDM_SAMPLE_RATE_ADX(x) (uint32_t) (SCRATCH_VARIABLE_ADX(pdm_sample_rate) + x * sizeof(uint32_t))
#define SCRATCH_PDM_RATE_ADX(x) (uint32_t) (SCRATCH_VARIABLE_ADX(pdm_rate) + x * sizeof(uint32_t))
#define SCRATCH_PDM_CLK_MODE_ADX(x) (uint32_t) (SCRATCH_VARIABLE_ADX(pdm_clk_mode) + x * sizeof(uint32_t))
#define SCRATCH_PDM_MAX_SPL(x) (uint32_t) (SCRATCH_VARIABLE_ADX(max_spl) + x * sizeof(uint32_t))
#define SCRATCH_PDM_MODE_ADX(x) (uint32_t) (SCRATCH_VARIABLE_ADX(pdm_mode) + x * sizeof(uint32_t))
#define SCRATCH_PDM_MAIN_CLK_AT_LAST_CONFIG_ADX (SCRATCH_VARIABLE_ADX(pdm_main_clock_at_last_config))
#define SCRATCH_EXT_CLK_FREQ_ADX (SCRATCH_VARIABLE_ADX(ext_clk_freq))
#define SCRATCH_PLL_CLK_FREQ_ADX (SCRATCH_VARIABLE_ADX(pll_clk_freq))
#define SCRATCH_FLL_CLK_FREQ_ADX (SCRATCH_VARIABLE_ADX(fll_clk_freq))
#define SCRATCH_PLL_PRESET (SCRATCH_VARIABLE_ADX(pll_preset))
#define SCRATCH_CRC_REGION_BEGIN_ADX SCRATCH_VALID_ADX
#define SCRATCH_CRC_REGION_LENGTH (sizeof(syntiant_ndp120_scratch_t) - offsetof(syntiant_ndp120_scratch_t, valid))

/**
 * @brief config layout as it is stored in scratch
 */
typedef struct {
    uint32_t checksum; /* must be first */
    uint32_t valid;    /* must be second */

    uint32_t input_clk;

    uint32_t ext_clk_freq;
    uint32_t pll_clk_freq;
    uint32_t fll_clk_freq;
    uint32_t pll_preset;
    uint32_t pdm_main_clock_at_last_config;

    uint32_t pdm_sample_rate[2]; 
    uint32_t pdm_rate[2]; 
    uint32_t pdm_clk_mode[2];
    uint32_t pdm_mode[2];
    uint32_t max_spl[2];

    uint32_t label_size;
    uint8_t labels[LABELS_MAX_SIZE];
    uint32_t fw_version_size;
    uint8_t fw_version[VERSION_MAX_SIZE];
    uint32_t params_version_size;
    uint8_t params_version[VERSION_MAX_SIZE];
    uint32_t pkg_version_size;
    uint8_t pkg_version[VERSION_MAX_SIZE];
    uint32_t dsp_fw_version_size;
    uint8_t dsp_fw_version[VERSION_MAX_SIZE];
} syntiant_ndp120_scratch_t;

SYNTIANT_CASSERT(sizeof(syntiant_ndp120_scratch_t) < 0xC00, "syntiant_ndp120_scratch_t has grown too large")

/**
 * @brief NDP120 status and debugging set flags
 */
enum syntiant_ndp120_status_set_e {
    SYNTIANT_NDP120_STATUS_SET_CLEAR = 0x01,
    /**< clear status state (counters, etc.) */
    SYNTIANT_NDP120_STATUS_SET_MAILBOX_TRACE = 0x02,
    /**< set or clear mailbox tracing */
    SYNTIANT_NDP120_STATUS_SET_ALL_M
    = ((SYNTIANT_NDP120_STATUS_SET_MAILBOX_TRACE << 1) - 1)
};

/**
 * @brief NDP120 status and debug mailbox directions
 */
enum syntiant_ndp120_status_mailbox_direction_e {
    SYNTIANT_NDP120_STATUS_MAILBOX_DIRECTION_HOST_TO_MCU = 0,
    SYNTIANT_NDP120_STATUS_MAILBOX_DIRECTION_MCU_TO_HOST = 1,
    SYNTIANT_NDP120_STATUS_MAILBOX_DIRECTION_MAX
        = SYNTIANT_NDP120_STATUS_MAILBOX_DIRECTION_MCU_TO_HOST
};

/**
 * @brief NDP120 status and debugging information
 */
struct syntiant_ndp120_status_s {
    unsigned int set;               /**< set flags */
    int mailbox_trace;              /**< enable mailbox tracing */
    uint32_t h2m_mailbox_req;       /**< host to mcu mailbox requests */
    uint32_t h2m_mailbox_rsp;       /**< host to mcu mailbox responses */
    uint32_t h2m_mailbox_unexpected;
    /**< host to mcu mailbox unexpected messages */
    uint32_t h2m_mailbox_error;
    /**< host to mcu mailbox protocol errors */
    uint32_t m2h_mailbox_req;       /**< mcu to host mailbox requests */
    uint32_t m2h_mailbox_rsp;       /**< mcu to host mailbox responses */
    uint32_t m2h_mailbox_unexpected;
    /**< mcu to host mcu mailbox unexpected messages */
    uint32_t m2h_mailbox_error;
    /**< mcu to host mcu mailbox protocol errors */
    uint32_t missed_frames;         /**< missed MATCH reports */
};

/**
 * @brief NDP120 metadata set
 */
enum syntiant_ndp120_nn_metadata_set_e {
    SYNTIANT_NDP120_SET_NN_NUM = 0x0001,
    SYNTIANT_NDP120_SET_LAYER_PER_NN = 0x0002,
    SYNTIANT_NDP120_SET_IS_NN_CACHED = 0x0004,
    SYNTIANT_NDP120_SET_INPUT_ISA_IDX = 0x0008,
    SYNTIANT_NDP120_SET_OUTPUT_ISA_IDX = 0x0010,
    SYNTIANT_NDP120_SET_INPUT_LAYER_TYPE = 0x0020,
    SYNTIANT_NDP120_SET_INPUT_SIZE = 0x0040,
    SYNTIANT_NDP120_SET_INPUT_COORD = 0x0080,
    SYNTIANT_NDP120_SET_OUTPUT_COORD = 0x0100,
    SYNTIANT_NDP120_SET_CACHE_INST = 0x0200
};

/**
 * @brief NDP120 neural network metadata information
 */
typedef struct {
    unsigned int set;                  /**< set flags */
    uint32_t nn_num;                    /**< number of neural networks */
    uint32_t nn_idx;
    /**< the index of network for which the following would be configured */
    uint32_t layer_idx;
    /**< the index of the layer in the network for which input, output and cache
         instructions need to be written. */
    uint32_t layers_per_nn;             /**< number rof layers per network */
    uint32_t is_nn_cached;              /**< is the network cached */
    uint32_t nn_input_isa_idx;          /**< input layer instruction index */
    uint32_t nn_output_isa_idx;         /**< output layer instruction index */
    uint32_t nn_input_layer_type;       /**< the type of input layer */
    uint32_t nn_input_layer_size[3];    /**< input layer size x, y, z */
    uint32_t input_coord;               /**< input coordinates of a layer */
    uint32_t output_coord;              /**< output coordinates of a layer */
    struct {
        uint32_t input_base_coord_max;
        uint32_t output_base_coord_max;
        uint32_t input_base_coord_add;
        uint16_t input_offset_add;
        uint16_t input_offset_max;
        uint16_t output_base_coord_add;
        uint16_t output_base_coord_stride;
    } cache_inst;                       /**< cache instruction params */
} syntiant_ndp120_nn_metadata_t;

/**
 * @brief NDP120 MCU orchestrator set enum
 */
enum syntiant_ndp120_mcu_orch_set_e {
    SYNTIANT_NDP120_SET_NUM_NODES = 0x0001,
    SYNTIANT_NDP120_SET_NODE = 0x0002,
    SYNTIANT_NDP120_SET_FLOWMAP = 0x0004
};

/**
 * @brief NDP120 neural network metadata information
 */
typedef struct {
    uint32_t set;
    uint32_t node_idx;
    uint32_t num_nodes;
    uint8_t id;
    uint8_t flow_id;
    uint8_t status;
    uint8_t type;
    uint8_t action;
    uint8_t num_inputs;
    uint8_t num_outputs;
    uint8_t flowmap;
    uint8_t input_edges[4];
    uint8_t next_ids[4];
} syntiant_ndp120_mcu_orch_t;

enum {
    PLL_PRESET_OP_VOLTAGE_0p9,
    PLL_PRESET_OP_VOLTAGE_1p1
};

typedef struct {
    uint32_t offset;
    uint32_t value;
} ndp120_pll_preset_value_t;

typedef struct {
    const char *name;
    int operating_voltage;
    uint32_t input_freq;
    uint32_t output_freq;
    ndp120_pll_preset_value_t *values;
} ndp120_pll_preset_t;

/**
 * @brief retrieve NDP120 status and debugging information
 *
 * Configuration variables with the corresponding
 * @c SYNTIANT_NDP120_STATUS_SET_* bit set in @c debug->set will
 * be updated.  After performing any requested updates, the current state
 * will be returned in @c status.
 *
 * @param ndp NDP state object
 * @param status status data object
 * @return a @c SYNTIANT_NDP_ERROR_* code
 */
extern int syntiant_ndp120_status(struct syntiant_ndp_device_s *ndp,
                                  struct syntiant_ndp120_status_s *status);


/* generic */

int
syntiant_ndp120_read_block(struct syntiant_ndp_device_s *ndp, int mcu,
    uint32_t address, void *value, uint32_t count);

int
syntiant_ndp120_write_block(struct syntiant_ndp_device_s *ndp, int mcu,
    uint32_t address, void *value, uint32_t count);

int
syntiant_ndp120_write(struct syntiant_ndp_device_s *ndp, int mcu,
    uint32_t address, uint32_t value);

int syntiant_ndp120_read(struct syntiant_ndp_device_s *ndp, int mcu,
    uint32_t address, void *value);

void syntiant_ndp120_mbin_send(struct syntiant_ndp_device_s *ndp, uint8_t data);

int syntiant_ndp120_mbin_resp_get(
    struct syntiant_ndp_device_s *ndp, uint8_t *data);

int syntiant_ndp120_do_mailbox_req(
    struct syntiant_ndp_device_s *ndp, uint8_t req, uint32_t *resp);

int syntiant_ndp120_do_mailbox_req_no_sync(
    struct syntiant_ndp_device_s *ndp, uint8_t req, uint32_t *resp);

int syntiant_ndp120_config_misc(struct syntiant_ndp_device_s *ndp,
            syntiant_ndp120_config_misc_t *config);

int syntiant_ndp120_debug_extract(struct syntiant_ndp_device_s *ndp, int type,
        void *data, unsigned int *len);

uint32_t syntiant_get_last_network_id(struct syntiant_ndp_device_s *ndp);

int syntiant_ndp120_test_function(struct syntiant_ndp_device_s *ndp);

int syntiant_ndp120_get_dsp_fw_state_header(
    struct syntiant_ndp_device_s *ndp, ndp120_dsp_fw_state_header_t *header);

int syntiant_ndp120_is_a0(struct syntiant_ndp_device_s *ndp);

/* config related */

int syntiant_ndp120_config_notify_on_sample_ready(struct syntiant_ndp_device_s *ndp, uint32_t enable);

int syntiant_ndp120_config_pdm(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_pdm_t *config);

int syntiant_ndp120_config_decimation(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_decimation_t *config);

int syntiant_ndp120_config_gain(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_gain_t *config);

int syntiant_ndp120_config_farrow(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_farrow_t *config);

int syntiant_ndp120_config_clk_src(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_clk_src_t *config);

int syntiant_ndp120_config_clk_pll( struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_clk_pll_t *config);

int syntiant_ndp120_config_clk_div( struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_clk_div_t *config);

int syntiant_ndp120_config_gpio(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_gpio_t *config);

int syntiant_ndp120_config_filter(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_config_filter_t *config);

int syntiant_ndp120_get_put_pll_clk_freq(struct syntiant_ndp_device_s *ndp, uint32_t *freq);

int syntiant_ndp120_get_put_fll_clk_freq(struct syntiant_ndp_device_s *ndp, uint32_t *freq);

int syntiant_ndp120_get_put_main_clk_src(struct syntiant_ndp_device_s *ndp, uint32_t *src);

int syntiant_ndp120_get_put_ext_clk_freq(struct syntiant_ndp_device_s *ndp, uint32_t *ext_freq);

int syntiant_ndp120_get_main_clk_freq(struct syntiant_ndp_device_s *ndp, uint32_t *freq);

uint32_t syntiant_ndp120_get_dsp_fw_pointer(struct syntiant_ndp_device_s *ndp);

uint32_t syntiant_ndp120_get_mcu_fw_pointer(struct syntiant_ndp_device_s *ndp);

int syntiant_ndp120_config_nn_metadata(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_nn_metadata_t * config);

int syntiant_ndp120_config_mcu_orchestrator(struct syntiant_ndp_device_s *ndp, syntiant_ndp120_mcu_orch_t * config);

/* DSP flow related */
int syntiant_ndp120_dsp_flow_setup_reset(ndp120_dsp_data_flow_setup_t *setup);

int syntiant_ndp120_dsp_flow_setup_add_rule(ndp120_dsp_data_flow_setup_t *setup, const ndp120_dsp_data_flow_rule_t *rule, int src);

int syntiant_ndp120_dsp_flow_setup_del_rule(ndp120_dsp_data_flow_setup_t *setup, const ndp120_dsp_data_flow_rule_t *rule, int src);

int syntiant_ndp120_dsp_flow_setup_apply(struct syntiant_ndp_device_s *ndp, ndp120_dsp_data_flow_setup_t *setup);

int syntiant_ndp120_dsp_flow_setup_get(struct syntiant_ndp_device_s *ndp, ndp120_dsp_data_flow_setup_t *setup);

int syntiant_ndp120_dsp_flow_get_put_set_id(struct syntiant_ndp_device_s *ndp, int *set_id);

/* only works if NDP120_DEBUG is set */
void syntiant_ndp120_dsp_flow_setup_dump(ndp120_dsp_data_flow_setup_t *setup);

/* NN related */

int syntiant_ndp120_get_matched_network_id(struct syntiant_ndp_device_s *ndp, uint32_t *network);

int syntiant_ndp120_num_networks(struct syntiant_ndp_device_s *ndp, uint8_t *num_networks);

uint32_t syntiant_ndp120_get_nn_graph(struct syntiant_ndp_device_s *ndp);

#endif
