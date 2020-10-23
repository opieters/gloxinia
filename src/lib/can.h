/* 
 * File:   
 * Author: opieters
 * Comments:
 * Revision history: 0.0.1
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __CAN_H__
#define	__CAN_H__

#include <xc.h>
#include <utilities.h>

/**
 * @brief Maximum number of bytes that can be sent in a single CAN message.
 */
#define CAN_MAX_N_BYTES 8 

#define CAN_HEADER(cmd, id) ((((cmd) & 0xFF) << 8) | ((id) & 0xFF)) 
#define CAN_NO_REMOTE_FRAME 0
#define CAN_REMOTE_FRAME 1
#define CAN_EXTENDED_FRAME 1
#define CAN_NO_EXTENDED_FRAME 0

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
    /**
     * @brief Fields associated with an ECAN message.
     * 
     * @param identifier: 11-bit primary ECAN identifier
     * @param remote_frame: 0 if no remote frame, 1 if remote frame
     * @param extended_frame: 0 if standard (11-bit address), 1 if 29-bit identifier
     * @param extended_identifier: 18 bit extended identifier field
     * @param data_length: number of bytes to transmit (max. 8)
     * @param data: data bytes of ECAN message
     */
    typedef struct {
        uint16_t identifier;
        uint8_t remote_frame;
        uint8_t extended_frame;
        uint32_t extended_identifier;
        uint8_t data_length;
        uint8_t* data;
    } can_message_t;
    
    /**
     * @brief Possible commands that can be used to exchange information over
     * the CAN bus that is not sensor data.s
     */
    
    typedef enum {
        NO_CAN_CMD                      = 0x00,
        CAN_DATA_CMD_APDS9301           = 0x01,
        CAN_DATA_CMD_OPT3001Q1          = 0x02,
        CAN_DATA_CMD_BH1721FVC          = 0x03,
        CAN_DATA_CMD_APDS9306           = 0x04,
        CAN_DATA_CMD_SHT35              = 0x05,
        CAN_DATA_CMD_SYLVATICA          = 0x06,
        CAN_DATA_CMD_PLANALTA           = 0x07,
        CAN_DATA_CMD_LICOR              = 0x08,
        CAN_DATA_CMD_PUMP               = 0x09,
        CAN_DATA_CMD_GROWTH_CHAMBER     = 0x0A,
        CAN_DATA_CMD_RELAY_BOARD        = 0x0B,
        CAN_CONFIG_CMD_APDS9301         = 0x31,
        CAN_CONFIG_CMD_OPT3001Q1        = 0x32,
        CAN_CONFIG_CMD_BH1721FVC        = 0x33,
        CAN_CONFIG_CMD_APDS9306         = 0x34,
        CAN_CONFIG_CMD_SHT35            = 0x35,
        CAN_CONFIG_CMD_SYLVATICA        = 0x36,
        CAN_CONFIG_CMD_PLANALTA         = 0x37,
        CAN_CONFIG_CMD_LICOR            = 0x38,
        CAN_CONFIG_CMD_PUMP             = 0x39,
        CAN_CONFIG_CMD_GROWTH_CHAMBER   = 0x3A,
        CAN_MSG_SENSOR_STATUS           = 0x50,
        CAN_MSG_SENSOR_ERROR            = 0x51,
        CAN_MSG_ACTUATOR_ERROR          = 0x52,
        CAN_INFO_MSG_TEXT,
        CAN_INFO_MSG_START_MEAS,
        CAN_INFO_MSG_STOP_MEAS,
        CAN_INFO_MSG_RESET,
        CAN_INFO_MSG_ACTUATOR_STATUS,
        CAN_INFO_MSG_INIT_SENSORS,
        CAN_INFO_MSG_SAMPLE,
        CAN_INFO_MSG_HELLO,
        CAN_INFO_MSG_INT_ADC,
        CAN_INFO_MSG_INIT_DAC,
        CAN_INFO_MSG_SCHEDULE_I2C,
        CAN_INFO_LIA_GAIN_SET,
        CAN_INFO_CMD_MEASUREMENT_START,
        CAN_INFO_CMD_ACTUATOR_START,
        CAN_CONFIG_CMD,
        N_CAN_MSG
    } can_cmd_t;
    
    /**
    * @brief Symbolic representation of CAN module status
    */
   typedef enum {
       CAN_NO_ERROR, 
       CAN_TX_PENDING,
       CAN_RX_PENDING,
   } can_status_t;
    
    typedef enum {
        CAN_MODULE_OFF = 0,
        CAN_MODULE_ON = 1,
    } can_module_status_t;
    
    
    extern volatile uint8_t __init_sensors;
    
    void can_cmd_info_rx(uint8_t cmd, uint8_t* data, uint8_t length);
    void can_cmd_info_tx(uint8_t cmd, uint8_t* data, uint8_t length);
    
    extern volatile uint8_t received_ecan_message;

    /**
     * @brief Initialises the DMA channel handling ECAN messages.
     * 
     * @attention Should be executed only after initialising the ECAN module!
     */
    void can_init_dma_channel(void);
    
    /**
     * @brief Initialises ECAN module.
     */
    void can_init(void);
    
    void can_disable(void);
    
    /**
     * @brief Places a CAN message in the buffer of the corresponding channel.
     * 
     * @details Does not actually handle CAN transfer. This is handled by the
     * DMA. For now, user configured priorities cannot be assigned to messages.
     * 
     * @param message: the ECAN message to transmit
     * @param channel: the ECAN channel number (0-7) on which to transmit the 
     * data.
     * 
     * @return failure (message in buffer not yet sent) or success in the form 
     * of a `can_status_t`.
     */
    inline __attribute__((always_inline)) can_status_t can_send_message(can_message_t* message, uint8_t channel);
    inline can_status_t can_send_message_any_ch(can_message_t* m);
    
    inline void can_process_node_message(void);
    inline void can_process_gateway_message(void);
    
    inline void can_parse_message(can_message_t* m, uint16_t* raw_data);
    
    void can_init_message(can_message_t* m, 
            uint16_t identifier,
            uint8_t remote_frame,
            uint8_t extended_frame,
            uint32_t extended_identifier,
            uint8_t* data,
            uint8_t data_length);
    

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* __CAN_H__ */
