#include <xc.h> 
#include <pga.h>
#include <spi.h>
#include <utilities.h>


uint8_t pga_convert_channel_to_bits(pga_mux_channel_t channel){
    switch(channel){
        case PGA_MUX_VCAL_CH0:
            return 0b0000;
            break;
        case PGA_MUX_CH1:
            return 0b0001;
            break;
        case PGA_CAL1:
            return 0b1100;
            break;
        case PGA_CAL2:
            return 0b1101;
            break;
        case PGA_CAL3:
            return 0b1110;
            break;
        case PGA_CAL4:
            return 0b1111;
            break;
        default:
            return 0b0000;
            break;
    }
}

uint8_t pga_convert_gain_to_bits(pga_gain_t channel){
    switch(channel){
        case PGA_GAIN_1:
            return 0b0000;
            break;
        case PGA_GAIN_2:
            return 0b0001;
            break;
        case PGA_GAIN_5:
            return 0b0010;
            break;
        case PGA_GAIN_10:
            return 0b0011;
            break;
        case PGA_GAIN_20:
            return 0b0100;
            break;
        case PGA_GAIN_50:
            return 0b0101;
            break;
        case PGA_GAIN_100:
            return 0b0110;
            break;
        case PGA_GAIN_200:
            return 0b0111;
            break;
        default:
            return 0b0000;
            break;
    }
}

void init_pga(pga_config_t* config){
    // configure nCS pin as output 
    CLEAR_BIT(config->cs_pin.tris_r, config->cs_pin.n);
    SET_BIT(config->cs_pin.lat_r, config->cs_pin.n);
    
    update_pga_status(config);
}

void update_pga_status(pga_config_t* config){
    spi_message_t m;
    uint16_t write_buffer[1], read_buffer[1];
    m.cs_pin = config->cs_pin;
    
    m.write_data = write_buffer;
    m.read_data = read_buffer;
    m.data_length = 1;
    
    switch(config->status){
        case PGA_STATUS_ON:
            // enable device
            write_buffer[0] = 0xE100;
            
            m.status = SPI_TRANSFER_PENDING;
            config->spi_message_handler(&m);
            
            // send the switch channel command
            write_buffer[0] = 0x2A00 
                    | (pga_convert_gain_to_bits(config->gain) << 4) 
                    | pga_convert_channel_to_bits(config->channel);
            m.status = SPI_TRANSFER_PENDING;
            config->spi_message_handler(&m);
            
            
            break;
        case PGA_STATUS_OFF:
        case PGA_STATUS_ERROR:
            // disable device
            
            write_buffer[0] = 0xE1F1;
            
            m.status = SPI_TRANSFER_PENDING;
            config->spi_message_handler(&m);
            break;
        default:
            break;
    }

    
}

