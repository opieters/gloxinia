#ifndef __SPI_H__
#define	__SPI_H__

#include <xc.h>// include processor files - each processor file is guarded.  
#include <utilities.h>

// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    typedef enum {
        SPI_TRANSFER_PENDING,
        SPI_TRANSFER_DONE,
        SPI_TRANSFER_ONGOING,
        N_SPI_STATUS_T
    } spi_status_t;
    
    typedef struct {
        uint16_t* write_data;
        uint16_t* read_data;
        uint8_t data_length;
        pin_t cs_pin;
        spi_status_t status;
    } spi_message_t;
    
    typedef enum {
        SPI_MODULE_SELECTOR_1,
        SPI_MODULE_SELECTOR_2,
        SPI_MODULE_SELECTOR_3,
        //SPI_MODULE_SELECTOR_4,
    } spi_module_selector_t;
    
    
    spi_message_t* spi_init_message(spi_message_t* m, uint16_t* write_data, uint16_t* read_data, uint8_t length, const pin_t* pin);
    
    void spi1_init(void);
    void spi2_init(void);
    void spi3_init(void);
    
    void spi1_send_message(spi_message_t* m);
    void spi2_send_message(spi_message_t* m);
    void spi3_send_message(spi_message_t* m);
    
    typedef void (*spi_handler_t)(spi_message_t*);
    
    void spi_software_send(pin_t* sck, pin_t* sdo, pin_t* sdi, pin_t* cs, uint8_t* tx_data, uint8_t* rx_data, uint8_t n, uint16_t period);
    
    spi_handler_t spi_get_handler(const spi_module_selector_t spi_module);

#ifdef	__cplusplus
}
#endif

#endif	/* __SPI_H__ */

