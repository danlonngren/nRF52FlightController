#include "twi.h"
#include "nrf_port.h"

#include "nrf_drv_twi.h"

#include "board_config.h"


static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);


static uint8_t tx[24];
static uint8_t rx[24];

#if 0
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}
#endif


error_t twiWriteData(uint8_t address, uint8_t reg, uint8_t *txIn, uint8_t size)
{
  error_t errCode = 1;

  if (size < sizeof(tx))
  {
      tx[0] = reg;
      uint32_t count = 0;
      for (count = 0; count < size; count++)
      {
         tx[count + 1] = txIn[count];
      }

      errCode = nrf_drv_twi_tx(&m_twi, address, tx, size, false);
  }
  else
  {
      errCode = NRF_ERROR_INVALID_LENGTH;
  }
  return errCode;
}

error_t twiReadData(uint8_t address, uint8_t reg, uint8_t *rxOut, uint8_t size)
{
  uint32_t errCode;

  errCode = nrf_drv_twi_tx(&m_twi, address, &reg, sizeof(reg), false);
  if (errCode != SUCCESS)
  {
      return errCode;
  }
  errCode = nrf_drv_twi_rx(&m_twi, address, rxOut, size);
  return errCode;
}


error_t twiInit(uint8_t scl, uint8_t sda, uint8_t irqPrio)
{
    uint32_t errCode;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = scl,
       .sda                = sda,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = irqPrio,
       .clear_bus_init     = false
    };

    errCode = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    ERROR_CHECK(errCode);

    nrf_drv_twi_enable(&m_twi);
    return errCode;
}
