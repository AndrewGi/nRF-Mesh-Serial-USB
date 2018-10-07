#include "nrf_soc.h"
#include "nrf_drv_clock.h"
static volatile uint32_t requests = 0;
void nrf_drv_clock_hfclk_request(nrf_drv_clock_handler_item_t * p_handler_item){
  uint32_t current_requests;
  CRITICAL_REGION_ENTER();
  requests++;
  current_requests = requests;
  CRITICAL_REGION_EXIT();
  uint32_t is_running = false;;
  sd_clock_hfclk_is_running(&is_running);
  if(is_running){
    if(p_handler_item!= NULL){
      p_handler_item->event_handler(NRF_DRV_CLOCK_EVT_HFCLK_STARTED);
    }
    return;
  }
  if(requests==1) { //Only first request should request power
    sd_clock_hfclk_request();
  }
  while(is_running==false){sd_clock_hfclk_is_running(&is_running);}; //Busy wait
  if(p_handler_item!= NULL){
    p_handler_item->event_handler(NRF_DRV_CLOCK_EVT_HFCLK_STARTED);
  }
}
void nrf_drv_clock_hfclk_release(void)
{
  CRITICAL_REGION_ENTER();
  if(requests>0)
    requests--;
  CRITICAL_REGION_EXIT();
  ASSERT(requests>=0);
  if(requests==0){
    uint32_t is_running = false;
    sd_clock_hfclk_is_running(&is_running);
    if(is_running)
      sd_clock_hfclk_release();
  }
  

}