#include "usb.hpp"

extern "C" void USBD_CDC_TxCpltcallback(uint8_t* Buf, uint32_t* Len) {
    bsp::usb::GetInstance()->Txcallback();
}
extern "C" void USBD_CDC_RxCpltcallback(uint8_t* Buf, const uint32_t* Len) {
    bsp::usb::GetInstance()->SetRxBuffer(Buf, *Len);
    bsp::usb::GetInstance()->Rxcallback();
}