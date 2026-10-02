#include <string.h>
#include <pico/unique_id.h>
#include <tusb.h>

#define _PID_MAP(itf, n) ((CFG_TUD_##itf) << (n))
#define USB_PID (0x4000 | _PID_MAP(CDC,0) | _PID_MAP(MSC,1) | _PID_MAP(HID,2) | _PID_MAP(MIDI,3) | _PID_MAP(VENDOR,4) | _PID_MAP(ECM_RNDIS,5) | _PID_MAP(NCM,5))

enum { STRID_LANGID=0, STRID_MANUFACTURER, STRID_PRODUCT, STRID_SERIAL, STRID_INTERFACE, STRID_MAC };
enum { ITF_NUM_CDC=0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };
enum {
#if CFG_TUD_ECM_RNDIS
  CONFIG_ID_RNDIS=0, CONFIG_ID_ECM=1,
#else
  CONFIG_ID_NCM=0,
#endif
  CONFIG_ID_COUNT
};

tusb_desc_device_t const desc_device = {
  .bLength=sizeof(tusb_desc_device_t), .bDescriptorType=TUSB_DESC_DEVICE,
  .bcdUSB=0x0110, .bDeviceClass=TUSB_CLASS_MISC,
  .bDeviceSubClass=MISC_SUBCLASS_COMMON, .bDeviceProtocol=MISC_PROTOCOL_IAD,
  .bMaxPacketSize0=CFG_TUD_ENDPOINT0_SIZE, .idVendor=0xCafe, .idProduct=USB_PID,
  .bcdDevice=0x0101, .iManufacturer=STRID_MANUFACTURER, .iProduct=STRID_PRODUCT,
  .iSerialNumber=STRID_SERIAL, .bNumConfigurations=CONFIG_ID_COUNT
};
uint8_t const *tud_descriptor_device_cb(void){ return (uint8_t const *)&desc_device; }

#define MAIN_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_RNDIS_DESC_LEN)
#define ALT_CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_ECM_DESC_LEN)
#define NCM_CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

#if CFG_TUSB_MCU == OPT_MCU_LPC175X_6X || CFG_TUSB_MCU == OPT_MCU_LPC177X_8X || CFG_TUSB_MCU == OPT_MCU_LPC40XX
#define EPNUM_NET_NOTIF 0x81
#define EPNUM_NET_OUT   0x02
#define EPNUM_NET_IN    0x82
#elif CFG_TUSB_MCU == OPT_MCU_CXD56
#define EPNUM_NET_NOTIF 0x83
#define EPNUM_NET_OUT   0x02
#define EPNUM_NET_IN    0x81
#elif defined(TUD_ENDPOINT_ONE_DIRECTION_ONLY)
#define EPNUM_NET_NOTIF 0x81
#define EPNUM_NET_OUT   0x02
#define EPNUM_NET_IN    0x83
#else
#define EPNUM_NET_NOTIF 0x81
#define EPNUM_NET_OUT   0x02
#define EPNUM_NET_IN    0x82
#endif

#if CFG_TUD_ECM_RNDIS
static uint8_t const rndis_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_RNDIS+1, ITF_NUM_TOTAL, 0, MAIN_CONFIG_TOTAL_LEN, 0, 100),
  TUD_RNDIS_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, EPNUM_NET_NOTIF, 8, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE),
};
static uint8_t const ecm_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_ECM+1, ITF_NUM_TOTAL, 0, ALT_CONFIG_TOTAL_LEN, 0, 100),
  TUD_CDC_ECM_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, STRID_MAC, EPNUM_NET_NOTIF, 64, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};
#else
static uint8_t const ncm_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_NCM+1, ITF_NUM_TOTAL, 0, NCM_CONFIG_TOTAL_LEN, 0, 100),
  TUD_CDC_NCM_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, STRID_MAC, EPNUM_NET_NOTIF, 64, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};
#endif

static uint8_t const *const configuration_arr[2] = {
#if CFG_TUD_ECM_RNDIS
  [CONFIG_ID_RNDIS]=rndis_configuration, [CONFIG_ID_ECM]=ecm_configuration
#else
  [CONFIG_ID_NCM]=ncm_configuration
#endif
};
uint8_t const *tud_descriptor_configuration_cb(uint8_t index){ return (index<CONFIG_ID_COUNT)?configuration_arr[index]:NULL; }

#if CFG_TUD_NCM
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define MS_OS_20_DESC_LEN 0xB2
uint8_t const desc_bos[] = { TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN,1), TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN,1) };
uint8_t const *tud_descriptor_bos_cb(void){ return desc_bos; }
uint8_t const desc_ms_os_20[] = {
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0,0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_CDC,0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W','I','N','N','C','M',0,0,0,0,0,0,0,0,0,0,
  U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08-0x08-0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
  'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,'r',0,'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,'s',0,0,0,
  U16_TO_U8S_LE(0x0050),
  '{',0,'1',0,'2',0,'3',0,'4',0,'5',0,'6',0,'7',0,'8',0,'-',0,'0',0,'D',0,'0',0,'8',0,'-',0,'4',0,'3',0,'F',0,'D',0,'-',0,'8',0,'B',0,'3',0,'E',0,'-',0,'1',0,'2',0,'7',0,'C',0,'A',0,'8',0,'A',0,'F',0,'F',0,'F',0,'9',0,'D',0,'}',0,0,0,0,0
};
TU_VERIFY_STATIC(sizeof(desc_ms_os_20)==MS_OS_20_DESC_LEN, "Incorrect size");
bool tud_vendor_control_xfer_cb(uint8_t rhport,uint8_t stage,tusb_control_request_t const *request){
  if(stage!=CONTROL_STAGE_SETUP) return true;
  if(request->bmRequestType_bit.type==TUSB_REQ_TYPE_VENDOR && request->bRequest==1 && request->wIndex==7){
    uint16_t total_len; memcpy(&total_len,desc_ms_os_20+8,2); return tud_control_xfer(rhport,request,(void *)(uintptr_t)desc_ms_os_20,total_len);
  }
  return false;
}
#endif

static char const *string_desc_arr[] = {
  [STRID_LANGID]=(const char[]){0x09,0x04}, [STRID_MANUFACTURER]="TinyUSB",
  [STRID_PRODUCT]="TinyUSB Device", [STRID_SERIAL]=NULL,
  [STRID_INTERFACE]="TinyUSB Network Interface"
};
static uint16_t _desc_str[33];
uint16_t const *tud_descriptor_string_cb(uint8_t index,uint16_t langid){
  (void)langid; unsigned chr_count=0;
  switch(index){
    case STRID_LANGID: memcpy(&_desc_str[1],string_desc_arr[0],2); chr_count=1; break;
    case STRID_SERIAL:{ pico_unique_board_id_t id; pico_get_unique_board_id(&id); for(unsigned i=0;i<sizeof(id.id);i++){ _desc_str[1+chr_count++]="0123456789ABCDEF"[(id.id[i]>>4)&0xf]; _desc_str[1+chr_count++]="0123456789ABCDEF"[id.id[i]&0xf]; } break; }
    case STRID_MAC: for(unsigned i=0;i<sizeof(tud_network_mac_address);i++){ _desc_str[1+chr_count++]="0123456789ABCDEF"[(tud_network_mac_address[i]>>4)&0xf]; _desc_str[1+chr_count++]="0123456789ABCDEF"[tud_network_mac_address[i]&0xf]; } break;
    default:{ if(!(index<sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL; const char *str=string_desc_arr[index]; chr_count=strlen(str); size_t max_count=sizeof(_desc_str)/sizeof(_desc_str[0])-1; if(chr_count>max_count) chr_count=max_count; for(size_t i=0;i<chr_count;i++) _desc_str[1+i]=str[i]; break; }
  }
  _desc_str[0]=(uint16_t)((TUSB_DESC_STRING<<8)|(2*chr_count+2)); return _desc_str;
}
