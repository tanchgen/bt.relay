/* Includes ------------------------------------------------------------------*/
#include "my_main.h"
#include "cube_hal.h"

#include "osal.h"
#include "sample_service.h"
#include "role_type.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bt01.h"
#include "my_bt01_def.h"

//#include "logger.h"

#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define BDADDR_SIZE 6

/* Private macros ------------------------------------------------------------ */
/*
// HCI Packet types 
#define HCI_COMMAND_PKT		0x01
#define HCI_ACLDATA_PKT		0x02
#define HCI_SCODATA_PKT		0x03
#define HCI_EVENT_PKT		0x04
#define HCI_VENDOR_PKT		0xff

#define HCI_TYPE_OFFSET                 0
#define HCI_VENDOR_CMDCODE_OFFSET       1
#define HCI_VENDOR_LEN_OFFSET           2
#define HCI_VENDOR_PARAM_OFFSET         4

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    6

// Commands 
#define VERSION         0x01
#define EEPROM_READ     0x02
#define EEPROM_WRITE    0x03
#define BLUENRG_RESET   0x04
#define HW_BOOTLOADER   0x05

#define MAX_RESP_SIZE 255

#define RESP_VENDOR_CODE_OFFSET     1
#define RESP_LEN_OFFSET_LSB         2
#define RESP_LEN_OFFSET_MSB         3
#define RESP_CMDCODE_OFFSET         4
#define RESP_STATUS_OFFSET          5
#define RESP_PARAM_OFFSET           6

// Types of vendor codes 
#define ERROR               0
// Error codes
#define UNKNOWN_COMMAND	    0x01
#define INVALID_PARAMETERS	0x12

#define RESPONSE            1

// Private macros ------------------------------------------------------------
#define PACK_2_BYTE_PARAMETER(ptr, param)  do{\
                *((uint8_t *)ptr) = (uint8_t)(param);   \
                *((uint8_t *)ptr+1) = (uint8_t)(param)>>8; \
                }while(0)

// Private variables ---------------------------------------------------------
// UART handler declaration
UART_HandleTypeDef UartHandle;
extern UART_HandleTypeDef huart1;

// Buffer used for transmission 
// Buffer used for reception 
uint8_t uart_header[UARTHEADERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];
*/
// SPI handler declaration 
extern SPI_HandleTypeDef SpiHandle;

// Uncomment the line corresponding to the role you want to have 
BLE_RoleTypeDef BLE_Role = SERVER;

extern volatile uint8_t set_connectable;
extern volatile int connected;
extern volatile uint8_t notification_enabled;

extern volatile uint8_t start_read_tx_char_handle;
extern volatile uint8_t start_read_rx_char_handle;
extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;

extern volatile uint32_t disconnCount; // Таймаут аутентификации по SHA1-хэш 120 сек. 
extern volatile uint16_t connection_handle;
extern volatile uint32_t relayCount1, relayCount2;
extern volatile uint32_t resetCount;

//uint8_t data[20]; //data for send to characteristic

/* Private function prototypes -----------------------------------------------*/
void User_Process(void);

/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Main function to show how to use the BlueNRG based device 
 * @param  None
 * @retval None
 */
int main(void)
{ 
#ifdef CLIENT_ROLE
  uint8_t BDADDR[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02}; // для клиента
  int ret;
#else // CLIENT_ROLE
  uint8_t BDADDR[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02}; // для сервера
#endif // CLIENT_ROLE

 //uint8_t SERVER_BDADDR[] = {0xfd, 0x00, 0x25, 0xec, 0x02, 0x04}; //BT address for HRM test
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  /* STM32Cube HAL library initialization:
   *  - Configure the Flash prefetch, Flash preread and Buffer caches
   *  - Systick timer is configured by default as source of time base, but user 
   *    can eventually implement his proper time base source (a general purpose 
   *    timer for example or other time source), keeping in mind that Time base 
   *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
   *    handled in milliseconds basis.
   *  - Low Level Initialization
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure RELAY1 (GPIO1) PB1*/
  relayInit(RELAY1);
 
  /* Configure RELAY2 (GPIO1) PB2*/
  relayInit(RELAY2);
     
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();

  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();
  
  Osal_MemCpy(bdaddr, BDADDR, sizeof(BDADDR));
  
#ifdef TERMINAL_ON  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    printf("Setting BD_ADDR failed.\n\r");
  }

  ret = aci_gatt_init();    
  if(ret){
    printf("GATT_Init failed.\n\r");
  }

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret != BLE_STATUS_SUCCESS){
    printf("GAP_Init failed.\n\r");
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    printf("BLE Stack Initialized.\n\r");
  }

#ifndef CLIENT_ROLE
  ret = Add_Sample_Service();

  if(ret == BLE_STATUS_SUCCESS)
      printf("Service added successfully.\n\r");
  else
      printf("Error while adding service.\n\r");
#endif // CLIENT_ROLE
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

#else
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, 
                            CONFIG_DATA_PUBADDR_LEN,
                            bdaddr);

  aci_gatt_init();

  aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                               OOB_AUTH_DATA_ABSENT,
                               NULL,
                               7,
                               16,
                               USE_FIXED_PIN_FOR_PAIRING,
                               123456,
                               BONDING);

  Add_Sample_Service();

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);
  
#endif  
  
  
  while(1)
  {
    HCI_Process();
    User_Process();
  }
}

void User_Process(void)
{
  if ( resetCount == 1 ) {
    tBleStatus ret;
    ret = aci_gap_terminate(connection_handle, SHA_TIMEOUT_REASON);
    if(ret){
#ifdef TERMINAL_ON
      printf("Terminate connection failed.\n\r");
#endif // TERMINAL_ON 
    }
    GAP_DisconnectionComplete_CB();
    HAL_Delay(2000);
    NVIC_SystemReset();
  }


  if(set_connectable){
    /* Establish connection with remote device */
    Make_Connection();
    set_connectable = FALSE;
  }

  if ( disconnCount == 1 ){
    tBleStatus ret;
    ret = aci_gap_terminate(connection_handle, SHA_TIMEOUT_REASON);
    if(ret){
#ifdef TERMINAL_ON
      printf("Terminate connection failed.\n\r");
#endif // TERMINAL_ON 
    }
    GAP_DisconnectionComplete_CB();
    disconnCount = 0;
  }

  if ( relayCount1 == 1 ) {
    relayOff( RELAY1 );
    relayCount1 = 0;
  }

  if ( relayCount2 == 1 ) {
    relayOff( RELAY2 );
    relayCount2 = 0;
  }
    
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}
