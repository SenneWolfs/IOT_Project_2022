/*
 * TaskUDPClient.h
 *
 *  Created on: Nov 16, 2022
 *      Author: Eduardo Bemelmans
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

/*******************************************************************************
* Macros
********************************************************************************/
/* Wi-Fi Credentials: Modify WIFI_SSID, WIFI_PASSWORD and WIFI_SECURITY_TYPE
 * to match your Wi-Fi network credentials.
 * Note: Maximum length of the Wi-Fi SSID and password is set to
 * CY_WCM_MAX_SSID_LEN and CY_WCM_MAX_PASSPHRASE_LEN as defined in cy_wcm.h file.
 */

// #define WIFI_SSID                         "xxxx"
// #define WIFI_PASSWORD                     "<3<3<3<3"
#define WIFI_SSID                         "NETGEAR98"
#define WIFI_PASSWORD                     "543DA3C9"

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                 CY_WCM_SECURITY_WPA2_AES_PSK

/* Maximum number of connection retries to the Wi-Fi network. */
#define MAX_WIFI_CONN_RETRIES             (10u)

/* Wi-Fi re-connection time interval in milliseconds */
#define WIFI_CONN_RETRY_INTERVAL_MSEC     (1000)

#define MAKE_IPV4_ADDRESS(a, b, c, d)     ((((uint32_t) d) << 24) | \
                                          (((uint32_t) c) << 16) | \
                                          (((uint32_t) b) << 8) |\
                                          ((uint32_t) a))

/* Change the server IP address to match the UDP server address (IP address
 * of the PC).
 */
#define UDP_SERVER_IP_ADDRESS             MAKE_IPV4_ADDRESS(192, 168, 0, 121)
#define UDP_SERVER_PORT                   (57345)

/*******************************************************************************
* Global variables
********************************************************************************/
extern QueueHandle_t queue_controller_handle;
/* Controller task handle. */
extern TaskHandle_t TaskControllerHandle;

/*******************************************************************************
* Function Prototype
********************************************************************************/
void TaskController(void *arg);

#endif /* CONTROLLER_H_ */

/* [] END OF FILE */

