/*
 * TaskController.c
 *
 *  Created on: Dec 28, 2022
 *      Author: Eduardo Bemelmans
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <inttypes.h>
#include "stdlib.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files. */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

#include "ControllerData.h"
#include "TaskController.h"

#define INVALID_CMD                        '2'

/* Acknowledgment messages to be sent UDP Server. */
#define ACK_LED_ON                         "LED ON ACK"
#define ACK_LED_OFF                        "LED OFF ACK"
#define INVALID_CMD_MSG                    "INVALID CMD RECEIVED"

/* Initial message sent to UDP Server to confirm client availability. */
#define START_COMM_MSG                         "A"

/* Buffer size to store the incoming messages from server, in bytes. */
#define MAX_UDP_RECV_BUFFER_SIZE          (20)

/* RTOS related macros for UDP client task. */
#define RTOS_TASK_TICKS_TO_WAIT           (1000)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static cy_rslt_t create_udp_client_socket(void);
static cy_rslt_t udp_client_recv_handler(cy_socket_t socket_handle, void *arg);
static cy_rslt_t connect_to_wifi_ap(void);
void print_heap_usage(char* msg);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* UDP client socket handle */
cy_socket_t client_handle;
cy_socket_sockaddr_t peer_addr;
QueueHandle_t queue_controller_handle;

void TaskController(void *arg)
{
    cy_rslt_t result;

    /* Variable to store the number of bytes sent to the UDP server. */
    uint32_t bytes_sent = 0;
    uint32_t bytes_received = 0;

    /* Data struct to receive controller from udp_client_recv_handler. */
    controller_data_msg_t* controller_data_msg;
    controller_data_msg = malloc(1*sizeof(controller_data_msg_t));

    controller_data_msg->id = 0;
    controller_data_msg->value = 0;

    /* IP address and UDP port number of the UDP server */
    cy_socket_sockaddr_t udp_server_addr = {
        .ip_address.ip.v4 = UDP_SERVER_IP_ADDRESS,
        .ip_address.version = CY_SOCKET_IP_VER_V4,
        .port = UDP_SERVER_PORT
    };

    /* Connect to Wi-Fi AP */
    if(connect_to_wifi_ap() != CY_RSLT_SUCCESS )
    {
        printf("\nTask Controller: Failed to connect to Wi-FI AP.\n");
        CY_ASSERT(0);
    }

    /* Secure Sockets initialized */
    result = cy_socket_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Task Controller: Secure Sockets initialization failed!\n");
        CY_ASSERT(0);
    }
    printf("Task Controller: Secure Sockets initialized\n");

    result = create_udp_client_socket();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Task Controller: UDP Client Socket creation failed!\n");
        CY_ASSERT(0);
    }

    /* First send data to Server and wait to receive command */
    result = cy_socket_sendto(client_handle, START_COMM_MSG, strlen(START_COMM_MSG), CY_SOCKET_FLAGS_NONE,
                                &udp_server_addr, sizeof(cy_socket_sockaddr_t), &bytes_sent);
    if(result == CY_RSLT_SUCCESS)
    {
        printf("Task Controller: Data sent to server\n");
        cyhal_gpio_write(CYBSP_USER_LED, 1);
    }
    else
    {
        printf("Task Controller: Failed to send data to server. Error : %"PRIu32"\n", result);
    }

    for(;;)
    {
        printf("Task Controller: Waiting for gamepad data...\r\n");
        /* Wait till Controller command is received from UDP Server . */
        /* Receive incoming message from UDP server. */
        result = cy_socket_recvfrom(client_handle, (void*)controller_data_msg, sizeof(controller_data_msg_t),
                                    CY_SOCKET_FLAGS_RECVFROM_NONE, NULL, 0, &bytes_received);
        printf("Task Controller: received gamepad data: id = %d, value = %d\r\n",
            controller_data_msg->id, controller_data_msg->value);

        xQueueSend(queue_controller_handle, (void*)controller_data_msg, 0UL);
        
    }
 }

/*******************************************************************************
 * Function Name: connect_to_wifi_ap()
 *******************************************************************************
 * Summary:
 *  Connects to Wi-Fi AP using the user-configured credentials, retries up to a
 *  configured number of times until the connection succeeds.
 *
 *******************************************************************************/
cy_rslt_t connect_to_wifi_ap(void)
{
    cy_rslt_t result;

    /* Variables used by Wi-Fi connection manager.*/
    cy_wcm_connect_params_t wifi_conn_param;

    cy_wcm_config_t wifi_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };

    cy_wcm_ip_address_t ip_address;

     /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wifi_config);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Task Controller: Wi-Fi Connection Manager initialization failed!\n");
        return result;
    }
    printf("Task Controller: Wi-Fi Connection Manager initialized.\r\n");

     /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));
    memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    wifi_conn_param.ap_credentials.security = WIFI_SECURITY_TYPE;

    /* Join the Wi-Fi AP. */
    for(uint32_t conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES; conn_retries++ )
    {
        result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

        if(result == CY_RSLT_SUCCESS)
        {
            printf("Task Controller: Successfully connected to Wi-Fi network '%s'.\n",
                                wifi_conn_param.ap_credentials.SSID);
            printf("Task Controller: IP Address Assigned: %d.%d.%d.%d\n", (uint8)ip_address.ip.v4,
                    (uint8)(ip_address.ip.v4 >> 8), (uint8)(ip_address.ip.v4 >> 16),
                    (uint8)(ip_address.ip.v4 >> 24));
            return result;
        }

        printf("Task Controller: Connection to Wi-Fi network failed with error code %d."
               "Retrying in %d ms...\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);

        vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
    }

    /* Stop retrying after maximum retry attempts. */
    printf("Task Controller: Exceeded maximum Wi-Fi connection attempts\n");

    return result;
}

/*******************************************************************************
 * Function Name: create_udp_client_socket
 *******************************************************************************
 * Summary:
 *  Function to create a socket and set the socket options
 *  to set callback function for handling incoming messages.
 *
 *******************************************************************************/
cy_rslt_t create_udp_client_socket(void)
{
    cy_rslt_t result;

    /* Create a UDP socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM, CY_SOCKET_IPPROTO_UDP, &client_handle);
    if(result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Variable used to set socket receive callback function. */
    cy_socket_opt_callback_t udp_recv_option = {
            .callback = udp_client_recv_handler,
            .arg = NULL
    };

    /* Register the callback function to handle messages received from UDP client. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET, CY_SOCKET_SO_RECEIVE_CALLBACK,
                                    &udp_recv_option, sizeof(cy_socket_opt_callback_t));

    return result;
}

/*******************************************************************************
 * Function Name: udp_client_recv_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle incoming UDP server messages.
 *
 * Parameters:
 *  cy_socket_t socket_handle: Connection handle for the UDP client socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t udp_client_recv_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result;
    /* Variable to store the number of bytes received. */
    uint32_t bytes_received = 0;
    /* Buffer to store received data. */
    char rx_buffer[1] = {0};

    /* Receive incoming message from UDP server. */
    result = cy_socket_recvfrom(client_handle, rx_buffer, MAX_UDP_RECV_BUFFER_SIZE,
                                    CY_SOCKET_FLAGS_RECVFROM_NONE, NULL, 0, &bytes_received);

    return result;
}

/* [] END OF FILE */



