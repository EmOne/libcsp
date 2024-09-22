#include <csp/csp_debug.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <csp/csp.h>
#include <csp/drivers/usart.h>
#include <csp/drivers/can_socketcan.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/interfaces/csp_if_zmqhub.h>


/* These three functions must be provided in arch specific way */
int router_start(void);
int server_start(void);
int client_start(void);

/* Server port, the port the server listens on for incoming connections from the client. */
#define MY_SERVER_PORT		10
#define SERVER_TC_PORT		13
#define SERVER_ACK_PORT		14
#define SERVER_STATUS_PORT	15

/* Commandline options */
static uint8_t server_address = 10;

/* test mode, used for verifying that host & client can exchange packets over the loopback interface */
static bool test_mode = false;
static unsigned int server_received = 0;
static unsigned int run_duration_in_sec = 3;
static unsigned int successful_ping = 0;
static uint32_t can_bps = 125000;
enum DeviceType {
	DEVICE_UNKNOWN,
	DEVICE_CAN,
	DEVICE_KISS,
	DEVICE_ZMQ,
};

#define __maybe_unused __attribute__((__unused__))

/* Server task - handles requests from clients */
void server(void) {

	csp_packet_t *packet;
	int dport;
	// int sport;

	csp_print("Server task started\n");

	/* Create socket with no specific socket options, e.g. accepts CRC32, HMAC, etc. if enabled during compilation */
	csp_socket_t sock = {0};

	/* Bind socket to all ports, e.g. all incoming connections will be handled here */
	csp_bind(&sock, CSP_ANY);

	/* Create a backlog of 10 connections, i.e. up to 10 new connections can be queued */
	csp_listen(&sock, 10);

	/* Wait for connections and then process packets on the connection */
	while (1) {

		/* Wait for a new connection, 10000 mS timeout */
		csp_conn_t *conn;
		if ((conn = csp_accept(&sock, 10000)) == NULL) {
			/* timeout */
			csp_print("CSP Accept timeout\n");
			continue;
		}

		/* Read packets on connection, timout is 50 mS */
		while ((packet = csp_read(conn, 50)) != NULL) {
			dport = csp_conn_dport(conn);
			// sport = csp_conn_sport(conn);
			switch (dport) {
			case SERVER_STATUS_PORT:
				csp_print("Status received on PORT %d: %s\n", dport, (char *) packet->data);
				csp_buffer_free(packet);
				break;
			case SERVER_TC_PORT:
				/* Process packet here */
				csp_print("Packet received on PORT %d: %s\n", dport, (char *) packet->data);
				csp_buffer_free(packet);
				++server_received;
				break;
			case SERVER_ACK_PORT:	
				csp_print("ACK received on PORT %d: %s\n", dport, (char *) packet->data);
				csp_buffer_free(packet);
				break;
			default:
				/* Call the default CSP service handler, handle pings, buffer use, etc. */
				csp_service_handler(packet);
				break;
			}
		}

		/* Close current connection */
		csp_close(conn);

	}

	return;

}
/* End of server task */

/* Client task sending requests to server task */
void client(void) {

	csp_print("Client task started\n");

	uint16_t count = 0;
    uint8_t csend[8] = {0};
    server_address = 11;

	csp_packet_t * packet;

	while (1) {

		usleep(test_mode ? 200000 : 1000000);

		/* Send ping to server, timeout 1000 mS, ping size 10 bytes */
		int result = csp_ping(server_address, 1000, 10, CSP_O_NONE);
		csp_print("Ping address: %u, result %d [mS]\n", server_address, result);
		// Increment successful_ping if ping was successful
		if (result >= 0) {
			++successful_ping;
		} 
		else 
		{
			/* Send reboot request to server, the server has no actual implementation of csp_sys_reboot() and fails to reboot */
			csp_reboot(server_address);
			csp_print("reboot system request sent to address: %u\n", server_address);
			usleep(1000000);
			continue;
		}
		
		
		/* Send data packet (string) to server */

		/* 1. Get packet buffer for message/data */
		packet = csp_buffer_get(0);
		if (packet == NULL) {
			/* Could not get buffer element */
			csp_print("Failed to get CSP buffer\n");
			successful_ping = 0;
			continue;
		}

		/* 2. Copy data to packet */
        #if 0
        memcpy(packet->data, "Hello world ", 12);
        memcpy(packet->data + 12, &count, 1);
        memset(packet->data + 13, 0, 1);
        #else
        csp_print("Send TC (%d): 0x%03X [%d] ", SERVER_TC_PORT, 0x200 & 0xfff, 8);
		unsigned int alen = sprintf((char*) packet->data, "%03X#",
				0x200 & 0xfff);

		csend[0] = ((uint8_t) (count >> 8)) & 0xFF;
		csend[1] = ((uint8_t) (count >> 0)) & 0xFF;
		for (int i = 0; i < 8; i++)
		{
			csp_print("%02X ", csend[i]);
			alen += sprintf((char*) packet->data + alen, "%02X", csend[i]);
		}
		
		memset(packet->data + alen, 0, 1); /* include the 0 termination */
		alen++;
		csp_print("(%d)\n", alen);
        #endif
        count++;

		/* 3. Set packet length */
		packet->length = alen; 

		/* 4. Connect to host on 'server_address', port MY_SERVER_PORT with regular UDP-like protocol and 1000 ms timeout */
		csp_conn_t * conn = csp_connect(CSP_PRIO_NORM, server_address, SERVER_TC_PORT, 1000, CSP_O_NONE);
		if (conn == NULL) {
			/* Connect failed */
			csp_print("Connection failed\n");
			csp_buffer_free(packet);
			successful_ping = 0;
			continue;
		}

		/* 5. Send packet */
		csp_send(conn, packet);

		/* 6. Close connection */
		csp_buffer_free(packet);
		csp_close(conn);
	}

	return;
}
/* End of client task */


csp_iface_t * add_interface(enum DeviceType device_type, const char * device_name)
{
    csp_iface_t * default_iface = NULL;

    if (device_type == DEVICE_KISS) {
        csp_usart_conf_t conf = {
			.device = device_name,
            .baudrate = 115200, /* supported on all platforms */
            .databits = 8,
            .stopbits = 1,
            .paritysetting = 0,
		};
        int error = csp_usart_open_and_add_kiss_interface(&conf, CSP_IF_KISS_DEFAULT_NAME, server_address, &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add KISS interface [%s], error: %d\n", device_name, error);
            exit(1);
        }
		default_iface->addr = server_address;
        default_iface->is_default = 1;
    }

    if (CSP_HAVE_LIBSOCKETCAN && (device_type == DEVICE_CAN)) {
        int error = csp_can_socketcan_open_and_add_interface(device_name, CSP_IF_CAN_DEFAULT_NAME, server_address, can_bps, true, 11, 0, &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add CAN interface [%s], error: %d\n", device_name, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }

    if (CSP_HAVE_LIBZMQ && (device_type == DEVICE_ZMQ)) {
        int error = csp_zmqhub_init(server_address, device_name, 0, &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add ZMQ interface [%s], error: %d\n", device_name, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }

	return default_iface;
}

static void print_usage(void)
{
	csp_print("Usage:\n"
			  " -c <can interface>	set CAN interface\n"
			  " -b <bit rate>	 	set CAN bit rate\n"
			  " -R <rtable>      	set routing table\n"
			  " -a <address>	 	set Host CSP Address\n"
			  " -C <address>	 	set Connect to server address\n"
			  " -t               	enable test mode\n"
			  " -T <duration>    	enable test mode with running time in seconds\n"
			  " -h               	print help\n");
}

/* main - initialization of CSP and start of server/client tasks */
int main(int argc, char * argv[]) {

	const char * device_name = NULL;
	enum DeviceType device_type = DEVICE_UNKNOWN;
    uint8_t address = 0;
    int opt;
	const char * rtable __maybe_unused = NULL;
	csp_iface_t * default_iface;

    while ((opt = getopt(argc, argv, "a:b:c:C:R:tT:hv")) != -1) {
        switch (opt) {
			case 'c':
				device_name = optarg;
				device_type = DEVICE_CAN;
                break;
			case 'b':
				can_bps = atoi(optarg);
				break;
			case 'R':
                rtable = optarg;
                break;
            case 'a':
                address = atoi(optarg);
                break;
            case 'C':
                server_address = atoi(optarg);
                break;
            case 't':
                test_mode = true;
                break;
            case 'T':
                test_mode = true;
                run_duration_in_sec = atoi(optarg);
                break;
			case 'v':
				csp_dbg_packet_print = 1;
				break;
            case 'h':
				print_usage();
				exit(0);
                break;
            default:
				print_usage();
                exit(1);
                break;
        }
    }

	// If more than one of the interfaces are set, print a message and exit
	if (device_type == DEVICE_UNKNOWN) {
		csp_print("Only one of the interfaces can be set.\n");
        print_usage();
        exit(EXIT_FAILURE);
    } else {
		csp_print("%s interfaces set.\n", device_name);
	}

    csp_print("Initialising CSP\n");

    /* Init CSP */
    csp_init();

    /* Start router */
    router_start();

    /* Add interface(s) */
    default_iface = device_type == DEVICE_UNKNOWN ? NULL :
		add_interface(device_type, device_name);

    if (!default_iface) {
        /* no interfaces configured - run server and client in process, using loopback interface */
        server_address = address;
    }

	// /* Setup routing table */
	if (rtable) {
		int error = csp_rtable_load(rtable);
		if (error < 1) {
			csp_print("csp_rtable_load(%s) failed, error: %d\n", rtable, error);
			exit(1);
		}
	} else if (default_iface) {
		csp_rtable_set(0, 0, default_iface, CSP_NO_VIA_ADDRESS);
	}


    csp_print("Connection table\r\n");
    csp_conn_print_table();

    csp_print("Interfaces\r\n");
    csp_iflist_print();

    /* Start server thread */
    server_start();

    /* Start client thread */
    client_start();

    /* Wait for execution to end (ctrl+c) */
    while(1) {
        sleep(run_duration_in_sec);

        if (test_mode) {
            /* Test mode is intended for checking that host & client can exchange packets over loopback */
            if (server_received < 5) {
                csp_print("Server received %u packets\n", server_received);
                exit(1);
            }
            csp_print("Server received %u packets\n", server_received);
            exit(0);
        }
    }

    return 0;
}
