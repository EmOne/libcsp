#include <csp/csp_debug.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <csp/csp.h>
#include <csp/drivers/usart.h>
#include <csp/drivers/can_socketcan.h>
#include <csp/interfaces/csp_if_zmqhub.h>

#include <stdio.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/* This function must be provided in arch specific way */
int router_start(void);

/* Server port, the port the server listens on for incoming connections from the client. */
#define SERVER_PORT		10

/* Commandline options */
static uint8_t server_address = 0;
static uint8_t client_address = 0;

/* Test mode, check that server & client can exchange packets */
static bool test_mode = false;
static unsigned int successful_ping = 0;
static unsigned int run_duration_in_sec = 3;

static uint32_t can_bps = 1000000;

enum DeviceType {
	DEVICE_UNKNOWN,
	DEVICE_CAN,
	DEVICE_KISS,
	DEVICE_ZMQ,
};

#define __maybe_unused __attribute__((__unused__))

static struct option long_options[] = {
	{"kiss-device", required_argument, 0, 'k'},
#if (CSP_HAVE_LIBSOCKETCAN)
	#define OPTION_c "cb:"
    {"can-device", required_argument, 0, 'c'},
	{"can-bps", required_argument, 0, 'b'},
#else
	#define OPTION_c
#endif
#if (CSP_HAVE_LIBZMQ)
	#define OPTION_z "z:"
    {"zmq-device", required_argument, 0, 'z'},
#else
	#define OPTION_z
#endif
#if (CSP_USE_RTABLE)
	#define OPTION_R "R:"
    {"rtable", required_argument, 0, 'R'},
#else
	#define OPTION_R
#endif
    {"interface-address", required_argument, 0, 'a'},
    {"connect-to", required_argument, 0, 'C'},
    {"test-mode", no_argument, 0, 't'},
    {"test-mode-with-sec", required_argument, 0, 'T'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
};

void print_help() {
	csp_print("Usage: csp_client [options]\n");
	if (CSP_HAVE_LIBSOCKETCAN) {
		csp_print(" -c <can-device>  set CAN device\n");
		csp_print(" -b <can-device-bitrate>  set CAN device bitrate\n");
	}
	if (1) {
		csp_print(" -k <kiss-device> set KISS device\n");
	}
	if (CSP_HAVE_LIBZMQ) {
		csp_print(" -z <zmq-device>  set ZeroMQ device\n");
	}
	if (CSP_USE_RTABLE) {
		csp_print(" -R <rtable>      set routing table\n");
	}
	if (1) {
		csp_print(" -a <address>     set interface address\n"
				  " -C <address>     connect to server at address\n"
				  " -t               enable test mode\n"
				  " -T <duration>    enable test mode with running time in seconds\n"
				  " -h               print help\n");
	}
}

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
        int error = csp_usart_open_and_add_kiss_interface(&conf, CSP_IF_KISS_DEFAULT_NAME,  &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add KISS interface [%s], error: %d\n", device_name, error);
            exit(1);
        }
		default_iface->addr = client_address;
        default_iface->is_default = 1;
    }

	if (CSP_HAVE_LIBSOCKETCAN && (device_type == DEVICE_CAN)) {
		int error = csp_can_socketcan_open_and_add_interface(device_name, CSP_IF_CAN_DEFAULT_NAME, client_address, can_bps, true, &default_iface);
        if (error != CSP_ERR_NONE) {
			csp_print("failed to add CAN interface [%s], error: %d\n", device_name, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }

	if (CSP_HAVE_LIBZMQ && (device_type == DEVICE_ZMQ)) {
        int error = csp_zmqhub_init(client_address, device_name, 0, &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add ZMQ interface [%s], error: %d\n", device_name, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }

	return default_iface;
}

/* main - initialization of CSP and start of client task */
int main(int argc, char * argv[]) {

	const char * device_name = NULL;
	enum DeviceType device_type = DEVICE_UNKNOWN;
	const char * rtable __maybe_unused = NULL;
	csp_iface_t * default_iface;
	struct timespec start_time;
	unsigned int count;
	int ret = EXIT_SUCCESS;
    int opt;

	int s, i; 
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;
	char frame_buff[32];
	size_t alen;

	while ((opt = getopt_long(argc, argv, OPTION_c OPTION_z OPTION_R "k:a:C:tT:h", long_options, NULL)) != -1) {
        switch (opt) {
            case 'c':
				device_name = optarg;
				device_type = DEVICE_CAN;
                break;
            case 'b':
				can_bps = atoi(optarg);
                break;
            case 'k':
				device_name = optarg;
				device_type = DEVICE_KISS;
                break;
            case 'z':
				device_name = optarg;
				device_type = DEVICE_ZMQ;
                break;
#if (CSP_USE_RTABLE)
            case 'R':
                rtable = optarg;
                break;
#endif
            case 'a':
                client_address = atoi(optarg);
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
            case 'h':
				print_help();
				exit(EXIT_SUCCESS);
            case '?':
                // Invalid option or missing argument
				print_help();
                exit(EXIT_FAILURE);
        }
    }

	// Unless one of the interfaces are set, print a message and exit
	if (device_type == DEVICE_UNKNOWN) {
		csp_print("One and only one of the interfaces can be set.\n");
        print_help();
        exit(EXIT_FAILURE);
    }

	csp_print("CAN Sockets Receive \n");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("CAN Socket");
		exit(EXIT_FAILURE);
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("CAN Bind");
		exit(EXIT_FAILURE);
	}

    csp_print("Initialising CSP\n");

    /* Init CSP */
    csp_init();

    /* Start router */
    router_start();

    /* Add interface(s) */
	default_iface = add_interface(device_type, device_name);

	/* Setup routing table */
	if (CSP_USE_RTABLE) {
		if (rtable) {
			int error = csp_rtable_load(rtable);
			if (error < 1) {
				csp_print("csp_rtable_load(%s) failed, error: %d\n", rtable, error);
				exit(1);
			}
		} else if (default_iface) {
			csp_rtable_set(0, 0, default_iface, CSP_NO_VIA_ADDRESS);
		}
	}

    csp_print("Connection table\r\n");
    csp_conn_print_table();

    csp_print("Interfaces\r\n");
    csp_iflist_print();

	if (CSP_USE_RTABLE) {
		csp_print("Route table\r\n");
		csp_rtable_print();
	}

    /* Start client work */
	csp_print("Client started\n");
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	count = 'A';

	while (1) {
		struct timespec current_time;

		usleep(test_mode ? 200000 : 1000000);

		/* Send ping to server, timeout 1000 mS, ping size 100 bytes */
		int result = csp_ping(server_address, 1000, 100, CSP_O_NONE);
		csp_print("Ping address: %u, result %d [mS]\n", server_address, result);
        // Increment successful_ping if ping was successful
        if (result >= 0) {
            ++successful_ping;
        }

		/* Send reboot request to server, the server has no actual implementation of csp_sys_reboot() and fails to reboot */
		csp_reboot(server_address);
		csp_print("reboot system request sent to address: %u\n", server_address);

		/* Send data packet (string) to server */

		/* 1. Connect to host on 'server_address', port SERVER_PORT with regular UDP-like protocol and 1000 ms timeout */
		csp_conn_t * conn = csp_connect(CSP_PRIO_NORM, server_address, SERVER_PORT, 1000, CSP_O_NONE);
		if (conn == NULL) {
			/* Connect failed */
			csp_print("Connection failed\n");
			ret = EXIT_FAILURE;
			break;
		}


		nbytes = read(s, &frame, sizeof(struct can_frame));

		if (nbytes < 0) {
			perror("CAN Read");
			break;
		}
		
		csp_print("0x%03X [%d] ",frame.can_id  & 0xfff, frame.can_dlc);	
		alen = sprintf(frame_buff, "%03X#", frame.can_id & 0xfff);	
				
		for (i = 0; i < frame.can_dlc; i++) 
		{
			csp_print("%02X ",frame.data[i]);
			alen += sprintf(frame_buff + alen, "%02X", frame.data[i]);
		}
		
		csp_print("\n");

		/* 2. Get packet buffer for message/data */
		csp_packet_t * packet = csp_buffer_get(100);
		if (packet == NULL) {
			/* Could not get buffer element */
			csp_print("Failed to get CSP buffer\n");
			ret = EXIT_FAILURE;
			break;
		}

		/* 3. Copy data to packet */
		
		// memcpy(packet->data, "EmOne->word: ", 12);
		// memcpy(packet->data + 12, &count, 1);
		// memset(packet->data + 13, 0, 1);
		memcpy(packet->data, &frame_buff, alen);
		memset(packet->data + alen, 0, 1);
		csp_print("Sending MSG: %s\n", packet->data);
		count++;

		/* 4. Set packet length */
		packet->length = (strlen((char *) packet->data) + 1); /* include the 0 termination */

		/* 5. Send packet */
		csp_send(conn, packet);

		/* 6. Close connection */
		csp_close(conn);

		/* 7. Check for elapsed time if test_mode. */
		if (test_mode) {
			clock_gettime(CLOCK_MONOTONIC, &current_time);

			/* We don't really care about the precision of it. */
			if (current_time.tv_sec - start_time.tv_sec > (long int) run_duration_in_sec) {
				/* Test mode, check that server & client can exchange packets */
				if (successful_ping < 5) {
					csp_print("Client successfully pinged the server %u times\n", successful_ping);
					ret = EXIT_FAILURE;
					break;
				}
				csp_print("Client successfully pinged the server %u times\n", successful_ping);
				break;
			}
		}
	}

    /* Wait for execution to end (ctrl+c) */

    return ret;
}
