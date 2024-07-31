#include <csp/csp_debug.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>

#include <stdio.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <csp/csp.h>
#include <csp/drivers/usart.h>
#include <csp/drivers/can_socketcan.h>
#include <csp/interfaces/csp_if_zmqhub.h>

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */
#define CANXL_PRIO_MASK CAN_SFF_MASK /* 11 bit priority mask */

/* the 8-bit VCID is optionally placed in the canxl_frame.prio element */
#define CANXL_VCID_OFFSET 16 /* bit offset of VCID in prio element */
#define CANXL_VCID_VAL_MASK 0xFFUL /* VCID is an 8-bit value */
#define CANXL_VCID_MASK (CANXL_VCID_VAL_MASK << CANXL_VCID_OFFSET)

/* CAN CC/FD/XL frame union */
typedef union {
	struct can_frame cc;
	struct canfd_frame fd;
	struct canxl_frame xl;
} cu_t;

/* These three functions must be provided in arch specific way */
int router_start(void);
int server_start(void);

int parse_canframe(char *cs, cu_t *cu);

/* Server port, the port the server listens on for incoming connections from the client. */
#define SERVER_PORT		10

/* Commandline options */
static uint8_t server_address = 0;

/* Test mode, check that server & client can exchange packets */
static bool test_mode = false;
static unsigned int server_received = 0;
static unsigned int run_duration_in_sec = 3;

static int forwarding = 0;

static uint32_t can_bps = 1000000;

const char *ip = "127.0.0.1";
const int port = 10015;
int sockfd;
struct sockaddr_in ccsds_addr;
socklen_t addr_size;
char ccsds_buffer[1024];

enum DeviceType {
	DEVICE_UNKNOWN,
	DEVICE_CAN,
	DEVICE_KISS,
	DEVICE_ZMQ,
};

#define __maybe_unused __attribute__((__unused__))


#define CANID_DELIM '#'
#define CC_DLC_DELIM '_'
#define XL_HDR_DELIM ':'
#define DATA_SEPERATOR '.'

const char hex_asc_upper[] = "0123456789ABCDEF";

#define hex_asc_upper_lo(x) hex_asc_upper[((x)&0x0F)]
#define hex_asc_upper_hi(x) hex_asc_upper[((x)&0xF0) >> 4]


unsigned char asc2nibble(char c)
{
	if ((c >= '0') && (c <= '9'))
		return c - '0';

	if ((c >= 'A') && (c <= 'F'))
		return c - 'A' + 10;

	if ((c >= 'a') && (c <= 'f'))
		return c - 'a' + 10;

	return 16; /* error */
}


int parse_canframe(char *cs, cu_t *cu)
{
	/* documentation see lib.h */

	int i, idx, dlen, len;
	int maxdlen = CAN_MAX_DLEN;
	int mtu = CAN_MTU;
	__u8 *data = cu->fd.data; /* fill CAN CC/FD data by default */
	canid_t tmp;

	len = strlen(cs);
	//printf("'%s' len %d\n", cs, len);

	memset(cu, 0, sizeof(*cu)); /* init CAN CC/FD/XL frame, e.g. LEN = 0 */

	if (len < 4)
		return 0;

	if (cs[3] == CANID_DELIM) { /* 3 digits SFF */

		idx = 4;
		for (i = 0; i < 3; i++) {
			if ((tmp = asc2nibble(cs[i])) > 0x0F)
				return 0;
			cu->cc.can_id |= tmp << (2 - i) * 4;
		}

	} else if (cs[5] == CANID_DELIM) { /* 5 digits CAN XL VCID/PRIO*/

		idx = 6;
		for (i = 0; i < 5; i++) {
			if ((tmp = asc2nibble(cs[i])) > 0x0F)
				return 0;
			cu->xl.prio |= tmp << (4 - i) * 4;
		}

		/* the VCID starts at bit position 16 */
		tmp = (cu->xl.prio << 4) & CANXL_VCID_MASK;
		cu->xl.prio &= CANXL_PRIO_MASK;
		cu->xl.prio |= tmp;

	} else if (cs[8] == CANID_DELIM) { /* 8 digits EFF */

		idx = 9;
		for (i = 0; i < 8; i++) {
			if ((tmp = asc2nibble(cs[i])) > 0x0F)
				return 0;
			cu->cc.can_id |= tmp << (7 - i) * 4;
		}
		if (!(cu->cc.can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
			cu->cc.can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

	} else
		return 0;

	if ((cs[idx] == 'R') || (cs[idx] == 'r')) { /* RTR frame */
		cu->cc.can_id |= CAN_RTR_FLAG;

		/* check for optional DLC value for CAN 2.0B frames */
		if (cs[++idx] && (tmp = asc2nibble(cs[idx++])) <= CAN_MAX_DLEN) {
			cu->cc.len = tmp;

			/* check for optional raw DLC value for CAN 2.0B frames */
			if ((tmp == CAN_MAX_DLEN) && (cs[idx++] == CC_DLC_DELIM)) {
				tmp = asc2nibble(cs[idx]);
				if ((tmp > CAN_MAX_DLEN) && (tmp <= CAN_MAX_RAW_DLC))
					cu->cc.len8_dlc = tmp;
			}
		}
		return mtu;
	}

	if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */
		maxdlen = CANFD_MAX_DLEN;
		mtu = CANFD_MTU;

		/* CAN FD frame <canid>##<flags><data>* */
		if ((tmp = asc2nibble(cs[idx + 1])) > 0x0F)
			return 0;

		cu->fd.flags = tmp;
		cu->fd.flags |= CANFD_FDF; /* dual-use */
		idx += 2;

	} else if (cs[idx + 14] == CANID_DELIM) { /* CAN XL frame '#80:00:11223344#' */
		maxdlen = CANXL_MAX_DLEN;
		mtu = CANXL_MTU;
		data = cu->xl.data; /* fill CAN XL data */

		if ((cs[idx + 2] != XL_HDR_DELIM) || (cs[idx + 5] != XL_HDR_DELIM))
			return 0;

		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		cu->xl.flags = tmp << 4;
		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		cu->xl.flags |= tmp;

		/* force CAN XL flag if it was missing in the ASCII string */
		cu->xl.flags |= CANXL_XLF;

		idx++; /* skip XL_HDR_DELIM */

		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		cu->xl.sdt = tmp << 4;
		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		cu->xl.sdt |= tmp;

		idx++; /* skip XL_HDR_DELIM */

		for (i = 0; i < 8; i++) {
			if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
				return 0;
			cu->xl.af |= tmp << (7 - i) * 4;
		}

		idx++; /* skip CANID_DELIM */
	}

	for (i = 0, dlen = 0; i < maxdlen; i++) {
		if (cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
			idx++;

		if (idx >= len) /* end of string => end of data */
			break;

		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		data[i] = tmp << 4;
		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		data[i] |= tmp;
		dlen++;
	}

	if (mtu == CANXL_MTU)
		cu->xl.len = dlen;
	else
		cu->fd.len = dlen;

	/* check for extra DLC when having a Classic CAN with 8 bytes payload */
	if ((maxdlen == CAN_MAX_DLEN) && (dlen == CAN_MAX_DLEN) && (cs[idx++] == CC_DLC_DELIM)) {
		unsigned char dlc = asc2nibble(cs[idx]);

		if ((dlc > CAN_MAX_DLEN) && (dlc <= CAN_MAX_RAW_DLC))
			cu->cc.len8_dlc = dlc;
	}

	return mtu;
}

/* Server task - handles requests from clients */
void server(void) {

	int s = 0; /* can raw socket */
	int required_mtu;
	// int mtu;
	// int enable_canfx = 1;
	struct sockaddr_can can_addr;
	// struct can_raw_vcid_options vcid_opts = {
	// 	.flags = CAN_RAW_XL_VCID_TX_PASS,
	// };
	static cu_t cu;
	struct ifreq ifr;

	csp_print("Server task started\n");

	/* Create socket with no specific socket options, e.g. accepts CRC32, HMAC, etc. if enabled during compilation */
	csp_socket_t sock = {0};

	/* Bind socket to all ports, e.g. all incoming connections will be handled here */
	csp_bind(&sock, CSP_ANY);

	/* Create a backlog of 10 connections, i.e. up to 10 new connections can be queued */
	csp_listen(&sock, 10);

	switch(forwarding)
	{
		case 1:
			if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
				perror("CAN Socket");
				exit(EXIT_FAILURE);
			}

			strcpy(ifr.ifr_name, "can0" );
			ioctl(s, SIOCGIFINDEX, &ifr);

			memset(&can_addr, 0, sizeof(can_addr));
			can_addr.can_family = AF_CAN;
			can_addr.can_ifindex = ifr.ifr_ifindex;

			if (bind(s, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
				perror("CAN Bind");
				exit(EXIT_FAILURE);
			}
			break;
		case 2: //TC CCSDS forwarding
			//#1 : socket
			sockfd = socket(AF_INET, SOCK_DGRAM, 0);
			if(sockfd < 0)
			{
				perror("UDP socket");
				exit(EXIT_FAILURE);
			}
			memset(&ccsds_addr, '\0', sizeof(ccsds_addr));
			ccsds_addr.sin_family = AF_INET;
			ccsds_addr.sin_port = htons(port);
			ccsds_addr.sin_addr.s_addr = inet_addr(ip);

			break;
		default:
			break;
	}

	/* Wait for connections and then process packets on the connection */
	while (1) {

		/* Wait for a new connection, 10000 mS timeout */
		csp_conn_t *conn;
		if ((conn = csp_accept(&sock, 10000)) == NULL) {
			/* timeout */
			continue;
		}

		/* Read packets on connection, timout is 100 mS */
		csp_packet_t *packet;
		while ((packet = csp_read(conn, 50)) != NULL) {
			switch (csp_conn_dport(conn)) {
			case SERVER_PORT:
				/* Process packet here */
				csp_print("Packet received on SERVER_PORT: %s\n", (char *) packet->data);

				/* parse CAN frame */
				required_mtu = parse_canframe((char *) packet->data, &cu);
				if (!required_mtu) {
					fprintf(stderr, "\nWrong CAN-frame format!\n\n");
					break;
				}

				switch(forwarding)
				{
					case 1:
						/* send frame */
						if (write(s, &cu, required_mtu) != required_mtu) {
							perror("write");
							// return 1;
						}
						break;
					case 2:
						//TODO: Parsing CCSDS format
						memcpy(ccsds_buffer, cu.cc.data, cu.cc.len);
						sendto(sockfd, ccsds_buffer, cu.cc.len, 0, (struct sockaddr*)&ccsds_addr, sizeof(ccsds_addr));
  						csp_print("[+]TM Data send: %s len: %d\n", &cu.cc.data, cu.cc.len);
						break;
					default:
						break;
				}

				csp_buffer_free(packet);
				++server_received;

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

static struct option long_options[] = {
    {"kiss-device", required_argument, 0, 'k'},
#if (CSP_HAVE_LIBSOCKETCAN)
	#define OPTION_c "c:"
    {"can-device", required_argument, 0, 'c'},
	#define OPTION_b "b:"
	{"can-bps", required_argument, 0, 'b'},
#else
	#define OPTION_c
	#define OPTION_b
#endif
#if (CSP_HAVE_LIBZMQ)
	#define OPTION_z "z:"
    {"zmq-device", required_argument, 0, 'z'},
	#define OPTION_f "f:"
	{"zmq2can", required_argument, 0, 'f'},
#else
	#define OPTION_z
	#define OPTION_f
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
    csp_print("Usage: csp_server [options]\n");
	if (CSP_HAVE_LIBSOCKETCAN) {
		csp_print(" -c <can-device>  set CAN device receiver\n");
		csp_print(" -b <can-bitrate>  set CAN device bitrate\n");
	}
	if (1) {
		csp_print(" -k <kiss-device> set KISS device\n");
	}
	if (CSP_HAVE_LIBZMQ) {
		csp_print(" -z <zmq-device>  set ZeroMQ device\n");
		csp_print(" -f <0|1|2>        ZeroMQ forwarding 0:disable 1:CAN raw 2: YAMCS CCSDS\n");
	}
	if (CSP_USE_RTABLE) {
		csp_print(" -R <rtable>      set routing table\n");
	}
	if (1) {
		csp_print(" -a <address>     set interface address\n"
				  " -t               enable test mode\n"
				  " -T <dration>     enable test mode with running time in seconds\n"
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
		default_iface->addr = server_address;
        default_iface->is_default = 1;
    }

    if (CSP_HAVE_LIBSOCKETCAN && (device_type == DEVICE_CAN)) {
        int error = csp_can_socketcan_open_and_add_interface(device_name, CSP_IF_CAN_DEFAULT_NAME, server_address, can_bps, true, &default_iface);
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

/* main - initialization of CSP and start of server task */
int main(int argc, char * argv[]) {

	const char * device_name = NULL;
	enum DeviceType device_type = DEVICE_UNKNOWN;
	const char * rtable __maybe_unused = NULL;
	csp_iface_t * default_iface;
    int opt;

	while ((opt = getopt_long(argc, argv, OPTION_c OPTION_z OPTION_f OPTION_R "k:a:tT:h:" , long_options, NULL)) != -1) {
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
			case 'f':
				forwarding = atoi(optarg);
				csp_print("Forwarding: %d\n",forwarding);
                break;
#if (CSP_USE_RTABLE)
            case 'R':
                rtable = optarg;
                break;
#endif
            case 'a':
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

	// If more than one of the interfaces are set, print a message and exit
	if (device_type == DEVICE_UNKNOWN) {
		csp_print("Only one of the interfaces can be set.\n");
        print_help();
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

    /* Start server thread */
    server_start();

    /* Wait for execution to end (ctrl+c) */
    while(1) {
        sleep(run_duration_in_sec);

        if (test_mode) {
            /* Test mode, check that server & client can exchange packets */
            if (server_received < 5) {
                csp_print("Server received %u packets\n", server_received);
                exit(EXIT_FAILURE);
            }
            csp_print("Server received %u packets\n", server_received);
            exit(EXIT_SUCCESS);
        }
    }

    return 0;
}
