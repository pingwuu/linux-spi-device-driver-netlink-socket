#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netlink/msg.h>
#include <netlink/attr.h>

#include "genl_ex.h"

#ifndef GRADING
#define HCSR04_TRIGGER_PIN 0
#define HCSR04_ECHO_PIN 1
#define MAX7219_CS_PIN 12
#endif

#define MAX7219_MOSI_PIN 11
#define MAX7219_SCK_PIN 13

static unsigned int mcgroups;		/* Mask of groups */

static void add_group(char* group)
{
	unsigned int grp = strtoul(group, NULL, 10);

	if (grp > GENL_TEST_MCGRP_MAX-1) {
		fprintf(stderr, "Invalid group number %u. Values allowed 0:%u\n",
			grp, GENL_TEST_MCGRP_MAX-1);
		exit(EXIT_FAILURE);
	}

	mcgroups |= 1 << (grp);
}

static int send_msg_to_kernel(struct nl_sock *sock, struct netlink_userdata* user_data)
{
	struct nl_msg* msg;
	int family_id, err = 0;

	family_id = genl_ctrl_resolve(sock, GENL_TEST_FAMILY_NAME);
	if(family_id < 0){
		fprintf(stderr, "Unable to resolve family name!\n");
		exit(EXIT_FAILURE);
	}

	msg = nlmsg_alloc();
	if (!msg) {
		fprintf(stderr, "failed to allocate netlink message\n");
		exit(EXIT_FAILURE);
	}

	if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_id, 0, 
		NLM_F_REQUEST, GENL_TEST_C_MSG, 0)) {
		fprintf(stderr, "failed to put nl hdr!\n");
		err = -ENOMEM;
		goto out;
	}

	err = nla_put(msg, GENL_TEST_ATTR_MSG, sizeof(struct netlink_userdata), user_data);
	if (err) {
		fprintf(stderr, "failed to put nl string!\n");
		goto out;
	}

	err = nl_send_auto(sock, msg);
	if (err < 0) {
		fprintf(stderr, "failed to send nl message!\n");
	}

out:
	nlmsg_free(msg);
	return err;
}

static int skip_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

static int process_rx_msg(struct nl_msg *msg, void* arg)
{
	struct nl_sock* nlsock = *((struct nl_sock**)arg);
	struct nlattr *attr[GENL_TEST_ATTR_MAX+1];
	struct netlink_userdata* rx_data = (struct netlink_userdata*) malloc(sizeof(struct netlink_userdata));
	int pattern_dir = 0;

	printf("process_rx_msg\n");

	genlmsg_parse(nlmsg_hdr(msg), 0, attr, 
			GENL_TEST_ATTR_MAX, genl_test_policy);

	if (!attr[GENL_TEST_ATTR_MSG]) {
		fprintf(stdout, "Kernel sent empty message!!\n");
		return NL_STOP;
	}
		
	memcpy(rx_data, nla_data(attr[GENL_TEST_ATTR_MSG]), sizeof(struct netlink_userdata));	

	if(rx_data && rx_data->msg == MSG_READ_DISTANCE) {
		printf("MSG_READ_DISTANCE distance_cm=%d\n", rx_data->distance_cm);
		if(rx_data->distance_cm == 0) {
			pattern_dir = -1;
		} else if(rx_data->distance_cm > 10 && rx_data->distance_cm < 100) {
			pattern_dir = 1;
		} else {
			pattern_dir = 0;
		}
		//Send pattern direction
		printf("process_rx_msg: send_msg_to_kernel, update pattern\n");
		memset(rx_data, 0, sizeof(struct netlink_userdata));
		rx_data->msg = MSG_CONFIG_PATTERN;
		rx_data->pattern_dir = pattern_dir;
		send_msg_to_kernel(nlsock, rx_data);
	}

	free(rx_data);
	return NL_STOP;
}

static void prep_nl_sock(struct nl_sock** nlsock)
{
	int family_id, grp_id;
	unsigned int bit = 0;
	
	*nlsock = nl_socket_alloc();
	if(!*nlsock) {
		fprintf(stderr, "Unable to alloc nl socket!\n");
		exit(EXIT_FAILURE);
	}

	/* disable seq checks on multicast sockets */
	nl_socket_disable_seq_check(*nlsock);
	nl_socket_disable_auto_ack(*nlsock);

	/* connect to genl */
	if (genl_connect(*nlsock)) {
		fprintf(stderr, "Unable to connect to genl!\n");
		goto exit_err;
	}

	/* resolve the generic nl family id*/
	family_id = genl_ctrl_resolve(*nlsock, GENL_TEST_FAMILY_NAME);
	if(family_id < 0){
		fprintf(stderr, "Unable to resolve family name!\n");
		goto exit_err;
	}

	if (!mcgroups)
		return;

	while (bit < sizeof(unsigned int)) {
		if (!(mcgroups & (1 << bit)))
			goto next;

		grp_id = genl_ctrl_resolve_grp(*nlsock, GENL_TEST_FAMILY_NAME,
				genl_test_mcgrp_names[bit]);

		if (grp_id < 0)	{
			fprintf(stderr, "Unable to resolve group name for %u!\n",
				(1 << bit));
            goto exit_err;
		}
		if (nl_socket_add_membership(*nlsock, grp_id)) {
			fprintf(stderr, "Unable to join group %u!\n", 
				(1 << bit));
            goto exit_err;
		}
next:
		bit++;
	}

    return;

exit_err:
    nl_socket_free(*nlsock); // this call closes the socket as well
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv)
{
	struct nl_sock* nlsock = NULL;
	struct nl_cb *cb = NULL;
	int ret, i = 0;
	struct netlink_userdata* user_data = (struct netlink_userdata*) malloc(sizeof(struct netlink_userdata));

	add_group("0"); //0 for kernel
	prep_nl_sock(&nlsock);
	/* prep the cb */
	cb = nl_cb_alloc(NL_CB_DEFAULT);
	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, skip_seq_check, NULL);
	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, process_rx_msg, &nlsock);


	user_data->msg = MSG_CONFIG_PINS;
	user_data->trigger_pin = HCSR04_TRIGGER_PIN;
	user_data->echo_pin = HCSR04_ECHO_PIN;
	user_data->cs_pin = MAX7219_CS_PIN;
	user_data->mosi_pin = MAX7219_MOSI_PIN;
	user_data->sck_pin = MAX7219_SCK_PIN;
	user_data->m_samples = 5;
	user_data->delta = 5000;
	memcpy(user_data->pattern_array, pattern, sizeof(struct led_rep) * 10);
	ret = send_msg_to_kernel(nlsock, user_data); //Setup pins and send pattern, so we only have to pass indices later
	printf("send_msg_to_kernel: ret=%d\n", ret);
	
	for(i = 0; i < 10; i++) {
		memset(user_data, 0, sizeof(struct netlink_userdata));
		user_data->msg = MSG_START_MEASUREMENT;
		user_data->enable = 1;
		ret = send_msg_to_kernel(nlsock, user_data);
		printf("start measurement\n");

		ret = nl_recvmsgs(nlsock, cb);
		printf("nl_recvmsgs %d\n", ret);
		
		sleep(user_data->delta/1000); //sleep until its time to collect a new value
	}
	nl_cb_put(cb);
    nl_socket_free(nlsock);
	free(user_data);
	return 0;
}