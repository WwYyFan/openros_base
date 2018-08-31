#include "base/modbus_rtu.h"
#include <ros/ros.h>
/*
 * bufcmp - Compare the recv buf with what we 
 * already have
 *
 * Input: s1 - addr of the first buf
 *        s2 - addr of the second buf
 * Return Value: 0 - same
 *               1 - different
 * We don't have to know which buf is smaller, we
 * only care whether they are same.
 */
int bufcmp(unsigned char *s1, unsigned char *s2) {
	int len1 = strlen((char *)s1);
	int len2 = strlen((char *)s2);
	if (len1 != len2)
		return 1; /* match fail */
	for (int i = 0; i < len1; ++i)
		if (s1[i] != s2[i])
			return 1;
	return 0;
}

/*
 * open_modbus - open the serial port
 *
 * Return Value: >0 - the fd. success
 *            	 <=0 - fail
 */
int open_modbus() {
	int fd;

	fd = open(MODBUS_DEV, O_RDWR);
	if (fd < 0) {
		perror("open tty error");
        ROS_INFO("open tty error：1.check devices name :/dev/ttyUSB0  2.add permissions: sudo -i && usermod -a -G YourUserName");
        
		return -1;
	}

	struct termios options;
	tcgetattr(fd, &options);
	memset(&options, 0, sizeof(options));
	// options.c_cflag |= CLOCAL | CREAD;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8; /* 8 data bit */

	options.c_cflag &= ~PARENB; /* no parity */
	options.c_cflag &= ~CSTOPB; /* 1 stop bit */

	/* set the baudrate */
	if (cfsetispeed(&options, BAUDRATE) < 0) {
		perror("baudrate seti error");
		return -1;
	}
	if (cfsetospeed(&options, BAUDRATE) < 0) {
		perror("baudrate seto error");
		return -1;
	}
	/* set the wait time */
	options.c_cc[VTIME] = 10;
	options.c_cc[VMIN] = 4;

	/* bind the options to fd */
	if (tcsetattr(fd, TCSANOW, &options) < 0) {
		perror("attr set error");
		return -1;
	}

	return fd;
}

/*
 * gripper_activate - activate the gripper
 *
 * Return Value: 0 - success
 *               -1 - fail
 */
int gripper_activate() {
	int fd;
	int read_cnt;
	unsigned char recv_buf[BUF_SIZE];

	if ((fd = open_modbus()) < 0)
		return -1;

	/* activate */
	if (write(fd, activate, sizeof(activate)) < 0) {
		perror("write error");
		return -1;
	}

#ifdef DEBUG	
	if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
		perror("read error");
		return -1;
	}

	fprintf(stdout, "Activate Receive: ");
	for (int i = 0; i < read_cnt; ++i)
		fprintf(stdout, "0x%x ", recv_buf[i]);
	fprintf(stdout, "\n");
#endif

	while (1) {
		if (write(fd, read_gripper_status, sizeof(read_gripper_status)) < 0) {
			perror("write error");
			return -1;
		}
		/* recv gripper status */
		if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
			perror("read error");
			return -1;
		}
		if (!bufcmp(activate_success, recv_buf))
			break; /* complete */
		else
			continue; /* not complete */
	}

	close(fd);
	return 0;
}

/*
 * gripper_close - close the gripper
 *
 * Return Value: 0 - success
 *               -1 - fail
 */
int gripper_close() {
	int fd;
	int read_cnt;
	unsigned char recv_buf[BUF_SIZE];

	if ((fd = open_modbus()) < 0)
		return -1;

	/* grip */
	if (write(fd, close_with_full_speed_full_force, 
			sizeof(close_with_full_speed_full_force)) < 0) {
		perror("write error");
		return -1;
	}
	while (1) {
		if (write(fd, read_gripper_status, 
				sizeof(read_gripper_status)) < 0) {
			perror("write error");
			return -1;
		}
		/* recv gripper status */
		if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
			perror("read error");
			return -1;
		}
#ifdef DEBUG
	// fprintf(stdout, "Close Receive: ");
	// for (int i = 0; i < read_cnt; ++i)
	// 	fprintf(stdout, "0x%x ", recv_buf[i]);
	// fprintf(stdout, "\n");
#endif
		if (!bufcmp(grip_is_completed, recv_buf))
			break; /* complete */
		else
			continue; /* not complete */
	}

	close(fd);
	return 0;
}

/*
 * gripper_open - open the gripper
 *
 * Return Value: 0 - success
 *               -1 - fail
 */
int gripper_open() {
	int fd;
	int read_cnt;
	unsigned char recv_buf[BUF_SIZE];

	if ((fd = open_modbus()) < 0)
		return -1;

	/* open */
	if (write(fd, open_with_full_speed_full_force, 
			sizeof(open_with_full_speed_full_force)) < 0) {
		perror("write error");
		return -1;
	}
	while (1) {
		if (write(fd, read_until_open_completed, 
				sizeof(read_until_open_completed)) < 0) {
			perror("write error");
			return -1;
		}
		/* recv gripper status */
		if ((read_cnt = read(fd, recv_buf, BUF_SIZE)) < 0) {
			perror("read error");
			return -1;
		}
#ifdef DEBUG
	// fprintf(stdout, "Open Receive: ");
	// for (int i = 0; i < read_cnt; ++i)
	// 	fprintf(stdout, "0x%x ", recv_buf[i]);
	// fprintf(stdout, "\n");
#endif
		if (!bufcmp(open_is_completed, recv_buf))
			break; /* complete */
		else
			continue; /* not complete */
	}

	close(fd);
	return 0;
}
