/*
 * This is the main file for modbus tcp communication
 * Modified by xfyu on Jan 25, 2018
 * Main Function：
 * a.using modbus rtu to activate gripper
 * b.close the gripper with full force and full speed
 * c.open the gripper with full force and full speed
 *
 * Note: some of the instruction is different from 
 * the Instruction Manual provided by Universal Robot.
 * I don't know why they are different, but I get these
 * from real robot test.
 */
 
/*
 * Libraries
 */
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#define MODBUS_DEV "/dev/ttyUSB0"
#define BAUDRATE B115200

/* already defined otherwhere */
#define BUF_SIZE 512

#define DEBUG

/*
 * Global Variables
 */
unsigned char activate[] = {
	0x09, 0x10,
	0x03, 0xe8,
	0x00, 0x03,
	0x06,
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00,
	0x73, 0x30
};

unsigned char read_gripper_status[] = {
	0x09, 0x03,
	0x07, 0xd0,
	0x00, 0x01,
	0x85, 0xcf
};

unsigned char activate_success[] = {
	0x09, 0x03,
	0x02,
	0x00, 0x00,
	0x59, 0x85
};

unsigned char close_with_full_speed_full_force[] = {
	0x09, 0x10,
	0x03, 0xe8,
	0x00, 0x03,
	0x06,
	0x09, 0x00,
	0x00, 0xff,
	0xff, 0xff,
	0x42, 0x29
};

unsigned char read_until_grip_completed[] = {
	0x09, 0x03,
	0x07, 0xd0,
	0x00, 0x03,
	0x04, 0x0e
};

unsigned char grip_is_completed[] = {
	0x09, 0x03,
	0x02, 0xf9,
	0x00, 0x1b, 0xd5
};

unsigned char open_with_full_speed_full_force[] = {
	0x09, 0x10,
	0x03, 0xe8,
	0x00, 0x03,
	0x06,
	0x09, 0x00,
	0x00, 0x00,
	0xff, 0xff,
	0x72, 0x19
};

unsigned char read_until_open_completed[] = {
	0x09, 0x03,
	0x07, 0xd0,
	0x00, 0x03,
	0x04, 0x0e
};

unsigned char open_is_completed[] = {
	0x09, 0x03,
	0x06,
	0xf9, 0x00,
	0x00, 0x00,
	0x03, 0x00,
	0x52, 0x2c
};

/*
 * Function Definitions
 */
int bufcmp(unsigned char *s1, unsigned char *s2);
int open_modbus();
int gripper_activate();
int gripper_close();
int gripper_open();
