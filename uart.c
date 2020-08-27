#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#define TTY_USB		"/dev/ttyUSB0"
#define IWR6843_FW_BIN	"v103_len_1861.bin"//"v103_len_977.bin"//"pattern.txt"
#define MAX_RX_SIZE 2048
unsigned char buffer[MAX_RX_SIZE]={0};
int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof(tty));
	if (tcgetattr (fd, &tty) != 0) {
		printf("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
		printf("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr (fd, &tty) != 0) {
		printf("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf("error %d setting term attributes", errno);
}

int uart_recv(int fd);

#define META_IMG1	0x04
#define META_IMG2	0x05
#define META_IMG3	0x06
#define META_IMG4	0x07

#define TYPE_SFLASH	0x02
#define TYPE_SRAM	0x04

#define CMD_PING	0x20
#define CMD_OPEN	0x21
#define CMD_CLOSE	0x22
#define CMD_WFLASH	0x24
#define CMD_ERASE	0x28
#define CMD_VERSION	0x2F

typedef struct {	// 5 bytes
	unsigned char sync;
	unsigned char revserved;
	unsigned char length;
	unsigned char sum;
	unsigned char cmd;
	unsigned char payload[240];
}IWR6843;

typedef struct {
	unsigned char sync;
	unsigned char revserved;
	unsigned char length;
	unsigned char sum;
	unsigned char cmd;
	unsigned char file_size[4];
	unsigned char reserved0[3];
	unsigned char storage_type;
	unsigned char reserved1[3];
	unsigned char file_type;
	unsigned char reserver3[4];
}IWR6843_OPEN_FILE;

typedef struct {	// 9 bytes
        unsigned char sync;
        unsigned char revserved;
        unsigned char length;
        unsigned char sum;
        unsigned char cmd;
        unsigned char reserved0[3];
        unsigned char storage_type;
}IWR6843_CLOSE_FILE;

int send_get_version(int fd)
{
	IWR6843 iwr6843_t;
	int ret = 0;

	memset(&iwr6843_t, 0, sizeof(IWR6843));

	iwr6843_t.sync = 0xAA;
	iwr6843_t.length = 0x03;
	iwr6843_t.sum = 0x2F;
	iwr6843_t.cmd = CMD_VERSION;

	ret = write(fd, &iwr6843_t, 5);

	return 0;
}

int send_ping(int fd)
{
	IWR6843 iwr6843_t;
	int ret = 0;

	memset(&iwr6843_t, 0, sizeof(IWR6843));

	iwr6843_t.sync = 0xAA;
	iwr6843_t.length = 0x03;
	iwr6843_t.sum = 0x20;
	iwr6843_t.cmd = CMD_PING;

	ret = write(fd, &iwr6843_t, 5);

	return 0;
}

int send_open_file(int fd)
{
	IWR6843_OPEN_FILE iwr6843_t;
	FILE *fp;
	int sum = 0, i = 0;
	int ret = 0;
	int size = 0;

	memset(&iwr6843_t, 0, sizeof(IWR6843_OPEN_FILE));
	iwr6843_t.sync = 0xAA;
	iwr6843_t.length = 0x13;
	iwr6843_t.cmd = CMD_OPEN;
	iwr6843_t.storage_type = TYPE_SFLASH;
	iwr6843_t.file_type = META_IMG1;

	fp = fopen(IWR6843_FW_BIN, "r");
	if (fp == NULL)
		return -1;

	fseek(fp, 0, SEEK_END);	// seek to end of file
	size = ftell(fp);	// get current file pointer
	fseek(fp, 0, SEEK_SET);	// seek back to beginning of file

	iwr6843_t.file_size[0] = (unsigned char)((size >> 24) & 0xff);
	iwr6843_t.file_size[1] = (unsigned char)((size >> 16) & 0xff);
	iwr6843_t.file_size[2] = (unsigned char)((size >> 8) & 0xff);
	iwr6843_t.file_size[3] = (unsigned char)(size & 0xff);

	for (i = 0; i < 4; i++) {
		sum += iwr6843_t.file_size[i];
	}
	sum = sum + iwr6843_t.file_type + iwr6843_t.storage_type + iwr6843_t.cmd;
	iwr6843_t.sum = (unsigned char)(sum & 0x000000ff);

	ret = write(fd, &iwr6843_t, 21);

	fclose(fp);

	return 0;
}

int send_write(int fd)
{
	IWR6843 iwr6843_t;
	FILE *fp;
	int len = 0;
	int i = 0;
	int sum = 0;
	int ret = 0;
	unsigned char tmp;
	unsigned int loop = 50;

	//memset(&iwr6843_t, 0, sizeof(IWR6843));
	memset(buffer, 0, MAX_RX_SIZE);

	fp = fopen(IWR6843_FW_BIN, "r");
	if (fp == NULL)
		return -1;

	do {
		//memset(&iwr6843_t, 0, sizeof(IWR6843));
		//memset(buffer, 0, MAX_RX_SIZE);

		//len = fread(iwr6843_t.payload, 1, 240, fp);
		len = fread(buffer, 1, MAX_RX_SIZE, fp);
		if (len == 0) {
			if (feof(fp)) {
				break;
			}
		}

		printf("Length = %d\n", len);
		//ret = write(fd, &iwr6843_t.payload, len);
		while(loop){
			ret = write(fd, buffer, len);
			usleep(50*1000);
			loop--;
		}

		//uart_recv(fd);

	} while (1);

	fclose(fp);

	return 0;
}

int send_close(int fd)
{
	IWR6843_CLOSE_FILE iwr6843_t;
	int ret = 0;

	memset(&iwr6843_t, 0, sizeof(IWR6843_CLOSE_FILE));
	iwr6843_t.sync = 0xAA;
	iwr6843_t.length = 0x07;
	iwr6843_t.sum = 0x24;
	iwr6843_t.cmd = CMD_CLOSE;
	iwr6843_t.storage_type = TYPE_SFLASH;

	ret = write(fd, &iwr6843_t, 9);

	return 0;
}

int send_erase(int fd)
{
	IWR6843 iwr6843_t;
	int ret = 0;

	iwr6843_t.sync = 0xAA;
	iwr6843_t.length = 0x03;
	iwr6843_t.sum = 0x28;
	iwr6843_t.cmd = CMD_ERASE;

	ret = write(fd, &iwr6843_t, 5);

	return 0;
}

int response_chirp(int fd)
{
	int ret = 0;
	char ACK[]="Done";
	ret = write(fd, ACK, sizeof(ACK));

	return 0;
}

int uart_recv(int fd)
{
	unsigned char buf [80];
	int i = 0, n = 0;

	//usleep ((7 + 25) * 100);

	while (1) {
                n = read (fd, buf, sizeof(buf));
                if (n > 0)
                        break;
		else
			usleep(1000);
        }
	printf("Recv Ack len = %d ", n);
	for (i = 0; i < n; i++) {
		printf("0x%02x ", buf[i]);
	}
	printf("\n");


	return n;
}

int main()
{
	unsigned char buf [80];
	int i = 0, n = 0;
	int ret = 0;
	char ACK[]="Done";

	int fd;
	//unsigned char data[5] = {0xAA, 0x00, 0x03, 0x2F, 0x2F};

	fd = open(TTY_USB, O_RDWR);
	//set_interface_attribs(fd, B921600, 0);
	printf("\r\nTest UART ttyUSB0, 115200\n");
	set_interface_attribs(fd, B115200, 0);
	//set_blocking(fd, 0);

	/*
	 * Send a 100 millisecond break
	*/
	//printf("Send break signal...\n");
	//tcsendbreak(fd, 10);
	//uart_recv(fd);

	//printf("Send open command...\n");
	//send_open_file(fd);
	//uart_recv(fd);

	while (1) {
		n = read (fd, buf, sizeof(buf));
		if (n > 0){
			printf("\r\nRecv Ack len = %d", n);
			ret = write(fd, ACK, sizeof(ACK));
			//if(n == 12)
			//break;
		}
		else
			usleep(1000);
    }

	printf("Firmware OTA\n");
	send_write(fd);

	sleep(1);

	/* Send clode command */
	//printf("Send close command\n");
	//send_close(fd);
	//uart_recv(fd);

	close(fd);

	return 0;
}
