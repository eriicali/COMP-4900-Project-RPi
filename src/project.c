#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <hw/i2c.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <devctl.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

//#define DEBUG

int g_tds = 0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
uint8_t *recvbuf;
FILE* f;
const char * buffer = "curl -s http://3.148.167.10:3000/salinity";
uint8_t desired_salinity;

// inter-integrated circuit context struct
struct i2c_ctx {
	struct {
		i2c_send_t send;
		/* ANS1115 takes a 3 byte initialization command */
		uint8_t buf[3];
	} send_buffer;

	struct {
		i2c_recv_t recv;
		/* ANS1115 sends 16bits in each frame so we parse two unsigned bytes */
		uint8_t buf[2];
	} recv_buffer;

	int i2c_fd;
} g_i2c_ctx =
		{ .i2c_fd = 0, .send_buffer = { .send = { .slave = { .addr = 0x48,
				.fmt = I2C_ADDRFMT_7BIT, }, .len = 0x3, .stop = 0x1, }, .buf = {
				0 }, }, .recv_buffer = { .recv = { .slave = { .addr = 0x48,
				.fmt = I2C_ADDRFMT_7BIT, }, .len = 0x2, .stop = 0x1, }, .buf = {
				0 }, }, };

#ifdef DEBUG
static void i2c_print_status()
{
	int ret;
	i2c_status_t stat = {0};
	ret = devctl(g_i2c_ctx.i2c_fd, DCMD_I2C_STATUS, &stat, sizeof(i2c_status_t), NULL);
	fprintf(stderr, "DEBUG: devctl STATUS ret:%d:%s i2c_status:0x%04x\n", ret, strerror(ret), (unsigned)stat);
}
#endif

static int i2c_init_ctx() {
	/* in our QNX bsp we set i2c1 to be the resmgr for i2c driver */
	int fd;
	if (g_i2c_ctx.i2c_fd == 0) {
		fd = open("/dev/i2c1", O_RDWR);
		if (fd > 0) {
			g_i2c_ctx.i2c_fd = fd;
			return EOK;
		}
	}
	return EEXIST;
}

#ifdef DEBUG
static void i2c_print_driver_info() {
	i2c_driver_info_t i2c_info;
	int ret = devctl(g_i2c_ctx.i2c_fd, DCMD_I2C_DRIVER_INFO, &i2c_info,
			sizeof(i2c_driver_info_t), NULL);
	if (ret == EOK) {
		fprintf(stderr,
				"DEBUG: DRIVER_INFO speed_mode=0x%04x addr_mode=0x%04x, verbosity=0x%04x\n",
				i2c_info.speed_mode, i2c_info.addr_mode, i2c_info.verbosity);
	} else {
		fprintf(stderr, "DEBUG: devctl failed DRIVER_INFO ret:%d:%s\n", ret,
				strerror(ret));
	}
	i2c_print_status();
}
#endif

static int i2c_bus_reset() {
	int ret = devctl(g_i2c_ctx.i2c_fd, DCMD_I2C_BUS_RESET, NULL, 0, NULL);
	if (ret != EOK) {
		fprintf(stderr, "%s devctl failed BUS_RESET ret:%d:%s\n", __func__, ret, strerror(ret));
	}
#ifdef DEBUG
	i2c_print_status();
#endif
	return ret;
}

static int i2c_set_addr(uint8_t addr) {
	i2c_addr_t i2caddr = {
			.addr = addr,
			.fmt = I2C_ADDRFMT_7BIT,
	};
	int ret = devctl(g_i2c_ctx.i2c_fd, DCMD_I2C_SET_SLAVE_ADDR, &i2caddr, sizeof(i2c_addr_t), NULL);
	if (ret != EOK) {
		fprintf(stderr, "%s devctl failed SET_SLAVE_ADDR ret:%d:%s\n", __func__, ret, strerror(ret));
	}
#ifdef DEBUG
	i2c_print_status();
#endif
	return ret;
}

static void i2c_set_send_buf(uint8_t buf0, uint8_t buf1, uint8_t buf2) {
	g_i2c_ctx.send_buffer.buf[0] = buf0;
	g_i2c_ctx.send_buffer.buf[1] = buf1;
	g_i2c_ctx.send_buffer.buf[2] = buf2;
}

static int i2c_send(size_t bufsz) {
	int ret;
	g_i2c_ctx.send_buffer.send.len = min(bufsz, sizeof(g_i2c_ctx.send_buffer.buf));
	ret = devctl(g_i2c_ctx.i2c_fd,
	DCMD_I2C_SEND, &(g_i2c_ctx.send_buffer),
			sizeof(i2c_send_t) + bufsz,
			NULL);
	if (ret != EOK) {
		fprintf(stderr, "%s devctl failed SEND ret:%d:%s\n", __func__, ret, strerror(ret));
	}
#ifdef DEBUG
	i2c_print_status();
#endif
	return ret;
}

static void i2c_get_recv_buf(uint8_t **out) {
	*out = g_i2c_ctx.recv_buffer.buf;
}

static int i2c_recv(size_t bufsz) {
	int ret;
	g_i2c_ctx.recv_buffer.recv.len = min(bufsz, sizeof(g_i2c_ctx.recv_buffer.buf));
	ret = devctl(g_i2c_ctx.i2c_fd,
	DCMD_I2C_RECV, &g_i2c_ctx.recv_buffer,
			sizeof(i2c_recv_t) + g_i2c_ctx.recv_buffer.recv.len,
			NULL);
	if (ret != EOK) {
		fprintf(stderr, "%s devctl failed RECV ret:%d:%s\n",__func__, ret, strerror(ret));
	}
#ifdef DEBUG
	i2c_print_status();
#endif
	return ret;
}

void turn_on_salty_pump() {
	// set pin 23 output pull neutral drive low
	system("gpio-bcm2711 set 23 op pn dl");
}

void turn_on_fresh_water_pump () {
	// set pin 24 output pull neutral drive low
	system("gpio-bcm2711 set 24 op pn dl");
}

void turn_off_salty_pump() {
	// set pin 23 output pull neutral drive high
	system("gpio-bcm2711 set 23 op pn dh");
}

void turn_off_fresh_water_pump() {
	// set pin 24 output pull neutral drive high
	system("gpio-bcm2711 set 24 op pn dh");
}

float tds_to_volts(int tds) {
	/**
	 * 2.4 = 5800tds
	 * 0.22 = 50tds
	 */
	float y = (0.00037913043478261 * ((float)tds)) + 0.20104347826087;
	return y;
}

int tds_in_threshold_check(float vin, float tgt, float thresh)
{
	if (vin >= (tgt - thresh) &&
			vin <= (tgt + thresh)) {
		return 1;
	}
	return 0;
}

void* get_current_salinity(void * arg) {
	int ret = 0;
	/**
	* set buffer to 3 byte command to initialize ans1115
	* if necessary see the TI ANS1115 datasheet to decode
	*/
	i2c_set_send_buf(0x1, 0x42, 0x85);
	ret = i2c_send(3);
	if (ret != EOK) {
		fprintf(stderr, "Failed to send ANS1115 init command\n");
		errno = ret;
		return NULL;
	}

	/**
	 * spin and wait for signal back from asn1115
	 * which confirms that the send command was successful
	 */

	while (1) {
		/* set buf to 0 and send 1 byte */
		i2c_set_send_buf(0x0, 0x0, 0x0);
		ret = i2c_send(1);
		if (ret != EOK) {
			fprintf(stderr, "Write register select failed ret:%d:%s\n", ret, strerror(ret));
		}

		/* recv the next 16 bits */
		ret = i2c_recv(2);
		if (ret != EOK) {
			fprintf(stderr, "Read volts failed\n");
			errno = ret;
		}

		/* concat the two uint8_t into a correctly ordered uint16_t */
		uint16_t vol = (uint16_t) recvbuf[0] * 256 + (uint16_t) recvbuf[1];
		float volts = (float) vol * 4.096 / 32768.0;

		int tds;
		pthread_mutex_lock(&mutex);
		tds = g_tds;
		pthread_mutex_unlock(&mutex);
		if (tds == 0) {
			printf("CB: tds target not set\n");
			continue;
		}
		float tgt = tds_to_volts(tds);

		printf("volts:%f tgt:%f\n", volts, tgt);

		if (tds_in_threshold_check(volts, tgt, 0.5)) {
			/* close enough, stop switching the pumps */
			turn_off_salty_pump();
			turn_off_fresh_water_pump();
			continue;
		} else {
			if (volts > tgt) {
				/* turn off salty pump */
				/* turn on fresh water pump */
				turn_off_salty_pump();
				turn_on_fresh_water_pump();
				printf("CB: volts > tgt\n");
				usleep(500);
			}
			if (volts < tgt) {
				/* turn on salty pump */
				/* turn off fresh water pump */
				turn_on_salty_pump();
				turn_off_fresh_water_pump();
				printf("CB: volts < tgt\n");
				usleep(500);
			}
		}
	}
}

void* get_desired_salinity(void * arg) {
	while(1) {
		if( ( f = popen( buffer, "rw" ) ) == NULL ) {
				perror( "popen" );
				return NULL;
		}

		int out;
		(void)fscanf(f, "{\"desired_salinity\":\"%d\"}", &out);
		pthread_mutex_lock(&mutex);
		g_tds = out;
		pthread_mutex_unlock(&mutex);
	}
}

int main(void) {
	pthread_t threads[2];
	// set pin 23 output pull neutral drive high
	system("gpio-bcm2711 set 23 op pn dh");
	// set pin 24 output pull neutral drive high
	system("gpio-bcm2711 set 24 op pn dh");

	// create thread to get desired salinity
	pthread_create(&threads[0], NULL, get_desired_salinity, NULL);
	int ret = 0;

	/* initialize the global variables for i2c functions */
	(void)i2c_init_ctx();

#ifdef DEBUG
	/* in debug flag mode print the specifics of the i2c driver */
	i2c_print_driver_info();
#endif

	/* retrieve the buffer the i2c_recv writes to */
	i2c_get_recv_buf(&recvbuf);

	/* set the addr of the ANS1115 we communicate with */
	ret = i2c_set_addr(0x48);
	if (ret != EOK) {
		fprintf(stderr, "Setting addr of ANS1115 failed\n");
		return ret;
	}

	/**
	 *  clear the bus of any possible incomplete actions made by
	 *  other processes or thread which are incomplete before we
	 *  start using the i2c resmgr
	 *  Ignoring output because this may fail
	 */
	(void)i2c_bus_reset();
	// init thread to get current salinity
	pthread_create(&threads[1], NULL, get_current_salinity, NULL);
	pthread_join(threads[1], NULL);
	pthread_join(threads[0], NULL);

	return EXIT_SUCCESS;
}
