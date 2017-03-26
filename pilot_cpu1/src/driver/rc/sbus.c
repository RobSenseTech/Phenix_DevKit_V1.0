#include "driver.h"
#include "hrt/drv_hrt.h"
#include "sbus.h"

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;

static uint8_t	frame[SBUS_FRAME_SIZE];

static unsigned partial_frame_count;

unsigned sbus_frame_drops;

int sbus_init(const char *device, bool singlewire)
{
	int sbus_fd = open(device, O_RDWR | O_NONBLOCK);
    int baudrate=100000;//sbus特有波特率
    UartDataFormat_t data_format;

	/* 100000bps, even parity, two stop bits */
    data_format.iDataBits = UART_DATA_8_BIT;
    data_format.iParity = UART_EVEN_PARITY;
    data_format.iStopBits = UART_2_STOP_BIT;

    if(sbus_fd != -1)
    {
        ioctl(sbus_fd, UART_IOC_SET_BAUDRATE, &baudrate);
        ioctl(sbus_fd, UART_IOC_SET_DATA_FORMAT, &data_format);

        partial_frame_count = 0;
        last_rx_time = hrt_absolute_time();

    }

    return sbus_fd;
}

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};

static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static bool sbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{
	/* check frame boundary markers to avoid out-of-sync cases */
	if ((frame[0] != 0x0f)) {
//        Print_Err("frame[0]=%x\n", frame[0]);
		sbus_frame_drops++;
		return false;
	}

	switch (frame[24]) {
	case 0x00:
		/* this is S.BUS 1 */
		break;

	case 0x03:
		/* S.BUS 2 SLOT0: RX battery and external voltage */
		break;

	case 0x83:
		/* S.BUS 2 SLOT1 */
		break;

	case 0x43:
	case 0xC3:
	case 0x23:
	case 0xA3:
	case 0x63:
	case 0xE3:
		break;

	default:
		/* we expect one of the bits above, but there are some we don't know yet */
		break;
	}

	/* we have received something we think is a frame */
	last_frame_time = frame_time;

    unsigned channel;
    unsigned pick;
	unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
			     SBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	for (channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}


		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	}

	/* decode switch channels if data fields are wide enough */
	if (max_values > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
		/* channel 18 (index 17) */
		values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	/* decode and handle failsafe and frame-lost flags */
	if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		*sbus_failsafe = true;
		*sbus_frame_drop = true;

	} else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 *
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
		 * condition as fail-safe greatly reduces the reliability and range of the radio link,
		 * e.g. by prematurely issueing return-to-launch!!! */

		*sbus_failsafe = false;
		*sbus_frame_drop = true;

	} else {
		*sbus_failsafe = false;
		*sbus_frame_drop = false;
	}

	return true;
}


bool sbus_input(int sbus_fd, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_channels)
{
	size_t		ret;
	hrt_abstime	now;

	/*
	 * The S.bus protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of morepartial_frame_count than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();
    if((now - last_rx_time) < 3000)
        return false;

    while(partial_frame_count == 0)
    {
	    ret = read(sbus_fd, (char *)&frame[0], 1);
        if(ret != 1)
        {
            break;
        }
        else
        {
            if(frame[0] == 0x0f)
            {
                partial_frame_count++;
            }
        }
    }

    //找到头，则拷数据，没找到直接返回
    if(partial_frame_count != 0)
    {
	    ret = read(sbus_fd, (char *)&frame[partial_frame_count], SBUS_FRAME_SIZE - partial_frame_count);
    }
    else
        return false;

	/*
	 * Add bytes to the current frame
	 */
	partial_frame_count += ret;


	/*
	 * If we don't have a full frame, return
	 */
	if (partial_frame_count < SBUS_FRAME_SIZE) {
		return false;
	}
    else
    {
        //没找到结束符，丢弃所有之前的数据
        if(frame[SBUS_FRAME_SIZE-1] != 0x00)
        {
            partial_frame_count = 0;
            return false;
        }
    }

    //找到结束符，记录当前时间
    if(partial_frame_count == SBUS_FRAME_SIZE)
    {
        last_rx_time  = hrt_absolute_time();
    }
	/*
	 * Great, it looks like we might have a frame.  Go ahead and
	 * decode it.
	 */
	partial_frame_count = 0;
	return sbus_decode(now, values, num_values, sbus_failsafe, sbus_frame_drop, max_channels);
}

