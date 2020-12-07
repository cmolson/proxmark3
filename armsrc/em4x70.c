//-----------------------------------------------------------------------------
// Copyright (C) 2020 sirloins based on em4x50
//
// This code is licensed to you under the terms of the GNU GPL, version 2 or,
// at your option, any later version. See the LICENSE.txt file for the text of
// the license.
//-----------------------------------------------------------------------------
// Low frequency EM4170 commands
//-----------------------------------------------------------------------------

#include "fpgaloader.h"
#include "ticks.h"
#include "dbprint.h"
#include "lfadc.h"
#include "commonutil.h"
#include "em4x70.h"
#include "appmain.h" // tear

static em4x70_tag_t tag = { 0 };

// EM4170 requires a parity bit on commands, other variants do not.
static bool command_parity = true;

#define EM4X70_T_TAG_QUARTER_PERIOD         8
#define EM4X70_T_TAG_HALF_PERIOD            16
#define EM4X70_T_TAG_THREE_QUARTER_PERIOD   24
#define EM4X70_T_TAG_FULL_PERIOD            32
#define EM4X70_T_TAG_TWA                   128 // Write Access Time
#define EM4X70_T_TAG_DIV                   224 // Divergency Time
#define EM4X70_T_TAG_AUTH                 4224 // Authentication Time
#define EM4X70_T_TAG_WEE                  3072 // EEPROM write Time
#define EM4X70_T_TAG_TWALB                 672 // Write Access Time of Lock Bits

#define EM4X70_T_WAITING_FOR_SNGLLIW       160   // Unsure

#define TICKS_PER_FC                        12 // 1 fc = 8us, 1.5us per tick = 12 ticks
#define EM4X70_MIN_AMPLITUDE                10 // Minimum difference between a high and low signal

#define EM4X70_TAG_TOLERANCE                10
#define EM4X70_TAG_WORD                     48


/**
 * These IDs are from the EM4170 datasheet
 * Some versions of the chip require a
 * (even) parity bit, others do not
 */
#define EM4X70_COMMAND_ID                   0x1
#define EM4X70_COMMAND_UM1                  0x2
#define EM4X70_COMMAND_AUTH                 0x3
#define EM4X70_COMMAND_PIN                  0x4
#define EM4X70_COMMAND_WRITE                0x5
#define EM4X70_COMMAND_UM2                  0x7

static uint8_t gHigh = 0;
static uint8_t gLow  = 0;

#define IS_HIGH(sample) (sample>gLow ? true : false)
#define IS_LOW(sample) (sample<gHigh ? true : false)
#define IS_TIMEOUT(timeout_ticks) (GetTicks() > timeout_ticks)


static uint8_t bits2byte(uint8_t *bits, int length);
static void bits2bytes(uint8_t *bits, int length, uint8_t *out);
static int em4x70_receive(uint8_t *bits);
static bool find_listen_window(bool command);
static void em4x70_send_nibble(uint8_t command, bool with_parity);
static void em4x70_send_words(const uint8_t bytes[2]);
static void em4x70_send_byte(uint8_t byte);
static void em4x70_send_bit(int bit);
//tatic void em4x70_send_byte_lsb(uint8_t byte);

static void init_tag(void) {
    memset(tag.data, 0x00, sizeof(tag.data)/sizeof(tag.data[0]));
}

static void EM4170_setup_read(void) {

    FpgaDownloadAndGo(FPGA_BITSTREAM_LF);
    FpgaWriteConfWord(FPGA_MAJOR_MODE_LF_ADC | FPGA_LF_ADC_READER_FIELD);

    // 50ms for the resonant antenna to settle.
    SpinDelay(50);

    // Now set up the SSC to get the ADC samples that are now streaming at us.
    FpgaSetupSsc(FPGA_MAJOR_MODE_LF_READER);

    FpgaSendCommand(FPGA_CMD_SET_DIVISOR, LF_DIVISOR_125);

    // Connect the A/D to the peak-detected low-frequency path.
    SetAdcMuxFor(GPIO_MUXSEL_LOPKD);

    // Steal this pin from the SSP (SPI communication channel with fpga) and
    // use it to control the modulation
    AT91C_BASE_PIOA->PIO_PER = GPIO_SSC_DOUT;
    AT91C_BASE_PIOA->PIO_OER = GPIO_SSC_DOUT;

    // Disable modulation at default, which means enable the field
    LOW(GPIO_SSC_DOUT);

    // Start the timer
    StartTicks();

    SpinDelay(10);

    // Watchdog hit
    WDT_HIT();
}

static bool get_signalproperties(void) {

    // calculate signal properties (mean amplitudes) from measured data:
    // 32 amplitudes (maximum values) -> mean amplitude value -> gHigh -> gLow
    bool signal_found = false;
    int no_periods = 32, pct = 25, noise = 140; // pct originally 75, found 25-50 was working better for me
    uint8_t sample_ref = 127;
    uint8_t sample_max_mean = 0;
    uint8_t sample_max[no_periods];
    uint32_t sample_max_sum = 0;
    
    memset(sample_max, 0x00, sizeof(sample_max));

    // wait until signal/noise > 1 (max. 32 periods)
    for (int i = 0; i < TICKS_PER_FC * EM4X70_T_TAG_FULL_PERIOD * no_periods; i++) {

        // about 2 samples per bit period
        WaitTicks(TICKS_PER_FC * EM4X70_T_TAG_HALF_PERIOD);

        if (AT91C_BASE_SSC->SSC_RHR > noise) {
            signal_found = true;
            break;
        }

    }

    if (signal_found == false)
        return false;

    // calculate mean maximum value of 32 periods, each period has a length of
    // 3 single "full periods" to eliminate the influence of a listen window
    for (int i = 0; i < no_periods; i++) {

        uint32_t start_ticks = GetTicks();
        //AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG;
        while (GetTicks() - start_ticks < TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD) {

            volatile uint8_t sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;

            if (sample > sample_max[i])
                sample_max[i] = sample;

        }

        sample_max_sum += sample_max[i];
    }

    sample_max_mean = sample_max_sum / no_periods;

    // set global envelope variables
    gHigh = sample_ref + pct * (sample_max_mean - sample_ref) / 100;
    gLow  = sample_ref - pct * (sample_max_mean - sample_ref) / 100;

    // Basic sanity check
    if(gHigh - gLow < EM4X70_MIN_AMPLITUDE) {
        return false;
    }

    Dbprintf("%s: gHigh %d gLow: %d", __func__, gHigh, gLow);
    return true;
}



/**
 * record_liw
 * 
 * prints the timing from 1->0->1... for LIW_TEST_LENGTH
 * 
 */ 
/*#define LIW_TEST_LENGTH 128
static void record_liw(void) {

    uint32_t intervals[LIW_TEST_LENGTH];

    uint8_t sample;

    // Count duration low, then duration high.
    for(int count = 0; count < LIW_TEST_LENGTH-1; count+=2) {

        uint32_t start_ticks = GetTicks();
        do {
            sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
        }while (IS_LOW(sample));
        intervals[count] = GetTicks() - start_ticks;

        start_ticks = GetTicks();
        do {
            sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
        }while (IS_HIGH(sample));
        intervals[count+1] = GetTicks() - start_ticks;
    }

    for(int count = 0; count < LIW_TEST_LENGTH-1; count+=2){
        Dbprintf("%d 0", intervals[count]/TICKS_PER_FC);
        Dbprintf("%d 1", intervals[count+1]/TICKS_PER_FC);
    }
}*/

/**
 *  get_pulse_length
 * 
 *      Times falling edge pulses
 */ 
static uint32_t get_pulse_length(void) {

    uint8_t sample;
    uint32_t timeout = GetTicks() + (TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD);

    do {
        sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }while (IS_HIGH(sample) && !IS_TIMEOUT(timeout));

    if (IS_TIMEOUT(timeout))
        return 0;

    uint32_t start_ticks = GetTicks();
    timeout = start_ticks + (TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD);

    do {
        sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }while (IS_LOW(sample) && !IS_TIMEOUT(timeout));

    if (IS_TIMEOUT(timeout))
        return 0;

    timeout = (TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD) + GetTicks();
    do {
        sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }while (IS_HIGH(sample) && !IS_TIMEOUT(timeout));

    if (IS_TIMEOUT(timeout))
        return 0;

    return GetTicks() - start_ticks;
}

/**
 *  get_pulse_invert_length
 * 
 *      Times rising edge pules
 *  TODO: convert to single function with get_pulse_length()
 */ 
static uint32_t get_pulse_invert_length(void) {

    uint8_t sample;
    uint32_t timeout = GetTicks() + (TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD);

    do {
        sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }while (IS_LOW(sample) && !IS_TIMEOUT(timeout));

    if (IS_TIMEOUT(timeout))
        return 0;

    uint32_t start_ticks = GetTicks();
    timeout = start_ticks + (TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD);

    do {
        sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }while (IS_HIGH(sample) && !IS_TIMEOUT(timeout));

    if (IS_TIMEOUT(timeout))
        return 0;

    timeout = GetTicks() + (TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD);
    do {
        sample = (uint8_t)AT91C_BASE_SSC->SSC_RHR;
    }while (IS_LOW(sample) && !IS_TIMEOUT(timeout));

    if (IS_TIMEOUT(timeout))
        return 0;

    return GetTicks() - start_ticks;

}

static bool check_pulse_length(uint32_t pl, int length, int margin) {
    // check if pulse length <pl> corresponds to given length <length>
    //Dbprintf("%s: pulse length %d vs %d", __func__, pl, length * TICKS_PER_FC);
    return ((pl >= TICKS_PER_FC * (length - margin)) & (pl <= TICKS_PER_FC * (length + margin)));
}

static void em4x70_send_bit(int bit) {

    // send single bit according to EM4170 application note and datasheet

    uint32_t start_ticks = GetTicks();

    if (bit == 0) {

        // disable modulation (drop the field) for 4 cycles of carrier
        LOW(GPIO_SSC_DOUT);
        while (GetTicks() - start_ticks <= TICKS_PER_FC * 4);

        // enable modulation (activates the field) for remaining first
        // half of bit period
        HIGH(GPIO_SSC_DOUT);
        while (GetTicks() - start_ticks <= TICKS_PER_FC * EM4X70_T_TAG_HALF_PERIOD);

        // disable modulation for second half of bit period
        LOW(GPIO_SSC_DOUT);
        while (GetTicks() - start_ticks <= TICKS_PER_FC * EM4X70_T_TAG_FULL_PERIOD);

    } else {

        // bit = "1" means disable modulation for full bit period
        LOW(GPIO_SSC_DOUT);
        while (GetTicks() - start_ticks <= TICKS_PER_FC * EM4X70_T_TAG_FULL_PERIOD);
    }
    
}


static bool check_ack(bool bliw) {

    // returns true if signal structue corresponds to ACK, anything else is
    // counted as NAK (-> false)
    // Only relevant for pasword writing function:
    // If <bliw> is true then within the single listen window right after the
    // ack signal a RM request has to be sent.
    
    //AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG;
    uint32_t start_ticks = GetTicks();
    while (GetTicks()-start_ticks < TICKS_PER_FC * 4 * EM4X70_T_TAG_FULL_PERIOD) {

        /*
            ACK
              64 (48+16)
              64 (48+16)
            NACK
              64 (48+16)
              48 (32+16)
        */
        if (check_pulse_length(get_pulse_length(), 2 * EM4X70_T_TAG_FULL_PERIOD, EM4X70_TAG_TOLERANCE)) {

            // The received signal is either ACK or NAK.

            if (check_pulse_length(get_pulse_length(), 2 * EM4X70_T_TAG_FULL_PERIOD, EM4X70_TAG_TOLERANCE)) {

                // Now the signal must be ACK.

                if (!bliw) {

                    return true;

                } /*else {

                    // send RM request after ack signal

                    // wait for 2 bits (remaining "bit" of ACK signal + first
                    // "bit" of listen window)
                    wait_timer(FPGA_TIMER_0, T0 * 2 * EM4X50_T_TAG_FULL_PERIOD);

                    // check for listen window (if first bit cannot be interpreted
                    // as a valid bit it must belong to a listen window)
                    if (get_next_bit() == EM4X50_BIT_OTHER) {

                        // send RM for request mode
                        em4x50_send_bit(0);
                        em4x50_send_bit(0);

                        return true;
                    }*/
                //}
            } else {

                // It's NAK -> stop searching
                break;
            }
        }
    }

    return false;
}

//==============================================================================
// auth functions
//==============================================================================
static int authenticate( uint8_t rnd[7], uint8_t frnd[4]) {

    Dbprintf("RND: %02X %02X %02X %02X %02X %02X %02X", rnd[0], rnd[1], rnd[2], rnd[3], rnd[4], rnd[5], rnd[6]);
    Dbprintf("FRND: %02X %02X %02X %02X", frnd[0], frnd[1], frnd[2], frnd[3]);

    // writes <word> to specified <address>
    if (find_listen_window(true)) {

        // Send Authenticate Command
        em4x70_send_nibble(EM4X70_COMMAND_AUTH, true);
        
        // Send 56-bit Random number
        for(int i=0;i<7;i++) {
            em4x70_send_byte(rnd[i]);
        }
        
        // Send 7 x 0's
        for(int i=0; i<7; i++) {
            em4x70_send_bit(0);
        }
        
        // Send 28-bit f(RN)

        // Send first 24 bits
        for(int i=0; i < 3; i++) {
            em4x70_send_byte(frnd[i]);
        }
        // Send last 4 bits (no parity)
        em4x70_send_nibble((frnd[3] >> 4) & 0xf, false);

        // Receive header, 20-bit g(RN), LIW
        uint8_t grnd[32] = {0};
        int num = em4x70_receive(grnd);
        if(num < 10) {
            Dbprintf("Auth failed");
            return PM3_ESOFT;
        }
        uint8_t response[3];
        bits2bytes(grnd, 24, response);
        Dbprintf("Auth Response: %02X %02X %02X", response[2], response[1], response[0]);
        return PM3_SUCCESS;

    } else {
        Dbprintf("Failed to find listen window");
    }

    return PM3_ESOFT;
}


//==============================================================================
// send_pin functions
//==============================================================================
static int send_pin( uint8_t pin[4]) {

    // writes <word> to specified <address>
    if (find_listen_window(true)) {

        // send PIN command
        em4x70_send_nibble(EM4X70_COMMAND_PIN, true);

        // --> Send TAG ID (4-7)
        for(int i=0; i < 4; i++) {
            em4x70_send_byte(tag.data[7-i]);
            //Dbprintf("Sending TAG %02X", tag.data[7-i]);
        }

        // --> Send PIN
        for(int i=0; i < 4; i++) {
            em4x70_send_byte(pin[i]);
            //Dbprintf("Sending PIN %02X", pin[i]);
        }

        // Wait TWALB
        WaitTicks(TICKS_PER_FC * EM4X70_T_TAG_TWALB);

        // <-- Receive ACK
        if (check_ack(false)) {

            // <w> Writes Lock Bits
            WaitTicks(TICKS_PER_FC * EM4X70_T_TAG_WEE);
            // <-- Header + ID
            uint8_t tag_id[64];
            int num  = em4x70_receive(tag_id);
            if(num < 32) {
                Dbprintf("Invalid ID Received");
                return PM3_ESOFT;
            }
            bits2bytes(tag_id, num, &tag.data[4]);
            return PM3_SUCCESS;

        } else {
            Dbprintf("Failed first ack");
        }


    } else {
        Dbprintf("Failed to find listen window");
    }

    return PM3_ESOFT;
}

//==============================================================================
// write functions
//==============================================================================
static int write(uint8_t word[2], uint8_t address) {

    // writes <word> to specified <address>

    if (find_listen_window(true)) {

        // send write command
        em4x70_send_nibble(EM4X70_COMMAND_WRITE, true);

        // send address data with parity  bit
        em4x70_send_nibble(address, true);

        // send data
        em4x70_send_words(word);

        WaitTicks(TICKS_PER_FC * EM4X70_T_TAG_TWA);

        // look for ACK sequence
        if (check_ack(false)) {

            // now EM4x50 needs T0 * EM4X50_T_TAG_TWEE (EEPROM write time)
            // for saving data and should return with ACK
            WaitTicks(TICKS_PER_FC * EM4X70_T_TAG_WEE);
            if (check_ack(false)) {

                return PM3_SUCCESS;
            } else {
                Dbprintf("Failed second ack");
            }

        } else {
            Dbprintf("Failed first ack");
        }


    } else {
        Dbprintf("Failed to find listen window");
    }

    return PM3_ESOFT;
}

static void em4x70_send_byte(uint8_t byte) {

    // send byte (without parity)

    for (int i = 0; i < 8; i++)
        em4x70_send_bit((byte >> (7 - i)) & 1);

}

/*static void em4x70_send_byte_lsb(uint8_t byte) {

    // send byte (without parity)

    for (int i = 0; i < 8; i++)
        em4x70_send_bit((byte>>i) & 1);

}*/

static void em4x70_send_words(const uint8_t bytes[2]) {

    // Splt into nibbles
    uint8_t nibbles[4];
    uint8_t j = 0;
    for(int i = 0; i < 2; i++) {
        nibbles[j++] = (bytes[i] >> 4) & 0xf;
        nibbles[j++] = bytes[i] & 0xf;
    }

    // send 16 bit word with parity bits according to EM4x70 datasheet
    // sent as 4 x nibbles (4 bits + parity)
    for (int i = 0; i < 4; i++) {
        em4x70_send_nibble(nibbles[i], true);
        //Dbprintf("Sending %X", nibbles[i]);
    }

    // send column parities (4 bit)
    em4x70_send_nibble(nibbles[0] ^ nibbles[1] ^ nibbles[2] ^ nibbles[3], false);
    //Dbprintf("Sending checksum %X", nibbles[0] ^ nibbles[1] ^ nibbles[2] ^ nibbles[3]);
    // send final stop bit (always "0")
    em4x70_send_bit(0);
}

/**
 * em4x70_send_nibble
 * 
 *  sends 4 bits of data + 1 bit of parity (with_parity)
 * 
 */
static void em4x70_send_nibble(uint8_t nibble, bool with_parity) {
    int parity = 0;
    
    for (int i = 0; i < 4; i++) {
        int bit = (nibble >> (3 - i)) & 1;
        em4x70_send_bit(bit);
        parity ^= bit;
    }

    if(with_parity)
        em4x70_send_bit(parity);

}

static bool find_listen_window(bool command) {
    
    int cnt = 0;
    while(cnt < EM4X70_T_WAITING_FOR_SNGLLIW) {
        /*
        80 ( 64 + 16 )
        80 ( 64 + 16 )
        Flip Polarity
        96 ( 64 + 32 )
        64 ( 32 + 16 +16 )*/

        if (check_pulse_length(get_pulse_invert_length(), 80, EM4X70_TAG_TOLERANCE)) {
            if (check_pulse_length(get_pulse_invert_length(), 80, EM4X70_TAG_TOLERANCE)) {
                if (check_pulse_length(get_pulse_length(), 96, EM4X70_TAG_TOLERANCE)) {
                    if (check_pulse_length(get_pulse_length(), 64, EM4X70_TAG_TOLERANCE)) {
                        if(command) {
                            /* Here we are after the 64 duration edge.
                             *   em4170 says we need to wait about 48 RF clock cycles.
                             *   depends on the delay between tag and us
                             * 
                             *   I've found between 4-5 quarter periods (32-40) works best
                             */
                            WaitTicks(TICKS_PER_FC * 4 * EM4X70_T_TAG_QUARTER_PERIOD);
                            // Send RM Command
                            em4x70_send_bit(0);
                            em4x70_send_bit(0);
                        }
                        return true;
                    }
                }
            }
        }
        cnt++;
    }

    return false;
}

static void bits2bytes(uint8_t *bits, int length, uint8_t *out) {
    
    if(length%8 != 0) {
        Dbprintf("Should have a multiple of 8 bits, was sent %d", length);
    }
    
    int num_bytes = length / 8; // We should have a multiple of 8 here

    for(int i=1; i <= num_bytes; i++) {
        out[num_bytes-i] = bits2byte(bits, 8);
        bits+=8;
        //Dbprintf("Read: %02X", out[num_bytes-i]);
    } 
}

static uint8_t bits2byte(uint8_t *bits, int length) {

    // converts <length> separate bits into a single "byte"
    uint8_t byte = 0;
    for (int i = 0; i < length; i++) {

        byte |= bits[i];

        if (i != length - 1)
            byte <<= 1;
    }

    return byte;
}

/*static void print_array(uint8_t *bits, int len) {

    if(len%8 != 0) {
        Dbprintf("Should have a multiple of 8 bits, was sent %d", len);
    }
    
    int num_bytes = len / 8; // We should have a multiple of 8 here

    uint8_t bytes[8];

    for(int i=0;i<num_bytes;i++) {
        bytes[i] = bits2byte(bits, 8);
        bits+=8;
        Dbprintf("Read: %02X", bytes[i]);
    }
}*/


/**
 * em4x70_read_id
 * 
 *  read pre-programmed ID (4 bytes)
 */ 
static bool em4x70_read_id(void) {

    if(find_listen_window(true)) {
        uint8_t bits[64] = {0};
        em4x70_send_nibble(EM4X70_COMMAND_ID, command_parity);
        int num = em4x70_receive(bits);
        if(num < 32) {
            Dbprintf("Invalid ID Received");
            return false;
        }
        bits2bytes(bits, num, &tag.data[4]);
        return true;
    }
    return false;
}

/**
 *  em4x70_read_um1
 * 
 *  read user memory 1 (4 bytes including lock bits)
 */
static bool em4x70_read_um1(void) {
    if(find_listen_window(true)) {
        uint8_t bits[64] = {0};
        em4x70_send_nibble(EM4X70_COMMAND_UM1, command_parity);
        int num = em4x70_receive(bits);
        if(num < 32) {
            Dbprintf("Invalid UM1 data received");
            return false;
        }
        bits2bytes(bits, num, &tag.data[0]);
        return true;
    }
    return false;
}


/**
 *  em4x70_read_um2
 * 
 *  read user memory 2 (8 bytes)
 */
static bool em4x70_read_um2(void) {
    if(find_listen_window(true)) {
        uint8_t bits[64] = {0};
        em4x70_send_nibble(EM4X70_COMMAND_UM2, command_parity);
        int num = em4x70_receive(bits);
        if(num < 64) {
            Dbprintf("Invalid UM2 data received");
            return false;
        }
        bits2bytes(bits, num, &tag.data[24]);
        return true;
    }
    return false;
}

static bool find_EM4X70_Tag(void) {
    // Dbprintf("%s: Start", __func__);
    // function is used to check wether a tag on the proxmark is an
    // EM4170 tag or not -> speed up "lf search" process
    return find_listen_window(false);
}

static int em4x70_receive(uint8_t *bits) {

    uint32_t pl;
    int bit_pos = 0;
    uint8_t edge = 0;

    
    bool foundheader = false;

    // Read out the header
    //   12 Manchester 1's (may miss some during settle period)
    //    4 Manchester 0's
    
    // Skip a few leading 1's as it could be noisy
    WaitTicks(TICKS_PER_FC * 3 * EM4X70_T_TAG_FULL_PERIOD);

    // wait until we get the transition from 1's to 0's which is 1.5 full windows
    int pulse_count = 0;
    while(pulse_count < 12){
        pl = get_pulse_invert_length();
        pulse_count++;
        if(check_pulse_length(pl, 3 * EM4X70_T_TAG_HALF_PERIOD, EM4X70_TAG_TOLERANCE)) {
            foundheader = true;
            break;
        }
    }

    if(!foundheader) {
        Dbprintf("Failed to find read header");
        return 0;
    }

    // Skip next 3 0's, header check consumes the first 0
    for(int i = 0; i < 3; i++) {
        get_pulse_invert_length();
    }

    // identify remaining bits based on pulse lengths
    // between two listen windows only pulse lengths of 1, 1.5 and 2 are possible
    while (true) {

        if(edge)
            pl = get_pulse_length();
        else
            pl = get_pulse_invert_length();

        if (check_pulse_length(pl, EM4X70_T_TAG_FULL_PERIOD, EM4X70_T_TAG_QUARTER_PERIOD)) {

            // pulse length = 1
            bits[bit_pos++] = edge;

        } else if (check_pulse_length(pl, 3 * EM4X70_T_TAG_HALF_PERIOD, EM4X70_T_TAG_QUARTER_PERIOD)) {

            // pulse length = 1.5 -> flip edge detection
            if(edge) {
                bits[bit_pos++] = 0;
                bits[bit_pos++] = 0;
                edge = 0;
            } else {
                bits[bit_pos++] = 1;
                bits[bit_pos++] = 1;
                edge = 1;
            }

        } else if (check_pulse_length(pl, 2 * EM4X70_T_TAG_FULL_PERIOD, EM4X70_T_TAG_QUARTER_PERIOD)) {

            // pulse length of 2
            if(edge) {
                bits[bit_pos++] = 0;
                bits[bit_pos++] = 1;
            } else {
                bits[bit_pos++] = 1;
                bits[bit_pos++] = 0;
            }

        } else if ( (edge && check_pulse_length(pl, 3 * EM4X70_T_TAG_FULL_PERIOD, EM4X70_T_TAG_QUARTER_PERIOD)) ||
                    (!edge && check_pulse_length(pl, 80, EM4X70_T_TAG_QUARTER_PERIOD))) {

            // LIW detected (either invert or normal)
            return --bit_pos;
        }
    }
    return bit_pos;

}

void em4x70_info(em4x70_data_t *etd) {

    uint8_t status = 0;
    
    // Support tags with and without command parity bits
    command_parity = etd->parity;

    init_tag();
    EM4170_setup_read();

    // Find the Tag
    if (get_signalproperties() && find_EM4X70_Tag()) {
        // Read ID, UM1 and UM2
        status = em4x70_read_id() && em4x70_read_um1() && em4x70_read_um2();
    }

    StopTicks();
    lf_finalize();
    reply_ng(CMD_LF_EM4X70_INFO, status, tag.data, sizeof(tag.data));
}


void em4x70_write(em4x70_data_t *etd) {

    uint8_t status = 0;
    Dbprintf("Writing tag...");
    command_parity = etd->parity;

    init_tag();
    EM4170_setup_read();

    // Find the Tag
    if (get_signalproperties() && find_EM4X70_Tag()) {
        // Write

        status = write(etd->word, etd->address) == PM3_SUCCESS;

        if(status) {
            // Read Tag
            em4x70_read_id();
            em4x70_read_um1();
            em4x70_read_um2();
        }

    }

    StopTicks();
    lf_finalize();
    reply_ng(CMD_LF_EM4X70_WRITE, status, tag.data, sizeof(tag.data));
}


void em4x70_send_pin(em4x70_data_t *etd) {

    uint8_t status = 0;
    Dbprintf("Sending PIN tag %02X %02X %02X %02X", etd->pin[0], etd->pin[1], etd->pin[2], etd->pin[3]);
    command_parity = etd->parity;

    init_tag();
    EM4170_setup_read();

    // Find the Tag
    if (get_signalproperties() && find_EM4X70_Tag()) {
        
        // Read ID (required for send_pin command)
        if(em4x70_read_id()) {
            
            // Send PIN
            status = send_pin(etd->pin) == PM3_SUCCESS;

            /*
            for(int i = 0; i<= 0x0F; i++) {
                if(find_listen_window(true)) {
                    em4x70_send_nibble(i, false);
                    record_liw();
                    Dbprintf("Attempt command %02X", i);
                }
            }
            */

            if(status) {
                // Read Tag
                em4x70_read_id();
                em4x70_read_um1();
                em4x70_read_um2();
            }
        } else {
            Dbprintf("Failed to read ID");
        }

    }

    StopTicks();
    lf_finalize();
    reply_ng(CMD_LF_EM4X70_SEND_PIN, status, tag.data, sizeof(tag.data));
}


void em4x70_auth(em4x70_data_t *etd) {

    uint8_t status = 0;
    //Dbprintf("Sending PIN tag %02X %02X %02X %02X", etd->pin[0], etd->pin[1], etd->pin[2], etd->pin[3]);
    command_parity = etd->parity;

    init_tag();
    EM4170_setup_read();

    // Find the Tag
    if (get_signalproperties() && find_EM4X70_Tag()) {
        
        // Read ID (required for send_pin command)
        if(em4x70_read_id()) {
            
            // Authenticate and get tag response
            status = authenticate(etd->rnd, etd->frnd) == PM3_SUCCESS;

            if(status) {
                // Read Tag
                em4x70_read_id();
                em4x70_read_um1();
                em4x70_read_um2();
            }
        } else {
            Dbprintf("Failed to read ID");
        }

    }

    StopTicks();
    lf_finalize();
    reply_ng(CMD_LF_EM4X70_AUTH, status, tag.data, sizeof(tag.data));
}