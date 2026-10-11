// SPDX-License-Identifier: MIT

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Time.hpp>

#include <cassert>
#include <chrono>
#include <climits>
#include <fstream>
#include <mutex>
#include <string.h>
#include <thread>

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>

extern const char *SoapyMuCell_tag;
extern const char *SoapyMuCell_commit;

#include "eeprom_auth.hpp"

#define DRIVER_KEY_STR "mucell"
#define HARDWARE_KEY_STR "mucell"
#define DEVICE_LABEL_STR "mucell"
#define DEVICE_DRIVER_STR "mucell"
#define REGISTER_STR "mucell"

// Streaming mode, affecting how starting, stopping, overruns and underruns
// are handled.
enum stream_mode {
    // Behave like most SDRs: RX overrun or TX underrun
    // may cause samples to be dropped, but streams will keep running.
    // Application can use timestamps to maintain correct timing.
    STREAM_MODE_NORMAL,
    // Behave like linked ALSA PCMs with default software parameters.
    // Overrun or underrun causes both RX and TX streams to stop.
    // TX buffer must be always kept filled to avoid underrun.
    // First write to TX stream starts both streams.
    // This was a stop-gap solution before implementation of timestamps
    // and is primarily kept here for compatibility with applications
    // that have not been modified to use timestamps yet.
    STREAM_MODE_LINK,
};

// Clamp, offset, scale and quantize a value based on a SoapySDR::Range
// and convert it to an integer.
// The value is offset so that the minimum value becomes 0
// and scaled so that step becomes 1.
static int32_t scale_from_range(SoapySDR::Range range, double value) {
    return (int)std::round(
        (std::min(std::max(value, range.minimum()), range.maximum()) - range.minimum()) / range.step());
}

// Inverse of scale_from_range.
static double scale_to_range(SoapySDR::Range range, int32_t value) {
    return std::min(std::max(
                        range.minimum() + range.step() * (double)value,
                        range.minimum()),
                    range.maximum());
}

// Read a number from a file in format 0x1234.
static uint16_t read_hex_file(const char *filename) {
    std::ifstream file(filename);
    std::string str;
    std::getline(file, str);
    return std::stoul(str, NULL, 16);
}

struct hat_info {
    uint16_t product_id;
    uint16_t product_ver;
    bool read_success;
    EepromInfo eeprom; // parsed + authenticated EEPROM data
};

static struct hat_info read_hat_info(void) {
    const uint16_t product_id_expected = 0x1255;
    // Default assumption if ID cannot be read
    struct hat_info info = {product_id_expected, 0x0101, false, {}};
    try {
        info.product_id   = read_hex_file("/proc/device-tree/hat/product_id");
        info.product_ver  = read_hex_file("/proc/device-tree/hat/product_ver");
        info.read_success = true;
        SoapySDR_logf(SOAPY_SDR_INFO, "Hardware version %d.%d",
                      info.product_ver >> 8, info.product_ver & 0xFF);
    } catch (std::exception &e) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Could not read HAT ID. Assuming hardware version %d.%d",
                      info.product_ver >> 8, info.product_ver & 0xFF);
    }
    if (info.product_id != product_id_expected) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Unexpected product ID 0x%04x. Are you sure the correct HAT is connected?", info.product_id);
    }

    // ── Parse EEPROM atoms and verify signature ───────────────────────────
    info.eeprom = read_and_verify_eeprom();

    // Log vendor / serial / auth result at init time
    SoapySDR_logf(SOAPY_SDR_INFO, "HAT EEPROM: %s", info.eeprom.summary().c_str());

    switch (info.eeprom.auth) {
    case AuthResult::OK:
        SoapySDR_logf(SOAPY_SDR_INFO,
                      "HAT authenticity: GENUINE - signed by key \"%s\"",
                      info.eeprom.matched_key_name.c_str());
        break;
    case AuthResult::NO_SIGNATURE:
        SoapySDR_logf(SOAPY_SDR_WARNING,
                      "HAT authenticity: unsigned EEPROM (no RSIG atom)");
        break;
    case AuthResult::KEY_NOT_FOUND:
        SoapySDR_logf(SOAPY_SDR_WARNING,
                      "HAT authenticity: signature present but did NOT match any compiled-in key");
        break;
    case AuthResult::NO_KEYS:
        SoapySDR_logf(SOAPY_SDR_WARNING,
                      "HAT authenticity: no public keys compiled in — rebuild with keys in public_keys/");
        break;
    case AuthResult::EEPROM_NOT_FOUND:
        SoapySDR_logf(SOAPY_SDR_WARNING,
                      "HAT authenticity: EEPROM image not found in device tree");
        break;
    case AuthResult::VERIFY_ERROR:
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "HAT authenticity: OpenSSL error during signature verification");
        break;
    case AuthResult::PARSE_ERROR:
        SoapySDR_logf(SOAPY_SDR_WARNING,
                      "HAT authenticity: EEPROM image could not be parsed");
        break;
    }

    return info;
}

// Convert raw received samples to CF32.
// TODO: Support other formats and add format as a parameter.
static inline void convert_rx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length) {
    const int32_t *src_ = (const int32_t *)src + src_offset * 2;
    float *dest_        = (float *)dest + dest_offset * 2;
    const float scaling = 1.0f / 0x80000000L;
    for (size_t i = 0; i < length * 2; i++) {
        dest_[i] = scaling * (float)src_[i];
    }
}

// Convert CF32 to raw transmit samples.
// TODO: Support other formats and add format as a parameter.
static inline void convert_tx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length, float tx_threshold2) {
    const float *src_   = (const float *)src + src_offset * 2;
    int32_t *dest_      = (int32_t *)dest + dest_offset * 2;
    const float scaling = (float)0x7FFFFFFFL;
    for (size_t i = 0; i < length * 2; i += 2) {
        float fi = src_[i], fq = src_[i + 1];
        int32_t vi = scaling * std::max(std::min(fi, 1.0f), -1.0f);
        int32_t vq = scaling * std::max(std::min(fq, 1.0f), -1.0f);
        // Second lowest bit of each "I" sample controls RX/TX switching.
        // Set the lowest bit to the same value just in case.
        // Let's also reserve the 2 lowest bits of "Q" samples
        // for future extensions and keep them as 0.
        vi &= 0xFFFFFFFCL;
        vq &= 0xFFFFFFFCL;
        if (fi * fi + fq * fq >= tx_threshold2)
            vi |= 0b11L;
        dest_[i]     = vi;
        dest_[i + 1] = vq;
    }
}

#define MAX_REGS 0x80

// Number of initial register values for SX1255.
// Only write the documented registers from 0x00 to 0x13.
#define N_INIT_REGISTERS 0x14
// Initial register values for SX1255.
static const uint8_t init_registers[N_INIT_REGISTERS] = {
    // 0x00: default value from datasheet (enable oscillator)
    0b00000001,
    // 0x01-0x06: RX and TX frequencies: 433.92 MHz
    0xD8,
    0xF5,
    0xC3,
    0xD8,
    0xF5,
    0xC3,
    // 0x07: read-only register, written value does not matter
    0x11,
    // 0x08: TX gains, default value from datasheet
    0b00101110,
    // 0x09: default value from datasheet
    0b00100100,
    // 0x0A: default value from datasheet
    0b00110000,
    // 0x0B: default value from datasheet
    0b00000010,
    // 0x0C: RX gains, default value from datasheet
    0b00111111,
    // 0x0D: RX filters, make them narrow
    // ADCTRIM value of 7 minimized ADC spurs
    // but maybe 6 worked better with 38.4 MHz clock in some cases.
    0b00111011,
    // 0x0E: default value from datasheet
    0b00000110,
    // 0x0F: IO_MAP, default value from datasheet
    0b00000000,
    // 0x10, CK_SEL, default value from datasheet
    0b00000010,
    // 0x11: read-only register(?), written value does not matter
    0b00000000,
    // 0x12-0x13: I2S at 125 kHz: CLKOUT divider 4, decimate by 256
    0b00100010,
    0b00101100,
};

// Register values for a given sample rate
struct sampleRateRegs {
    // Ratio of reference clock to sample rate
    uint16_t div;
    // iism_clk_div value (register 0x12 bits 3-0)
    uint8_t clkout : 4;
    // int_dec_mantisse value (register 0x13 bit 7)
    uint8_t mant : 1;
    // int_dec_m_parameter value (register 0x13 bit 6)
    uint8_t m : 1;
    // int_dec_n_parameter value (register 0x13 bits 5-3)
    uint8_t n : 3;
};

#define N_SAMPLE_RATES 6

// Register values for different sample rates
static const struct sampleRateRegs sample_rates[N_SAMPLE_RATES] = {
    {1536, 0b0110, 0, 1, 6},
    {768, 0b0100, 0, 1, 5},
    {512, 0b0011, 0, 0, 6},
    //{ 384, 0b0011, 0, 1, 4 }, // 24 bit samples (did not work correctly)
    {256, 0b0010, 0, 0, 5},
    //{ 192, 0b0010, 0, 1, 3 }, // 24 bit samples (did not work correctly)
    {128, 0b0001, 0, 0, 4},
    //{  96, 0b0001, 0, 1, 2 }, // 24 bit samples (did not work correctly)
    {64, 0b0000, 0, 0, 3},
    //{  48, 0b0000, 0, 1, 1 }, // 24 bit samples (did not work correctly)
    //{  32, 0b0000, 0, 0, 2 }, // 16 bit samples (did not work)
};

// Class to use SPI through the SPI userspace API (SPIDEV) in Linux.
// Putting it in a separate class helps use RAII to ensure
// the file descriptor gets closed in all situations.
class Spi {
private:
    // SPIDEV file descriptor
    int fd;

public:
    Spi(const char *spidev_path) {
        assert(spidev_path != NULL);
        fd = open(spidev_path, O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", fd);
        if (fd < 0) {
            // TODO: add more detailed error messages from strerror or something
            throw std::runtime_error("Failed to open SPI");
        }
    }

    ~Spi() {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Closing SPIDEV fd %d", fd);
        close(fd);
    }

    int transfer(const uint8_t *tx_data, uint8_t *rx_data, const size_t len) const {
        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));

        transfer.tx_buf   = (__u64)tx_data;
        transfer.rx_buf   = (__u64)rx_data;
        transfer.len      = (__u32)len;
        transfer.speed_hz = 10000000;

        int ret           = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV ioctl: %d", ret);

        if ((size_t)ret != len)
            throw std::runtime_error("SPI transfer failed");

        return ret;
    }

    int transfer(const std::vector<uint8_t> &tx_data, std::vector<uint8_t> &rx_data) const {
        // Make sure data fits in both buffers
        const size_t transfer_len = std::min(tx_data.size(), rx_data.size());
        return transfer(tx_data.data(), rx_data.data(), transfer_len);
    }
};

// Similar to class Spi, but for GPIO pins.
class GpioChip {
public:
    int chip_fd;

    GpioChip(const char *gpiochip_path) {
        assert(gpiochip_path != NULL);
        chip_fd = open(gpiochip_path, O_RDWR);
        if (chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO");
        }
    }

    ~GpioChip() {
        close(chip_fd);
    }
};

class GpioLine {
public:
    int line_fd;

    GpioLine(class GpioChip &chip, __u32 pin_number, const char *name, __u64 flags, bool initial_value) {
        struct gpio_v2_line_request req = {
            .offsets  = {pin_number},
            .consumer = "",
            .config   = (struct gpio_v2_line_config){
                .flags     = flags,
                .num_attrs = 0,
                .padding   = {},
                .attrs     = {},
            },
            .num_lines         = 1,
            .event_buffer_size = 0,
            .padding           = {},
            .fd                = 0,
        };
        strncpy(req.consumer, name, GPIO_MAX_NAME_SIZE - 1);

        int ret = ioctl(chip.chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
        if (ret < 0) {
            throw std::runtime_error("Failed to request GPIO line");
        }

        line_fd = req.fd;

        set_value(initial_value);
    }

    ~GpioLine() {
        close(line_fd);
    }

    void set_value(bool value) {
        struct gpio_v2_line_values val = {
            .bits = value,
            .mask = 1,
        };
        int ret = ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &val);
        if (ret < 0) {
            throw std::runtime_error("Failed to write GPIO line");
        }
    }
};

// Convert an ALSA recording error to a corresponding SoapySDR readStream error.
static int alsa_error_to_soapy_rx(int alsa_error_code) {
    if (alsa_error_code == -EPIPE) {
        // Overrun
        return SOAPY_SDR_OVERFLOW;
    } else {
        // Some other error
        return SOAPY_SDR_STREAM_ERROR;
    }
}

// Convert an ALSA playback error to a corresponding SoapySDR writeStream error.
static int alsa_error_to_soapy_tx(int alsa_error_code) {
    if (alsa_error_code == -EPIPE) {
        // Underrun
        return SOAPY_SDR_UNDERFLOW;
    } else {
        // Some other error
        return SOAPY_SDR_STREAM_ERROR;
    }
}

#define ALSACHECK(a)                                                                                \
    do {                                                                                            \
        int retcheck = (a);                                                                         \
        if (retcheck < 0) {                                                                         \
            SoapySDR_logf(SOAPY_SDR_ERROR, "\nALSA error in %s: %s\n", #a, snd_strerror(retcheck)); \
            goto alsa_error;                                                                        \
        }                                                                                           \
    } while (0)

class AlsaPcm {
public:
    const char *name;
    snd_pcm_t *pcm;
    mutable std::mutex mutex;
    snd_pcm_stream_t dir;
    enum stream_mode stream_mode;
    bool setup_done;
    bool activated;
    bool primed;
    int64_t position;
    snd_pcm_uframes_t hwp_period_size;
    snd_pcm_uframes_t hwp_buffer_size;

    AlsaPcm(const char *name, snd_pcm_stream_t dir) : name(name),
                                                      pcm(NULL),
                                                      dir(dir),
                                                      stream_mode(STREAM_MODE_NORMAL),
                                                      primed(0),
                                                      setup_done(0),
                                                      activated(0),
                                                      position(0),
                                                      hwp_period_size(0),
                                                      hwp_buffer_size(0) {
    }

    void open(void) {
        ALSACHECK(snd_pcm_open(&pcm, name, dir, 0));
        return;

    alsa_error:
        if (pcm != NULL)
            snd_pcm_close(pcm);

        // TODO more detailed error messages?
        throw std::runtime_error("Error opening ALSA device");
    }

    ~AlsaPcm() {
        if (pcm != NULL)
            snd_pcm_close(pcm);
    }

    bool is_tx(void) {
        return dir == SND_PCM_STREAM_PLAYBACK;
    }

    int reset() {
        int ret = 0;
        // Make sure stream is stopped. If it was stopped already,
        // drop will fail but that is fine, so do not check the return value.
        snd_pcm_drop(pcm);

        ret = snd_pcm_prepare(pcm);
        if (ret < 0)
            return ret;
        position = 0;
        ret      = snd_pcm_reset(pcm);
        return ret;
    }

    void configure(snd_pcm_uframes_t period) {
        if (pcm == NULL)
            return;
        snd_pcm_hw_params_t *hwp = NULL;
        snd_pcm_sw_params_t *swp = NULL;

        unsigned int hwp_periods = 0;

        // Period size basically determines the I2S DMA interrupt rate.
        // A higher period size somewhat reduces CPU use
        // but increases the minimum latency.
        // If an application reads samples in fixed size blocks,
        // it is usually best to use a period a size equal to the block size
        // to minimize latency while avoiding unnecessary CPU use.
        // Period size can configured using the "period" stream argument.
        // Use default value if 0.
        hwp_period_size = period > 0 ? period : 256;

        // Based on some tests requesting various period and buffer sizes,
        // it seems like, on a Raspberry Pi:
        // * Maximum buffer size is 65536.
        // * Buffer size should be a multiple of period size.
        //   Otherwise ALSA forces period size to be a submultiple of buffer size.
        // * There should be at least 2 periods per buffer, but we do not really need
        //   to check for this. ALSA handles it nicely.
        // * Minimum period size is 2 but this does not really need
        //   a separate check either.
        // Given these constraints, find the maximum possible buffer size
        // to achieve the period requested.
        const snd_pcm_uframes_t max_buffer_size = 65536;
        hwp_period_size                         = std::min(hwp_period_size, max_buffer_size);
        hwp_buffer_size                         = max_buffer_size / hwp_period_size * hwp_period_size;

        snd_pcm_uframes_t swp_boundary          = 0;

        ALSACHECK(snd_pcm_hw_params_malloc(&hwp));
        ALSACHECK(snd_pcm_hw_params_any(pcm, hwp));
        ALSACHECK(snd_pcm_hw_params_set_access(pcm, hwp, SND_PCM_ACCESS_RW_INTERLEAVED));
        ALSACHECK(snd_pcm_hw_params_set_format(pcm, hwp, SND_PCM_FORMAT_S32_LE));
        // Sample rate given to ALSA does not affect the actual sample rate,
        // so just use a fixed "dummy" value.
        ALSACHECK(snd_pcm_hw_params_set_rate(pcm, hwp, 192000, 0));
        ALSACHECK(snd_pcm_hw_params_set_channels(pcm, hwp, 2));
        ALSACHECK(snd_pcm_hw_params_set_buffer_size_near(pcm, hwp, &hwp_buffer_size));
        ALSACHECK(snd_pcm_hw_params_set_period_size_near(pcm, hwp, &hwp_period_size, 0));
        ALSACHECK(snd_pcm_hw_params_get_periods(hwp, &hwp_periods, 0));

        ALSACHECK(snd_pcm_hw_params(pcm, hwp));
        snd_pcm_hw_params_free(hwp);

        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA parameters: buffer_size=%d, period_size=%d, periods=%d", hwp_buffer_size, hwp_period_size, hwp_periods);

        ALSACHECK(snd_pcm_sw_params_malloc(&swp));
        ALSACHECK(snd_pcm_sw_params_current(pcm, swp));
        ALSACHECK(snd_pcm_sw_params_get_boundary(swp, &swp_boundary));
        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA SW parameters: boundary=%lu", swp_boundary);
        if (stream_mode == STREAM_MODE_NORMAL) {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, swp_boundary));
            // https://stackoverflow.com/a/20515251
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, swp_boundary));
        } else {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, hwp_buffer_size));
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, 0));
        }

        ALSACHECK(snd_pcm_sw_params(pcm, swp));
        snd_pcm_sw_params_free(swp);

        ALSACHECK(reset());

        return;

    alsa_error:
        if (hwp != NULL)
            snd_pcm_hw_params_free(hwp);
        if (swp != NULL)
            snd_pcm_sw_params_free(swp);
        // TODO more detailed error messages?
        throw std::runtime_error("Error configuring ALSA device");
    }
};

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapyMuCell : public SoapySDR::Device {
private:
    double masterClock;
    double sampleRate;

    // Mutex for anything involving SX1255 registers
    // to avoid problems if an application calls methods from multiple threads.
    mutable std::recursive_mutex reg_mutex;

    Spi spi;
    GpioChip gpio;
    GpioLine gpio_reset, gpio_rx, gpio_tx;
    AlsaPcm alsa_rx;
    AlsaPcm alsa_tx;

    // Transmitter is turned on when squared magnitude of a TX sample
    // exceeds this threshold.
    float tx_threshold2;
    // If true, RX and TX streams have been linked using snd_pcm_link
    bool linked;

    // Values of registers (to be) written to the chip.
    // Storing them here makes it easier and faster to change
    // specific bits since they do not need to be read
    // from the chip every time.
    uint8_t regs[MAX_REGS];

    // Buffer for RX samples before type conversion.
    // Data is not actually uint64_t but it simplifies code a bit
    // by making one vector element correspond to one 32+32 bit complex sample.
    std::vector<uint64_t> buffer_rx;
    // Buffer for TX samples after type conversion.
    std::vector<uint64_t> buffer_tx;

    struct hat_info hat_info;

    // Convert a SoapySDR nanosecond timestamp to a sample counter.
    int64_t timestamp_to_samples(long long timestamp) const {
        return SoapySDR::timeNsToTicks(timestamp, sampleRate);
    }

    // Convert a sample counter to a SoapySDR nanosecond timestamp.
    long long samples_to_timestamp(int64_t samples) const {
        return SoapySDR::ticksToTimeNs(samples, sampleRate);
    }

    // Set given bits of a register.
    // The registers are not actually written to the chip.
    void set_register_bits(size_t address, unsigned lowestbit, unsigned nbits, unsigned value) {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        regs[address] = (regs[address] & (~mask)) | ((value << lowestbit) & mask);
    }

    // Get given bits of a cached register.
    // The registers are not actually read from the chip.
    unsigned get_cached_register_bits(size_t address, unsigned lowestbit, unsigned nbits) const {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        return (regs[address] & mask) >> lowestbit;
    }

    // Write a range of registers to the chip.
    void write_registers_to_chip(size_t firstreg, size_t nregs) {
        if ((firstreg >= MAX_REGS) || (nregs > MAX_REGS) || (firstreg > MAX_REGS - nregs))
            throw std::runtime_error("Invalid register address");

        size_t transfer_len = nregs + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = firstreg | 0x80;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = regs[firstreg + i - 1];

        spi.transfer(buf, buf);
    }

    void reset_chip(void) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Resetting chip");
        // Timing from datasheet Figure 6-2: Manual Reset Timing Diagram
        gpio_reset.set_value(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        gpio_reset.set_value(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void init_chip(void) {
        for (size_t i = 0; i < N_INIT_REGISTERS; i++)
            set_register_bits(i, 0, 8, init_registers[i]);
        // Enable RX and TX, just for initial testing. This should be done somewhere else.
        set_register_bits(0, 1, 3, 0b111);
        write_registers_to_chip(0, N_INIT_REGISTERS);
    }

    bool does_synth_tune(double frequency) {
        setFrequency(SOAPY_SDR_RX, 0, frequency, {});
        setFrequency(SOAPY_SDR_TX, 0, frequency, {});
        // Give some time for PLL to lock
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto status_register = readRegister("", 0x11);
        return (status_register & 3) == 3;
    }

    void detect_clock(void) {
        // Try to detect whether SX1255 clock is 32 MHz or 38.4 MHz
        // by tuning near edges of tuning range.
        // First assume it is 32 MHz and try tuning accordingly.
        masterClock = 32.0e6;
        // If clock is 38.4 MHz instead, 510 MHz ends up at 612 MHz
        // where synthesizers (hopefully) do not lock anymore.
        bool tunes_high = does_synth_tune(510e6);
        // Synthesizers do not (hopefully) go down to 330 MHz, but
        // if clock is 38.4 MHz instead, they end up at 396 MHz.
        bool tunes_low = does_synth_tune(330e6);
        if (tunes_low && (!tunes_high)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 38.4 MHz");
            masterClock = 38.4e6;
        } else if (tunes_high && (!tunes_low)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 32.0 MHz");
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "Clock detection failed, assuming 38.4 MHz");
            masterClock = 38.4e6;
        }

        // Update default values for new masterClock value
        sampleRate = masterClock / 256.0;
        setFrequency(SOAPY_SDR_RX, 0, 433.92e6, {});
        setFrequency(SOAPY_SDR_TX, 0, 433.92e6, {});
    }

    /***********************************************************************
     * Initialization and destruction
     **********************************************************************/

public:
    SoapyMuCell(const SoapySDR::Kwargs &args, struct hat_info hat_info) : masterClock(32.0e6),
                                                                          sampleRate(125.0e3),

                                                                          spi("/dev/spidev0.0"),

                                                                          gpio("/dev/gpiochip0"),
                                                                          gpio_reset(gpio,
                                                                                     5,
                                                                                     "SX reset",
                                                                                     GPIO_V2_LINE_FLAG_OUTPUT | GPIO_V2_LINE_FLAG_OPEN_SOURCE,
                                                                                     0),
                                                                          gpio_rx(gpio,
                                                                                  23,
                                                                                  "SX RX",
                                                                                  GPIO_V2_LINE_FLAG_OUTPUT,
                                                                                  1),
                                                                          gpio_tx(gpio,
                                                                                  22,
                                                                                  "SX TX",
                                                                                  GPIO_V2_LINE_FLAG_OUTPUT,
                                                                                  1),

                                                                          alsa_rx(AlsaPcm("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE)),
                                                                          alsa_tx(AlsaPcm("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK)),
                                                                          tx_threshold2(0.0f),
                                                                          linked(false),
                                                                          regs{0},

                                                                          // Allocate reasonably large buffers by default.
                                                                          // Their size will be increased if needed.
                                                                          buffer_rx(8192),
                                                                          buffer_tx(8192),

                                                                          hat_info(hat_info) {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapyMuCell");

        reset_chip();
        init_chip();
        detect_clock();
        // Open ALSA devices now when I2S clocks are already running.
        // I am not sure if this makes any difference but just in case.
        alsa_rx.open();
        alsa_tx.open();
    }

    ~SoapyMuCell(void) {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapyMuCell");

        // Put SX1255 to sleep
        set_register_bits(0, 0, 4, 0);
        write_registers_to_chip(0, 1);

        // Make sure PA is turned off
        writeSetting("PA", "OFF");

        // Turn off RX and TX LEDs / RF switches
        gpio_rx.set_value(0);
        gpio_tx.set_value(0);
    }

    /***********************************************************************
     * Sample streams
     **********************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args) {
        (void)channels; // Only one channel
        (void)args;     // Unused for now

        std::scoped_lock lock(alsa_rx.mutex, alsa_tx.mutex);

        if (format != "CF32")
            throw std::runtime_error("Only CF32 format is currently supported");
        if (
            (snd_pcm_state(alsa_rx.pcm) == SND_PCM_STATE_RUNNING) ||
            (snd_pcm_state(alsa_tx.pcm) == SND_PCM_STATE_RUNNING))
            throw std::runtime_error("Streams can be setup only if none of the streams are running");

        auto *stream = direction == SOAPY_SDR_RX ? &alsa_rx : &alsa_tx;

        // Allow setting up only one stream per direction.
        if (stream->setup_done)
            throw std::runtime_error("Stream has been setup already");

        if (stream->is_tx()) {
            const float tx_threshold_default = 1.0e-3;
            // Enable TX when magnitude of a sample exceeds a given threshold.
            float tx_threshold =
                (args.count("threshold") > 0)
                    ? std::stof(args.at("threshold"))
                    : tx_threshold_default;
            tx_threshold2 = tx_threshold * tx_threshold;
        }

        bool arg_link       = (args.count("link") > 0 && args.at("link") == "1");
        stream->stream_mode = arg_link ? STREAM_MODE_LINK : STREAM_MODE_NORMAL;

        stream->configure(args.count("period") > 0 ? std::stoul(args.at("period")) : 0);

        stream->setup_done = 1;

        // Link RX and TX PCMs only in LINK mode. In NORMAL mode the
        // application manages timing via timestamps; linking starts TX
        // (with an empty buffer) as soon as RX starts, which underruns
        // into corrupted stream accounting and hangs the first
        // avail_delay/read (issue #2).
        if ((!linked) && alsa_rx.setup_done && alsa_tx.setup_done && alsa_rx.stream_mode == STREAM_MODE_LINK && alsa_tx.stream_mode == STREAM_MODE_LINK) {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Linking streams");
            ALSACHECK(snd_pcm_link(alsa_rx.pcm, alsa_tx.pcm));
            linked = 1;
        }

        return reinterpret_cast<SoapySDR::Stream *>(stream);
    alsa_error:
        // TODO more detailed error messages?
        throw std::runtime_error("ALSA error");
    }

    void closeStream(SoapySDR::Stream *handle) {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        stream->setup_done = 0;
    }

    int activateStream(
        SoapySDR::Stream *handle,
        const int flags,
        const long long timeNs,
        const size_t numElems) {
        (void)flags;
        (void)timeNs;
        (void)numElems;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already activated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 1;
        SoapySDR_logf(SOAPY_SDR_INFO, "activateStream: %s (mode %d, state %d)",
                      stream->is_tx() ? "TX" : "RX",
                      (int)stream->stream_mode, (int)snd_pcm_state(stream->pcm));

        if (stream->stream_mode == STREAM_MODE_NORMAL) {
            if (stream->is_tx() && !stream->primed) {
                // Prime the playback buffer with silence. RX and TX PCMs are
                // linked, so starting RX (activated first by most clients)
                // also starts TX with an empty buffer: the underrun opens
                // immediately, pcm_delay goes negative and getHardwareTime
                // reads far into the future (issue #2). Prime regardless of
                // PCM state: writei on a RUNNING stream fills available space,
                // which during the underrun is the whole buffer. Zeros are
                // valid silence (the RF switch bit is clear).
                std::vector<int32_t> silence(stream->hwp_buffer_size * 2);
                snd_pcm_sframes_t primed = snd_pcm_writei(
                    stream->pcm, silence.data(), stream->hwp_buffer_size);
                if (primed < 0) {
                    SoapySDR_logf(SOAPY_SDR_ERROR,
                                  "TX priming failed: %s", snd_strerror(primed));
                } else {
                    stream->position += primed;
                    stream->primed = 1;
                    SoapySDR_logf(SOAPY_SDR_INFO,
                                  "Primed TX with %d frames of silence (state %d)",
                                  (int)primed, (int)snd_pcm_state(stream->pcm));
                }
            }
            if (snd_pcm_state(stream->pcm) == SND_PCM_STATE_PREPARED) {
                ALSACHECK(snd_pcm_start(stream->pcm));
            }
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    int deactivateStream(
        SoapySDR::Stream *handle,
        const int flags,
        const long long timeNs) {
        (void)flags;
        (void)timeNs;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (!stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already deactivated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 0;

        // If both streams have been deactivated, stop them.
        if ((!alsa_rx.activated) && (!alsa_tx.activated)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Stopping and resetting streams");
            ALSACHECK(alsa_rx.reset());
            ALSACHECK(alsa_tx.reset());
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    size_t getStreamMTU(SoapySDR::Stream *handle) const {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        return stream->hwp_period_size;
    }

    int readStream(
        SoapySDR::Stream *handle,
        void *const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs) {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        flags = 0;

        if (stream->is_tx())
            throw std::runtime_error("Wrong direction");

        // Always report the timestamp of the next sample, like SoapySX:
        // rust-soapysdr clients detect timestamp availability by change,
        // and the stack anchors initial_time from it (issue #2).
        timeNs = samples_to_timestamp(stream->position);
        flags |= SOAPY_SDR_HAS_TIME;

        if (!stream->activated)
            return 0;

        snd_pcm_t *pcm = stream->pcm;
        if (numElems * 2 > buffer_rx.size())
            buffer_rx.resize(numElems * 2);

        size_t buff_offset = 0;
        int ret            = 0;
        while (buff_offset < numElems) {
            snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;

            // snd_pcm_avail_delay issues a status/hwsync ioctl that wedges
            // on this kernel while the capture stream is overruning (issue #2).
            // arecord (readi -> snd_pcm_avail_update) works fine on the same
            // stream; use the pure hw_ptr arithmetic instead.
            pcm_avail = snd_pcm_avail_update(pcm);
            pcm_delay = 0;
            ret       = (pcm_avail < 0) ? (int)pcm_avail : 0;
            fprintf(stderr, "RS avail ret=%d avail=%ld\n", ret, (long)pcm_avail);
            fflush(stderr);

            if (ret < 0)
                break;

            // RX buffer overrun: hw_ptr ran ahead of the app because
            // NORMAL mode's stop_threshold=boundary lets the stream run
            // through overruns (only 0.34 s of buffer at 192 kHz, and the
            // stack may not read for seconds during init). avail then
            // exceeds the buffer size, and readi wedges on the runaway
            // accounting. Skip to the live edge like the v1.0 driver did
            // (issue #2): forward past the stale samples, then read fresh.
            if (pcm_avail > (snd_pcm_sframes_t)stream->hwp_buffer_size) {
                snd_pcm_uframes_t overwritten =
                    (snd_pcm_uframes_t)pcm_avail - stream->hwp_buffer_size;
                snd_pcm_uframes_t skip =
                    (overwritten / stream->hwp_period_size + 2) * stream->hwp_period_size;
                snd_pcm_sframes_t forwarded = snd_pcm_forward(pcm, skip);
                if (forwarded < 0) {
                    ret = (int)forwarded;
                    break;
                }
                stream->position += forwarded;
                SoapySDR_logf(SOAPY_SDR_WARNING,
                              "RX overrun: skipped %d stale samples", (int)forwarded);
                continue;
            }

            size_t n = numElems - buff_offset;

            if (pcm_avail <= 0 || (size_t)pcm_avail < n) {
                // Wait for more data, bounded by the requested timeout
                // (-1 = indefinitely, like SoapySX when no timeout given).
                int wait_ms = (timeoutUs > 0) ? (int)(timeoutUs / 1000) : -1;
                ret         = snd_pcm_wait(pcm, wait_ms);
                if (ret == 0)
                    break; // timed out; return what we have
                if (ret < 0)
                    break;
                continue;
            }

            snd_pcm_sframes_t samples_read = snd_pcm_readi(pcm, buffer_rx.data(), n);
            if (samples_read < 0) {
                ret = (int)samples_read;
                break;
            }
            convert_rx_buffer(buffer_rx.data(), 0, buffs[0], buff_offset, samples_read);
            buff_offset += samples_read;
            stream->position += samples_read;
        }

        if (ret == -EPIPE) // Overrun
            return SOAPY_SDR_OVERFLOW;
        if (ret < 0)
            return SOAPY_SDR_STREAM_ERROR;

        return buff_offset;
    }

    int writeStream(
        SoapySDR::Stream *handle,
        const void *const *buffs,
        const size_t numElems,
        int &flags,
        long long timeNs,
        const long timeoutUs) {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        if (!stream->is_tx())
            throw std::runtime_error("Wrong direction");

        if (!stream->activated)
            return 0;

        snd_pcm_t *pcm              = stream->pcm;
        snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
        // Avoid the status ioctl (see readStream): approximate the queued
        // delay as the filled part of the buffer, which for playback is
        // buffer_size - free_space.
        pcm_avail = snd_pcm_avail_update(pcm);
        if (pcm_avail < 0)
            return alsa_error_to_soapy_tx((int)pcm_avail);
        pcm_delay = (snd_pcm_sframes_t)(stream->hwp_buffer_size - (snd_pcm_uframes_t)pcm_avail);
        if (pcm_delay < 0)
            pcm_delay = 0;
        int ret                   = 0;

        int64_t playback_position = stream->position - (int64_t)pcm_delay;
        int64_t write_position;

        if (flags & SOAPY_SDR_HAS_TIME) {
            write_position = timestamp_to_samples(timeNs);
            if (playback_position - write_position > 0) {
                // Timestamp in the past: discard, pretend written
                return (int)numElems;
            }
        } else {
            write_position = stream->position;
        }

        // Forward the stream to write_position, blocking as needed
        // like SoapySX.
        int64_t posdiff = write_position - stream->position;
        while (posdiff > 0) {
            snd_pcm_sframes_t forwardable = snd_pcm_forwardable(pcm);
            if (forwardable < 0)
                return SOAPY_SDR_STREAM_ERROR;
            if (posdiff < (int64_t)forwardable)
                forwardable = snd_pcm_forward(pcm, (snd_pcm_sframes_t)posdiff);
            else
                forwardable = snd_pcm_forward(pcm, forwardable);
            if (forwardable < 0)
                return SOAPY_SDR_STREAM_ERROR;
            stream->position += forwardable;
            posdiff -= forwardable;
            if (posdiff > 0) {
                int w = snd_pcm_wait(pcm, (timeoutUs > 0) ? (int)(timeoutUs / 1000) : -1);
                if (w == 0)
                    return 0; // timed out
                if (w < 0)
                    return SOAPY_SDR_STREAM_ERROR;
            }
        }

        // Write until the full buffer is in, blocking like SoapySX.
        if (numElems * 2 > buffer_tx.size())
            buffer_tx.resize(numElems * 2);
        size_t buff_offset = 0;
        while (buff_offset < numElems) {
            pcm_avail = snd_pcm_avail_update(pcm);
            if (pcm_avail < 0) {
                ret = (int)pcm_avail;
                break;
            }
            size_t n = numElems - buff_offset;
            if (pcm_avail <= 0 || (size_t)pcm_avail < n) {
                ret = snd_pcm_wait(pcm, (timeoutUs > 0) ? (int)(timeoutUs / 1000) : -1);
                if (ret == 0)
                    break; // timed out; return partial
                if (ret < 0)
                    break;
                continue;
            }
            convert_tx_buffer(buffs[0], buff_offset, buffer_tx.data(), 0, n, tx_threshold2);
            snd_pcm_sframes_t written = snd_pcm_writei(pcm, buffer_tx.data(), n);
            if (written < 0) {
                ret = (int)written;
                break;
            }
            buff_offset += written;
            stream->position += written;
        }

        if (ret == -EPIPE) // Underrun
            return SOAPY_SDR_UNDERFLOW;
        if (ret < 0)
            return SOAPY_SDR_STREAM_ERROR;
        return buff_offset;
    }

    long long getHardwareTime(const std::string &what) const {
        if (what == "") {
            // RX and TX streams should give similar results here.
            //
            // Use TX stream to avoid locking the RX stream mutex.
            // Reasoning is that getHardwareTime seems most useful
            // for determining timing of producing TX signals,
            // and some applications might run RX and TX in different threads,
            // we want to avoid locks between them, and I guess
            // getHardwareTime is most likely to be called from a TX thread.
            //
            // The mutex is needed to avoid race conditions between updating
            // our stream->position and the ALSA internal stream position,
            // in case getHardwareTime gets called from some other thread
            // while writeStream is running.

            auto *stream = &alsa_tx;
            std::scoped_lock lock(stream->mutex);
            snd_pcm_t *pcm              = stream->pcm;

            snd_pcm_sframes_t pcm_avail = 0;
            // Same wedging status ioctl as the read path (issue #2): use
            // snd_pcm_avail_update and approximate the playback delay as
            // the filled part of the buffer.
            pcm_avail = snd_pcm_avail_update(pcm);
            if (pcm_avail < 0)
                throw std::runtime_error("ALSA error");
            snd_pcm_sframes_t pcm_delay =
                (snd_pcm_sframes_t)stream->hwp_buffer_size - pcm_avail;
            if (pcm_delay < 0)
                pcm_delay = 0;

            return samples_to_timestamp(stream->position - (int64_t)pcm_delay);
        } else {
            throw std::runtime_error("Unsupported time");
        }
    }

    /***********************************************************************
     * Sample rates
     **********************************************************************/

    std::vector<double> listSampleRates(const int direction, const size_t channel) const {
        (void)direction;
        (void)channel;
        std::vector<double> sampleRates;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            sampleRates.push_back(masterClock / (double)sample_rates[i].div);
        }
        return sampleRates;
    }

    // Wrapper for listSampleRates to support both methods
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const {
        const auto sampleRates = listSampleRates(direction, channel);
        SoapySDR::RangeList ranges;
        for (const auto sampleRate : sampleRates) {
            ranges.push_back({sampleRate, sampleRate, 0});
        }
        return ranges;
    }

    void setSampleRate(
        const int direction,
        const size_t channel,
        const double rate) {
        (void)direction;
        (void)channel;

        std::scoped_lock lock(reg_mutex);

        if (rate != rate || rate <= 0)
            throw std::runtime_error("Sample rate must be positive");

        double divider = round(masterClock / rate);
        struct sampleRateRegs r;
        bool found = false;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            r = sample_rates[i];
            if ((double)r.div == divider) {
                found = true;
                break;
            }
        }
        if (!found) {
            throw std::runtime_error("Unsupported sample rate");
        }
        // Disable RX and TX before changing sample rate.
        // Changing sample rate while RX/TX was running sometimes seemed
        // to break ordering of data over I2S, causing things like swapped
        // left and right channels or otherwise corrupted data.
        // This seems to fix the problem.
        set_register_bits(0x00, 1, 2, 0);
        write_registers_to_chip(0x00, 1);
        // Change the sample rate
        set_register_bits(0x12, 0, 4, r.clkout);
        set_register_bits(0x13, 7, 1, r.mant);
        set_register_bits(0x13, 6, 1, r.m);
        set_register_bits(0x13, 3, 3, r.n);
        write_registers_to_chip(0x12, 2);
        sampleRate = masterClock / divider;
        // Enable RX and TX again
        set_register_bits(0x00, 1, 2, 3);
        write_registers_to_chip(0x00, 1);
    }

    double getSampleRate(
        const int direction,
        const size_t channel) const {
        (void)direction;
        (void)channel;
        std::scoped_lock lock(reg_mutex);
        return sampleRate;
    }

    /***********************************************************************
     * Center frequency
     **********************************************************************/

    void setFrequency(
        const int direction,
        const size_t channel,
        const double frequency,
        const SoapySDR::Kwargs &args) {
        (void)channel;
        (void)args;

        std::scoped_lock lock(reg_mutex);

        const double step        = masterClock * (1.0 / (double)(1L << 20));
        const uint32_t quantized = (uint32_t)scale_from_range(
            SoapySDR::Range(0, step * (double)((1L << 24) - 1), step),
            frequency);
        if (direction == SOAPY_SDR_RX) {
            set_register_bits(0x01, 0, 8, quantized >> 16);
            set_register_bits(0x02, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x03, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x01, 3);
        } else {
            set_register_bits(0x04, 0, 8, quantized >> 16);
            set_register_bits(0x05, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x06, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x04, 3);
        }
    }

    double getFrequency(
        const int direction,
        const size_t channel) const {
        (void)channel;

        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L << 20));
        if (direction == SOAPY_SDR_RX)
            return step * ((((uint32_t)regs[1]) << 16) |
                           (((uint32_t)regs[2]) << 8) |
                           ((uint32_t)regs[3]));
        else
            return step * ((((uint32_t)regs[4]) << 16) |
                           (((uint32_t)regs[5]) << 8) |
                           ((uint32_t)regs[6]));
    }

    /***********************************************************************
     * Gains
     **********************************************************************/

    std::vector<std::string> listGains(
        const int direction,
        const size_t channel) const {
        (void)channel;
        if (direction == SOAPY_SDR_RX)
            return std::vector<std::string>{"LNA", "PGA"};
        else
            return std::vector<std::string>{"DAC", "MIXER"};
    }

    SoapySDR::Range getGainRange(
        const int direction,
        const size_t channel,
        const std::string &name) const {
        (void)channel;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") return {0.0, 48.0, 6.0};
            if (name == "PGA") return {0.0, 30.0, 2.0};
        } else {
            if (name == "DAC") return {0.0, 9.0, 3.0};
            if (name == "MIXER") return {0.0, 30.0, 2.0};
        }
        return {0, 0, 0};
    }

    void setGain(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double value) {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = scale_from_range(getGainRange(direction, channel, name), value);
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                // LNA gain does not have a constant step,
                // so some extra logic is needed.
                if (quantized <= 6) // -48 to -12 dB
                    set_register_bits(0x0C, 5, 3, 6 - quantized / 2);
                else if (quantized == 7) // -6 dB
                    set_register_bits(0x0C, 5, 3, 2);
                else // 0 dB
                    set_register_bits(0x0C, 5, 3, 1);
            } else if (name == "PGA") {
                set_register_bits(0x0C, 1, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RXFE1=0x%02x", regs[0x0C]);
            write_registers_to_chip(0x0C, 1);
        } else {
            if (name == "DAC") {
                set_register_bits(0x08, 4, 3, quantized);
            } else if (name == "MIXER") {
                set_register_bits(0x08, 0, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "TXFE1=0x%02x", regs[0x08]);
            write_registers_to_chip(0x08, 1);
        }
    }

    double getGain(
        const int direction,
        const size_t channel,
        const std::string &name) const {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = 0;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                const int32_t map[8] = {0, 8, 7, 6, 4, 2, 0, 0};
                quantized            = map[get_cached_register_bits(0x0C, 5, 3)];
            } else if (name == "PGA") {
                quantized = get_cached_register_bits(0x0C, 1, 4);
            }
        } else {
            if (name == "DAC") {
                quantized = get_cached_register_bits(0x08, 4, 3);
            } else if (name == "MIXER") {
                quantized = get_cached_register_bits(0x08, 0, 4);
            }
        }
        return scale_to_range(getGainRange(direction, channel, name), quantized);
    }

    void setGain(
        const int direction,
        const size_t channel,
        const double value) {
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            // Keep PGA gain around pga_gain_target over most of the
            // gain range while adjusting LNA gain over a wide range.
            // PGA gain has a smaller step, so use it to fine tune gain.
            const double pga_gain_target = 12.0;
            setGain(direction, channel, "LNA", value - pga_gain_target);
            double lna_gain = getGain(direction, channel, "LNA");
            setGain(direction, channel, "PGA", value - lna_gain);
        } else {
            // Not sure about best TX gain distribution yet.
            // Use similar logic as RX gains for now.
            const double mixer_gain_target = 26.0;
            setGain(direction, channel, "DAC", value - mixer_gain_target);
            double dac_gain = getGain(direction, channel, "DAC");
            setGain(direction, channel, "MIXER", value - dac_gain);
        }
    }

    /***********************************************************************
     * Antennas
     **********************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const {
        (void)channel; // Only one channel
        std::vector<std::string> antennas;
        if (direction == SOAPY_SDR_RX) {
            antennas.push_back("RX");
            antennas.push_back("LB");
            // Digital loopback did not seem to work, so do not list it
            // antennas.push_back("DLB");
        } else {
            antennas.push_back("TX");
            antennas.push_back("NONE");
        }
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name) {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            if (name == "RX") {
                // Disable loopback
                set_register_bits(0x10, 2, 2, 0);
            } else if (name == "LB") {
                // RF loopback
                set_register_bits(0x10, 2, 2, 1);
            } else if (name == "DLB") {
                // Digital loopback
                set_register_bits(0x10, 2, 2, 3);
            }
            write_registers_to_chip(0x10, 1);
        } else {
            if (name == "TX") {
                // Enable PA
                set_register_bits(0x00, 3, 1, 1);
            } else if (name == "NONE") {
                // Disable PA
                set_register_bits(0x00, 3, 1, 0);
            }
            write_registers_to_chip(0x00, 1);
        }
    }

    std::string getAntenna(const int direction, const size_t channel) const {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            unsigned lb = get_cached_register_bits(0x10, 2, 2);
            if (lb & 2)
                return "DLB";
            if (lb & 1)
                return "LB";
            return "RX";
        } else {
            if (get_cached_register_bits(0x00, 3, 1)) {
                // PA is enabled
                return "TX";
            } else {
                // PA is disabled
                return "NONE";
            }
        }
    }

    /***********************************************************************
     * Other settings
     **********************************************************************/

    void writeSetting(
        const std::string &key,
        const std::string &value) {
        // PA control modes
        if (key == "PA") {
            if (value == "ON") {
                // PA always on
                gpio_tx.set_value(1);
                gpio_rx.set_value(0);
            } else if (value == "OFF") {
                // PA always off
                gpio_tx.set_value(0);
                gpio_rx.set_value(1);
            } else if (value == "AUTO") {
                // PA on/off controlled by TX stream (default)
                gpio_tx.set_value(1);
                gpio_rx.set_value(1);
            }
        }
    }

    // TODO: readSetting, getSettingInfo

    /***********************************************************************
     * Low level interfaces
     **********************************************************************/

    std::vector<unsigned> readRegisters(
        const std::string &name,
        const unsigned addr,
        const size_t length) const {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        size_t transfer_len = length + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = addr;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = 0;

        spi.transfer(buf, buf);

        std::vector<unsigned> result(length, 0);
        for (size_t i = 0; i < length; i++)
            result[i] = buf[i + 1];

        return result;
    }

    unsigned readRegister(
        const std::string &name,
        const unsigned addr) const {
        auto r = readRegisters(name, addr, 1);
        return r.at(0);
    }

    void writeRegisters(
        const std::string &name,
        const unsigned addr,
        const std::vector<unsigned> &value) {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        for (size_t i = 0; i < value.size(); i++)
            set_register_bits(addr + i, 0, 8, value[i]);

        write_registers_to_chip(addr, value.size());
    }

    void writeRegister(
        const std::string &name,
        const unsigned addr,
        const unsigned value) {
        (void)name;
        std::scoped_lock lock(reg_mutex);
        set_register_bits(addr, 0, 8, value);
        write_registers_to_chip(addr, 1);
    }

    /***********************************************************************
     * Other hardware and driver information
     **********************************************************************/

    std::string getDriverKey(void) const {
        return DRIVER_KEY_STR;
    }

    std::string getHardwareKey(void) const {
        return HARDWARE_KEY_STR;
    }

    SoapySDR::Kwargs getHardwareInfo(void) const {
        SoapySDR::Kwargs args;
        args["soapysx_tag"]    = SoapyMuCell_tag;
        args["soapysx_commit"] = SoapyMuCell_commit;

        if (hat_info.read_success) {
            args["hardware_version"] = std::to_string(hat_info.product_ver >> 8) + "." + std::to_string(hat_info.product_ver & 0xFF);
        } else {
            args["hardware_version"] = "unknown";
        }

        // EEPROM vendor info
        const EepromInfo &e = hat_info.eeprom;
        if (e.has_vendor_info) {
            args["eeprom_vendor"]  = e.vendor_str;
            args["eeprom_product"] = e.product_str;
            args["eeprom_uuid"]    = e.uuid;
        }
        if (e.has_serial)
            args["eeprom_serial"] = e.serial;

        // Authenticity
        switch (e.auth) {
        case AuthResult::OK:
            args["eeprom_auth"]     = "genuine";
            args["eeprom_auth_key"] = e.matched_key_name;
            break;
        case AuthResult::NO_SIGNATURE:
            args["eeprom_auth"] = "unsigned";
            break;
        case AuthResult::KEY_NOT_FOUND:
            args["eeprom_auth"] = "mismatch";
            break;
        case AuthResult::NO_KEYS:
            args["eeprom_auth"] = "no_keys";
            break;
        case AuthResult::EEPROM_NOT_FOUND:
            args["eeprom_auth"] = "not_found";
            break;
        case AuthResult::VERIFY_ERROR:
            args["eeprom_auth"] = "error";
            break;
        case AuthResult::PARSE_ERROR:
            args["eeprom_auth"] = "parse_error";
            break;
        }

        return args;
    }

    size_t getNumChannels(const int direction) const {
        (void)direction; // Same for both directions
        return 1;
    }

    std::string getNativeStreamFormat(
        const int direction,
        const size_t channel,
        double &fullScale) const {
        (void)direction;
        (void)channel;
        fullScale = 1;
        // This is not really the "native" format
        // but the only one currently supported.
        return "CF32";
    }

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
        (void)direction;
        (void)channel;
        std::vector<std::string> streamFormats;
        streamFormats.push_back("CF32");
        return streamFormats;
    }

    bool hasHardwareTime(const std::string &what) const {
        if (what == "")
            return true;
        return false;
    }
};

/***********************************************************************
 * Find available devices
 **********************************************************************/
static SoapySDR::KwargsList findDevice(const SoapySDR::Kwargs &args) {
    (void)args;
    SoapySDR::KwargsList devices;

    // TODO: check whether a device is actually found

    SoapySDR::Kwargs device;
    device["label"]  = DEVICE_LABEL_STR;
    device["driver"] = DEVICE_DRIVER_STR;
    devices.push_back(device);

    return devices;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *makeDevice(const SoapySDR::Kwargs &args) {
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyMuCell version %s %s", SoapyMuCell_tag, SoapyMuCell_commit);
    return new SoapyMuCell(args, read_hat_info());
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerDevice(DEVICE_DRIVER_STR, &findDevice, &makeDevice, SOAPY_SDR_ABI_VERSION);
