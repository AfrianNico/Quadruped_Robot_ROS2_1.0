#include <cstring>
#include <driver/spi_slave.h>
#include "driver/CommandInputSpi.h"

static spi_slave_transaction_t spiTransaction_;
static bool spiInitialized_ = false;

#ifndef WORD_ALIGNED_ATTR
#if defined(__GNUC__)
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#else
#define WORD_ALIGNED_ATTR
#endif
#endif

static WORD_ALIGNED_ATTR uint8_t spiRecvBuf_[32];

CommandInputSpi::CommandInputSpi(ILogger &logger, BatterySharedState &batteryState, ModeSharedState &modeState)
    : logger_(logger), batteryState_(batteryState), modeState_(modeState)
{
    std::memset(recvBuf_, 0, BUF_SIZE);
    std::memset(txBuf_, 0, BUF_SIZE);
    std::memset(spiRecvBuf_, 0, sizeof(spiRecvBuf_));
}

void CommandInputSpi::fillTelemetryTxBuffer()
{
    TelemetryPacket pkt;
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;

    BatteryState batState;
    batState.voltage = 0.0f;
    batState.current = 0.0f;
    batState.power = 0.0f;
    batState.socPercent = 0.0f;
    batteryState_.get(batState);

    pkt.voltage = batState.voltage; // V
    pkt.current = batState.current; // A
    pkt.socPercent = static_cast<uint8_t>(constrain(batState.socPercent, 0.0f, 100.0f) + 0.5f);
    pkt.robotBehavior = behaviorCode_;
    pkt.crc = 0;

    uint16_t crc = 0xFFFF;
    uint8_t *raw = reinterpret_cast<uint8_t *>(&pkt);
    for (size_t i = 0; i < 12; i++)
    {
        crc ^= raw[i];
        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    pkt.crc = crc;
    std::memset(txBuf_, 0, BUF_SIZE);
    std::memcpy(txBuf_, &pkt, sizeof(TelemetryPacket));
}

void CommandInputSpi::begin()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = GPIO_MOSI;
    buscfg.miso_io_num = GPIO_MISO;
    buscfg.sclk_io_num = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num = GPIO_CS;
    slvcfg.flags = 0;
    slvcfg.queue_size = 1;
    slvcfg.mode = 0; // SPI mode 0 (CPOL=0, CPHA=0)

    esp_err_t err = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
    if (err != ESP_OK)
    {
        logger_.error("SPI", "spi_slave_initialize failed");
        return;
    }

    spiInitialized_ = true;
    queueTransaction();

    logger_.info("SPI", "ESP32 SPI slave initialized on VSPI_HOST");
}

void CommandInputSpi::queueTransaction()
{
    if (!spiInitialized_ || transactionQueued_)
        return;

    fillTelemetryTxBuffer();

    std::memset(spiRecvBuf_, 0, sizeof(spiRecvBuf_));
    std::memset(&spiTransaction_, 0, sizeof(spiTransaction_));

    spiTransaction_.length = sizeof(spiRecvBuf_) * 8; // in bits
    spiTransaction_.rx_buffer = spiRecvBuf_;
    spiTransaction_.tx_buffer = txBuf_;

    if (spi_slave_queue_trans(VSPI_HOST, &spiTransaction_, 0) == ESP_OK)
    {
        transactionQueued_ = true;
    }
}

bool CommandInputSpi::parseMessage(const char *msg, Command &command)
{
    if (std::strcmp(msg, "c:forward") == 0)
    {
        command = Command::FORWARD;
        return true;
    }

    if (std::strcmp(msg, "c:backward") == 0)
    {
        command = Command::BACKWARD;
        return true;
    }

    if (std::strcmp(msg, "c:right") == 0)
    {
        command = Command::TURN_RIGHT;
        return true;
    }

    if (std::strcmp(msg, "c:left") == 0)
    {
        command = Command::TURN_LEFT;
        return true;
    }

    if (std::strcmp(msg, "c:stop") == 0)
    {
        command = Command::NONE;
        return true;
    }

    return false;
}

bool CommandInputSpi::update(Command &command, Mode &mode)
{
    if (!spiInitialized_)
        return false;

    if (!transactionQueued_)
    {
        queueTransaction();
        return false;
    }

    spi_slave_transaction_t *outT = nullptr;
    esp_err_t err = spi_slave_get_trans_result(VSPI_HOST, &outT, 0);

    if (err != ESP_OK)
        return false;

    transactionQueued_ = false;
    lastTransactionMs_ = millis();

    std::memcpy(recvBuf_, spiRecvBuf_, BUF_SIZE - 1);
    recvBuf_[BUF_SIZE - 1] = '\0';

    for (size_t i = 0; recvBuf_[i] != '\0' && i < BUF_SIZE; ++i)
    {
        if (recvBuf_[i] >= 'A' && recvBuf_[i] <= 'Z')
            recvBuf_[i] = recvBuf_[i] - 'A' + 'a';
    }

    if (std::strcmp(recvBuf_, "m:auto") == 0)
    {
        logger_.info("SPI", "Change Mode to AUTO");
        mode = Mode::AUTO;
        modeState_.set(Mode::AUTO);
        queueTransaction();
        return true;
    }

    if (std::strcmp(recvBuf_, "m:manual") == 0)
    {
        logger_.info("SPI", "Change Mode to MANUAL");
        mode = Mode::MANUAL;
        modeState_.set(Mode::MANUAL);
        command = Command::NONE;
        lastAutoCommand_ = Command::NONE;
        queueTransaction();
        return true;
    }

    if (mode == Mode::AUTO)
    {
        Command parsed = Command::NONE;
        if (parseMessage(recvBuf_, parsed))
        {
            lastAutoCommand_ = parsed;
            command = parsed;
            queueTransaction();
            return true;
        }
        if (lastAutoCommand_ != Command::NONE)
        {
            command = lastAutoCommand_;
            queueTransaction();
            return true;
        }
    }

    if (lastTransactionMs_ != 0 && millis() - lastTransactionMs_ > WATCHDOG_TIMEOUT_MS)
    {
        logger_.warn("SPI", "Watchdog timeout: no SPI transaction, stopping");
        command = Command::NONE;
        lastAutoCommand_ = Command::NONE;
        queueTransaction();
        return true;
    }

    queueTransaction();
    return false;
}