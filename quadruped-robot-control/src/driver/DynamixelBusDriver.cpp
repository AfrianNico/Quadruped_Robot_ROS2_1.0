#include "DynamixelBusDriver.h"
#include "config/pin_config.h"
#include "config/servo_config.h"
#include "config/system_config.h"

/**
 * @brief Protocol 2.0 instruction codes.
 */
namespace
{
    constexpr uint8_t INST_READ = 0x02;
    constexpr uint8_t STATUS_OK = 0x00;

    /**
     * @brief Receives a status packet response from a Dynamixel servo.
     *
     * Sits in a timed read loop expecting exactly expectedLen bytes.
     * A valid Protocol 2.0 status packet looks like:
     *   FF FF FD 00  ID  LEN_L LEN_H  [0x55 instr]  [ERR]  [PARAM...]  CRC_L CRC_H
     *
     * @param port Reference to the HardwareSerial UART.
     * @param id Expected servo ID in the response.
     * @param buf  Buffer to store the full packet (including header).
     * @param expectedLen Total packet length expected (header + payload + CRC).
     * @return Number of bytes read on success, -1 on timeout/crc/ID mismatch.
     */
    int readResponse(HardwareSerial &port, uint8_t id, uint8_t *buf, uint8_t expectedLen)
    {
        const unsigned long deadline = millis() + READ_TIMEOUT_MS;
        uint8_t idx = 0;

        while (millis() < deadline && idx < expectedLen)
        {
            if (port.available())
            {
                buf[idx++] = port.read();
            }
        }

        if (idx < expectedLen)
            return -1; // timeout

        // Validate header: 0xFF 0xFF 0xFD 0x00
        if (buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != 0xFD || buf[3] != 0x00)
            return -1;

        // Validate ID (accept broadcast 0xFE as well)
        if (buf[4] != id && buf[4] != 0xFE)
            return -1;

        // Validate instruction = 0x55 (status)
        const uint8_t instrIdx = STATUS_HEADER_LEN - 4; // 6
        if (buf[instrIdx] != 0x55)
            return -1;

        // Validate error byte (must be 0x00 for a successful read)
        if (buf[instrIdx + 1] != STATUS_OK)
            return -1;

        // Validate CRC — covers bytes 0 through expectedLen-3
        const uint16_t calcCrc = DynamixelBusDriver::updateCRC(0, buf, expectedLen - 2);
        const uint16_t rcvCrc = static_cast<uint16_t>(buf[expectedLen - 2]) |
                                (static_cast<uint16_t>(buf[expectedLen - 1]) << 8);
        if (calcCrc != rcvCrc)
            return -1;

        return idx;
    }
}

DynamixelBusDriver::DynamixelBusDriver(HardwareSerial &port, uint8_t dirPin)
    : port_(port), dirPin_(dirPin)
{
}

void DynamixelBusDriver::begin(int txPin, int rxPin)
{
    pinMode(dirPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
    port_.begin(DXL_BAUDRATE, SERIAL_8N1, rxPin, txPin);
}

uint16_t DynamixelBusDriver::updateCRC(uint16_t crcAccum, const uint8_t *data, uint16_t size)
{
    for (uint16_t j = 0; j < size; j++)
    {
        crcAccum ^= static_cast<uint16_t>(data[j] << 8);
        for (uint16_t i = 0; i < 8; i++)
        {
            if (crcAccum & 0x8000)
            {
                crcAccum = (crcAccum << 1) ^ 0x8005;
            }
            else
            {
                crcAccum <<= 1;
            }
        }
    }
    return crcAccum;
}

void DynamixelBusDriver::sendPacket(uint8_t *packet, uint8_t len)
{
    digitalWrite(dirPin_, HIGH);
    port_.write(packet, len);
    port_.flush();
    digitalWrite(dirPin_, LOW);
}

void DynamixelBusDriver::write1Byte(uint8_t id, uint16_t addr, uint8_t value)
{
    uint8_t packet[13] = {
        0xFF, 0xFF, 0xFD, 0x00,
        id,
        0x06, 0x00,
        0x03,
        static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>((addr >> 8) & 0xFF),
        value,
        0x00, 0x00};

    const uint16_t crc = updateCRC(0, packet, 11);
    packet[11] = static_cast<uint8_t>(crc & 0xFF);
    packet[12] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sendPacket(packet, 13);
}

void DynamixelBusDriver::write4Byte(uint8_t id, uint16_t addr, uint32_t value)
{
    uint8_t packet[16] = {
        0xFF, 0xFF, 0xFD, 0x00,
        id,
        0x09, 0x00,
        0x03,
        static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>((addr >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF),
        0x00, 0x00};

    const uint16_t crc = updateCRC(0, packet, 14);
    packet[14] = static_cast<uint8_t>(crc & 0xFF);
    packet[15] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sendPacket(packet, 16);
}

void DynamixelBusDriver::torqueEnable(uint8_t id, bool enable)
{
    write1Byte(id, ADDR_TORQUE_ENABLE, enable ? 1 : 0);
}

void DynamixelBusDriver::setProfileAccel(uint8_t id, uint32_t accel)
{
    write4Byte(id, ADDR_PROFILE_ACCELERATION, accel);
}

void DynamixelBusDriver::setProfileVelocity(uint8_t id, uint32_t vel)
{
    write4Byte(id, ADDR_PROFILE_VELOCITY, vel);
}

int32_t DynamixelBusDriver::readPresentPosition(uint8_t id)
{
    // Build read instruction packet: header + id + length(7) + INST_READ + addr(2) + size(2)
    uint8_t packet[14] = {
        0xFF, 0xFF, 0xFD, 0x00,
        id,
        0x07, 0x00, // length = 7
        INST_READ,  // 0x02
        static_cast<uint8_t>(ADDR_PRESENT_POSITION & 0xFF),
        static_cast<uint8_t>((ADDR_PRESENT_POSITION >> 8) & 0xFF),
        0x04, 0x00,  // read 4 bytes
        0x00, 0x00}; // CRC placeholder

    const uint16_t crc = updateCRC(0, packet, 12);
    packet[12] = static_cast<uint8_t>(crc & 0xFF);
    packet[13] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sendPacket(packet, 14);

    // Expected response: header(4) + reserved(1) + id(1) + length(2) + instr(1) + err(1) + data(4) + crc(2) = 16
    uint8_t response[16];
    const int rc = readResponse(port_, id, response, 16);
    if (rc < 0)
        return INT32_MIN;

    // Payload data bytes at indices 8..11 (after header + instr + err)
    // instr at offset 6, err at offset 7, data starts at offset 8
    const int32_t position = static_cast<int32_t>(response[8]) |
                             (static_cast<int32_t>(response[9]) << 8) |
                             (static_cast<int32_t>(response[10]) << 16) |
                             (static_cast<int32_t>(response[11]) << 24);
    return position;
}

bool DynamixelBusDriver::isMoving(uint8_t id)
{
    // Build read instruction packet for 1 byte at ADDR_MOVING
    uint8_t packet[14] = {
        0xFF, 0xFF, 0xFD, 0x00,
        id,
        0x07, 0x00,
        INST_READ,
        static_cast<uint8_t>(ADDR_MOVING & 0xFF),
        static_cast<uint8_t>((ADDR_MOVING >> 8) & 0xFF),
        0x01, 0x00, // read 1 byte
        0x00, 0x00};

    const uint16_t crc = updateCRC(0, packet, 12);
    packet[12] = static_cast<uint8_t>(crc & 0xFF);
    packet[13] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sendPacket(packet, 14);

    // Expected response: header(4) + reserved(1) + id(1) + length(2) + instr(1) + err(1) + data(1) + crc(2) = 13
    uint8_t response[13];
    const int rc = readResponse(port_, id, response, 13);
    if (rc < 0)
        return false;

    // data byte at index 8 (after header + instr + err)
    return (response[8] != 0);
}

void DynamixelBusDriver::writeLegGoal(int32_t posHip, int32_t posKnee, uint8_t idHip, uint8_t idKnee)
{
    uint8_t packet[24] = {
        0xFF, 0xFF, 0xFD, 0x00,
        0xFE,
        0x11, 0x00,
        0x83,
        0x74, 0x00,
        0x04, 0x00,

        idHip,
        0, 0, 0, 0,

        idKnee,
        0, 0, 0, 0,

        0, 0};

    const int32_t hipValue = posHip + 2048;
    packet[13] = static_cast<uint8_t>(hipValue & 0xFF);
    packet[14] = static_cast<uint8_t>((hipValue >> 8) & 0xFF);
    packet[15] = static_cast<uint8_t>((hipValue >> 16) & 0xFF);
    packet[16] = static_cast<uint8_t>((hipValue >> 24) & 0xFF);

    const int32_t kneeValue = posKnee + KNEE_CENTER_OFFSET(idKnee);
    packet[18] = static_cast<uint8_t>(kneeValue & 0xFF);
    packet[19] = static_cast<uint8_t>((kneeValue >> 8) & 0xFF);
    packet[20] = static_cast<uint8_t>((kneeValue >> 16) & 0xFF);
    packet[21] = static_cast<uint8_t>((kneeValue >> 24) & 0xFF);

    const uint16_t crc = updateCRC(0, packet, 22);
    packet[22] = static_cast<uint8_t>(crc & 0xFF);
    packet[23] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sendPacket(packet, 24);
}

void DynamixelBusDriver::writeLegSpeedRaw(int32_t speedHip, int32_t speedKnee, uint8_t idHip, uint8_t idKnee)
{
    uint8_t packet[24] = {
        0xFF, 0xFF, 0xFD, 0x00,
        0xFE,
        0x11, 0x00,
        0x83,
        0x70, 0x00,
        0x04, 0x00,

        idHip,
        0, 0, 0, 0,

        idKnee,
        0, 0, 0, 0,

        0, 0};

    packet[13] = static_cast<uint8_t>(speedHip & 0xFF);
    packet[14] = static_cast<uint8_t>((speedHip >> 8) & 0xFF);
    packet[15] = static_cast<uint8_t>((speedHip >> 16) & 0xFF);
    packet[16] = static_cast<uint8_t>((speedHip >> 24) & 0xFF);

    packet[18] = static_cast<uint8_t>(speedKnee & 0xFF);
    packet[19] = static_cast<uint8_t>((speedKnee >> 8) & 0xFF);
    packet[20] = static_cast<uint8_t>((speedKnee >> 16) & 0xFF);
    packet[21] = static_cast<uint8_t>((speedKnee >> 24) & 0xFF);

    const uint16_t crc = updateCRC(0, packet, 22);
    packet[22] = static_cast<uint8_t>(crc & 0xFF);
    packet[23] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sendPacket(packet, 24);
}