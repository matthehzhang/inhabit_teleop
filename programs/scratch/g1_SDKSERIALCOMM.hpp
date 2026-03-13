  #pragma once

  #include <array>
  #include <chrono>
  #include <cstdint>
  #include <optional>
  #include <string>

  struct Packet {
    uint16_t sequence;
    std::array<float, 14> values;
    std::chrono::steady_clock::time_point
  received_at;
  };

  struct WristTargets {
    float roll;
    float pitch;
    float yaw;
  };

  class SerialPort {
  public:
    SerialPort(const std::string& path, int
  baud_rate);
    ~SerialPort();
    ssize_t Read(uint8_t* dst, std::size_t size);

  private:
    int fd_;
  };

  class SerialPacketReader {
  public:
    explicit SerialPacketReader(SerialPort&
  serial);
    std::optional<Packet> ReadLatestPacket();

  private:
    SerialPort& serial_;
  };
