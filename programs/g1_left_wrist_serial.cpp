#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

namespace {

constexpr char kCmdTopic[] = "rt/lowcmd";
constexpr char kStateTopic[] = "rt/lowstate";
constexpr char kDefaultSerialPort[] = "/dev/ttyACM0";
constexpr int kDefaultBaudRate = 115200;

constexpr std::uint8_t kHeader0 = 0xAA;
constexpr std::uint8_t kHeader1 = 0x55;
constexpr std::size_t kFloatCount = 14;
constexpr std::size_t kPayloadBytes = 2 + kFloatCount * sizeof(float);
constexpr std::size_t kPacketBytes = 2 + kPayloadBytes + 2;
constexpr std::size_t kLegacyPacketBytes = 2 + kFloatCount * sizeof(float);
constexpr double kCommandDtSec = 0.001;
constexpr double kPrintPeriodSec = 2.0;
constexpr float kPosStopF = 2.146E+9F;
constexpr float kVelStopF = 16000.0F;
constexpr float kWristKp = 24.0F;
constexpr float kWristKd = 1.6F;
constexpr std::size_t kAverageWindowSize = 4;

constexpr int kNumMotors = 35;
constexpr int kJointLeftWristRoll = 19;
constexpr int kJointLeftWristPitch = 20;
constexpr int kJointLeftWristYaw = 21;
constexpr std::array<int, 3> kPacketLeftWristIndices = {4, 5, 6};
constexpr std::array<int, 3> kLeftWristJointIndices = {
    kJointLeftWristRoll,
    kJointLeftWristPitch,
    kJointLeftWristYaw,
};

struct Packet {
    std::uint16_t sequence = 0;
    std::array<float, kFloatCount> values{};
    std::chrono::steady_clock::time_point received_at{};
};

float Clamp(float value, float lower, float upper) {
    return std::max(lower, std::min(upper, value));
}

float Average(const std::array<float, kAverageWindowSize> &values) {
    float sum = 0.0F;
    for (float value : values) {
        sum += value;
    }
    return sum / static_cast<float>(kAverageWindowSize);
}

std::uint16_t Crc16Ccitt(const std::uint8_t *data, std::size_t len) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

std::uint32_t Crc32Core(std::uint32_t *ptr, std::uint32_t len) {
    std::uint32_t xbit = 0;
    std::uint32_t data = 0;
    std::uint32_t crc = 0xFFFFFFFF;
    constexpr std::uint32_t kPolynomial = 0x04C11DB7;

    for (std::uint32_t i = 0; i < len; i++) {
        xbit = 1U << 31;
        data = ptr[i];
        for (std::uint32_t bits = 0; bits < 32; bits++) {
            if ((crc & 0x80000000U) != 0U) {
                crc <<= 1;
                crc ^= kPolynomial;
            } else {
                crc <<= 1;
            }

            if ((data & xbit) != 0U) {
                crc ^= kPolynomial;
            }
            xbit >>= 1;
        }
    }

    return crc;
}

speed_t ResolveBaud(int baud_rate) {
    switch (baud_rate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        default:
            throw std::runtime_error("Unsupported baud rate");
    }
}

class SerialPort {
public:
    SerialPort(const std::string &path, int baud_rate) {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port " + path + ": " + std::strerror(errno));
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            Close();
            throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
        }

        cfmakeraw(&tty);
        const speed_t speed = ResolveBaud(baud_rate);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            Close();
            throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
        }

        tcflush(fd_, TCIFLUSH);
    }

    ~SerialPort() { Close(); }

    ssize_t Read(std::uint8_t *dst, std::size_t size) {
        return read(fd_, dst, size);
    }

private:
    void Close() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    int fd_ = -1;
};

class SerialPacketReader {
public:
    explicit SerialPacketReader(SerialPort &serial) : serial_(serial) {}

    std::optional<Packet> ReadLatestPacket() {
        ReadIntoBuffer();
        std::optional<Packet> latest;

        while (buffer_.size() >= kLegacyPacketBytes) {
            const std::size_t header_index = FindHeader();
            if (header_index == buffer_.size()) {
                stats_.desync_bytes += buffer_.size();
                buffer_.clear();
                break;
            }

            if (header_index > 0) {
                stats_.desync_bytes += header_index;
                buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header_index));
            }

            if (buffer_.size() < kLegacyPacketBytes) {
                break;
            }

            if (buffer_.size() >= kPacketBytes) {
                const std::uint16_t expected_crc =
                    static_cast<std::uint16_t>(buffer_[kPacketBytes - 2] | (buffer_[kPacketBytes - 1] << 8));
                const std::uint16_t actual_crc = Crc16Ccitt(buffer_.data() + 2, kPayloadBytes);
                if (actual_crc == expected_crc) {
                    Packet packet{};
                    std::memcpy(&packet.sequence, buffer_.data() + 2, sizeof(packet.sequence));
                    std::memcpy(packet.values.data(), buffer_.data() + 4, packet.values.size() * sizeof(float));
                    packet.received_at = std::chrono::steady_clock::now();
                    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(kPacketBytes));
                    if (SequenceIsNew(packet.sequence) && ValuesFinite(packet.values)) {
                        stats_.valid_packets++;
                        latest = packet;
                    }
                    continue;
                }
                stats_.crc_failures++;
            }

            Packet legacy{};
            std::memcpy(legacy.values.data(), buffer_.data() + 2, legacy.values.size() * sizeof(float));
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(kLegacyPacketBytes));
            legacy.sequence = has_sequence_ ? static_cast<std::uint16_t>(last_sequence_ + 1) : 0;
            legacy.received_at = std::chrono::steady_clock::now();

            if (ValuesFinite(legacy.values)) {
                stats_.legacy_packets++;
                if (SequenceIsNew(legacy.sequence)) {
                    latest = legacy;
                }
            } else {
                stats_.invalid_packets++;
            }
        }

        PrintStatusIfNeeded();
        return latest;
    }

private:
    struct Stats {
        std::size_t valid_packets = 0;
        std::size_t legacy_packets = 0;
        std::size_t crc_failures = 0;
        std::size_t invalid_packets = 0;
        std::size_t stale_packets = 0;
        std::size_t desync_bytes = 0;
    };

    void ReadIntoBuffer() {
        std::array<std::uint8_t, 256> chunk{};
        while (true) {
            const ssize_t bytes_read = serial_.Read(chunk.data(), chunk.size());
            if (bytes_read > 0) {
                buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + bytes_read);
                continue;
            }

            if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                throw std::runtime_error("Serial read failed: " + std::string(std::strerror(errno)));
            }
            break;
        }
    }

    std::size_t FindHeader() const {
        for (std::size_t i = 0; i + 1 < buffer_.size(); ++i) {
            if (buffer_[i] == kHeader0 && buffer_[i + 1] == kHeader1) {
                return i;
            }
        }
        return buffer_.size();
    }

    bool SequenceIsNew(std::uint16_t sequence) {
        if (!has_sequence_) {
            has_sequence_ = true;
            last_sequence_ = sequence;
            return true;
        }

        const std::uint16_t delta = static_cast<std::uint16_t>(sequence - last_sequence_);
        if (delta == 0 || delta > 0x8000U) {
            stats_.stale_packets++;
            return false;
        }

        last_sequence_ = sequence;
        return true;
    }

    bool ValuesFinite(const std::array<float, kFloatCount> &values) const {
        for (float value : values) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return true;
    }

    void PrintStatusIfNeeded() {
        const double now = std::chrono::duration<double>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        if (now - last_print_time_ < kPrintPeriodSec) {
            return;
        }

        std::cout << "serial stats:"
                  << " valid=" << stats_.valid_packets
                  << " legacy=" << stats_.legacy_packets
                  << " crc_fail=" << stats_.crc_failures
                  << " invalid=" << stats_.invalid_packets
                  << " stale=" << stats_.stale_packets
                  << " desync=" << stats_.desync_bytes
                  << std::endl;
        last_print_time_ = now;
    }

    SerialPort &serial_;
    std::vector<std::uint8_t> buffer_;
    Stats stats_{};
    bool has_sequence_ = false;
    std::uint16_t last_sequence_ = 0;
    double last_print_time_ = 0.0;
};

class G1LeftWristSerialBridge {
public:
    void Init() {
        InitLowCmd();

        lowcmd_publisher_.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kCmdTopic));
        lowcmd_publisher_->InitChannel();

        lowstate_subscriber_.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kStateTopic));
        lowstate_subscriber_->InitChannel(
            std::bind(&G1LeftWristSerialBridge::LowStateMessageHandler, this, std::placeholders::_1), 1);
    }

    void WaitForFirstState() {
        std::cout << "Waiting for robot state..." << std::endl;
        while (!have_state_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "Got robot state." << std::endl;
    }

    void ApplyPacket(const Packet &packet) {
        for (std::size_t i = 0; i < kLeftWristJointIndices.size(); ++i) {
            const int joint = kLeftWristJointIndices[i];
            const float raw_target = packet.values[kPacketLeftWristIndices[i]];
            if (!have_filtered_targets_) {
                input_history_[i].fill(raw_target);
            } else {
                input_history_[i][input_history_index_] = raw_target;
            }
            const float filtered_target = Average(input_history_[i]);

            const float predicted_target = filtered_target;
            last_targets_[i] = raw_target;
            filtered_targets_[i] = filtered_target;

            low_cmd_.motor_cmd()[joint].mode() = 1;
            low_cmd_.motor_cmd()[joint].q() = predicted_target;
            low_cmd_.motor_cmd()[joint].dq() = 0.0F;
            low_cmd_.motor_cmd()[joint].kp() = kWristKp;
            low_cmd_.motor_cmd()[joint].kd() = kWristKd;
            low_cmd_.motor_cmd()[joint].tau() = 0.0F;
        }

        have_filtered_targets_ = true;
        input_history_index_ = (input_history_index_ + 1) % kAverageWindowSize;

        if (!reported_packet_) {
            std::cout << std::fixed << std::setprecision(3)
                      << "First packet mapped to left wrist:"
                      << " roll=" << last_targets_[0]
                      << " pitch=" << last_targets_[1]
                      << " yaw=" << last_targets_[2]
                      << std::endl;
            reported_packet_ = true;
        }
    }

    void Publish() {
        low_cmd_.crc() = Crc32Core(reinterpret_cast<std::uint32_t *>(&low_cmd_),
                                   (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        lowcmd_publisher_->Write(low_cmd_);
    }

private:
    void InitLowCmd() {
        low_cmd_.mode_pr() = 0;
        low_cmd_.mode_machine() = 0;

        for (int i = 0; i < kNumMotors; ++i) {
            low_cmd_.motor_cmd()[i].mode() = 1;
            low_cmd_.motor_cmd()[i].q() = kPosStopF;
            low_cmd_.motor_cmd()[i].dq() = kVelStopF;
            low_cmd_.motor_cmd()[i].kp() = 0.0F;
            low_cmd_.motor_cmd()[i].kd() = 0.0F;
            low_cmd_.motor_cmd()[i].tau() = 0.0F;
        }
    }

    void LowStateMessageHandler(const void *message) {
        low_state_ = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
        have_state_ = true;
    }

    unitree_hg::msg::dds_::LowCmd_ low_cmd_{};
    unitree_hg::msg::dds_::LowState_ low_state_{};
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
    bool have_state_ = false;
    bool have_filtered_targets_ = false;
    bool reported_packet_ = false;
    std::array<float, 3> last_targets_{0.0F, 0.0F, 0.0F};
    std::array<float, 3> filtered_targets_{0.0F, 0.0F, 0.0F};
    std::array<std::array<float, kAverageWindowSize>, 3> input_history_{};
    std::size_t input_history_index_ = 0;
};

}  // namespace

int main(int argc, const char **argv) {
    const std::string serial_port = argc >= 2 ? argv[1] : kDefaultSerialPort;
    const bool sim_mode = argc < 3;
    const char *network_interface = sim_mode ? "lo" : argv[2];
    const int domain_id = sim_mode ? 1 : 0;

    try {
        ChannelFactory::Instance()->Init(domain_id, network_interface);
        SerialPort serial(serial_port, kDefaultBaudRate);
        SerialPacketReader packet_reader(serial);
        G1LeftWristSerialBridge bridge;

        std::cout << "Opened serial port " << serial_port << std::endl;
        std::cout << "DDS mode: " << (sim_mode ? "sim" : "robot")
                  << " domain=" << domain_id
                  << " interface=" << network_interface << std::endl;

        bridge.Init();
        bridge.WaitForFirstState();

        std::cout << "Controlling left wrist joints from potentiometer packets." << std::endl;
        std::cout << "Packet mapping: left[4]->roll, left[5]->pitch, left[6]->yaw" << std::endl;

        auto next_tick = std::chrono::steady_clock::now();
        while (true) {
            const auto packet = packet_reader.ReadLatestPacket();
            if (packet.has_value()) {
                bridge.ApplyPacket(*packet);
            }

            bridge.Publish();
            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(kCommandDtSec));
            std::this_thread::sleep_until(next_tick);
        }
    } catch (const std::exception &ex) {
        std::cerr << ex.what() << std::endl;
        std::cerr << "Usage: ./g1_left_wrist_serial [serial_port] [network_interface]" << std::endl;
        std::cerr << "Example sim:   ./g1_left_wrist_serial /dev/ttyACM0" << std::endl;
        std::cerr << "Example robot: ./g1_left_wrist_serial /dev/ttyACM0 enp2s0" << std::endl;
        return 1;
    }

    return 0;
}
