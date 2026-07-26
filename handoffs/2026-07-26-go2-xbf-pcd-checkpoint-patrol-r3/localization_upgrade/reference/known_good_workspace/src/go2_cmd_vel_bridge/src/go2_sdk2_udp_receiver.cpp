#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace unitree::robot;

namespace
{
constexpr uint32_t kCmdPacketMagic = 0x4732434dU;  // "G2CM"
constexpr uint16_t kCmdPacketVersion = 2;

#pragma pack(push, 1)
struct CmdPacket
{
  uint32_t magic;
  uint16_t version;
  uint16_t packet_size;
  uint64_t sequence;
  uint64_t send_steady_ns;
  uint64_t send_system_ns;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack(pop)

struct Summary
{
  Summary(double average_in = 0.0, double p95_in = 0.0, double maximum_in = 0.0)
  : average(average_in), p95(p95_in), maximum(maximum_in) {}

  double average;
  double p95;
  double maximum;
};

Summary summarize(std::vector<double> values)
{
  if (values.empty()) return Summary();
  std::sort(values.begin(), values.end());
  const auto p95_index = std::min(
    values.size() - 1,
    static_cast<size_t>(std::ceil(values.size() * 0.95)) - 1);
  const double total = std::accumulate(values.begin(), values.end(), 0.0);
  return Summary(total / values.size(), values[p95_index], values.back());
}

uint64_t steady_now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

int main(int argc, char **argv)
{
  std::string net_interface = "eth0";
  int port = 5005;

  if (argc >= 2) net_interface = argv[1];
  if (argc >= 3) port = std::stoi(argv[2]);

  ChannelFactory::Instance()->Init(0, net_interface);

  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);
  sport_client.Init();

  std::cout << "SDK2 receiver started on interface: " << net_interface
            << std::endl;
  std::cout << "UDP listen port: " << port
            << ", packet_v=" << kCmdPacketVersion << std::endl;
  std::cout << "SDK command limits: max_vx=0.500 max_vy=0.100 "
            << "max_vyaw=0.500" << std::endl;

  sport_client.StandUp();
  sleep(2);
  sport_client.BalanceStand();
  sleep(1);

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "socket failed errno=" << errno << std::endl;
    return 2;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    std::cerr << "bind failed port=" << port << " errno=" << errno
              << std::endl;
    close(sock);
    return 3;
  }

  // Wake periodically even if the upstream chain stops, so cmd_age is visible.
  timeval receive_timeout{};
  receive_timeout.tv_sec = 1;
  receive_timeout.tv_usec = 0;
  setsockopt(
    sock, SOL_SOCKET, SO_RCVTIMEO,
    &receive_timeout, sizeof(receive_timeout));

  CmdPacket pkt{};
  uint64_t last_receive_ns = 0;
  uint64_t last_sequence = 0;
  uint64_t report_started_ns = steady_now_ns();
  uint64_t last_alert_ns = 0;
  uint64_t receive_count = 0;
  uint64_t sequence_gaps = 0;
  uint64_t invalid_packets = 0;
  float last_vx = 0.0f;
  float last_vy = 0.0f;
  float last_vyaw = 0.0f;
  std::vector<double> receive_gaps_ms;
  std::vector<double> udp_transit_ms;
  std::vector<double> move_call_ms;
  std::vector<double> source_to_move_done_ms;

  auto report_timing = [&](uint64_t now_ns) {
    const auto receive_gap = summarize(receive_gaps_ms);
    const auto transit = summarize(udp_transit_ms);
    const auto move = summarize(move_call_ms);
    const auto total = summarize(source_to_move_done_ms);
    const double cmd_age_ms = last_receive_ns > 0
      ? static_cast<double>(now_ns - last_receive_ns) / 1e6
      : -1.0;

    std::cout << std::fixed << std::setprecision(3)
              << "TIMING_RECEIVER "
              << "udp_gap_ms=" << receive_gap.average << "/"
              << receive_gap.p95 << "/" << receive_gap.maximum << " "
              << "udp_transit_ms=" << transit.average << "/"
              << transit.p95 << "/" << transit.maximum << " "
              << "move_call_ms=" << move.average << "/"
              << move.p95 << "/" << move.maximum << " "
              << "sender_to_move_done_ms=" << total.average << "/"
              << total.p95 << "/" << total.maximum << " "
              << "cmd_age_ms=" << cmd_age_ms << " "
              << "count=" << receive_count << " "
              << "seq_gaps=" << sequence_gaps << " "
              << "invalid=" << invalid_packets << " "
              << "last_seq=" << last_sequence << " "
              << "cmd=(" << last_vx << "," << last_vy << ","
              << last_vyaw << ")" << std::endl;

    const bool alert =
      receive_gap.maximum > 100.0 || transit.maximum > 20.0 ||
      move.maximum > 25.0 || total.maximum > 50.0 ||
      sequence_gaps > 0 || invalid_packets > 0;
    if (alert && now_ns - last_alert_ns >= 1000000000ULL) {
      last_alert_ns = now_ns;
      std::cerr << std::fixed << std::setprecision(3)
                << "TIMING_ALERT_RECEIVER "
                << "udp_gap_max_ms=" << receive_gap.maximum << " "
                << "udp_transit_max_ms=" << transit.maximum << " "
                << "move_call_max_ms=" << move.maximum << " "
                << "sender_to_move_done_max_ms=" << total.maximum << " "
                << "seq_gaps=" << sequence_gaps << " "
                << "invalid=" << invalid_packets << std::endl;
    }

    receive_gaps_ms.clear();
    udp_transit_ms.clear();
    move_call_ms.clear();
    source_to_move_done_ms.clear();
    receive_count = 0;
    sequence_gaps = 0;
    invalid_packets = 0;
    report_started_ns = now_ns;
  };

  while (true)
  {
    const ssize_t n = recv(sock, &pkt, sizeof(pkt), 0);
    const uint64_t receive_ns = steady_now_ns();
    if (n == static_cast<ssize_t>(sizeof(pkt)) &&
        pkt.magic == kCmdPacketMagic &&
        pkt.version == kCmdPacketVersion &&
        pkt.packet_size == sizeof(CmdPacket))
    {
      if (last_receive_ns > 0) {
        receive_gaps_ms.push_back(
          static_cast<double>(receive_ns - last_receive_ns) / 1e6);
      }
      if (pkt.send_steady_ns <= receive_ns) {
        udp_transit_ms.push_back(
          static_cast<double>(receive_ns - pkt.send_steady_ns) / 1e6);
      }
      if (last_sequence > 0 && pkt.sequence > last_sequence + 1) {
        sequence_gaps += pkt.sequence - last_sequence - 1;
      }

      last_receive_ns = receive_ns;
      last_sequence = pkt.sequence;
      last_vx = std::max(-0.5f, std::min(0.5f, pkt.vx));
      last_vy = std::max(-0.10f, std::min(0.10f, pkt.vy));
      last_vyaw = std::max(-0.5f, std::min(0.5f, pkt.vyaw));

      const uint64_t move_start_ns = steady_now_ns();
      sport_client.Move(last_vx, last_vy, last_vyaw);
      const uint64_t move_done_ns = steady_now_ns();
      move_call_ms.push_back(
        static_cast<double>(move_done_ns - move_start_ns) / 1e6);
      if (pkt.send_steady_ns <= move_done_ns) {
        source_to_move_done_ms.push_back(
          static_cast<double>(move_done_ns - pkt.send_steady_ns) / 1e6);
      }
      ++receive_count;
    }
    else if (n >= 0) {
      ++invalid_packets;
    }
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      std::cerr << "recv failed errno=" << errno << std::endl;
    }

    if (receive_ns - report_started_ns >= 1000000000ULL) {
      report_timing(receive_ns);
    }
  }

  sport_client.StopMove();
  close(sock);
  return 0;
}
