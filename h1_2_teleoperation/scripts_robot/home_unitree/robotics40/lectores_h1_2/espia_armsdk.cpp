// espia_armsdk — SOLO LECTURA. Escucha rt/arm_sdk y rt/lowstate a la vez y
// contrasta la consigna que circula por el bus con lo que hace la junta.
// No publica nada.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <cmath>

static unitree_hg::msg::dds_::LowState_ g_ls;
static unitree_hg::msg::dds_::LowCmd_   g_as;
static std::atomic<uint64_t> n_ls{0}, n_as{0};

static void OnLs(const void *m){ g_ls=*(const unitree_hg::msg::dds_::LowState_*)m; n_ls++; }
static void OnAs(const void *m){ g_as=*(const unitree_hg::msg::dds_::LowCmd_*)m;   n_as++; }

int main(int argc, char const *argv[]) {
  if (argc < 3) { std::printf("Uso: %s <interfaz> <segundos>\n", argv[0]); return -1; }
  int seg = std::atoi(argv[2]);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> s1;
  s1.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  s1->InitChannel(OnLs, 1);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> s2;
  s2.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>("rt/arm_sdk"));
  s2->InitChannel(OnAs, 1);

  std::printf("  t  armsdk_msgs peso  L_Elb_cmd L_Elb_med  L_ShR_cmd L_ShR_med  kp_elb\n");
  for (int t = 1; t <= seg; ++t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    double peso = n_as.load() ? g_as.motor_cmd().at(27).q() : -1;
    double ec = n_as.load() ? g_as.motor_cmd().at(16).q() : 0;
    double sc = n_as.load() ? g_as.motor_cmd().at(14).q() : 0;
    double kp = n_as.load() ? g_as.motor_cmd().at(16).kp() : 0;
    std::printf("%4.1f %10lu %5.2f %9.1f %9.1f  %9.1f %9.1f  %6.0f\n",
      t*0.5, (unsigned long)n_as.load(), peso,
      ec*57.2958, g_ls.motor_state().at(16).q()*57.2958,
      sc*57.2958, g_ls.motor_state().at(14).q()*57.2958, kp);
    std::fflush(stdout);
  }
  std::printf("total: arm_sdk %lu msgs, lowstate %lu msgs\n",
              (unsigned long)n_as.load(), (unsigned long)n_ls.load());
  return 0;
}
