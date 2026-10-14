// log_bajo_nivel — SOLO LECTURA. Escucha rt/lowcmd y rt/lowstate y saca CSV con
// consigna vs medida de los 4 tobillos y las 2 munecas. No publica nada.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
static unitree_hg::msg::dds_::LowState_ g_ls;
static unitree_hg::msg::dds_::LowCmd_ g_cmd;
static std::atomic<uint64_t> n_ls{0}, n_cmd{0};
static void OnLs(const void *m){ g_ls=*(const unitree_hg::msg::dds_::LowState_*)m; n_ls++; }
static void OnCmd(const void *m){ g_cmd=*(const unitree_hg::msg::dds_::LowCmd_*)m; n_cmd++; }
int main(int argc, char const *argv[]) {
  if (argc < 3) { std::printf("Uso: %s <interfaz> <segundos>\n", argv[0]); return -1; }
  double seg = atof(argv[2]);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> s1;
  s1.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  s1->InitChannel(OnLs, 1);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> s2;
  s2.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>("rt/lowcmd"));
  s2->InitChannel(OnCmd, 1);
  // t, n_cmd, LAP_c,LAP_m, LAR_c,LAR_m, RAP_c,RAP_m, RAR_c,RAR_m, LWR_c,LWR_m, RWR_c,RWR_m, knee_m, kp_ank
  int idx[6] = {4, 5, 10, 11, 17, 24};
  auto t0 = std::chrono::steady_clock::now();
  std::printf("t,n_cmd,LAP_c,LAP_m,LAR_c,LAR_m,RAP_c,RAP_m,RAR_c,RAR_m,LWR_c,LWR_m,RWR_c,RWR_m,LKnee_m,kp_ank\n");
  while (true) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
    if (t > seg) break;
    std::printf("%.4f,%lu", t, (unsigned long)n_cmd.load());
    for (int k = 0; k < 6; ++k)
      std::printf(",%.5f,%.5f", g_cmd.motor_cmd().at(idx[k]).q(), g_ls.motor_state().at(idx[k]).q());
    std::printf(",%.5f,%.1f\n", g_ls.motor_state().at(3).q(), g_cmd.motor_cmd().at(4).kp());
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  std::fprintf(stderr, "lowcmd %lu, lowstate %lu\n",
    (unsigned long)n_cmd.load(), (unsigned long)n_ls.load());
  return 0;
}
