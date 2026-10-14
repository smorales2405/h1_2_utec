// lee_mando — SOLO LECTURA. Volca wireless_remote[40] del LowState_ y el kp que
// usa el control interno, para saber si el mando esta vivo y en que estado real esta.
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
  if (argc < 2) { std::printf("Uso: %s <interfaz>\n", argv[0]); return -1; }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> s1;
  s1.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  s1->InitChannel(OnLs, 1);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> s2;
  s2.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>("rt/lowcmd"));
  s2->InitChannel(OnCmd, 1);
  std::this_thread::sleep_for(std::chrono::seconds(4));
  if (!n_ls.load()) { std::printf("sin lowstate\n"); return 1; }

  std::printf("=== wireless_remote[40] ===\n ");
  int no_cero = 0;
  for (int i = 0; i < 40; ++i) {
    std::printf("%02X ", g_ls.wireless_remote()[i]);
    if (g_ls.wireless_remote()[i] != 0) no_cero++;
    if ((i+1) % 20 == 0) std::printf("\n ");
  }
  std::printf("\n  bytes no nulos: %d/40  ->  %s\n", no_cero,
              no_cero == 0 ? "MANDO MUDO (todo a cero)" : "el mando manda datos");

  std::printf("\n=== que publica el control interno en rt/lowcmd ===\n");
  if (!n_cmd.load()) { std::printf("  nadie publica\n"); }
  else {
    std::printf("  %lu msgs en 4 s (~%.0f Hz), mode_pr=%u mode_machine=%u\n",
      (unsigned long)n_cmd.load(), n_cmd.load()/4.0,
      g_cmd.mode_pr(), g_cmd.mode_machine());
    std::printf("  %-4s %8s %8s %8s %8s\n", "slot", "q_cmd", "kp", "kd", "tau");
    int slots[] = {3, 9, 12, 16, 23, 27};
    const char *nm[] = {"L_Knee","R_Knee","Waist","L_Elbow","R_Elbow","peso27"};
    for (int k = 0; k < 6; ++k) {
      int i = slots[k];
      std::printf("  %-4s %8.3f %8.1f %8.1f %8.2f   <- %s\n", nm[k],
        g_cmd.motor_cmd().at(i).q(), g_cmd.motor_cmd().at(i).kp(),
        g_cmd.motor_cmd().at(i).kd(), g_cmd.motor_cmd().at(i).tau(), nm[k]);
    }
  }
  return 0;
}
