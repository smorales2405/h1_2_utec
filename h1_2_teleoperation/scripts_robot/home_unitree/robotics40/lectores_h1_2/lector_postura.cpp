// lector_postura — SOLO LECTURA. Imprime la postura actual de las 27 juntas y
// comprueba si ALGUIEN MAS esta publicando en rt/lowcmd (control interno activo).
// No crea ningun publicador. No puede mover el robot.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <cmath>

static const char *kNombres[27] = {
  "L_HipYaw","L_HipPitch","L_HipRoll","L_Knee","L_AnklePitch","L_AnkleRoll",
  "R_HipYaw","R_HipPitch","R_HipRoll","R_Knee","R_AnklePitch","R_AnkleRoll",
  "WaistYaw",
  "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_Elbow","L_WristRoll",
  "L_WristPitch","L_WristYaw",
  "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_Elbow","R_WristRoll",
  "R_WristPitch","R_WristYaw"};

static unitree_hg::msg::dds_::LowState_ g_ls;
static std::atomic<uint64_t> g_n_ls{0}, g_n_cmd{0};
static std::atomic<int> g_cmd_kp_max{0};

static void OnLowState(const void *m) {
  g_ls = *(const unitree_hg::msg::dds_::LowState_ *)m; g_n_ls++;
}
static void OnLowCmd(const void *m) {
  auto *c = (const unitree_hg::msg::dds_::LowCmd_ *)m; g_n_cmd++;
  int k = 0;
  for (int i = 0; i < 27; ++i) k = std::max(k, (int)c->motor_cmd().at(i).kp());
  g_cmd_kp_max = k;
}

int main(int argc, char const *argv[]) {
  if (argc < 2) { std::printf("Uso: %s <interfaz>\n", argv[0]); return -1; }
  std::printf("=== lector_postura: SOLO LECTURA ===\n\n");
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> s1;
  s1.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  s1->InitChannel(OnLowState, 1);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> s2;
  s2.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>("rt/lowcmd"));
  s2->InitChannel(OnLowCmd, 1);

  std::this_thread::sleep_for(std::chrono::seconds(4));
  if (g_n_ls.load() == 0) { std::printf("sin lowstate\n"); return 1; }

  std::printf("--- POSTURA ACTUAL (27 juntas) ---\n");
  double max_abs = 0; int idx_max = -1;
  for (int i = 0; i < 27; ++i) {
    double q = g_ls.motor_state().at(i).q();
    if (std::fabs(q) > max_abs) { max_abs = std::fabs(q); idx_max = i; }
    std::printf("  %2d %-16s q=%+7.3f rad = %+7.1f deg   tau=%+7.2f Nm  %2d C\n",
                i, kNombres[i], q, q * 180.0 / M_PI,
                g_ls.motor_state().at(i).tau_est(),
                (int)g_ls.motor_state().at(i).temperature()[0]);
  }
  std::printf("\n  junta mas alejada del cero: %d (%s) con %.1f deg\n",
              idx_max, kNombres[idx_max], max_abs * 180.0 / M_PI);

  std::printf("\n--- QUIEN MANDA EN rt/lowcmd (4 s) ---\n");
  uint64_t n = g_n_cmd.load();
  if (n == 0) {
    std::printf("  NADIE publica en rt/lowcmd -> el bajo nivel esta libre\n");
  } else {
    std::printf("  *** HAY OTRO CONTROLADOR: %lu mensajes (~%.0f Hz), kp max = %d ***\n",
                (unsigned long)n, n / 4.0, g_cmd_kp_max.load());
    std::printf("  lanzar un ejemplo de bajo nivel AHORA = dos controladores peleando\n");
  }
  std::printf("\nlowstate: %lu msgs (~%.0f Hz)\n", (unsigned long)g_n_ls.load(), g_n_ls.load()/4.0);
  return 0;
}
