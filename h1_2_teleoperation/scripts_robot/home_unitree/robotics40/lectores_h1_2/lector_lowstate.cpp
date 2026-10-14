// lector_lowstate — sonda de SOLO LECTURA para el Unitree H1-2.
//
// Se suscribe a rt/lowstate y rt/lf/bmsstate y los imprime. NO crea ningun
// ChannelPublisher, no publica nada en ningun topic y no instancia ningun
// cliente RPC. No puede mover el robot.
//
// Uso:  lector_lowstate <interfaz> [segundos]      p.ej. lector_lowstate eth0 10

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

static const char *kTopicLowState = "rt/lowstate";
static const char *kTopicBmsState = "rt/lf/bmsstate";

// El H1-2 lleva una bateria 16S (67.2 V a plena carga = 16 x 4.2 V), 15 Ah.
static const int   kCells      = 16;
static const float kVoltFull   = 67.2f;

// Las 27 juntas que el H1-2 usa de los 35 slots del mensaje.
static const int kNumJoints = 27;

// Curva de descarga aproximada de Li-ion, por celda. Solo para orientar:
// bajo carga la tension cae y subestima el SOC.
static float SocDesdeCelda(float v) {
  static const float tab[][2] = {
      {4.20f, 100.f}, {4.10f, 90.f}, {4.00f, 80.f}, {3.90f, 70.f},
      {3.85f, 60.f},  {3.80f, 50.f}, {3.75f, 40.f}, {3.70f, 30.f},
      {3.65f, 20.f},  {3.55f, 10.f}, {3.40f, 5.f},  {3.00f, 0.f}};
  const int n = sizeof(tab) / sizeof(tab[0]);
  if (v >= tab[0][0]) return 100.f;
  if (v <= tab[n - 1][0]) return 0.f;
  for (int i = 0; i < n - 1; ++i) {
    if (v <= tab[i][0] && v > tab[i + 1][0]) {
      float t = (v - tab[i + 1][0]) / (tab[i][0] - tab[i + 1][0]);
      return tab[i + 1][1] + t * (tab[i][1] - tab[i + 1][1]);
    }
  }
  return 0.f;
}

struct ResumenLowState {
  std::mutex mtx;
  std::atomic<uint64_t> n{0};
  uint32_t tick = 0;
  uint8_t mode_machine = 0, mode_pr = 0;
  float rpy[3] = {0, 0, 0};
  float imu_temp = 0;
  int temp_min = 999, temp_max = -999, temp_idx_max = -1;
  int fallos = 0, primer_fallo = -1, cod_fallo = 0;
  float vol_min = 1e9f, vol_max = -1e9f, vol_sum = 0;
  int vol_n = 0;
};

struct ResumenBms {
  std::mutex mtx;
  std::atomic<uint64_t> n{0};
  uint8_t soc = 0, soh = 0;
  int32_t current = 0;
  uint32_t bmsvoltage[3] = {0, 0, 0};
  uint32_t bmsstate[5] = {0, 0, 0, 0, 0};
  int celdas_no_cero = 0, temps_no_cero = 0;
  uint16_t cell_min = 0xFFFF, cell_max = 0;
  int16_t t_min = 32767, t_max = -32768;
  bool todo_cero = true;
};

static ResumenLowState g_ls;
static ResumenBms g_bms;

static void OnLowState(const void *msg) {
  auto *s = (const unitree_hg::msg::dds_::LowState_ *)msg;
  std::lock_guard<std::mutex> lk(g_ls.mtx);
  g_ls.n++;
  g_ls.tick = s->tick();
  g_ls.mode_machine = s->mode_machine();
  g_ls.mode_pr = s->mode_pr();
  for (int i = 0; i < 3; ++i) g_ls.rpy[i] = s->imu_state().rpy()[i];
  g_ls.imu_temp = (float)s->imu_state().temperature();

  g_ls.temp_min = 999; g_ls.temp_max = -999; g_ls.temp_idx_max = -1;
  g_ls.fallos = 0; g_ls.primer_fallo = -1; g_ls.cod_fallo = 0;
  g_ls.vol_min = 1e9f; g_ls.vol_max = -1e9f; g_ls.vol_sum = 0; g_ls.vol_n = 0;

  for (int i = 0; i < kNumJoints; ++i) {
    const auto &m = s->motor_state()[i];
    int t = (int)m.temperature()[0];
    if (t > 0) {  // 0 = sensor sin dato
      if (t < g_ls.temp_min) g_ls.temp_min = t;
      if (t > g_ls.temp_max) { g_ls.temp_max = t; g_ls.temp_idx_max = i; }
    }
    if (m.motorstate() != 0) {
      g_ls.fallos++;
      if (g_ls.primer_fallo < 0) { g_ls.primer_fallo = i; g_ls.cod_fallo = (int)m.motorstate(); }
    }
    float v = m.vol();
    if (v > 1.0f) {  // descarta slots sin alimentar
      if (v < g_ls.vol_min) g_ls.vol_min = v;
      if (v > g_ls.vol_max) g_ls.vol_max = v;
      g_ls.vol_sum += v; g_ls.vol_n++;
    }
  }
}

static void OnBmsState(const void *msg) {
  auto *s = (const unitree_hg::msg::dds_::BmsState_ *)msg;
  std::lock_guard<std::mutex> lk(g_bms.mtx);
  g_bms.n++;
  g_bms.soc = s->soc();
  g_bms.soh = s->soh();
  g_bms.current = s->current();
  for (int i = 0; i < 3; ++i) g_bms.bmsvoltage[i] = s->bmsvoltage()[i];
  for (int i = 0; i < 5; ++i) g_bms.bmsstate[i] = s->bmsstate()[i];

  g_bms.celdas_no_cero = 0; g_bms.cell_min = 0xFFFF; g_bms.cell_max = 0;
  for (int i = 0; i < 40; ++i) {
    uint16_t c = s->cell_vol()[i];
    if (c != 0) {
      g_bms.celdas_no_cero++;
      if (c < g_bms.cell_min) g_bms.cell_min = c;
      if (c > g_bms.cell_max) g_bms.cell_max = c;
    }
  }
  g_bms.temps_no_cero = 0; g_bms.t_min = 32767; g_bms.t_max = -32768;
  for (int i = 0; i < 12; ++i) {
    int16_t t = s->temperature()[i];
    if (t != 0) {
      g_bms.temps_no_cero++;
      if (t < g_bms.t_min) g_bms.t_min = t;
      if (t > g_bms.t_max) g_bms.t_max = t;
    }
  }
  bool cero = (g_bms.soc == 0 && g_bms.soh == 0 && g_bms.current == 0 &&
               g_bms.celdas_no_cero == 0 && g_bms.temps_no_cero == 0 &&
               g_bms.bmsvoltage[0] == 0);
  g_bms.todo_cero = cero;
}

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::printf("Uso: %s <interfaz_de_red> [segundos]\n", argv[0]);
    std::printf("  p.ej. %s eth0 10\n", argv[0]);
    return -1;
  }
  const std::string iface = argv[1];
  const int segundos = (argc >= 3) ? std::atoi(argv[2]) : 10;

  std::printf("=== lector_lowstate: SOLO LECTURA, no publica nada ===\n");
  std::printf("interfaz: %s   duracion: %d s\n\n", iface.c_str(), segundos);

  unitree::robot::ChannelFactory::Instance()->Init(0, iface);

  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub_ls;
  sub_ls.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicLowState));
  sub_ls->InitChannel(OnLowState, 1);

  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::BmsState_> sub_bms;
  sub_bms.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>(kTopicBmsState));
  sub_bms->InitChannel(OnBmsState, 1);

  auto t0 = std::chrono::steady_clock::now();
  uint64_t prev_ls = 0, prev_bms = 0;

  for (int s = 1; s <= segundos; ++s) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint64_t n_ls = g_ls.n.load(), n_bms = g_bms.n.load();
    double hz_ls = (double)(n_ls - prev_ls), hz_bms = (double)(n_bms - prev_bms);
    prev_ls = n_ls; prev_bms = n_bms;

    std::printf("---------------- t=%2d s ----------------\n", s);

    {
      std::lock_guard<std::mutex> lk(g_ls.mtx);
      if (n_ls == 0) {
        std::printf("rt/lowstate      : SIN DATOS\n");
      } else {
        std::printf("rt/lowstate      : %.0f Hz  tick=%u  mode_machine=%u  mode_pr=%u (%s)\n",
                    hz_ls, g_ls.tick, g_ls.mode_machine, g_ls.mode_pr,
                    g_ls.mode_pr == 0 ? "PR" : "AB");
        std::printf("  imu rpy        : %+.3f %+.3f %+.3f rad   imu %.0f C\n",
                    g_ls.rpy[0], g_ls.rpy[1], g_ls.rpy[2], g_ls.imu_temp);
        if (g_ls.temp_idx_max >= 0)
          std::printf("  motores        : %d-%d C  (mas caliente: junta %d)\n",
                      g_ls.temp_min, g_ls.temp_max, g_ls.temp_idx_max);
        if (g_ls.fallos == 0)
          std::printf("  motorstate     : 27/27 a 0, sin fallos\n");
        else
          std::printf("  motorstate     : *** %d CON FALLO *** primero: junta %d codigo 0x%X\n",
                      g_ls.fallos, g_ls.primer_fallo, g_ls.cod_fallo);
        if (g_ls.vol_n > 0) {
          float media = g_ls.vol_sum / g_ls.vol_n;
          float celda = media / kCells;
          std::printf("  BATERIA (bus de %d motores): %.2f V  [%.2f - %.2f]\n",
                      g_ls.vol_n, media, g_ls.vol_min, g_ls.vol_max);
          std::printf("     -> %.3f V/celda de 16S   ~%.0f%% SOC   (%.0f%% de los %.1f V a tope)\n",
                      celda, SocDesdeCelda(celda), 100.f * media / kVoltFull, kVoltFull);
        } else {
          std::printf("  BATERIA         : motor_state[].vol() todo a cero, no hay tension\n");
        }
      }
    }

    {
      std::lock_guard<std::mutex> lk(g_bms.mtx);
      if (n_bms == 0) {
        std::printf("rt/lf/bmsstate   : SIN DATOS\n");
      } else if (g_bms.todo_cero) {
        std::printf("rt/lf/bmsstate   : %.0f Hz pero EL MENSAJE ENTERO A CERO (cascaron del PC2)\n", hz_bms);
      } else {
        std::printf("rt/lf/bmsstate   : %.0f Hz  soc=%u%%  soh=%u%%  current=%d mA\n",
                    hz_bms, g_bms.soc, g_bms.soh, g_bms.current);
        std::printf("  bmsvoltage     : %u %u %u mV\n",
                    g_bms.bmsvoltage[0], g_bms.bmsvoltage[1], g_bms.bmsvoltage[2]);
        std::printf("  celdas         : %d no nulas, %u-%u mV\n",
                    g_bms.celdas_no_cero, g_bms.cell_min, g_bms.cell_max);
        std::printf("  temperaturas   : %d no nulas, %d-%d C\n",
                    g_bms.temps_no_cero, g_bms.t_min, g_bms.t_max);
      }
    }
    std::printf("\n");
    std::fflush(stdout);
  }

  auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("=== fin: %.1f s, %lu lowstate (%.0f Hz), %lu bmsstate (%.0f Hz) ===\n",
              dt, (unsigned long)g_ls.n.load(), g_ls.n.load() / dt,
              (unsigned long)g_bms.n.load(), g_bms.n.load() / dt);
  return 0;
}
