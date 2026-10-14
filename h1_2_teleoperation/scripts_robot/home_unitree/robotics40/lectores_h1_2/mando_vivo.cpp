// mando_vivo — SOLO LECTURA. Decodifica los botones de wireless_remote[40] en
// vivo. Sirve para ver si una pulsacion del mando llega de verdad al PC2.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
static unitree_hg::msg::dds_::LowState_ g_ls;
static std::atomic<uint64_t> n{0};
static void OnLs(const void *m){ g_ls=*(const unitree_hg::msg::dds_::LowState_*)m; n++; }

int main(int argc, char const *argv[]) {
  if (argc < 3) { std::printf("Uso: %s <interfaz> <segundos>\n", argv[0]); return -1; }
  double seg = atof(argv[2]);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> s;
  s.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  s->InitChannel(OnLs, 1);
  std::printf("Escuchando el mando %.0f s. PULSA R2+X AHORA (y lo que quieras).\n", seg);
  std::printf("Cualquier cambio en los bytes de boton se imprime.\n\n");
  const char *b0[8] = {"R1","L1","START","SELECT","R2","L2","F1","F2"};
  const char *b1[8] = {"A","B","X","Y","UP","RIGHT","DOWN","LEFT"};
  uint8_t prev2 = 0xAA, prev3 = 0xAA;
  auto t0 = std::chrono::steady_clock::now();
  while (true) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
    if (t > seg) break;
    if (n.load()) {
      uint8_t c2 = g_ls.wireless_remote()[2], c3 = g_ls.wireless_remote()[3];
      if (c2 != prev2 || c3 != prev3) {
        std::printf("[%6.2f s] bytes 2,3 = %02X %02X   pulsado:", t, c2, c3);
        bool alguno = false;
        for (int i = 0; i < 8; ++i) if (c2 & (1<<i)) { std::printf(" %s", b0[i]); alguno = true; }
        for (int i = 0; i < 8; ++i) if (c3 & (1<<i)) { std::printf(" %s", b1[i]); alguno = true; }
        if (!alguno) std::printf(" (nada)");
        std::printf("\n"); std::fflush(stdout);
        prev2 = c2; prev3 = c3;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::printf("\nfin. lowstate recibidos: %lu\n", (unsigned long)n.load());
  if (prev2 == 0xAA) std::printf("*** NUNCA cambio ningun byte de boton ***\n");
  return 0;
}
