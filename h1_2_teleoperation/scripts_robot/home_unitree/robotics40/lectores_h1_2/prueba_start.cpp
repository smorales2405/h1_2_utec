// prueba_start — Start() y SetFsmId(204) imprimiendo el codigo de retorno.
// 8301 LocoState no disponible, 8302 fsm id invalido.
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <chrono>
#include <cstdio>
#include <thread>
int main(int argc, char const *argv[]) {
  if (argc < 2) { std::printf("Uso: %s <interfaz>\n", argv[0]); return -1; }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::h1::LocoClient c; c.Init(); c.SetTimeout(10.f);
  int id = -1; c.GetFsmId(id); std::printf("FSM antes: %d\n", id);
  int r = c.Start();
  std::printf("Start()        -> ret = %d %s\n", r, r == 0 ? "(OK)" : "(ERROR)");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  c.GetFsmId(id); std::printf("  FSM = %d\n", id);
  if (id != 204) {
    int r2 = c.SetFsmId(204);
    std::printf("SetFsmId(204)  -> ret = %d %s\n", r2, r2 == 0 ? "(OK)" : "(ERROR)");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    c.GetFsmId(id); std::printf("  FSM = %d\n", id);
  }
  int bal = -1; int rb = c.GetBalanceMode(bal);
  std::printf("GetBalanceMode -> ret = %d valor = %d\n", rb, bal);
  return 0;
}
