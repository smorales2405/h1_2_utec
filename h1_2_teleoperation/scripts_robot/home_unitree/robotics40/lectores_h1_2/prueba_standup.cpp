// prueba_standup — llama a StandUp() y SI imprime el codigo de retorno,
// que el ejemplo oficial tira a la basura. Luego relee la FSM.
// Codigos: 8301 LocoState no disponible, 8302 fsm id invalido,
//          8303 OdomState no disponible, 8304 task id invalido.
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <chrono>
#include <cstdio>
#include <thread>
int main(int argc, char const *argv[]) {
  if (argc < 2) { std::printf("Uso: %s <interfaz>\n", argv[0]); return -1; }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::h1::LocoClient c;
  c.Init();
  c.SetTimeout(10.f);

  int id = -1;
  c.GetFsmId(id);
  std::printf("FSM antes: %d\n", id);

  int r = c.StandUp();
  std::printf("StandUp()   -> ret = %d %s\n", r, r == 0 ? "(OK)" : "(ERROR)");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  c.GetFsmId(id);
  std::printf("FSM despues de StandUp: %d\n", id);

  if (id != 2) {
    int r2 = c.SetFsmId(2);
    std::printf("SetFsmId(2) -> ret = %d %s\n", r2, r2 == 0 ? "(OK)" : "(ERROR)");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    c.GetFsmId(id);
    std::printf("FSM despues de SetFsmId(2): %d\n", id);
  }
  return 0;
}
