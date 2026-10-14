// lee_fsm — SOLO LECTURA. Consulta GetFsmId / GetFsmMode / GetBalanceMode del
// LocoClient. Son getters: no cambian ningun estado del robot.
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <cstdio>
int main(int argc, char const *argv[]) {
  if (argc < 2) { std::printf("Uso: %s <interfaz>\n", argv[0]); return -1; }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::h1::LocoClient c;
  c.Init();
  c.SetTimeout(5.f);
  int id = -999, mode = -999, bal = -999;
  int r1 = c.GetFsmId(id);
  int r2 = c.GetFsmMode(mode);
  int r3 = c.GetBalanceMode(bal);
  std::printf("GetFsmId      -> ret=%d  valor=%d\n", r1, id);
  std::printf("GetFsmMode    -> ret=%d  valor=%d\n", r2, mode);
  std::printf("GetBalanceMode-> ret=%d  valor=%d\n", r3, bal);
  std::printf("\n  FSM: 0=par cero  1=Damp  2=de pie bloqueado  204=control principal\n");
  return 0;
}
