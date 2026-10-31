/**
 * @file stage2_navigation_progress_test.cpp
 * @brief 检查无收益循环识别及必要移动、开图、采集、原地防御的免触发边界。
 */
#include "rmtc_connection/stage2_navigation_progress.hpp"
#include <iostream>
#include <stdexcept>

/** @brief 用公开连续轨迹区分循环与有效移动，并检查临时代价到期及收益解除。 */
int main() {
  try {
    rmtc::Stage2Weights w;w.navigation_cycle_window=12;
    rmtc::NavigationProgress guard;
    rmtc::PublicPerception f;f.self.alive=true;f.self.hp=100;f.session_id="progress-tests";
    for(int t=0;t<12;++t) {f.tick=t;f.self.position={1+t%2,1};guard.observe(f,100,w);}
    if(!guard.active()||guard.breaks()!=1||guard.penalty({1,1},11,w)<=0||guard.penalty({3,1},11,w)!=0)
      throw std::runtime_error("unproductive local loop not identified");
    if(guard.penalty({1,1},23,w)!=0) throw std::runtime_error("temporary cost did not expire");
    f.tick=12;f.score.resource_value=10;guard.observe(f,100,w);
    if(guard.active()) throw std::runtime_error("real resource progress did not release loop guard");
    guard.reset();f.score.resource_value=0;
    for(int t=0;t<30;++t) {f.tick=t;f.self.position={1+t%2,1};guard.observe(f,100+t,w);}
    if(guard.active()) throw std::runtime_error("continued exploration falsely classified as looping");
    guard.reset();
    for(int t=0;t<30;++t) {f.tick=t;f.self.position={t,1};guard.observe(f,100,w);}
    if(guard.active()) throw std::runtime_error("long route falsely classified as local loop");
    guard.reset();
    for(int t=0;t<30;++t) {f.tick=t;f.self.position={1,1};guard.observe(f,100,w);}
    if(guard.active()) throw std::runtime_error("stationary defense falsely classified as movement loop");
    guard.reset();w.navigation_cycle_window=0;
    for(int t=0;t<30;++t) {f.tick=t;f.self.position={1+t%2,1};guard.observe(f,100,w);}
    if(guard.active()) throw std::runtime_error("disabled guard changed baseline");
    std::cout<<"Navigation progress checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
