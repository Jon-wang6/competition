/**
 * @file stage2_acquisition_curve_test.cpp
 * @brief 固定采集权重、纯探索半衰曲线、资源活动恢复和致死动作边界。
 */
#include "rmtc_connection/stage2_acquisition_curve.hpp"
#include "rmtc_connection/stage2_strategy.hpp"
#include <iostream>
#include <stdexcept>

namespace {
/// 测试不依赖Release下可能被移除的assert。
void check(bool okay,const char* message) { if(!okay) throw std::runtime_error(message); }
/// 完全公开的小图，仅提供本车视野内的资源和可选对手。
rmtc::PublicPerception frame(int tick,bool resource=false,bool enemy=false) {
  rmtc::PublicPerception f;f.session_id="curve-test";f.tick=tick;f.map={9,9};f.vision_radius=9;
  f.self.position={4,4};f.self.team="A";f.self.alive=true;f.self.hp=100;
  if (resource) {
    rmtc::VisibleEntity e;e.id="late";e.type="resource_normal";e.position={5,4};e.value=10;
    f.visible_entities.push_back(e);
  }
  if (enemy) {
    rmtc::VisibleEntity e;e.id="B";e.team="B";e.type="robot";e.position={4,6};e.hp=100;e.spawn_protected=false;
    f.visible_entities.push_back(e);
  }
  return f;
}
} // namespace

/** @brief 数学曲线与实际策略接入都必须保持“采集不衰减，只衰减探索”。 */
int main() {
  try {
    rmtc::AcquisitionCurveWeights w;w.enabled=true;w.validate();
    double previous=1e100;
    for (int tick=0;tick<=500;++tick) {
      const auto p=rmtc::acquisition_preference(w,tick,0,false);
      check(p.resource==w.resource_weight,"resource desire decayed with time");
      check(p.exploration<=previous+1e-12 && p.exploration>=w.explore_floor,
            "exploration did not decay monotonically within configured bounds");
      previous=p.exploration;
    }
    const auto fresh=rmtc::acquisition_preference(w,50,50,true);
    const auto quiet=rmtc::acquisition_preference(w,50,0,false);
    check(fresh.exploration>quiet.exploration,"new visible resource did not reset the silence discount");
    check(rmtc::acquisition_preference(w,500,500,true).resource==w.resource_weight,
          "late visible resource lost collection preference");
    auto invalid=w;invalid.explore_floor=invalid.explore_peak+1;
    bool rejected=false;try{invalid.validate();}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"invalid increasing exploration curve accepted");
    rmtc::Stage2Weights settings;settings.acquisition=w;
    auto temporal=settings;temporal.temporal.enabled=true;
    temporal.temporal.transition_ticks=100;temporal.temporal.resource_age_end=5;
    temporal.temporal.shape=1; // 独立数学端点场景，不依赖训练后的默认曲线形状。
    temporal.resource_age_ticks=35;
    const auto first=rmtc::temporal_weights_at(temporal,0);
    const auto middle=rmtc::temporal_weights_at(temporal,50);
    const auto last=rmtc::temporal_weights_at(temporal,500);
    check(first.resource_age_ticks==35 && middle.resource_age_ticks==20 && last.resource_age_ticks==5,
          "parameter-specific temporal interpolation did not reach its endpoints");
    check(temporal.resource_age_ticks==35 &&
          rmtc::temporal_weights_at(temporal,50).resource_age_ticks==20,
          "per-tick interpolation compounded or mutated the base configuration");
    check(first.acquisition.resource_weight==last.acquisition.resource_weight,
          "temporal parameter fitting decayed collection desire");
    rmtc::Stage2Strategy strategy(settings);
    strategy.decide(frame(0,true),500);
    check(strategy.plan().resource_weight==w.resource_weight,"opening fixed collection weight not applied");
    auto action=strategy.decide(frame(400,true,true),100);
    check(strategy.plan().resource_weight==w.resource_weight && strategy.plan().goal=="resource" &&
          action.type=="move" && action.direction=="right","late resource yielded to ordinary attack");
    auto low=frame(450,true,true);low.self.hp=15;
    rmtc::Stage2Strategy guarded(settings);
    check(guarded.decide(low,50).type=="defend","high collection weight promised a lethal pre-move pickup");
    strategy.decide(frame(451),49);
    check(strategy.plan().acquisition_silence_ticks>0,"missing resource did not start a silence period");
    auto reset=frame(3);reset.session_id="new-match";
    strategy.decide(reset,497);
    check(strategy.plan().acquisition_silence_ticks==3,"resource activity leaked across matches");
    std::cout<<"Fixed collection and exploration decay checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
