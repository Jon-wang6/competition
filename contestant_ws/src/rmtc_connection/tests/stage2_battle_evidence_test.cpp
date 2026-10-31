/** @file stage2_battle_evidence_test.cpp
 * @brief 检查公开战斗证据、误归因排除、尸体清理和来路软折扣。
 */
#include "rmtc_connection/stage2_battle_evidence.hpp"
#include <iostream>
#include <stdexcept>

/** @brief 对推断业务边界作断言，失败时给出可定位的原因。 */
void evidence_require(bool condition,const char* message) {
  if(!condition) throw std::runtime_error(message);
}

/** @brief 构造本人右侧视野边缘的一名敌人，不提供任何裁判私有字段。 */
rmtc::PublicPerception evidence_frame() {
  rmtc::PublicPerception f;f.session_id="battle-evidence";f.map={12,9};f.vision_radius=2;
  f.self.team="A";f.self.position={3,3};f.self.alive=true;f.self.hp=100;
  rmtc::VisibleEntity e;e.type="robot";e.team="B";e.id="B-life1";
  e.position={5,3};e.hp=100;f.visible_entities={e};return f;
}

/** @brief 本车攻击不能制造假第三方；明确死亡与重现、消失和推测来路各有独立语义。 */
int main() {
  try {
    rmtc::Stage2Weights w;w.combat_clue_weight=1;w.opening_depletion_weight=1;
    rmtc::Stage2Events events;
    auto before=evidence_frame(),after=before;after.tick=1;after.visible_entities[0].hp=80;
    rmtc::MapMemory memory;memory.update(before);memory.update(after);
    rmtc::BattleEvidence clues;
    clues.update(after,before,events,{"attack","right"},memory,w);
    evidence_require(clues.details.empty(),"own shot was misclassified as third-party fire");
    clues.reset();clues.update(after,before,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.details.size()==1&&clues.in_view({7,3},1)>0,
                     "unattributed visible damage did not create a nearby search clue");
    evidence_require(clues.supply_factor({9,3},1,w)<1&&
                     clues.supply_factor({9,3},1,w)>=w.opening_depletion_floor&&
                     clues.supply_factor({1,3},1,w)==1,
                     "enemy-origin prior blocked a region or discounted the wrong side");
    auto changed=after;changed.tick=2;changed.event_text="new event awaiting structured interpretation";
    changed.visible_entities[0].hp=60;
    clues.reset();clues.update(changed,after,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.details.empty(),"unexplained event damage created false battle evidence");
    auto gone=before;gone.tick=1;gone.visible_entities.clear();
    clues.reset();clues.update(gone,before,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.details.empty(),"disappearance was declared a death");
    auto dead=before;dead.tick=1;dead.visible_entities[0].hp=0;
    clues.reset();clues.update(dead,before,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.details.size()==1&&clues.details[0]["explicit_death"].get<bool>(),
                     "explicit zero-HP observation was not recorded");
    auto revived=dead;revived.tick=2;revived.visible_entities[0].hp=100;
    revived.visible_entities[0].id="B-life2";revived.visible_entities[0].position={4,3};
    clues.update(revived,dead,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.details.empty(),"new enemy life retained the old death-scene clue");
    before=evidence_frame();before.vision_radius=4;
    auto attacker=before.visible_entities[0];attacker.team="C";attacker.id="C";attacker.position={7,3};
    before.visible_entities.push_back(attacker);after=before;after.tick=1;after.visible_entities[0].hp=80;
    memory=rmtc::MapMemory{};memory.update(before);memory.update(after);
    clues.reset();clues.update(after,before,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.engagement("C",1,3)>0&&clues.in_view({7,3},2)==0,
                     "known third-party shooter was duplicated as a hidden additional enemy");
    // 天下无双改变射程和伤害；已知第三方的一次强化命中不能被重复认成隐藏攻击者。
    before=evidence_frame();before.vision_radius=2;
    attacker.position={7,5};before.visible_entities.push_back(attacker);
    after=before;after.tick=1;after.visible_entities[0].hp=50;
    memory=rmtc::MapMemory{};memory.update(before);events.reset();events.observe(before,memory);
    rmtc::Stage2EventEffect fortress;fortress.kind="unrivaled";fortress.actor="C";
    fortress.begin_tick=0;fortress.end_tick=5;fortress.attack_radius=2;
    fortress.attack_damage=50;fortress.defended_damage=25;
    rmtc::Stage2EventConstraints packet;packet.session_id=before.session_id;
    packet.source_text=before.event_text;packet.source_tick=0;packet.begin_tick=0;
    packet.end_tick=5;packet.timeline=true;packet.events={fortress};
    events.merge(packet,before,memory);memory.update(after);events.observe(after,memory);
    clues.reset();clues.update(after,before,events,rmtc::Action::wait(),memory,w);
    evidence_require(clues.engagement("C",1,3)>0&&
                     !clues.details[0]["unattributed"].get<bool>()&&clues.in_view({8,3},2)==0,
                     "known buffed third-party damage manufactured an additional hidden attacker");
    std::cout<<"Battle evidence and resource-origin checks passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
