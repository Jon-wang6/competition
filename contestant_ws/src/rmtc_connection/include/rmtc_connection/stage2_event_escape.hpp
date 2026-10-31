/**
 * @file stage2_event_escape.hpp
 * @brief 用已观察地形反向BFS，估计从预告危险区域撤到安全格至少还需几步。
 *
 * 只读取模型已确认的区域与本队可通行格，不把未知格猜成出口；每帧按当前
 * 地形重建，事件结束后不保留额外墙。用于首步风险估计，不修改裁判结算规则。
 */
#pragma once
#include "rmtc_connection/public_data.hpp"
#include <array>
#include <functional>
#include <queue>
#include <vector>

namespace rmtc {
class EventEscapeField {
 public:
  /** @brief 从所有已知安全格同时扩展，得到危险区各可通行格到最近出口的距离。 */
  EventEscapeField(MapSize size,const std::vector<Position>& danger,
                   const std::function<bool(Position)>& passable):size_(size),
      distances_(static_cast<std::size_t>(size.width*size.height),-1) {
    std::vector<bool> hazardous(distances_.size(),false);
    for(auto p:danger)if(inside(p))hazardous[index(p)]=true;
    std::queue<Position> queue;
    for(int y=0;y<size.height;++y)for(int x=0;x<size.width;++x){
      Position p{x,y};if(!hazardous[index(p)]&&passable(p)){
        distances_[index(p)]=0;queue.push(p);
      }
    }
    const std::array<Position,4> steps{{{0,-1},{0,1},{-1,0},{1,0}}};
    while(!queue.empty()){
      auto p=queue.front();queue.pop();
      for(auto d:steps){Position next{p.x+d.x,p.y+d.y};
        if(inside(next)&&passable(next)&&distances_[index(next)]<0){
          distances_[index(next)]=distances_[index(p)]+1;queue.push(next);
        }
      }
    }
  }
  /** @brief 未知、被堵或没有已知出口返回-1；安全区返回0，不伪造可达性。 */
  int distance(Position p) const {return inside(p)?distances_[index(p)]:-1;}
 private:
  /** @brief 检查整数格是否属于公开地图尺寸。 */
  bool inside(Position p) const {return p.x>=0&&p.y>=0&&p.x<size_.width&&p.y<size_.height;}
  /** @brief 将合法格转换为行主序索引，仅在inside确认后调用。 */
  std::size_t index(Position p) const {return static_cast<std::size_t>(p.y*size_.width+p.x);}
  MapSize size_;std::vector<int> distances_;
};
}  // namespace rmtc
