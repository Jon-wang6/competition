/**
 * @file gpu_parameter_screen.cpp
 * @brief 离线调参专用CUDA核回归筛选器；不参与比赛决策，也不调用语言模型。
 *
 * 输入为归一化参数、真实裁判样本分数及候选参数的二进制矩阵。每个GPU线程
 * 计算一个候选的高斯核加权分数与距离奖励，输出仅用于安排后续真实裁判试验。
 * 使用已安装的CUDA驱动加载PTX，避免为此安装完整CUDA工具链。
 *
 * @details 模块协作与维护约定
 * GPU输出是基于已测样本的候选预测，用于筛选下一批实验，不是实际裁判结算。
 * 核回归距离奖励与真实策略参数不同；最终入选参数仍需闭环独立地图验证。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <dlfcn.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Ptr = unsigned long long;
/** @brief 将CUDA错误转换为可读异常，驱动不可用时直接失败，不伪装成GPU执行。 */
void check(int code) { if (code) throw std::runtime_error("CUDA error " + std::to_string(code)); }
/** @brief 从已打开的驱动加载准确签名的函数指针。 */
template<class F> F symbol(void* lib, const char* name) {
  auto p = dlsym(lib, name); if (!p) throw std::runtime_error(name); return reinterpret_cast<F>(p);
}
/** @brief 读取定长数组；截断、非有限数据直接拒绝。 */
std::vector<float> read(std::ifstream& in, std::size_t n) {
  std::vector<float> x(n); in.read(reinterpret_cast<char*>(x.data()), n*sizeof(float));
  if (!in || !std::all_of(x.begin(), x.end(), [](float v){return std::isfinite(v);}))
    throw std::runtime_error("invalid matrix");
  return x;
}
const char* kernel = R"PTX(
.version 7.0
.target sm_70
.address_size 64
.visible .entry screen(.param .u64 C,.param .u64 X,.param .u64 Y,.param .u64 O,
 .param .u32 N,.param .u32 M,.param .u32 D,.param .f32 G) {
 .reg .pred %p<4>; .reg .b32 %r<20>; .reg .b64 %a<20>; .reg .f32 %f<20>;
 ld.param.u64 %a0,[C]; ld.param.u64 %a1,[X]; ld.param.u64 %a2,[Y]; ld.param.u64 %a3,[O];
 ld.param.u32 %r0,[N]; ld.param.u32 %r1,[M]; ld.param.u32 %r2,[D]; ld.param.f32 %f0,[G];
 mov.u32 %r3,%ctaid.x; mov.u32 %r4,%ntid.x; mov.u32 %r5,%tid.x;
 mad.lo.u32 %r6,%r3,%r4,%r5; setp.ge.u32 %p0,%r6,%r0; @%p0 bra DONE;
 mul.lo.u32 %r7,%r6,%r2; mul.wide.u32 %a4,%r7,4; add.u64 %a5,%a0,%a4;
 mov.f32 %f1,0f00000000; mov.f32 %f2,0f00000000; mov.f32 %f3,0f7f7fffff;
 mov.u32 %r8,0;
 OBS:
 mov.f32 %f4,0f00000000; mov.u32 %r9,0;
 mul.lo.u32 %r10,%r8,%r2; mul.wide.u32 %a6,%r10,4; add.u64 %a7,%a1,%a6;
 DIM:
 mul.wide.u32 %a8,%r9,4; add.u64 %a9,%a5,%a8; add.u64 %a10,%a7,%a8;
 ld.global.f32 %f5,[%a9]; ld.global.f32 %f6,[%a10]; sub.f32 %f7,%f5,%f6;
 fma.rn.f32 %f4,%f7,%f7,%f4;
 add.u32 %r9,%r9,1; setp.lt.u32 %p1,%r9,%r2; @%p1 bra DIM;
 min.f32 %f3,%f3,%f4; mul.f32 %f8,%f4,%f0; neg.f32 %f8,%f8;
 mul.f32 %f8,%f8,0f3fb8aa3b; ex2.approx.ftz.f32 %f9,%f8;
 mul.wide.u32 %a11,%r8,4; add.u64 %a12,%a2,%a11; ld.global.f32 %f10,[%a12];
 fma.rn.f32 %f1,%f9,%f10,%f1; add.f32 %f2,%f2,%f9;
 add.u32 %r8,%r8,1; setp.lt.u32 %p2,%r8,%r1; @%p2 bra OBS;
 max.f32 %f2,%f2,0f1e3ce508; div.rn.f32 %f11,%f1,%f2;
 sqrt.rn.f32 %f12,%f3; fma.rn.f32 %f13,%f12,0f3eb33333,%f11;
 mul.wide.u32 %a13,%r6,4; add.u64 %a14,%a3,%a13; st.global.f32 [%a14],%f13;
 DONE: ret;
})PTX";
} // namespace

/** @brief 执行GPU筛选并与CPU结果交叉核对（默认4096个，--cpu-all核对全部）；输出计时与误差。 */
int main(int argc, char** argv) {
 try {
  if (argc!=3 && !(argc==4 && std::string(argv[3])=="--cpu-all"))
    throw std::runtime_error("usage: gpu_parameter_screen input.bin output.bin [--cpu-all]");
  std::ifstream in(argv[1],std::ios::binary); std::uint32_t sizes[3]{};
  in.read(reinterpret_cast<char*>(sizes),sizeof(sizes)); unsigned n=sizes[0],m=sizes[1],d=sizes[2];
  if (!n || n>2000000 || !m || m>1000 || !d || d>128) throw std::runtime_error("matrix limits");
  auto train=read(in,std::size_t(m)*d), targets=read(in,m), candidates=read(in,std::size_t(n)*d);
  void* lib=dlopen("libcuda.so.1",RTLD_NOW); if (!lib) throw std::runtime_error("CUDA driver unavailable");
  auto init=symbol<int(*)(unsigned)>(lib,"cuInit");
  auto device=symbol<int(*)(int*,int)>(lib,"cuDeviceGet");
  auto name=symbol<int(*)(char*,int,int)>(lib,"cuDeviceGetName");
  auto context=symbol<int(*)(void**,unsigned,int)>(lib,"cuCtxCreate_v2");
  auto destroy=symbol<int(*)(void*)>(lib,"cuCtxDestroy_v2");
  auto module=symbol<int(*)(void**,const void*)>(lib,"cuModuleLoadData");
  auto function=symbol<int(*)(void**,void*,const char*)>(lib,"cuModuleGetFunction");
  auto alloc=symbol<int(*)(Ptr*,std::size_t)>(lib,"cuMemAlloc_v2");
  auto upload=symbol<int(*)(Ptr,const void*,std::size_t)>(lib,"cuMemcpyHtoD_v2");
  auto download=symbol<int(*)(void*,Ptr,std::size_t)>(lib,"cuMemcpyDtoH_v2");
  auto launch=symbol<int(*)(void*,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned,void*,void**,void**)>(lib,"cuLaunchKernel");
  auto sync=symbol<int(*)()>(lib,"cuCtxSynchronize");
  check(init(0));int dev{};check(device(&dev,0));char model[128]{};check(name(model,128,dev));
  void* ctx{};check(context(&ctx,0,dev));void* mod{};check(module(&mod,kernel));void* fun{};check(function(&fun,mod,"screen"));
  auto now=[](){return std::chrono::steady_clock::now();}; auto start=now();
  Ptr c{},x{},y{},o{};check(alloc(&c,candidates.size()*4));check(alloc(&x,train.size()*4));
  check(alloc(&y,targets.size()*4));check(alloc(&o,std::size_t(n)*4));
  check(upload(c,candidates.data(),candidates.size()*4));check(upload(x,train.data(),train.size()*4));check(upload(y,targets.data(),targets.size()*4));
  float gamma=2.f;void* args[]={&c,&x,&y,&o,&n,&m,&d,&gamma};
  auto kstart=now();check(launch(fun,(n+255)/256,1,1,256,1,1,0,nullptr,args,nullptr));check(sync());auto kend=now();
  std::vector<float> scores(n);check(download(scores.data(),o,std::size_t(n)*4));auto end=now();
  auto cpustart=now();double error=0;unsigned count=argc==4 ? n : std::min(n,4096u);
  for(unsigned i=0;i<count;++i) {
   double total=0,weight=0,nearest=1e30;
   for(unsigned j=0;j<m;++j) {double distance=0;
    for(unsigned k=0;k<d;++k){double delta=candidates[i*d+k]-train[j*d+k];distance+=delta*delta;}
    double w=std::exp(-gamma*distance);weight+=w;total+=w*targets[j];nearest=std::min(nearest,distance);
   }
   double expected=total/std::max(weight,1e-20)+.35*std::sqrt(nearest);
   error=std::max(error,std::abs(scores[i]-expected));
  }
  auto cpuend=now();if(error>1e-4) throw std::runtime_error("CPU/GPU validation failed");
  std::ofstream out(argv[2],std::ios::binary);out.write(reinterpret_cast<char*>(scores.data()),scores.size()*4);
  auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
  std::cout<<"{\"gpu\":\""<<model<<"\",\"candidates\":"<<n<<",\"samples\":"<<m<<",\"dimensions\":"<<d
   <<",\"kernel_ms\":"<<ms(kstart,kend)<<",\"transfer_and_kernel_ms\":"<<ms(start,end)
   <<",\"cpu_reference_count\":"<<count<<",\"cpu_reference_ms\":"<<ms(cpustart,cpuend)<<",\"max_absolute_error\":"<<error<<"}\n";
  check(destroy(ctx));dlclose(lib);return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
