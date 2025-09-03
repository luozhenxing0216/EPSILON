# SSC Map 架构与功能详解

## 文件概述
`ssc_map.cc` 实现了 SSC (Spatial-Spatial-Temporal Corridor) 算法，用于自动驾驶路径规划中的空间-时间走廊构建。

## 核心架构

### 1. 头文件和命名空间 (Lines 1-12)
```cpp
#include "ssc_planner/ssc_map.h"
#include <glog/logging.h>
namespace planning {
```
- 引入头文件和 Google 日志库
- 使用 planning 命名空间封装所有功能

### 2. 构造函数 (Lines 14-21)
```cpp
SscMap::SscMap(const SscMap::Config &config) : config_(config) {
  config_.Print();
  p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
  p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
}
```
**功能**：
- 初始化配置参数
- 创建两个3D网格地图：原始网格和膨胀网格
- 网格维度为 [s, d, t] (纵向位置、横向位置、时间)

### 3. 地图重置 (Lines 23-31)
```cpp
ErrorType SscMap::ResetSscMap(const common::FrenetState &ini_fs) {
  ClearDrivingCorridor();
  ClearGridMap();
  start_time_ = ini_fs.time_stamp;
  UpdateMapOrigin(ini_fs);
  return kSuccess;
}
```
**功能**：
- 清除之前的走廊和网格数据
- 设置新的起始时间和地图原点

### 4. 地图原点更新 (Lines 33-44)
```cpp
void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
  initial_fs_ = ori_fs;
  std::array<decimal_t, 3> map_origin;
  map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;  // s轴：当前位置-后退长度
  map_origin[1] = -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;  // d轴：居中
  map_origin[2] = ori_fs.time_stamp;  // t轴：当前时间
  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}
```
**功能**：
- 根据车辆当前状态设置3D网格的原点
- s轴：向后偏移一定距离以包含历史轨迹
- d轴：以当前位置为中心
- t轴：从当前时间开始

## 核心算法实现

### 5. 种子立方体生成 (Lines 46-56)
```cpp
ErrorType SscMap::GetInitialCubeUsingSeed(
    const Vec3i &seed_0, const Vec3i &seed_1,
    common::AxisAlignedCubeNd<int, 3> *cube) const {
  // 计算两个种子点的包围盒
  std::array<int, 3> lb, ub;
  for(int i = 0; i < 3; i++) {
    lb[i] = std::min(seed_0(i), seed_1(i));
    ub[i] = std::max(seed_0(i), seed_1(i));
  }
  *cube = common::AxisAlignedCubeNd<int, 3>(ub, lb);
  return kSuccess;
}
```
**功能**：
- 根据两个种子点创建初始的轴对齐立方体
- 形成走廊构建的起始单元

### 6. SSC地图构建主函数 (Lines 58-65)
```cpp
ErrorType SscMap::ConstructSscMap(
    const std::unordered_map<int, vec_E<common::FsVehicle>> &sur_vehicle_trajs_fs,
    const vec_E<Vec2f> &obstacle_grids) {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  FillStaticPart(obstacle_grids);        // 填充静态障碍物
  FillDynamicPart(sur_vehicle_trajs_fs); // 填充动态障碍物
  return kSuccess;
}
```
**功能**：
- 清空网格数据
- 填充静态障碍物（固定障碍物）
- 填充动态障碍物（其他车辆轨迹）

### 7. 膨胀方向设置 (Lines 67-77)
```cpp
ErrorType SscMap::GetInflationDirections(const bool &if_first_cube,
                                         std::array<bool, 6> *dirs_disabled) {
  (*dirs_disabled)[0] = false;  // X正方向
  (*dirs_disabled)[1] = false;  // X负方向
  (*dirs_disabled)[2] = false;  // Y正方向
  (*dirs_disabled)[3] = false;  // Y负方向
  (*dirs_disabled)[4] = false;  // Z正方向
  (*dirs_disabled)[5] = !if_first_cube;  // Z负方向（仅第一个立方体可向过去膨胀）
  return kSuccess;
}
```
**功能**：
- 设置立方体在6个方向的膨胀权限
- 时间轴负方向只有第一个立方体可以膨胀

## 走廊构建核心算法 (Lines 95-248)

### 8. 走廊构建主流程
```cpp
ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
```

#### Stage I: 种子点生成 (Lines 98-154)
```cpp
// ~ Stage I: Get seeds
vec_E<Vec3i> traj_seeds;
int num_states = static_cast<int>(trajs.size());
```
**功能**：
- 从输入轨迹中提取关键点作为种子
- 转换为网格坐标系
- 过滤超出范围的点

#### Stage II: 立方体膨胀 (Lines 156-248)
```cpp
// ~ Stage II: Inflate cubes
```
**关键逻辑**：
1. **初始立方体创建**：为每对相邻种子点创建初始立方体
2. **碰撞检测**：检查立方体是否与障碍物冲突
3. **立方体膨胀**：在保证无碰撞的前提下最大化立方体体积
4. **走廊连接**：处理立方体之间的连接关系

### 9. 立方体膨胀算法 (Lines 440-508)
```cpp
ErrorType SscMap::InflateCubeIn3dGrid(GridMap3D *p_grid,
                                      const std::array<bool, 6> &dir_disabled,
                                      const std::array<int, 6> &dir_step,
                                      common::AxisAlignedCubeNd<int, 3> *cube) {
```
**核心思想**：
- 在6个方向上迭代膨胀立方体
- 考虑运动学约束（最大加速度、速度限制）
- 保证膨胀后的立方体仍然无碰撞

### 10. 碰撞检测函数族 (Lines 650-746)

#### 立方体碰撞检测
```cpp
bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
```
**功能**：检查整个立方体区域是否无障碍物

#### 平面碰撞检测
```cpp
bool SscMap::CheckIfPlaneIsFreeOnXAxis(...) // X轴平面
bool SscMap::CheckIfPlaneIsFreeOnYAxis(...) // Y轴平面  
bool SscMap::CheckIfPlaneIsFreeOnZAxis(...) // Z轴平面
```
**功能**：检查立方体在特定轴上的边界平面是否无障碍物

### 11. 障碍物填充

#### 静态障碍物填充 (Lines 782-794)
```cpp
ErrorType SscMap::FillStaticPart(const vec_E<Vec2f> &obs_grid_fs) {
  for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
    // 在所有时间层填充静态障碍物
    for (int k = 0; k < config_.map_size[2]; ++k) {
      std::array<decimal_t, 3> pt = {{obs_grid_fs[i](0), obs_grid_fs[i](1),
                                      (double)k * config_.map_resolution[2]}};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
      if (p_3d_grid_->CheckCoordInRange(coord)) {
        p_3d_grid_->SetValueUsingCoordinate(coord, 100);
      }
    }
  }
}
```

#### 动态障碍物填充 (Lines 796-842)
```cpp
ErrorType SscMap::FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj) {
  // 使用OpenCV的fillPoly函数填充车辆轮廓
  cv::fillPoly(layer_mat, vv_coord_cv, 100);
}
```

## 数据结构设计

### 1. 3D网格地图
- **原始网格** (`p_3d_grid_`)：存储障碍物信息
- **膨胀网格** (`p_3d_inflated_grid_`)：考虑车辆尺寸的安全网格

### 2. 走廊表示
- `driving_corridor_vec_`：存储构建的走廊序列
- `final_corridor_vec_`：最终的全局度量走廊

### 3. 配置参数
- 地图尺寸、分辨率
- 运动学约束（最大速度、加速度）
- 膨胀参数

## 算法特点

### 1. 多阶段处理
1. **网格构建**：将连续空间离散化
2. **种子生成**：从轨迹提取关键点
3. **立方体膨胀**：最大化可行空间
4. **走廊优化**：连接和平滑处理

### 2. 安全保证
- 碰撞检测确保路径安全
- 运动学约束保证可执行性
- 车辆尺寸膨胀提供安全边界

### 3. 效率优化
- 平面检测减少计算量
- 增量膨胀避免重复计算
- 网格化提高空间查询效率

这个算法为自动驾驶提供了一个强大的空间-时间规划框架，能够在复杂的动态环境中生成安全、可行的行驶走廊。
