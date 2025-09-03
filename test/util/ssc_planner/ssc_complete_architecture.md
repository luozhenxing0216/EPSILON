# SSC Map 完整代码架构解析

## 总览

`ssc_map.cc` 实现了一个复杂的**空间-时间走廊 (Spatial-Temporal Corridor)** 规划算法，用于自动驾驶车辆在动态环境中的路径规划。该算法通过3D网格化将连续的空间-时间问题转换为离散优化问题。

## 核心概念

### 1. Frenet坐标系
- **s轴**: 沿参考线的纵向位置（前进方向）
- **d轴**: 垂直于参考线的横向偏移（左右方向）  
- **t轴**: 时间轴

### 2. 3D网格地图
```cpp
// 网格配置
map_size = {1000, 100, 81}        // s×d×t = 1000×100×81个网格
map_resolution = {0.25, 0.2, 0.1} // 分辨率: 0.25m × 0.2m × 0.1s
总覆盖范围: 250m × 20m × 8.1s
```

## 详细架构分析

### A. 数据结构层次

#### 1. 基础数据类型
```cpp
using ObstacleMapType = uint8_t;     // 障碍物网格数据类型
using SscMapDataType = uint8_t;      // SSC地图数据类型
using GridMap3D = common::GridMapND<ObstacleMapType, 3>;  // 3D网格地图
```

#### 2. 核心成员变量
```cpp
private:
    GridMap3D *p_3d_grid_;           // 原始障碍物网格
    GridMap3D *p_3d_inflated_grid_;  // 考虑车辆尺寸的膨胀网格
    Config config_;                   // 算法配置参数
    decimal_t start_time_;           // 规划起始时间
    common::FrenetState initial_fs_; // 初始Frenet状态
    
    // 结果存储
    vec_E<common::DrivingCorridor> driving_corridor_vec_;     // 驾驶走廊
    vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec_; // 最终走廊
    std::vector<int> if_corridor_valid_;  // 走廊有效性标记
```

### B. 功能模块分层

#### 第1层: 初始化与配置
```cpp
// 构造函数 - Lines 14-21
SscMap::SscMap(const Config &config)
    创建两个3D网格地图实例
    
// 地图重置 - Lines 23-31  
ResetSscMap(const FrenetState &ini_fs)
    清除历史数据，设置新的起始状态
    
// 原点更新 - Lines 33-44
UpdateMapOrigin(const FrenetState &ori_fs)
    根据车辆当前位置重新设置网格坐标系原点
```

#### 第2层: 环境建模
```cpp
// 主构建函数 - Lines 58-65
ConstructSscMap(obstacles, trajectories)
    └── FillStaticPart(obstacle_grids)     // 填充静态障碍物
    └── FillDynamicPart(vehicle_trajs)     // 填充动态障碍物

// 静态障碍物处理 - Lines 782-794
FillStaticPart(const vec_E<Vec2f> &obs_grid_fs)
    在所有时间层填充2D静态障碍物
    
// 动态障碍物处理 - Lines 826-842  
FillMapWithFsVehicleTraj(const vec_E<FsVehicle> traj)
    使用OpenCV的fillPoly填充车辆轮廓多边形
```

#### 第3层: 走廊构建核心
```cpp
// 主要算法 - Lines 95-248
ConstructCorridorUsingInitialTrajectory(grid, trajectories)

Stage I: 种子点生成 (Lines 98-154)
    └── 从输入轨迹提取关键点
    └── 转换为网格坐标
    └── 过滤无效点

Stage II: 立方体膨胀 (Lines 156-248)  
    └── GetInitialCubeUsingSeed()          // 创建初始立方体
    └── CheckIfCubeIsFree()                // 碰撞检测
    └── InflateCubeIn3dGrid()              // 立方体膨胀
    └── 走廊连接处理
```

#### 第4层: 几何算法
```cpp
// 立方体膨胀主函数 - Lines 440-508
InflateCubeIn3dGrid(grid, directions, steps, cube)
    运动学约束计算
    6方向迭代膨胀: X±, Y±, Z±
    
// 单轴膨胀函数 - Lines 510-629
InflateCubeOnXPosAxis()  // s轴正向膨胀
InflateCubeOnXNegAxis()  // s轴负向膨胀  
InflateCubeOnYPosAxis()  // d轴正向膨胀
InflateCubeOnYNegAxis()  // d轴负向膨胀
InflateCubeOnZPosAxis()  // t轴正向膨胀
InflateCubeOnZNegAxis()  // t轴负向膨胀
```

#### 第5层: 碰撞检测
```cpp
// 立方体完整性检查 - Lines 650-671
CheckIfCubeIsFree(grid, cube)
    遍历立方体内所有网格点，确保无障碍物
    
// 平面检查函数 - Lines 673-746
CheckIfPlaneIsFreeOnXAxis(grid, cube, x)  // X轴平面检查
CheckIfPlaneIsFreeOnYAxis(grid, cube, y)  // Y轴平面检查  
CheckIfPlaneIsFreeOnZAxis(grid, cube, z)  // Z轴平面检查
```

### C. 关键算法逻辑

#### 1. 种子点提取算法
```cpp
// 伪代码
for each trajectory_point in input_trajectory:
    if point is first_seed:
        add initial_state as seed_0
        add trajectory_point as seed_1
    else:
        if point is in_range and time_valid:
            add point as seed
```

#### 2. 立方体膨胀算法
```cpp
// 伪代码  
while not all_directions_finished:
    for direction in [X+, X-, Y+, Y-, Z+, Z-]:
        if not finished[direction]:
            try_inflate_in_direction()
            if collision_detected or boundary_reached:
                mark_direction_finished()
```

#### 3. 运动学约束计算
```cpp
// 位置边界计算
s_max = s0 + v0*t + 0.5*a_max*t²  // 最大可达位置
s_min = s0 + v0*t + 0.5*a_min*t²  // 最小可达位置

// 限制膨胀范围
if cube.upper_bound[0] >= s_max_index:
    stop_positive_s_expansion()
```

## 算法特色与优势

### 1. 多尺度时空建模
- **空间离散化**: 将连续空间问题转换为网格问题
- **时间分层**: 每个时间片独立处理动态障碍物
- **Frenet坐标**: 适应弯曲道路的自然坐标系

### 2. 渐进式优化
- **种子驱动**: 从轨迹关键点开始构建
- **增量膨胀**: 逐步扩大可行空间
- **碰撞约束**: 保证安全性

### 3. 高效计算策略
- **平面检测**: 减少3D碰撞检测计算量
- **方向分离**: 6个方向独立膨胀
- **早期终止**: 碰撞时立即停止膨胀

### 4. 车辆动力学集成
- **运动学边界**: 考虑加速度和速度限制
- **安全边界**: 车辆尺寸膨胀
- **时间一致性**: 确保轨迹可执行

## 复杂度分析

### 时间复杂度
- **网格填充**: O(N × M × T) - N×M为空间网格数，T为时间层数
- **碰撞检测**: O(V) - V为立方体体积
- **膨胀算法**: O(S × V) - S为膨胀步数
- **总体复杂度**: O(N × M × T + K × S × V) - K为立方体数量

### 空间复杂度
- **3D网格**: O(N × M × T) ≈ 8.1MB (1000×100×81×1byte)
- **走廊存储**: O(K × C) - K为走廊数，C为立方体数
- **总体**: O(N × M × T) 主导

## 实际应用考虑

### 1. 参数调优
```cpp
// 关键参数
inflate_steps = {20, 5, 10, 10, 1, 1}  // 6方向膨胀步长
kMaxNumOfGridAlongTime = 2              // 时间轴最大网格数  
kMaxLongitudinalAcc = 3.0               // 最大纵向加速度
kMaxLongitudinalDecel = -8.0            // 最大纵向减速度
```

### 2. 实时性优化
- **网格分辨率权衡**: 精度 vs 计算速度
- **膨胀步长调整**: 质量 vs 效率
- **并行计算机会**: 多方向膨胀可并行

### 3. 鲁棒性保证
- **边界检查**: 防止数组越界
- **有效性验证**: 每个阶段都有错误处理
- **降级策略**: 失败时提供基本可行解

## 总结

SSC算法通过巧妙的3D空间-时间建模，将复杂的动态路径规划问题转换为可处理的几何优化问题。其核心思想是在保证安全性的前提下，最大化车辆的机动空间，为上层轨迹优化器提供约束条件。

该算法的成功关键在于：
1. **合理的坐标系选择** (Frenet)
2. **高效的空间离散化** (3D网格)  
3. **渐进式的优化策略** (种子+膨胀)
4. **完善的约束处理** (碰撞+运动学)

这使得SSC成为自动驾驶领域中一个重要的规划工具。
