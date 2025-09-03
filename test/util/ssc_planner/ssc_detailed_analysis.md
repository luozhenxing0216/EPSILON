# SSC Map 逐行代码分析

## 1. 关键数据结构定义

### GridMap3D 类型定义
```cpp
using GridMap3D = common::GridMapND<ObstacleMapType, 3>;
```
- **功能**: 定义3维网格地图类型
- **模板参数**: `ObstacleMapType` (uint8_t), 维度3
- **用途**: 表示空间-时间网格，坐标轴为 [s, d, t]

### 配置结构体 Config
```cpp
struct Config {
    std::array<int, 3> map_size = {{1000, 100, 81}};               // s, d, t 网格数量
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};  // 分辨率: m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};      // 轴名称
    decimal_t s_back_len = 0.0;                                    // 向后扩展长度
    // ... 运动学约束参数
};
```

## 2. 构造函数详细分析

### Lines 14-21: 构造函数
```cpp
SscMap::SscMap(const SscMap::Config &config) : config_(config) {
  config_.Print();  // 打印配置信息，用于调试
  
  // 创建原始3D网格地图
  p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size,      // 网格尺寸 [1000, 100, 81]
      config_.map_resolution, // 分辨率 [0.25m, 0.2m, 0.1s]  
      config_.axis_name);    // 轴名称 ["s", "d", "t"]
      
  // 创建膨胀后的3D网格地图（考虑车辆尺寸）
  p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
}
```
**关键概念**:
- **s轴**: 纵向位置（前进方向），范围250m，分辨率0.25m
- **d轴**: 横向位置（左右偏移），范围20m，分辨率0.2m  
- **t轴**: 时间轴，范围8.1s，分辨率0.1s

## 3. 地图原点设置

### Lines 33-44: UpdateMapOrigin
```cpp
void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
  initial_fs_ = ori_fs;  // 保存初始Frenet状态
  
  std::array<decimal_t, 3> map_origin;
  // s轴原点: 当前位置向后偏移s_back_len
  map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;
  
  // d轴原点: 以当前位置为中心，向左偏移一半地图宽度
  map_origin[1] = -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;
  
  // t轴原点: 当前时间戳
  map_origin[2] = ori_fs.time_stamp;
  
  // 设置两个网格的原点
  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}
```
**数学计算**:
- d轴偏移量 = -(100-1) × 0.2 / 2.0 = -9.9m （向左偏移）
- 这样设计使得车辆当前位置处于d轴中心

## 4. 走廊构建核心算法

### Lines 95-154: Stage I - 种子点生成
```cpp
ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
  
  // Stage I: 从轨迹中提取种子点
  vec_E<Vec3i> traj_seeds;  // 存储网格坐标的种子点
  int num_states = static_cast<int>(trajs.size());
  
  if (num_states > 1) {
    bool first_seed_determined = false;
    
    for (int k = 0; k < num_states; ++k) {
      std::array<decimal_t, 3> p_w = {};
      
      if (!first_seed_determined) {
        // 处理第一个种子点（初始状态）
        decimal_t s_0 = initial_fs_.vec_s[0];    // 初始s位置
        decimal_t d_0 = initial_fs_.vec_dt[0];   // 初始d位置  
        decimal_t t_0 = initial_fs_.time_stamp;  // 初始时间
        std::array<decimal_t, 3> p_w_0 = {s_0, d_0, t_0};
        auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);
        
        // 处理轨迹中的第一个点
        decimal_t s_1 = trajs[k].frenet_state.vec_s[0];
        decimal_t d_1 = trajs[k].frenet_state.vec_dt[0];
        decimal_t t_1 = trajs[k].frenet_state.time_stamp;
        std::array<decimal_t, 3> p_w_1 = {s_1, d_1, t_1};
        auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);
        
        // 边界检查
        if (!p_grid->CheckCoordInRange(coord_1)) continue;
        if (coord_1[2] <= 0) continue;  // 时间不能早于起始时间
        
        first_seed_determined = true;
        traj_seeds.push_back(Vec3i(coord_0[0], coord_0[1], coord_0[2]));
        traj_seeds.push_back(Vec3i(coord_1[0], coord_1[1], coord_1[2]));
      } else {
        // 处理后续种子点
        decimal_t s = trajs[k].frenet_state.vec_s[0];
        decimal_t d = trajs[k].frenet_state.vec_dt[0];
        decimal_t t = trajs[k].frenet_state.time_stamp;
        p_w = {s, d, t};
        auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
        
        if (!p_grid->CheckCoordInRange(coord)) continue;
        traj_seeds.push_back(Vec3i(coord[0], coord[1], coord[2]));
      }
    }
  }
```

### Lines 156-248: Stage II - 立方体膨胀
```cpp
  // Stage II: 基于种子点构建和膨胀立方体
  common::DrivingCorridor driving_corridor;
  bool is_valid = true;
  auto seed_num = static_cast<int>(traj_seeds.size());
  
  if (seed_num < 2) {
    // 种子点不足，无法构建走廊
    driving_corridor.is_valid = false;
    driving_corridor_vec_.push_back(driving_corridor);
    return kWrongStatus;
  }
  
  for (int i = 0; i < seed_num; ++i) {
    if (i == 0) {
      // 处理第一个立方体
      common::AxisAlignedCubeNd<int, 3> cube;
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
      
      // 碰撞检测
      if (!CheckIfCubeIsFree(p_grid, cube)) {
        LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;
        // 记录失败的立方体并标记走廊无效
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);
        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }
      
      // 立方体膨胀
      std::array<bool, 6> dirs_disabled = {false, false, false, false, false, false};
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);
      
      // 创建驾驶立方体
      common::DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);
      
    } else {
      // 处理后续立方体
      if (CheckIfCubeContainsSeed(driving_corridor.cubes.back().cube, traj_seeds[i])) {
        // 种子点在当前立方体内，添加到种子列表
        driving_corridor.cubes.back().seeds.push_back(traj_seeds[i]);
        continue;
      } else {
        // 种子点超出当前立方体，需要创建新立方体
        
        // 获取当前立方体的最后一个种子
        Vec3i seed_r = driving_corridor.cubes.back().seeds.back();
        driving_corridor.cubes.back().seeds.pop_back();
        
        // 在时间轴上切割当前立方体
        driving_corridor.cubes.back().cube.upper_bound[2] = seed_r(2);
        i = i - 1;  // 重新处理当前种子点
        
        // 创建新的立方体
        common::AxisAlignedCubeNd<int, 3> cube;
        GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
        
        if (!CheckIfCubeIsFree(p_grid, cube)) {
          // 新立方体碰撞，标记失败
          LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;
          common::DrivingCube driving_cube;
          driving_cube.cube = cube;
          driving_cube.seeds.push_back(traj_seeds[i]);
          driving_cube.seeds.push_back(traj_seeds[i + 1]);
          driving_corridor.cubes.push_back(driving_cube);
          driving_corridor.is_valid = false;
          driving_corridor_vec_.push_back(driving_corridor);
          is_valid = false;
          break;
        }
        
        // 膨胀新立方体
        std::array<bool, 6> dirs_disabled = {false, false, false, false, false, false};
        InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);
        
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_corridor.cubes.push_back(driving_cube);
      }
    }
  }
  
  if (is_valid) {
    // 成功构建走廊，进行最后的时间轴切割
    driving_corridor.cubes.back().cube.upper_bound[2] = traj_seeds.back()(2);
    driving_corridor.is_valid = true;
    driving_corridor_vec_.push_back(driving_corridor);
  }
  
  return kSuccess;
}
```

## 5. 立方体膨胀算法

### Lines 440-508: InflateCubeIn3dGrid
```cpp
ErrorType SscMap::InflateCubeIn3dGrid(GridMap3D *p_grid,
                                      const std::array<bool, 6> &dir_disabled,
                                      const std::array<int, 6> &dir_step,
                                      common::AxisAlignedCubeNd<int, 3> *cube) {
  // 方向状态标记
  bool x_p_finish = dir_disabled[0];  // X正方向完成
  bool x_n_finish = dir_disabled[1];  // X负方向完成
  bool y_p_finish = dir_disabled[2];  // Y正方向完成
  bool y_n_finish = dir_disabled[3];  // Y负方向完成
  bool z_p_finish = dir_disabled[4];  // Z正方向完成
  
  // 膨胀步长
  int x_p_step = dir_step[0];  // 默认20步
  int x_n_step = dir_step[1];  // 默认5步
  int y_p_step = dir_step[2];  // 默认10步
  int y_n_step = dir_step[3];  // 默认10步
  int z_p_step = dir_step[4];  // 默认1步
  
  // 运动学约束计算
  int t_max_grids = cube->lower_bound[2] + config_.kMaxNumOfGridAlongTime;
  decimal_t t = t_max_grids * p_grid->dims_resolution(2);  // 时间长度
  decimal_t a_max = config_.kMaxLongitudinalAcc;           // 最大加速度 3.0 m/s²
  decimal_t a_min = config_.kMaxLongitudinalDecel;         // 最大减速度 -8.0 m/s²
  decimal_t d_comp = initial_fs_.vec_s[1] * 1;             // 速度补偿
  
  // 计算运动学可达边界
  decimal_t s_u = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t + 
                  0.5 * a_max * t * t + d_comp;  // 最大可达位置
  decimal_t s_l = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t + 
                  0.5 * a_min * t * t - d_comp;  // 最小可达位置
  
  // 转换为网格坐标
  int s_idx_u, s_idx_l;
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);
  s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len / 2.0) / 
                                               config_.map_resolution[0]));
  
  // 在X和Y方向上迭代膨胀
  while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
    if (!x_p_finish) x_p_finish = InflateCubeOnXPosAxis(p_grid, x_p_step, cube);
    if (!x_n_finish) x_n_finish = InflateCubeOnXNegAxis(p_grid, x_n_step, cube);
    if (!y_p_finish) y_p_finish = InflateCubeOnYPosAxis(p_grid, y_p_step, cube);
    if (!y_n_finish) y_n_finish = InflateCubeOnYNegAxis(p_grid, y_n_step, cube);
    
    // 运动学边界检查
    if (cube->upper_bound[0] >= s_idx_u) x_p_finish = true;
    if (cube->lower_bound[0] <= s_idx_l) x_n_finish = true;
  }
  
  // 在Z方向（时间轴）上膨胀
  while (!z_p_finish) {
    if (!z_p_finish) z_p_finish = InflateCubeOnZPosAxis(p_grid, z_p_step, cube);
    
    // 时间轴长度限制
    if (cube->upper_bound[2] - cube->lower_bound[2] >= config_.kMaxNumOfGridAlongTime) {
      z_p_finish = true;
    }
  }
  
  return kSuccess;
}
```

## 6. 碰撞检测函数

### Lines 650-671: CheckIfCubeIsFree
```cpp
bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
  // 获取立方体边界
  int f0_min = cube.lower_bound[0];  // s轴下界
  int f0_max = cube.upper_bound[0];  // s轴上界
  int f1_min = cube.lower_bound[1];  // d轴下界
  int f1_max = cube.upper_bound[1];  // d轴上界
  int f2_min = cube.lower_bound[2];  // t轴下界
  int f2_max = cube.upper_bound[2];  // t轴上界
  
  // 遍历立方体内的所有网格点
  int i, j, k;
  std::array<int, 3> coord;
  bool is_free;
  for (i = f0_min; i <= f0_max; ++i) {
    for (j = f1_min; j <= f1_max; ++j) {
      for (k = f2_min; k <= f2_max; ++k) {
        coord = {i, j, k};
        p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {  // 发现障碍物
          return false;
        }
      }
    }
  }
  return true;  // 整个立方体都是自由空间
}
```

## 7. 障碍物填充

### Lines 826-842: FillMapWithFsVehicleTraj (动态障碍物)
```cpp
ErrorType SscMap::FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj) {
  if (traj.size() == 0) {
    LOG(ERROR) << "[Ssc] SscMap - Trajectory is empty.";
    return kWrongStatus;
  }
  
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    // 验证轨迹点有效性
    bool is_valid = true;
    for (const auto v : traj[i].vertices) {
      if (v(0) <= 0) {  // s坐标无效
        is_valid = false;
        break;
      }
    }
    if (!is_valid) continue;
    
    // 获取时间信息
    decimal_t z = traj[i].frenet_state.time_stamp;
    int t_idx = 0;
    
    // 转换车辆顶点到网格坐标
    std::vector<common::Point2i> v_coord;
    std::array<decimal_t, 3> p_w;
    for (const auto v : traj[i].vertices) {
      p_w = {v(0), v(1), z};  // 3D点 [s, d, t]
      auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];
      if (!p_grid->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord.push_back(common::Point2i(coord[0], coord[1]));  // 2D投影
    }
    if (!is_valid) continue;
    
    // 使用OpenCV填充多边形
    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    std::vector<cv::Point2i> v_coord_cv;
    common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord, &v_coord_cv);
    vv_coord_cv.push_back(v_coord_cv);
    
    // 获取对应时间层的2D切片
    int w = p_3d_grid_->dims_size()[0];  // s轴大小
    int h = p_3d_grid_->dims_size()[1];  // d轴大小
    int layer_offset = t_idx * w * h;    // 时间层偏移
    cv::Mat layer_mat = cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
                                p_3d_grid_->get_data_ptr() + layer_offset);
    
    // 填充车辆占用区域
    cv::fillPoly(layer_mat, vv_coord_cv, 100);  // 100表示障碍物
  }
  
  return kSuccess;
}
```

## 算法精髓总结

1. **空间离散化**: 将连续的3D空间-时间问题转换为离散网格问题
2. **渐进式构建**: 从种子点开始，逐步膨胀立方体以最大化可行空间
3. **运动学约束**: 确保生成的走廊符合车辆动力学限制
4. **安全保证**: 通过碰撞检测确保整个走廊无障碍物
5. **时间一致性**: 考虑动态障碍物的时间演化

这个算法的核心思想是在保证安全性的前提下，为自动驾驶车辆提供最大的机动空间。
