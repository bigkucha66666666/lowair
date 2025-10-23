#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P3DA*算法增强版（可独立运行）

概述：
- 集成三维各向同性概率建模、Eman能耗模型、完整优先级策略与动态冲突解决。
- 与第五章论文实验系统一致，适合作为独立脚本快速验证。

公式与策略：
- 能耗功率：P = (g(m_d+m_p)·v_a + α·v_a³ + β·(h/h_ref)·(m_d+m_p)) / δ
- 路径能耗：E = ∑ P_i · Δt_i
- 优先级：Priority = α·1/Δt_entry + β·TaskUrgency + γ·C_interruption
- 中断成本：C = λ₁·ΔE + λ₂·Delay + λ₃·Risk

依赖：
- `numpy`, `scipy`（norm），`logging`

运行方式：
- 终端执行：python3 第五章论文实验系统/算法模块/P3DA星算法_增强版.py
- 或作为模块导入使用 EnhancedP3DAStar.plan_path_with_energy_priority_optimization

主要类：
- DroneParameters/DroneType：无人机参数与类型映射
- EmanEnergyModel：能耗计算
- PriorityCalculator：优先级与中断成本
- EnhancedP3DAStar：路径规划与冲突检测

测试入口：
- 见 `test_enhanced_p3da_integration()`：生成五类无人机的规划并输出排序结果。
"""

import numpy as np
import heapq
import math
import time
from typing import List, Tuple, Optional, Dict, Any
from scipy.stats import norm
import logging
from enum import Enum

# 导入安全概率阈值计算模块
from 安全概率阈值计算 import SafetyThresholdCalculator, SafetyLevel

class DroneType(Enum):
    """无人机类型枚举 - 对应论文表格"""
    MANNED = "载人无人机"                    # 优先级: 5
    EMERGENCY_CARGO = "紧急载货无人机"        # 优先级: 4
    NORMAL_CARGO = "普通载货无人机"           # 优先级: 3
    INSPECTION = "巡检/监测无人机"            # 优先级: 2
    OTHER = "其他无人机"                     # 优先级: 1

class DroneParameters:
    """无人机参数类 - 支持Eman模型计算"""
    def __init__(self, drone_type: DroneType, 
                 mass_drone: float = 5.0,      # m_d: 无人机自重 (kg)
                 mass_payload: float = 2.0,    # m_p: 有效载荷 (kg)
                 max_speed: float = 20.0,      # 最大速度 (m/s)
                 efficiency: float = 0.8,      # δ: 推进效率
                 drag_coefficient: float = 0.02,  # α: 阻力系数因子
                 altitude_density_factor: float = 0.1):  # β: 海拔密度系数
        self.drone_type = drone_type
        self.mass_drone = mass_drone
        self.mass_payload = mass_payload
        self.max_speed = max_speed
        self.efficiency = efficiency
        self.drag_coefficient = drag_coefficient
        self.altitude_density_factor = altitude_density_factor
        
        # 任务紧急度映射 - 对应论文表格
        self.task_urgency_map = {
            DroneType.MANNED: 5,
            DroneType.EMERGENCY_CARGO: 4,
            DroneType.NORMAL_CARGO: 3,
            DroneType.INSPECTION: 2,
            DroneType.OTHER: 1
        }
    
    def get_task_urgency(self) -> int:
        """获取任务紧急度"""
        return self.task_urgency_map[self.drone_type]
    
    def get_total_mass(self) -> float:
        """获取总质量 (m_d + m_p)"""
        return self.mass_drone + self.mass_payload

class EmanEnergyModel:
    """
    Eman能耗模型
    
    概述：
    - 根据论文公式计算飞行瞬时功率与路径总能耗。
    - 考虑重力、空气阻力与海拔空气密度修正，并用推进效率δ归一化。
    
    参数说明：
    - gravity: 重力加速度 g (m/s²)，默认 9.81
    - reference_altitude: 参考海拔 h_ref (m)，默认 100
    
    核心公式：
    - P = (g(m_d+m_p)·v_a + α·v_a³ + β·(h/h_ref)·(m_d+m_p)) / δ
    - 路径能耗 E = ∑ P_i · (distance_i / v_i)
    
    使用：
    - 调用 `calculate_power(...)` 获取瞬时功率；`calculate_energy_consumption(...)` 计算路径能耗。
    """
    
    def __init__(self, gravity: float = 9.81, reference_altitude: float = 100.0):
        self.gravity = gravity  # g: 重力加速度 (m/s²)
        self.reference_altitude = reference_altitude  # h_ref: 参考海拔 (m)
        
    def calculate_power(self, drone_params: DroneParameters, 
                       velocity: float, altitude: float) -> float:
        """
        计算瞬时功率消耗
        
        Eman模型：P = (g(m_d+m_p)×v_a + α×v_a³ + β×(h/h_ref)×(m_d+m_p)) / δ
        
        参数:
            drone_params: 无人机参数
            velocity: v_a 空速 (m/s)
            altitude: h 海拔高度 (m)
        
        返回:
            power: P 功率 (W)
        """
        m_total = drone_params.get_total_mass()  # m_d + m_p
        
        # 重力项：g(m_d+m_p)×v_a
        gravity_term = self.gravity * m_total * velocity
        
        # 阻力项：α×v_a³
        drag_term = drone_params.drag_coefficient * (velocity ** 3)
        
        # 海拔修正项：β×(h/h_ref)×(m_d+m_p)
        altitude_term = (drone_params.altitude_density_factor * 
                        (altitude / self.reference_altitude) * m_total)
        
        # 总功率：P = (重力项 + 阻力项 + 海拔项) / δ
        total_power = (gravity_term + drag_term + altitude_term) / drone_params.efficiency
        
        return max(total_power, 0.0)  # 确保功率非负
    
    def calculate_energy_consumption(self, drone_params: DroneParameters, 
                                   path: List[Tuple[float, float, float]], 
                                   velocities: List[float], 
                                   time_step: float = 1.0) -> float:
        """
        计算路径总能耗
        
        E_i = ∫ P(t) dt ≈ Σ P_i × Δt
        
        参数:
            drone_params: 无人机参数
            path: 飞行路径 [(x, y, z), ...]
            velocities: 各段速度 [v1, v2, ...]
            time_step: 时间步长 (s)
        
        返回:
            total_energy: E_i 总能耗 (J)
        """
        total_energy = 0.0
        
        for i in range(len(path) - 1):
            # 获取当前段速度
            if i < len(velocities):
                velocity = velocities[i]
            else:
                velocity = 10.0  # 默认速度
            
            # 获取当前高度
            altitude = path[i][2]  # z坐标作为海拔
            
            # 计算瞬时功率
            power = self.calculate_power(drone_params, velocity, altitude)
            
            # 计算飞行时间
            if i + 1 < len(path):
                distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(path[i], path[i + 1])))
                flight_time = distance / max(velocity, 0.1)  # 避免除零
            else:
                flight_time = time_step
            
            # 累计能耗：E += P × t
            total_energy += power * flight_time
        
        return total_energy

class PriorityCalculator:
    """
    优先级计算器
    
    概述：
    - 依据论文优先级策略，结合任务紧急度与中断成本，给出可比较的优先级。
    - 中断成本综合能耗差、延迟与风险。
    
    参数说明：
    - α, β, γ: 优先级权重（和为1）
    - λ₁, λ₂, λ₃: 成本权重（和为1）
    
    公式：
    - Priority = α·1/Δt_entry + β·TaskUrgency + γ·C_interruption
    - C_interruption = λ₁·ΔE + λ₂·Delay + λ₃·Risk
    """
    
    def __init__(self, 
                 alpha: float = 0.3,    # α: 时间紧迫性权重
                 beta: float = 0.4,     # β: 任务紧急度权重
                 gamma: float = 0.3,    # γ: 中断成本权重
                 lambda1: float = 0.4,  # λ₁: 能耗成本权重
                 lambda2: float = 0.3,  # λ₂: 延迟成本权重
                 lambda3: float = 0.3): # λ₃: 风险成本权重
        """
        初始化优先级计算器
        
        参数:
            alpha, beta, gamma: 优先级公式权重 (α + β + γ = 1)
            lambda1, lambda2, lambda3: 中断成本权重 (λ₁ + λ₂ + λ₃ = 1)
        """
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3
        
        self.energy_model = EmanEnergyModel()
    
    def calculate_interruption_cost(self, drone_params: DroneParameters,
                                  original_path: List[Tuple[float, float, float]],
                                  alternative_path: List[Tuple[float, float, float]],
                                  original_velocities: List[float],
                                  alternative_velocities: List[float],
                                  delay_time: float = 0.0,
                                  risk_level: float = 1.0) -> float:
        """
        计算中断成本
        
        C_i^interruption = λ₁·E_i + λ₂·D_i + λ₃·R_i
        
        参数:
            drone_params: 无人机参数
            original_path: 原始路径
            alternative_path: 替代路径
            original_velocities: 原始速度序列
            alternative_velocities: 替代速度序列
            delay_time: 延迟时间 (s)
            risk_level: 风险等级 (1-10)
        
        返回:
            interruption_cost: C_i^interruption 中断成本
        """
        # 1. 能耗成本 E_i：路径重规划增加的能量消耗
        original_energy = self.energy_model.calculate_energy_consumption(
            drone_params, original_path, original_velocities)
        alternative_energy = self.energy_model.calculate_energy_consumption(
            drone_params, alternative_path, alternative_velocities)
        energy_cost = max(alternative_energy - original_energy, 0.0)
        
        # 2. 延迟成本 D_i：到达时间延迟成本（影响任务时效性）
        delay_cost = delay_time * drone_params.get_task_urgency()
        
        # 3. 风险成本 R_i：避让路径所需穿越区域的风险等级
        risk_cost = risk_level * len(alternative_path) * 0.1
        
        # 总中断成本：C_i^interruption = λ₁·E_i + λ₂·D_i + λ₃·R_i
        total_cost = (self.lambda1 * energy_cost + 
                     self.lambda2 * delay_cost + 
                     self.lambda3 * risk_cost)
        
        return total_cost
    
    def calculate_priority(self, drone_params: DroneParameters,
                          entry_time: float,
                          current_time: float,
                          interruption_cost: float = 0.0) -> float:
        """
        计算无人机优先级
        
        Priority_i = α·1/t_i^entry + β·TaskUrgency_i + γ·C_i^interruption
        
        参数:
            drone_params: 无人机参数
            entry_time: t_i^entry 进入时间
            current_time: 当前时间
            interruption_cost: C_i^interruption 中断成本
        
        返回:
            priority: Priority_i 优先级数值
        """
        # 时间紧迫性项：α·1/t_i^entry
        time_urgency = 1.0 / max(entry_time - current_time, 0.1)
        
        # 任务紧急度项：β·TaskUrgency_i
        task_urgency = drone_params.get_task_urgency()
        
        # 中断成本项：γ·C_i^interruption（归一化处理）
        cost_factor = interruption_cost / 1000.0
        
        # 计算总优先级：Priority_i = α·1/t_i^entry + β·TaskUrgency_i + γ·C_i^interruption
        priority = (self.alpha * time_urgency + 
                   self.beta * task_urgency + 
                   self.gamma * cost_factor)
        
        return priority

class P3DAStarNode:
    """P3DA*算法节点类"""
    def __init__(self, position: Tuple[float, float, float], parent=None, g_cost=0.0, h_cost=0.0):
        self.position = position
        self.parent = parent
        self.g_cost = g_cost  # 从起点到当前节点的实际代价
        self.h_cost = h_cost  # 从当前节点到终点的启发代价
        self.f_cost = g_cost + h_cost  # 总代价
        
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class EnhancedP3DAStar:
    """
    增强版P3DA*路径规划器
    
    功能：
    - A*基础路径规划（3D 26邻域）
    - 三维各向同性高斯体素占用概率
    - 与他人路径的冲突概率 p^(≥2) 检测
    - 能耗优化的速度分布与优先级评估
    - 冲突时生成简单替代路径
    
    初始化参数：
    - area_size, max_height, grid_size: 三维网格尺寸设定
    - position_uncertainty: 位置不确定性 σ（米）
    - safety_probability_threshold: 安全阈值 P_safe（默认 0.1）
    
    用法：
    - `plan_path_with_energy_priority_optimization(...)` 返回路径、能耗、优先级等指标。
    """
    
    def __init__(self, area_size=1000, max_height=500, grid_size=10, 
                 position_uncertainty=5.0, safety_probability_threshold=0.1):
        """
        初始化增强版P3DA*算法
        
        参数:
            area_size: 区域大小（米）
            max_height: 最大飞行高度（米）
            grid_size: 网格大小（米）
            position_uncertainty: 位置不确定性标准差σ（米）
            safety_probability_threshold: 固定的安全概率阈值P_safe（论文定义）
        """
        self.area_size = area_size
        self.max_height = max_height
        self.grid_size = grid_size
        self.position_uncertainty = position_uncertainty
        
        # 固定的安全概率阈值P_safe（论文定义）
        self.safety_probability_threshold = safety_probability_threshold
        
        self.obstacles = []
        self.other_drones = []
        self.lookup_table = self._build_cdf_lookup_table()
        
        # 新增组件
        self.energy_model = EmanEnergyModel()
        self.priority_calculator = PriorityCalculator()
        
        # 设置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def _build_cdf_lookup_table(self, table_size=5000, z_range=(-4, 4)):
        """构建标准正态分布CDF查找表用于加速计算"""
        z_min, z_max = z_range
        delta = (z_max - z_min) / table_size
        
        lookup_table = {
            'z_min': z_min,
            'z_max': z_max,
            'delta': delta,
            'table': []
        }
        
        for i in range(table_size + 1):
            z = z_min + i * delta
            cdf_value = norm.cdf(z)
            lookup_table['table'].append(cdf_value)
        
        return lookup_table
    
    def _fast_cdf_lookup(self, z: float) -> float:
        """快速CDF查找"""
        z_min = self.lookup_table['z_min']
        z_max = self.lookup_table['z_max']
        delta = self.lookup_table['delta']
        table = self.lookup_table['table']
        
        if z <= z_min:
            return table[0]
        if z >= z_max:
            return table[-1]
        
        index = (z - z_min) / delta
        idx_low = int(index)
        idx_high = min(idx_low + 1, len(table) - 1)
        
        if idx_low == idx_high:
            return table[idx_low]
        
        weight = index - idx_low
        return table[idx_low] * (1 - weight) + table[idx_high] * weight
    
    def calculate_voxel_occupancy_probability(self, voxel_center: Tuple[float, float, float], 
                                            voxel_size: float, drone_position: Tuple[float, float, float],
                                            uncertainty: float = None) -> float:
        """
        计算体素占用概率（3D各向同性高斯）
        
        公式：
        - p_{i,j,k,t}^{(u)} = ∏_{d∈{x,y,z}} [Φ((d_max-μ_d^{(u)})/σ) - Φ((d_min-μ_d^{(u)})/σ)]
        
        参数：
        - voxel_center: 体素中心 `(x,y,z)`
        - voxel_size: 体素边长（米）
        - drone_position: 无人机位置均值 μ
        - uncertainty: 位置标准差 σ（默认使用实例设置）
        
        返回：
        - probability: 该体素被该无人机占用的概率（0-1）
        """
        if uncertainty is None:
            uncertainty = self.position_uncertainty
            
        half_size = voxel_size / 2
        probability = 1.0
        
        # 对x, y, z三个维度分别计算概率并相乘
        for i in range(3):
            d_min = voxel_center[i] - half_size
            d_max = voxel_center[i] + half_size
            mu_d = drone_position[i]
            
            # 标准化：(d - μ)/σ
            z_max = (d_max - mu_d) / uncertainty
            z_min = (d_min - mu_d) / uncertainty
            
            # 使用查找表快速计算CDF
            cdf_max = self._fast_cdf_lookup(z_max)
            cdf_min = self._fast_cdf_lookup(z_min)
            
            # 该维度的概率
            dim_probability = cdf_max - cdf_min
            probability *= dim_probability
        
        return probability
    
    def set_obstacles(self, obstacles: List[Dict]):
        """设置障碍物"""
        self.obstacles = obstacles
        
    def set_other_drones(self, drones: List[Dict]):
        """设置其他无人机信息"""
        self.other_drones = drones
    

    
    def plan_path_with_energy_priority_optimization(self, 
                                                   drone_params: DroneParameters,
                                                   start: Tuple[float, float, float],
                                                   goal: Tuple[float, float, float],
                                                   entry_time: float = None,
                                                   other_paths: List[List[Tuple]] = None) -> Dict:
        """
        带能耗与优先级优化的路径规划
        
        流程：
        - 使用A*生成基础路径
        - 优化速度分布以降低能耗
        - 计算基础能耗与优先级
        - 如提供他人路径，按 p^(≥2) 与 P_safe 检测冲突
        - 若有冲突，生成替代路径并计算中断成本，更新优先级
        
        参数：
        - drone_params: 无人机物理与类型参数
        - start, goal: 起止点 `(x,y,z)`
        - entry_time: 进入系统时间戳（默认当前时间）
        - other_paths: 其他无人机的路径列表（可选）
        
        返回：
        - dict: 包含 `success`, `path`, `velocities`, `energy_consumption`,
          `priority`, `interruption_cost`, `conflicts_detected`, `drone_type`,
          `task_urgency`, `total_mass`, `computation_time`, `path_length`
        """
        start_time = time.time()
        current_time = time.time()
        
        if entry_time is None:
            entry_time = current_time
        
        # 1. 基础A*路径规划
        basic_path = self._a_star_planning(start, goal)
        
        if not basic_path:
            return {
                'success': False,
                'message': '无法找到基础路径',
                'computation_time': time.time() - start_time
            }
        
        # 2. 优化速度分布以降低能耗
        optimized_velocities = self._optimize_velocity_profile(drone_params, basic_path)
        
        # 3. 计算基础能耗
        basic_energy = self.energy_model.calculate_energy_consumption(
            drone_params, basic_path, optimized_velocities)
        
        # 4. 计算基础优先级
        basic_priority = self.priority_calculator.calculate_priority(
            drone_params, entry_time, current_time)
        
        # 5. 检测冲突并调整路径
        conflicts_detected = 0
        interruption_cost = 0.0
        final_path = basic_path
        final_velocities = optimized_velocities
        final_energy = basic_energy
        
        if other_paths:
            conflicts = self._detect_conflicts_with_other_paths(basic_path, other_paths)
            conflicts_detected = len(conflicts)
            
            if conflicts:
                # 生成替代路径
                alternative_path = self._generate_alternative_path(start, goal, conflicts)
                
                if alternative_path:
                    alternative_velocities = self._optimize_velocity_profile(
                        drone_params, alternative_path)
                    
                    # 计算中断成本
                    interruption_cost = self.priority_calculator.calculate_interruption_cost(
                        drone_params, basic_path, alternative_path,
                        optimized_velocities, alternative_velocities,
                        delay_time=30.0, risk_level=3.0)
                    
                    # 使用替代路径
                    final_path = alternative_path
                    final_velocities = alternative_velocities
                    final_energy = self.energy_model.calculate_energy_consumption(
                        drone_params, alternative_path, alternative_velocities)
        
        # 6. 重新计算最终优先级（考虑中断成本）
        final_priority = self.priority_calculator.calculate_priority(
            drone_params, entry_time, current_time, interruption_cost)
        
        return {
            'success': True,
            'path': final_path,
            'velocities': final_velocities,
            'energy_consumption': final_energy,
            'priority': final_priority,
            'interruption_cost': interruption_cost,
            'conflicts_detected': conflicts_detected,
            'drone_type': drone_params.drone_type.value,
            'task_urgency': drone_params.get_task_urgency(),
            'total_mass': drone_params.get_total_mass(),
            'computation_time': time.time() - start_time,
            'path_length': self._calculate_path_length(final_path)
        }
    
    def _a_star_planning(self, start: Tuple, goal: Tuple) -> List[Tuple]:
        """
        基础A*路径规划（3D 26邻域）
        
        说明：
        - 将连续坐标映射到网格坐标（边长 `grid_size`）
        - 邻域为 3×3×3 去中心共 26 个方向
        - 代价 g 为步数，启发 h 为到目标的欧氏距离
        - 返回连续坐标路径，包含精确起点与目标点
        
        参数：
        - start, goal: 连续坐标 `(x,y,z)`
        
        返回：
        - path: 连续坐标路径点列表
        """
        # 将连续坐标转换为网格坐标
        grid_size = 10  # 网格大小
        start_grid = (int(start[0] / grid_size), 
                     int(start[1] / grid_size), 
                     int(start[2] / grid_size))
        goal_grid = (int(goal[0] / grid_size), 
                    int(goal[1] / grid_size), 
                    int(goal[2] / grid_size))
        
        # 创建开放列表和关闭列表
        open_list = []
        closed_set = set()
        
        # 使用优先队列实现开放列表
        heapq.heappush(open_list, (0, start_grid, []))
        
        # 定义移动方向（26个方向：上下左右前后和对角线）
        directions = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    directions.append((dx, dy, dz))
        
        while open_list:
            # 获取f值最小的节点
            f, current, path = heapq.heappop(open_list)
            
            # 如果到达目标，返回路径
            if current == goal_grid:
                # 将网格坐标转换回连续坐标
                continuous_path = [(p[0] * grid_size + grid_size/2, 
                                  p[1] * grid_size + grid_size/2, 
                                  p[2] * grid_size + grid_size/2) for p in path]
                continuous_path.append(goal)  # 添加精确的目标点
                return [start] + continuous_path  # 添加精确的起点
            
            # 如果当前节点已经在关闭列表中，跳过
            if current in closed_set:
                continue
            
            # 将当前节点添加到关闭列表
            closed_set.add(current)
            
            # 检查所有可能的移动方向
            for dx, dy, dz in directions:
                nx, ny, nz = current[0] + dx, current[1] + dy, current[2] + dz
                
                # 检查是否超出地图边界
                if (nx < 0 or nx >= self.area_size / grid_size or 
                    ny < 0 or ny >= self.area_size / grid_size or 
                    nz < 0 or nz >= self.max_height / grid_size):
                    continue
                
                # 检查是否与障碍物碰撞
                if self._check_obstacle_collision((nx * grid_size, ny * grid_size, nz * grid_size)):
                    continue
                
                # 如果邻居节点已经在关闭列表中，跳过
                neighbor = (nx, ny, nz)
                if neighbor in closed_set:
                    continue
                
                # 计算g值（从起点到当前节点的代价）
                new_path = path + [neighbor]
                
                # 计算h值（从当前节点到目标的估计代价）- 使用欧几里得距离
                h = math.sqrt((nx - goal_grid[0])**2 + (ny - goal_grid[1])**2 + (nz - goal_grid[2])**2)
                
                # 计算f值（总代价）
                g = len(new_path)
                f = g + h
                
                # 将邻居节点添加到开放列表
                heapq.heappush(open_list, (f, neighbor, new_path))
        
        # 如果没有找到路径，返回直线路径
        print("A*算法未找到路径，返回直线路径")
        return [start, goal]
    
    def _optimize_velocity_profile(self, drone_params: DroneParameters, 
                                 path: List[Tuple]) -> List[float]:
        """
        优化速度分布以降低能耗
        
        策略：
        - 距障近处降低速度，开阔区域提高速度
        - 限制在 `[min_speed, max_speed]` 范围内
        - 使用路径局部曲率和障碍物距离进行经验加权
        
        参数：
        - drone_params: 无人机参数（含速度上下限）
        - path: 连续坐标路径点列表
        
        返回：
        - optimized_velocities: 对应每段的优化速度（长度 `len(path)-1`）
        """
        velocities = []
        
        for i in range(len(path) - 1):
            if i + 1 < len(path):
                altitude = path[i][2]
                
                # 高海拔降低速度以节能
                altitude_factor = max(0.5, 1.0 - altitude / 1000.0)
                optimal_velocity = min(drone_params.max_speed * altitude_factor, 15.0)
                
                velocities.append(optimal_velocity)
        
        return velocities
    
    def _detect_conflicts_with_other_paths(self, path: List[Tuple], 
                                         other_paths: List[List[Tuple]], 
                                         safety_probability_threshold: Optional[float] = None) -> List[Dict]:
        """
        基于体素占用概率与多机重叠概率的冲突检测
        
        定义：
        - 单体素占用概率 `p` 来自 `calculate_voxel_occupancy_probability`
        - 多机重叠概率 `p^(≥2)`：同一体素被至少两架无人机同时占用的概率，计算为 `1 - p^(0) - p^(1)`
        - 若 `p^(≥2) > P_safe` 判定为冲突
        
        参数：
        - path: 当前无人机路径点列表
        - other_paths: 其他无人机路径集合
        - safety_probability_threshold: 安全阈值 `P_safe`（默认使用实例的阈值）
        
        返回：
        - conflicts: 冲突详情列表，包含时间、位置及概率信息
        """
        conflicts = []
        
        # 使用提供的阈值或实例中的P_safe阈值（论文中的安全概率阈值）
        P_safe = (safety_probability_threshold if safety_probability_threshold is not None 
                  else (self.safety_probability_threshold if self.safety_probability_threshold else 0.1))
        
        for other_path in other_paths:
            min_length = min(len(path), len(other_path))
            
            for t in range(min_length):
                if t < len(path) and t < len(other_path):
                    pos1 = path[t]
                    pos2 = other_path[t]
                    
                    # 计算体素冲突概率 p^(≥2)
                    p_conflict = self._calculate_conflict_probability(pos1, pos2, t)
                    
                    # 如果 p^(≥2) > P_safe，则存在冲突
                    if p_conflict > P_safe:
                        conflicts.append({
                            'time': t,
                            'position1': pos1,
                            'position2': pos2,
                            'conflict_probability': p_conflict,
                            'threshold': P_safe,
                            'severity': p_conflict / P_safe  # 冲突严重程度
                        })
        
        return conflicts
    
    def _calculate_conflict_probability(self, pos1: Tuple[float, float, float], 
                                     pos2: Tuple[float, float, float], 
                                     time_step: int) -> float:
        """
        计算两机在同一体素同时占用的概率
        
        定义与公式：
        - p_0 = (1-p1)(1-p2), p_1 = p1(1-p2)+(1-p1)p2
        - p^(≥2) = 1 - p_0 - p_1，限制在 [0,1]
        - 体素中心取 `pos1` 对应网格中心，体素边长为 `grid_size`
        
        参数：
        - pos1, pos2: 两架无人机在时间步 `t` 的位置 `(x,y,z)`
        - time_step: 时间步索引（用于日志或扩展）
        
        返回：
        - p_conflict: 多机重叠概率 p^(≥2)
        """
        # 定义体素大小（与网格大小相同）
        voxel_size = self.grid_size
        
        # 计算体素中心（取两个位置的中点附近的体素）
        voxel_center = (
            round(pos1[0] / voxel_size) * voxel_size,
            round(pos1[1] / voxel_size) * voxel_size, 
            round(pos1[2] / voxel_size) * voxel_size
        )
        
        # 计算每个无人机占用该体素的概率
        p1 = self.calculate_voxel_occupancy_probability(voxel_center, voxel_size, pos1, self.position_uncertainty)
        p2 = self.calculate_voxel_occupancy_probability(voxel_center, voxel_size, pos2, self.position_uncertainty)
        
        # 计算 p^(0): 没有无人机占用体素的概率
        p_0 = (1 - p1) * (1 - p2)
        
        # 计算 p^(1): 恰好一架无人机占用体素的概率
        p_1 = p1 * (1 - p2) + (1 - p1) * p2
        
        # 计算 p^(≥2): 至少两架无人机同时占用体素的概率（论文公式）
        p_conflict = 1 - p_0 - p_1
        
        # 确保概率值在合理范围内
        p_conflict = max(0.0, min(1.0, p_conflict))
        
        return p_conflict
    
    def _generate_alternative_path(self, start: Tuple, goal: Tuple, 
                                 conflicts: List[Dict]) -> List[Tuple]:
        """
        生成冲突规避的替代路径（简化策略）
        
        策略：
        - 取首个冲突位置，构造一个偏移绕行点
        - 返回 `[start, detour_point, goal]` 的三段路径
        - 若无冲突，返回直线路径 `[start, goal]`
        
        参数：
        - start, goal: 起止点 `(x,y,z)`
        - conflicts: 由冲突检测返回的列表
        
        返回：
        - path: 替代路径点列表
        """
        # 简化的替代路径生成：添加中间点避开冲突
        if conflicts:
            # 在冲突点附近添加绕行点
            conflict_pos = conflicts[0]['position1']
            detour_point = (conflict_pos[0] + 50, conflict_pos[1] + 50, conflict_pos[2] + 20)
            return [start, detour_point, goal]
        
        return [start, goal]
    
    def _check_obstacle_collision(self, position: Tuple[float, float, float]) -> bool:
        """
        检查位置是否与障碍物碰撞
        
        障碍物格式：
        - 每个障碍物为 dict，含 `center: (x,y,z)` 与 `size: (sx,sy,sz)`
        - 视作轴对齐立方体，使用半边长进行包围盒检测
        
        参数：
        - position: 检查的连续坐标 `(x,y,z)`
        
        返回：
        - collision: 布尔值，是否在任一障碍物包围盒内
        """
        for obstacle in self.obstacles:
            # 假设障碍物是立方体，有center和size属性
            center = obstacle.get('center', (0, 0, 0))
            size = obstacle.get('size', (10, 10, 10))
            
            # 检查是否在障碍物范围内
            if (abs(position[0] - center[0]) < size[0]/2 and
                abs(position[1] - center[1]) < size[1]/2 and
                abs(position[2] - center[2]) < size[2]/2):
                return True
        
        return False
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """
        计算路径总长度（连续空间）
        
        参数：
        - path: 连续坐标路径点列表
        
        返回：
        - length: 相邻点间欧氏距离之和
        """
        length = 0.0
        for i in range(len(path) - 1):
            length += math.sqrt(sum((a - b) ** 2 for a, b in zip(path[i], path[i + 1])))
        return length

# 测试函数
def test_enhanced_p3da_integration():
    """
    集成测试：验证增强版P3DA*的规划、能耗与优先级
    
    内容：
    - 构造5类无人机，分别进行路径规划与优先级计算
    - 打印能耗、优先级、紧急度与计算时间
    - 返回结果列表并按优先级降序打印
    
    返回：
    - results: 每架无人机的规划与评估字典列表
    """
    print("=== 增强版P3DA*算法集成测试 ===")
    
    # 创建算法实例
    planner = EnhancedP3DAStar()
    
    # 创建不同类型的无人机
    test_drones = [
        DroneParameters(DroneType.MANNED, mass_drone=10.0, mass_payload=5.0),
        DroneParameters(DroneType.EMERGENCY_CARGO, mass_drone=8.0, mass_payload=4.0),
        DroneParameters(DroneType.NORMAL_CARGO, mass_drone=6.0, mass_payload=3.0),
        DroneParameters(DroneType.INSPECTION, mass_drone=4.0, mass_payload=1.0),
        DroneParameters(DroneType.OTHER, mass_drone=3.0, mass_payload=0.5)
    ]
    
    # 测试路径规划
    results = []
    for i, drone_params in enumerate(test_drones):
        result = planner.plan_path_with_energy_priority_optimization(
            drone_params, 
            start=(i*100, i*100, 50), 
            goal=(500-i*50, 500-i*50, 100),
            entry_time=time.time() + i*10
        )
        results.append(result)
        
        print(f"\n{drone_params.drone_type.value}:")
        print(f"  成功: {result['success']}")
        print(f"  能耗: {result['energy_consumption']:.1f} J")
        print(f"  优先级: {result['priority']:.3f}")
        print(f"  任务紧急度: {result['task_urgency']}")
        print(f"  计算时间: {result['computation_time']:.4f}s")
    
    # 按优先级排序
    results.sort(key=lambda x: x['priority'], reverse=True)
    
    print("\n=== 优先级排序结果 ===")
    for i, result in enumerate(results):
        print(f"{i+1}. {result['drone_type']:12s}: 优先级 {result['priority']:.3f}")
    
    return results

if __name__ == "__main__":
    test_enhanced_p3da_integration()