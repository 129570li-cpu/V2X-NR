#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
网格型SUMO路网生成器 (1km x 1km)
基于城市路网生成器改造，支持车道类型判别(NT/TL/RT)

功能特点:
1. 1km x 1km网格拓扑 (可配置网格大小)
2. 支持直行/左转/右转路由
3. 完整输出：.net.xml, .rou.xml, .add.xml, fcd.out.xml
4. 车辆命名: car0, car1, car2... (与urban.rou.xml命名一致)
5. 与NS-3 VFVC模块完全兼容

使用方法：
python generate_grid_network.py --help
python generate_grid_network.py --output-dir ./grid_network --vehicles 200
"""

import os
import sys
import argparse
import subprocess
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math
import random
from typing import List, Dict, Tuple

class GridNetworkGenerator:
    def __init__(self, config):
        """初始化网格网络生成器

        Args:
            config: 配置字典，包含所有生成参数
        """
        self.config = config
        self.output_dir = config['output_dir']
        self.routes: Dict[str, List[str]] = {}

        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)

        # 文件路径
        self.nod_file = os.path.join(self.output_dir, 'grid.nod.xml')
        self.edg_file = os.path.join(self.output_dir, 'grid.edg.xml')
        self.con_file = os.path.join(self.output_dir, 'grid.con.xml')
        self.typ_file = os.path.join(self.output_dir, 'grid.typ.xml')
        self.net_file = os.path.join(self.output_dir, 'grid.net.xml')
        self.rou_file = os.path.join(self.output_dir, 'grid.rou.xml')
        self.add_file = os.path.join(self.output_dir, 'grid.add.xml')
        self.station_file = os.path.join(self.output_dir, 'stations.xml')
        self.cfg_file = os.path.join(self.output_dir, 'grid.sumocfg')
        self.fcd_file = os.path.join(self.output_dir, 'grid_fcd.out.xml')

        print(f"📁 输出目录: {self.output_dir}")

    def build_nodes(self):
        """构建网格节点文件 (.nod.xml)"""
        print("🔧 生成网格节点文件...")

        root = ET.Element('nodes',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/nodes_file.xsd'})

        grid_size = self.config['grid_size']  # 网格维度 (e.g., 5 表示 5x5 网格)
        spacing = self.config['spacing']  # 网格间距 (e.g., 250m)

        # 生成 grid_size x grid_size 网格节点
        for i in range(grid_size):
            for j in range(grid_size):
                node_id = f'N_{i}_{j}'
                x = j * spacing
                y = i * spacing

                # 使用优先级节点（支持转向连接）
                ET.SubElement(root, 'node',
                             attrib={'id': node_id, 'x': str(x), 'y': str(y),
                                    'type': 'traffic_light' if self.config['with_tls'] else 'priority'})

        self._save_xml(root, self.nod_file)
        print(f"✅ 节点文件已生成: {self.nod_file} ({grid_size}x{grid_size} = {grid_size*grid_size} 节点)")

    def build_edge_types(self):
        """构建道路类型文件 (.typ.xml)"""
        print("🔧 生成道路类型文件...")

        root = ET.Element('types',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/types_file.xsd'})

        # 网格道路类型（统一类型）
        ET.SubElement(root, 'type',
                     attrib={'id': 'grid_street',
                            'priority': '8',
                            'numLanes': str(self.config['num_lanes']),
                            'speed': str(self.config['max_speed']),
                            'allow': 'all',
                            'width': '3.5'})

        self._save_xml(root, self.typ_file)
        print(f"✅ 道路类型文件已生成: {self.typ_file}")

    def build_edges(self):
        """构建网格边文件 (.edg.xml)"""
        print("🔧 生成网格边文件...")

        root = ET.Element('edges',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/edges_file.xsd'})

        grid_size = self.config['grid_size']

        # 水平边 (东西向)
        for i in range(grid_size):
            for j in range(grid_size - 1):
                # 东向（从左到右）
                from_node = f'N_{i}_{j}'
                to_node = f'N_{i}_{j+1}'
                edge_id = f'E_H_{i}_{j}_E'
                ET.SubElement(root, 'edge',
                             attrib={'id': edge_id, 'from': from_node, 'to': to_node, 'type': 'grid_street'})

                # 西向（从右到左）
                edge_id = f'E_H_{i}_{j+1}_W'
                ET.SubElement(root, 'edge',
                             attrib={'id': edge_id, 'from': to_node, 'to': from_node, 'type': 'grid_street'})

        # 垂直边 (南北向)
        for j in range(grid_size):
            for i in range(grid_size - 1):
                # 北向（从下到上）
                from_node = f'N_{i}_{j}'
                to_node = f'N_{i+1}_{j}'
                edge_id = f'E_V_{i}_{j}_N'
                ET.SubElement(root, 'edge',
                             attrib={'id': edge_id, 'from': from_node, 'to': to_node, 'type': 'grid_street'})

                # 南向（从上到下）
                edge_id = f'E_V_{i+1}_{j}_S'
                ET.SubElement(root, 'edge',
                             attrib={'id': edge_id, 'from': to_node, 'to': from_node, 'type': 'grid_street'})

        self._save_xml(root, self.edg_file)
        print(f"✅ 边文件已生成: {self.edg_file}")

    def build_connections(self):
        """构建连接文件 (.con.xml) - 支持NT/TL/RT车道判别"""
        print("🔧 生成连接文件（支持车道类型判别）...")

        root = ET.Element('connections',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/connections_file.xsd'})

        grid_size = self.config['grid_size']
        num_lanes = self.config['num_lanes']

        # 为每个内部节点设置转向连接
        for i in range(grid_size):
            for j in range(grid_size):
                node_id = f'N_{i}_{j}'

                # === 东向车辆 (面朝东) ===
                if j > 0:  # 有来自西边的边
                    from_edge = f'E_H_{i}_{j}_E'

                    # 直行（继续向东）
                    if j < grid_size - 1:
                        to_straight = f'E_H_{i}_{j+1}_E'
                        if num_lanes >= 3:
                            for lane in range(1, num_lanes - 1):
                                ET.SubElement(root, 'connection',
                                             attrib={'from': from_edge, 'to': to_straight,
                                                    'fromLane': str(lane), 'toLane': str(lane), 'dir': 's'})
                        else:
                            ET.SubElement(root, 'connection',
                                         attrib={'from': from_edge, 'to': to_straight,
                                                'fromLane': '0', 'toLane': '0', 'dir': 's'})

                    # 左转（向北）- 最左车道
                    if i < grid_size - 1:
                        to_left = f'E_V_{i}_{j}_N'
                        from_lane = str(num_lanes - 1) if num_lanes >= 2 else '0'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_left,
                                            'fromLane': from_lane, 'toLane': '0', 'dir': 'l'})

                    # 右转（向南）- 最右车道
                    if i > 0:
                        to_right = f'E_V_{i}_{j}_S'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_right,
                                            'fromLane': '0', 'toLane': '0', 'dir': 'r'})

                # === 西向车辆 (面朝西) ===
                if j < grid_size - 1:  # 有来自东边的边
                    from_edge = f'E_H_{i}_{j+1}_W'

                    # 直行（继续向西）
                    if j > 0:
                        to_straight = f'E_H_{i}_{j}_W'
                        if num_lanes >= 3:
                            for lane in range(1, num_lanes - 1):
                                ET.SubElement(root, 'connection',
                                             attrib={'from': from_edge, 'to': to_straight,
                                                    'fromLane': str(lane), 'toLane': str(lane), 'dir': 's'})
                        else:
                            ET.SubElement(root, 'connection',
                                         attrib={'from': from_edge, 'to': to_straight,
                                                'fromLane': '0', 'toLane': '0', 'dir': 's'})

                    # 左转（向南）- 最左车道
                    if i > 0:
                        to_left = f'E_V_{i}_{j}_S'
                        from_lane = str(num_lanes - 1) if num_lanes >= 2 else '0'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_left,
                                            'fromLane': from_lane, 'toLane': '0', 'dir': 'l'})

                    # 右转（向北）- 最右车道
                    if i < grid_size - 1:
                        to_right = f'E_V_{i}_{j}_N'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_right,
                                            'fromLane': '0', 'toLane': '0', 'dir': 'r'})

                # === 北向车辆 (面朝北) ===
                if i > 0:  # 有来自南边的边
                    from_edge = f'E_V_{i}_{j}_N'

                    # 直行（继续向北）
                    if i < grid_size - 1:
                        to_straight = f'E_V_{i+1}_{j}_N'
                        if num_lanes >= 3:
                            for lane in range(1, num_lanes - 1):
                                ET.SubElement(root, 'connection',
                                             attrib={'from': from_edge, 'to': to_straight,
                                                    'fromLane': str(lane), 'toLane': str(lane), 'dir': 's'})
                        else:
                            ET.SubElement(root, 'connection',
                                         attrib={'from': from_edge, 'to': to_straight,
                                                'fromLane': '0', 'toLane': '0', 'dir': 's'})

                    # 左转（向西）- 最左车道
                    if j > 0:
                        to_left = f'E_H_{i+1}_{j}_W'
                        from_lane = str(num_lanes - 1) if num_lanes >= 2 else '0'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_left,
                                            'fromLane': from_lane, 'toLane': '0', 'dir': 'l'})

                    # 右转（向东）- 最右车道
                    if j < grid_size - 1:
                        to_right = f'E_H_{i+1}_{j}_E'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_right,
                                            'fromLane': '0', 'toLane': '0', 'dir': 'r'})

                # === 南向车辆 (面朝南) ===
                if i < grid_size - 1:  # 有来自北边的边
                    from_edge = f'E_V_{i+1}_{j}_S'

                    # 直行（继续向南）
                    if i > 0:
                        to_straight = f'E_V_{i}_{j}_S'
                        if num_lanes >= 3:
                            for lane in range(1, num_lanes - 1):
                                ET.SubElement(root, 'connection',
                                             attrib={'from': from_edge, 'to': to_straight,
                                                    'fromLane': str(lane), 'toLane': str(lane), 'dir': 's'})
                        else:
                            ET.SubElement(root, 'connection',
                                         attrib={'from': from_edge, 'to': to_straight,
                                                'fromLane': '0', 'toLane': '0', 'dir': 's'})

                    # 左转（向东）- 最左车道
                    if j < grid_size - 1:
                        to_left = f'E_H_{i}_{j}_E'
                        from_lane = str(num_lanes - 1) if num_lanes >= 2 else '0'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_left,
                                            'fromLane': from_lane, 'toLane': '0', 'dir': 'l'})

                    # 右转（向西）- 最右车道
                    if j > 0:
                        to_right = f'E_H_{i}_{j}_W'
                        ET.SubElement(root, 'connection',
                                     attrib={'from': from_edge, 'to': to_right,
                                            'fromLane': '0', 'toLane': '0', 'dir': 'r'})

        self._save_xml(root, self.con_file)
        print(f"✅ 连接文件已生成: {self.con_file}")

    def build_vehicle_types(self):
        """构建车辆类型定义"""
        print("🔧 生成车辆类型...")

        vehicle_types = {
            'passenger': {
                'maxSpeed': '22.2',  # 80 km/h
                'speedFactor': '1.0',
                'speedDev': '0.1',
                'length': '4.5',
                'minGap': '2.0',
                'accel': '2.6',
                'decel': '4.5',
                'sigma': '0.5'
            },
            'truck': {
                'maxSpeed': '25.0',  # 90 km/h
                'speedFactor': '0.9',
                'speedDev': '0.05',
                'length': '12.0',
                'minGap': '3.0',
                'accel': '1.8',
                'decel': '4.0',
                'sigma': '0.5'
            },
            'bus': {
                'maxSpeed': '22.2',  # 80 km/h
                'speedFactor': '0.85',
                'speedDev': '0.05',
                'length': '12.0',
                'minGap': '3.0',
                'accel': '2.0',
                'decel': '4.0',
                'sigma': '0.5'
            },
            'rsu_type': {
                'maxSpeed': '0.1',
                'accel': '0.1',
                'decel': '0.1',
                'length': '1.0',
                'width': '1.0',
                'minGap': '0.1',
                'sigma': '0.0',
                'lcStrategic': '0',
                'lcCooperative': '0',
                'lcSpeedGain': '0',
                'lcKeepRight': '0',
                'lcSublane': '0',
                'color': '0,0,1'
            }
        }

        self.vehicle_types = vehicle_types
        return vehicle_types

    def build_routes(self):
        """构建路由文件 (.rou.xml)"""
        print("🔧 生成路由文件...")

        root = ET.Element('routes',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/routes_file.xsd'})

        # 添加车辆类型定义
        vehicle_types = self.build_vehicle_types()
        for vtype_id, attrs in vehicle_types.items():
            ET.SubElement(root, 'vType', attrib=dict(id=vtype_id, **attrs))

        # 路由定义
        routes = self._generate_routes()
        self.routes = routes
        for route_id, edges in routes.items():
            ET.SubElement(root, 'route',
                         attrib={'id': route_id, 'edges': ' '.join(edges)})

        # 可选 RSU：先注入 RSU（depart=0），再生成车辆，确保 route 文件按出发时间排序
        if not self.config.get('no_rsu'):
            self._inject_static_rsus(root)
        self._generate_vehicles(root)

        self._save_xml(root, self.rou_file)
        print(f"✅ 路由文件已生成: {self.rou_file}")

    def _generate_routes(self) -> Dict[str, List[str]]:
        """生成循环路由，确保车辆在仿真期间持续存在"""
        grid_size = self.config['grid_size']

        if grid_size < 2:
            raise ValueError('网格维度至少为2，才能构建循环路由')

        spacing = self.config['spacing']
        max_speed = self.config['max_speed']
        target_duration = max(self.config['sim_time'], 100)
        rng = random.Random(self.config.get('seed', 42))

        loop_length = 4 * spacing
        if loop_length <= 0:
            raise ValueError('路段长度必须大于0')

        loop_time = loop_length / max_speed if max_speed > 0 else 60.0
        min_repetitions = math.ceil(target_duration / max(loop_time, 1e-6)) + 1
        loop_repetitions = max(2, min_repetitions)

        routes: Dict[str, List[str]] = {}
        max_templates = self.config['num_route_templates'] if self.config['num_route_templates'] > 0 else None

        def add_route(route_key: str, edges: List[str]):
            if max_templates is not None and len(routes) >= max_templates:
                return False
            routes[route_key] = edges
            return True

        # === 长走廊路线（覆盖整行/整列），确保车辆均匀覆盖网格 ===
        # 行走廊：往东到头再往西返回，形成闭合循环（避免使用最外一列，减小无连接风险）
        for i in range(grid_size):
            east = [f'E_H_{i}_{j}_E' for j in range(grid_size - 2)]
            west = [f'E_H_{i}_{j+1}_W' for j in reversed(range(grid_size - 2))]
            if east and west:
                routes[f'corridor_row_{i}'] = east + west
        # 列走廊：往北到头再往南返回
        for j in range(grid_size):
            north = [f'E_V_{i}_{j}_N' for i in range(grid_size - 2)]
            south = [f'E_V_{i+1}_{j}_S' for i in reversed(range(grid_size - 2))]
            if north and south:
                routes[f'corridor_col_{j}'] = north + south

        # 为所有网格单元生成顺/逆时针循环路由，先收集再随机打散，避免路由数量限制时偏向左上角
        loop_defs = []
        for i in range(grid_size - 1):
            for j in range(grid_size - 1):
                clockwise = [
                    f'E_H_{i}_{j}_E',
                    f'E_V_{i}_{j+1}_N',
                    f'E_H_{i+1}_{j+1}_W',
                    f'E_V_{i+1}_{j}_S'
                ]
                counter_clockwise = [
                    f'E_V_{i}_{j}_N',
                    f'E_H_{i+1}_{j}_E',
                    f'E_V_{i+1}_{j+1}_S',
                    f'E_H_{i}_{j+1}_W'
                ]
                loop_defs.append((f'loop_{i}_{j}_cw', clockwise))
                loop_defs.append((f'loop_{i}_{j}_ccw', counter_clockwise))

        rng.shuffle(loop_defs)
        for route_key, base_edges in loop_defs:
            repeated_edges = base_edges * loop_repetitions
            if not add_route(route_key, repeated_edges):
                break

        print(f"Generated {len(routes)} route templates with {loop_repetitions} loops each")
        return routes

    def _generate_vehicles(self, root):
        """生成车辆 - 使用car{i}命名"""
        num_vehicles = self.config['num_vehicles']
        depart_window = min(self.config['depart_window'], 100.0)

        truck_share = self.config['truck_share']
        bus_share = self.config['bus_share']

        route_ids = list(self.routes.keys())
        if not route_ids:
            self.routes = self._generate_routes()
            route_ids = list(self.routes.keys())

        if not route_ids:
            raise RuntimeError('未能生成循环路由，无法分配车辆')

        route_ids.sort()
        rng = random.Random(self.config.get('seed', 42))
        rng.shuffle(route_ids)

        corridor_routes = [rid for rid in route_ids if rid.startswith('corridor_')]
        other_routes = [rid for rid in route_ids if not rid.startswith('corridor_')]

        assigned_route_sequence: List[str] = []
        for rid in corridor_routes:
            if len(assigned_route_sequence) == num_vehicles:
                break
            assigned_route_sequence.append(rid)

        remaining = num_vehicles - len(assigned_route_sequence)
        if remaining > 0:
            pool = other_routes if other_routes else route_ids
            idx = 0
            while remaining > 0:
                assigned_route_sequence.append(pool[idx % len(pool)])
                idx += 1
                remaining -= 1
        route_sequence = assigned_route_sequence

        vehicles_data = []

        for i in range(num_vehicles):
            if num_vehicles > 1:
                depart_time = (depart_window * i) / (num_vehicles - 1)
            else:
                depart_time = 0.0

            rand = random.random()
            if rand < truck_share:
                vtype = 'truck'
            elif rand < truck_share + bus_share:
                vtype = 'bus'
            else:
                vtype = 'passenger'

            route = route_sequence[i]

            vehicle_data = {
                'temp_id': i,
                'type': vtype,
                'route': route,
                'depart_time': depart_time,
                'color': self._get_vehicle_color(vtype)
            }
            vehicles_data.append(vehicle_data)

        vehicles_data.sort(key=lambda v: v['depart_time'])
        for idx, vdata in enumerate(vehicles_data):
            vdata['id'] = f'car{idx}'

        first_depart = vehicles_data[0]['depart_time'] if vehicles_data else 0.0
        last_depart = vehicles_data[-1]['depart_time'] if vehicles_data else 0.0
        unique_routes = len(set(route_sequence))
        print(
            f"Generated {len(vehicles_data)} vehicles (depart {first_depart:.1f}s-{last_depart:.1f}s) using {unique_routes} routes"
        )

        for vehicle_data in vehicles_data:
            vehicle_attrs = {
                'id': vehicle_data['id'],
                'type': vehicle_data['type'],
                'route': vehicle_data['route'],
                'depart': f"{vehicle_data['depart_time']:.1f}",
                'color': vehicle_data['color']
            }
            ET.SubElement(root, 'vehicle', attrib=vehicle_attrs)

    def _get_vehicle_color(self, vtype: str) -> str:
        """根据车辆类型返回颜色"""
        colors = {
            'passenger': '1,1,0',      # 黄色
            'truck': '0,1,0',          # 绿色
            'bus': '0,0,1'             # 蓝色
        }
        return colors.get(vtype, '1,1,1')

    def _inject_static_rsus(self, root: ET.Element):
        """在路由文件中加入静态RSU车辆"""
        grid_size = self.config['grid_size']
        # 仅放置 4 个 RSU，位于四个子网格的“中心”路口。
        # 使用偏移 offset 和 high，适配不同 grid_size（8x8 时为 (2,2),(6,2),(2,6),(6,6)）。
        offset = max(1, grid_size // 4)
        high = max(0, grid_size - 1 - offset)
        positions = [
            (offset, offset),
            (high, offset),
            (offset, high),
            (high, high),
        ]
        total = len(positions)
        stop_pos = min(
            max(self.config['spacing'] * 0.02, 1.0),
            max(self.config['spacing'] - 1.0, 1.0)
        )

        print(f"🏛️  注入 {total} 个静态RSU车辆...")

        rsu_counter = 0
        base_rsu_id = 99990  # 避免与车辆 ID 混淆
        for i, j in positions:
            edge_id = self._select_rsu_anchor_edge(i, j)
            rsu_num = base_rsu_id + rsu_counter
            route_id = f'rsu{rsu_num}_route'
            rsu_id = f'rsu{rsu_num}'

            ET.SubElement(root, 'route', attrib={'id': route_id, 'edges': edge_id})

            vehicle_attrs = {
                'id': rsu_id,
                'type': 'rsu_type',
                'route': route_id,
                'depart': '0',
                'departSpeed': '0',
                'arrivalSpeed': '0',
                'color': '0,0,1'
            }
            vehicle_elem = ET.SubElement(root, 'vehicle', attrib=vehicle_attrs)
            ET.SubElement(vehicle_elem, 'stop', attrib={
                'lane': f'{edge_id}_0',
                'startPos': f'{stop_pos:.2f}',
                'endPos': f'{stop_pos:.2f}',
                'duration': '1000000',
                'parking': 'true'
            })
            rsu_counter += 1

    def _select_rsu_anchor_edge(self, i: int, j: int) -> str:
        """为每个路口选择一个挂载RSU的车道"""
        grid_size = self.config['grid_size']
        candidates = []

        if i < grid_size - 1:
            candidates.append(f'E_V_{i}_{j}_N')
        if j < grid_size - 1:
            candidates.append(f'E_H_{i}_{j}_E')
        if i > 0:
            candidates.append(f'E_V_{i}_{j}_S')
        if j > 0:
            candidates.append(f'E_H_{i}_{j}_W')

        if not candidates:
            raise ValueError(f'节点({i}, {j})没有可用边')

        return candidates[0]

    def build_station_file(self):
        """构建RSU POI文件，供Ns-3映射路口坐标"""
        # 如选择不生成 RSU，创建空的 additional 文件以避免 sumo 报错
        print("🔧 生成RSU坐标文件 (stations.xml)...")
        root = ET.Element('additional',
                          attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                  'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/additional_file.xsd'})

        if not self.config.get('no_rsu'):
            grid_size = self.config['grid_size']
            spacing = self.config['spacing']

            # 与路由文件一致，只放置 4 个 RSU，位置与 _inject_static_rsus 相同
            offset = max(1, grid_size // 4)
            high = max(0, grid_size - 1 - offset)
            positions = [
                (offset, offset),
                (high, offset),
                (offset, high),
                (high, high),
            ]

            base_rsu_id = 99990
            for idx, (i, j) in enumerate(positions):
                poi_id = f'rsu{base_rsu_id + idx}'
                x = j * spacing
                y = i * spacing
                ET.SubElement(root, 'poi', attrib={
                    'id': poi_id,
                    'color': 'red',
                    'layer': '202.0',
                    'x': f'{x:.2f}',
                    'y': f'{y:.2f}'
                })

        self._save_xml(root, self.station_file)
        print(f"✅ RSU坐标文件已生成: {self.station_file}")

    def build_additional_files(self):
        """构建附加文件 (.add.xml)"""
        if not self.config['with_tls']:
            print("⏭️  跳过附加文件生成（无信号灯）")
            return

        print("🔧 生成附加文件...")

        root = ET.Element('additional',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/additional_file.xsd'})

        self._save_xml(root, self.add_file)
        print(f"✅ 附加文件已生成: {self.add_file}")

    def generate_network_file(self):
        """使用netconvert生成网络文件"""
        print("🔧 运行netconvert生成网络...")

        cmd = [
            'netconvert',
            '--node-files', self.nod_file,
            '--edge-files', self.edg_file,
            '--type-files', self.typ_file,
            '--output-file', self.net_file,
            '--junctions.corner-detail', '2',
            '--junctions.limit-turn-speed', '8'
        ]

        # 投影信息在 netconvert 之后写入 net.xml 的 <location> 元素，避免将输入坐标误当经纬度解析

        # Only add connection file if it exists and we want manual connections
        # For now, let SUMO auto-generate connections
        # if os.path.exists(self.con_file):
        #     cmd.insert(5, '--connection-files')
        #     cmd.insert(6, self.con_file)

        if self.config['with_tls']:
            cmd.extend([
                '--tls.guess', 'true',
                '--tls.cycle.time', '90'
            ])

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            print(f"✅ 网络文件已生成: {self.net_file}")
            if result.stdout.strip():
                print(f"📝 netconvert输出: {result.stdout}")
            # 将投影元数据写入 net.xml，便于 TraCI 做 XY<->经纬度转换
            self._inject_location_metadata()
        except subprocess.CalledProcessError as e:
            print(f"❌ netconvert失败: {e}")
            print(f"stderr: {e.stderr}")
            raise

    def generate_sumo_config(self):
        """生成SUMO配置文件"""
        print("🔧 生成SUMO配置文件...")

        root = ET.Element('configuration',
                         attrib={'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
                                'xsi:noNamespaceSchemaLocation': 'http://sumo.dlr.de/xsd/sumoConfiguration.xsd'})

        # 输入文件
        input_elem = ET.SubElement(root, 'input')
        ET.SubElement(input_elem, 'net-file', attrib={'value': os.path.basename(self.net_file)})
        ET.SubElement(input_elem, 'route-files', attrib={'value': os.path.basename(self.rou_file)})
        additional_files = []
        if os.path.exists(self.add_file):
            additional_files.append(os.path.basename(self.add_file))
        if os.path.exists(self.station_file):
            additional_files.append(os.path.basename(self.station_file))
        if additional_files:
            ET.SubElement(input_elem, 'additional-files',
                          attrib={'value': ','.join(additional_files)})

        # 时间设置
        time_elem = ET.SubElement(root, 'time')
        ET.SubElement(time_elem, 'begin', attrib={'value': '0'})
        ET.SubElement(time_elem, 'end', attrib={'value': str(self.config['sim_time'])})
        ET.SubElement(time_elem, 'step-length', attrib={'value': '0.1'})

        # 输出设置
        output_elem = ET.SubElement(root, 'output')
        ET.SubElement(output_elem, 'fcd-output', attrib={'value': os.path.basename(self.fcd_file)})
        ET.SubElement(output_elem, 'netstate-dump', attrib={'value': 'netstate.xml'})

        # 处理设置
        processing_elem = ET.SubElement(root, 'processing')
        ET.SubElement(processing_elem, 'collision.action', attrib={'value': 'warn'})

        self._save_xml(root, self.cfg_file)
        print(f"✅ SUMO配置文件已生成: {self.cfg_file}")

    def run_simulation(self):
        """运行SUMO仿真并导出FCD"""
        if not self.config['run_simulation']:
            print("⏭️  跳过仿真运行")
            return

        print("🚀 运行SUMO仿真...")

        cfg_basename = os.path.basename(self.cfg_file)
        cmd = ['sumo', '-c', cfg_basename, '--no-step-log', '--no-warnings']

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True,
                                  cwd=self.output_dir)
            print(f"✅ 仿真完成，FCD文件: {self.fcd_file}")

            if os.path.exists(self.fcd_file):
                file_size = os.path.getsize(self.fcd_file) / (1024*1024)
                print(f"📊 FCD文件大小: {file_size:.2f} MB")
            else:
                print("⚠️  FCD文件未生成")

        except subprocess.CalledProcessError as e:
            print(f"❌ SUMO仿真失败: {e}")
            print(f"stderr: {e.stderr}")
            raise

    def _save_xml(self, root, filename):
        """保存XML文件，带格式化"""
        rough_string = ET.tostring(root, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent='    ')

        # 移除空行
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        pretty_xml = '\n'.join(lines)

        with open(filename, 'w', encoding='utf-8') as f:
            f.write(pretty_xml)

    def _inject_location_metadata(self):
        """在 net.xml 中写入投影元数据，供 TraCI/NS-3 做 XY<->经纬度转换"""
        if self.config.get('origin_lat') is None or self.config.get('origin_lon') is None:
            return
        lat0 = float(self.config['origin_lat'])
        lon0 = float(self.config['origin_lon'])
        try:
            tree = ET.parse(self.net_file)
            root = tree.getroot()
            conv_max = (self.config['grid_size'] - 1) * self.config['spacing']
            conv_boundary = f"0.00,0.00,{conv_max:.2f},{conv_max:.2f}"
            proj = f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"

            loc = root.find('location')
            if loc is None:
                loc = ET.Element('location')
                root.insert(0, loc)
            loc.set('netOffset', "0.00,0.00")
            loc.set('convBoundary', conv_boundary)
            loc.set('origBoundary', conv_boundary)
            loc.set('projParameter', proj)
            loc.set('projName', 'tmerc')

            tree.write(self.net_file, encoding='utf-8')
        except Exception as e:
            print(f"⚠️  写入投影元数据失败: {e}")

    def generate_all(self):
        """生成所有文件"""
        print("🌟 开始生成网格型SUMO路网...")
        print(f"📋 配置参数: {self.config}")

        try:
            # 1. 基础文件
            self.build_nodes()
            self.build_edge_types()
            self.build_edges()
            self.build_connections()

            # 2. 网络生成
            self.generate_network_file()

            # 3. 路由和附加文件
            self.build_routes()
            self.build_station_file()
            self.build_additional_files()

            # 4. 配置文件
            self.generate_sumo_config()

            # 5. 仿真运行
            self.run_simulation()

            print("🎉 网格型SUMO路网生成完成!")
            self._print_summary()

        except Exception as e:
            print(f"❌ 生成过程中出现错误: {e}")
            raise

    def _print_summary(self):
        """打印生成总结"""
        print("\n📋 生成文件总结:")
        files = [
            (self.net_file, "路网文件"),
            (self.rou_file, "路由文件"),
            (self.add_file, "附加文件"),
            (self.station_file, "RSU坐标文件"),
            (self.cfg_file, "配置文件"),
            (self.fcd_file, "FCD轨迹文件")
        ]

        for filepath, desc in files:
            if os.path.exists(filepath):
                size_kb = os.path.getsize(filepath) / 1024
                print(f"  ✅ {desc}: {os.path.basename(filepath)} ({size_kb:.1f} KB)")
            else:
                print(f"  ❌ {desc}: {os.path.basename(filepath)} (未生成)")

        print(f"\n🗂️  所有文件位于: {self.output_dir}")
        print(f"🚀 运行命令: sumo -c {os.path.basename(self.cfg_file)}")

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='网格型SUMO路网生成器 (1km x 1km)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例用法:
  # 基本生成（默认5x5网格，250m间距 = 1km x 1km）
  python generate_grid_network.py

  # 自定义参数（4x4网格，333m间距 ≈ 1km x 1km）
  python generate_grid_network.py --grid-size 4 --spacing 333 --vehicles 200

  # 不运行仿真，只生成文件
  python generate_grid_network.py --no-simulation

  # 可复现实验（指定随机种子）
  python generate_grid_network.py --seed 123 --vehicles 200
        ''')

    # 基本参数
    parser.add_argument('--output-dir', '-o', default='./grid_network',
                       help='输出目录 (默认: ./grid_network)')
    parser.add_argument('--vehicles', '-v', type=int, default=200,
                       help='车辆数量 (默认: 200)')

    # 网格参数
    parser.add_argument('--grid-size', type=int, default=5,
                       help='网格维度 (默认: 5, 表示5x5网格)')
    parser.add_argument('--spacing', type=float, default=250.0,
                       help='网格间距(m) (默认: 250, 5x5网格时=1km x 1km)')
    parser.add_argument('--num-lanes', type=int, default=3,
                       help='每条道路的车道数 (默认: 3)')
    parser.add_argument('--max-speed', type=float, default=16.67,
                       help='道路限速(m/s) (默认: 16.67, ~60km/h)')

    # 车辆分布
    parser.add_argument('--truck-share', type=float, default=0.15,
                       help='卡车比例 (默认: 0.15)')
    parser.add_argument('--bus-share', type=float, default=0.05,
                       help='公交车比例 (默认: 0.05)')

    # 路由参数
    parser.add_argument('--num-route-templates', type=int, default=0,
                       help='路由模板数量 (默认: 0=使用全部循环路由)')
    parser.add_argument('--min-route-edges', type=int, default=3,
                       help='路由最小边数 (默认: 3)')
    parser.add_argument('--max-route-edges', type=int, default=10,
                       help='路由最大边数 (默认: 10)')
    parser.add_argument('--turn-probability', type=float, default=0.3,
                       help='路由中转向概率 (默认: 0.3)')

    # 仿真参数
    parser.add_argument('--sim-time', type=int, default=600,
                       help='仿真时长(s) (默认: 600)')
    parser.add_argument('--depart-window', type=int, default=120,
                       help='车辆出发时间窗(s) (默认: 120)')

    # 可选功能
    parser.add_argument('--with-tls', action='store_true',
                       help='启用交通信号灯')
    parser.add_argument('--no-simulation', action='store_true',
                       help='不运行仿真，只生成文件')
    parser.add_argument('--origin-lat', type=float, default=0.0,
                       help='投影参考纬度 (默认: 0.0，经纬度投影到平面米坐标)')
    parser.add_argument('--origin-lon', type=float, default=0.0,
                       help='投影参考经度 (默认: 0.0，经纬度投影到平面米坐标)')
    parser.add_argument('--no-rsu', action='store_true',
                       help='不生成静态 RSU')

    # 可复现性参数
    parser.add_argument('--seed', type=int, default=42,
                       help='随机种子 (默认: 42)')

    return parser.parse_args()

def main():
    """主函数"""
    args = parse_arguments()

    # 设置随机种子确保可复现性
    random.seed(args.seed)
    print(f"🎯 设置随机种子: {args.seed}")

    # 构建配置
    config = {
        'output_dir': args.output_dir,
        'grid_size': args.grid_size,
        'spacing': args.spacing,
        'num_lanes': args.num_lanes,
        'max_speed': args.max_speed,
        'num_vehicles': args.vehicles,
        'truck_share': args.truck_share,
        'bus_share': args.bus_share,
        'num_route_templates': args.num_route_templates,
        'min_route_edges': args.min_route_edges,
        'max_route_edges': args.max_route_edges,
        'turn_probability': args.turn_probability,
        'sim_time': args.sim_time,
        'depart_window': args.depart_window,
        'with_tls': args.with_tls,
        'run_simulation': not args.no_simulation,
        'seed': args.seed,
        'origin_lat': args.origin_lat,
        'origin_lon': args.origin_lon,
        'no_rsu': args.no_rsu,
    }

    # 验证网格尺寸
    actual_size = (config['grid_size'] - 1) * config['spacing']
    print(f"📏 实际网格尺寸: {actual_size}m x {actual_size}m")

    # 生成器
    generator = GridNetworkGenerator(config)
    generator.generate_all()

if __name__ == '__main__':
    main()
