"""
海上船舶会遇场景生成器 - 专业版 (优化版)
Maritime Collision Avoidance Scenario Generator - Professional Edition (Optimized)

功能特性:
- 自动生成2-5船会遇场景，确保产生真实碰撞危险
- 基于配置文件的场景类型随机分配，覆盖所有会遇类型
- 高级选项自定义局面类型及数量
- CSV格式数据导出，支持打开、另存为、预览
- 极坐标可视化预览，支持鼠标滚轮缩放和拖拽
- 相对运动线显示逻辑与main.py完全一致
- 专业界面设计，全屏显示
- 详细的本船和目标船信息及避碰参数显示
"""

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys
import math
import random
import csv
import json
from datetime import datetime
import georef

# 安全参数常量
SAFE_TCPA = 30.0  # 安全TCPA阈值(秒)
SAFE_DCPA = 2.0   # 安全DCPA阈值(海里)
TT_DCPA = 1.0     # 目标DCPA

# 加载配置
try:
    with open('config.json', 'r', encoding='utf-8') as f:
        conf = json.load(f)
        TT_DCPA = conf.get('tt_DCPA', 1.0)
except:
    pass


def load_csv_config(filename):
    """从CSV文件加载配置数据，支持多种编码"""
    encodings = ['gbk', 'gb2312', 'utf-8', 'utf-8-sig', 'cp936']

    for encoding in encodings:
        try:
            with open(filename, 'r', encoding=encoding) as f:
                reader = csv.reader(f)
                next(reader)  # 跳过标题行
                data = list(reader)
                data = [row for row in data if row and any(row)]
                if data:
                    return data
        except Exception as e:
            continue

    print(f"警告：无法读取文件 {filename}")
    return []


# 加载配置文件
speed_data = load_csv_config('./data/speed.csv')
visibility_data = load_csv_config('./data/visibility.csv')
stage_data = load_csv_config('./data/stage.csv')
ownship_behavior_data = load_csv_config('./data/ownship_behavior.csv')
target_behavior_data = load_csv_config('./data/target_behavior.csv')
meeting_situation_data = load_csv_config('./data/meeting_situation.csv')

# 解析会遇类型配置
meeting_situations = []
for row in meeting_situation_data:
    if len(row) >= 7:
        try:
            meeting_situations.append({
                'id': int(row[0]),
                'name': row[1],
                'rel_brg_min': float(row[2]),
                'rel_brg_max': float(row[3]),
                'dist_min': float(row[4]),
                'dist_max': float(row[5]),
                'behavior': int(row[6])
            })
        except Exception as e:
            print(f"解析会遇类型配置出错: {row}, 错误: {e}")
            continue

# 解析速度配置
speed_ranges = []
for row in speed_data:
    if len(row) >= 4:
        speed_ranges.append({
            'id': int(row[0]),
            'name': row[1],
            'min': float(row[2]),
            'max': float(row[3])
        })

# 解析阶段配置
stages = []
for row in stage_data:
    if len(row) >= 4:
        stages.append({
            'id': int(row[0]),
            'name': row[1],
            'dist_min': float(row[2]),
            'dist_max': float(row[3])
        })

# 检查配置加载
if not meeting_situations:
    print("警告：未能加载任何会遇类型配置！")
if not stages:
    print("警告：未能加载任何阶段配置！")


def mod360(deg):
    """将角度归一化到[0, 360)范围"""
    v = math.fmod(deg, 360)
    if v < 0.0:
        v += 360.0
    return v


def get_rel_course_north(v_x_o, v_y_o, v_x_t, v_y_t):
    """计算相对航向（正北为0度）"""
    dvx = v_x_t - v_x_o
    dvy = v_y_t - v_y_o
    course = math.degrees(math.atan2(dvx, dvy))
    if course < 0.0:
        course += 360.0
    return course


def calc_rel_spd_cog(osog, ocog, tsog, tcog):
    """计算相对速度和相对航向"""
    v_x_o = osog * math.sin(math.radians(ocog))
    v_y_o = osog * math.cos(math.radians(ocog))
    v_x_t = tsog * math.sin(math.radians(tcog))
    v_y_t = tsog * math.cos(math.radians(tcog))

    v_x_rel = v_x_t - v_x_o
    v_y_rel = v_y_t - v_y_o
    rel_spd = math.sqrt(v_x_rel**2 + v_y_rel**2)

    rel_cog = get_rel_course_north(v_x_o, v_y_o, v_x_t, v_y_t)

    return rel_spd, rel_cog


def calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog):
    """计算TCPA和DCPA"""
    dist, brg = georef.DistanceBearingMercator(tlat, tlon, olat, olon)

    v_x_o = osog * math.sin(math.radians(ocog))
    v_y_o = osog * math.cos(math.radians(ocog))
    v_x_t = tsog * math.sin(math.radians(tcog))
    v_y_t = tsog * math.cos(math.radians(tcog))

    v_x_rel = v_x_t - v_x_o
    v_y_rel = v_y_t - v_y_o
    rel_spd = math.sqrt(v_x_rel**2 + v_y_rel**2)

    rel_course = get_rel_course_north(v_x_o, v_y_o, v_x_t, v_y_t)

    delta = rel_course - brg - 180.0
    TCPA = 99999.0
    if rel_spd > 0.001:
        TCPA = dist * math.cos(math.radians(delta)) / rel_spd * 60.0

    DCPA = abs(dist * math.sin(math.radians(delta)))

    return TCPA, DCPA


def gen_oship(lat, lon, sog_min, sog_max):
    """生成本船数据"""
    sog = random.uniform(sog_min, sog_max)
    cog = random.uniform(0, 360)
    return {
        'lat': lat,
        'lon': lon,
        'sog': sog,
        'cog': cog,
    }


def gen_tship(oship, meeting_config, sog_min, sog_max, max_attempts=100):
    """
    生成有碰撞危险的目标船 - 完全参考main.py第343-556行逻辑

    核心思路（参考main.py）：
    1. 随机确定相对方位relbrg（从配置表）
    2. 随机确定距离dist和目标船速度tsog
    3. 计算初始航向：tcog = ocog + relbrg + 180
    4. 计算相对运动方向rel_cog
    5. 使用delta=0，计算目标船真实方位：brg = rel_cog - 180
    6. 根据brg和dist确定目标船位置
    """
    olat = oship['lat']
    olon = oship['lon']
    osog = oship['sog']
    ocog = oship['cog']

    rel_brg_min = meeting_config['rel_brg_min']
    rel_brg_max = meeting_config['rel_brg_max']
    dist_min = meeting_config['dist_min']
    dist_max = meeting_config['dist_max']

    # 迭代尝试不同的参数组合
    for attempt in range(max_attempts):
        # 1. 随机确定相对方位（处理跨越0度的情况）
        # 参考main.py第227-228行
        if rel_brg_max < rel_brg_min:
            # 跨越0度，如354-6度
            if random.random() < 0.5:
                relbrg = random.uniform(rel_brg_min, 360.0)
            else:
                relbrg = random.uniform(0.0, rel_brg_max)
        else:
            relbrg = random.uniform(rel_brg_min, rel_brg_max)
        relbrg = mod360(relbrg)

        # 2. 随机确定距离和目标船速度
        # 参考main.py第234-237行
        dist = random.uniform(dist_min, dist_max)
        tsog = random.uniform(sog_min, sog_max)

        # 3. 计算目标船航向（参考main.py第238行）
        tcog = mod360(ocog + relbrg + 180.0)

        # 4. 计算相对运动方向（参考main.py第240行，注意参数顺序！）
        rel_spd_temp, rel_cog = calc_rel_spd_cog(tsog, tcog, osog, ocog)

        if rel_spd_temp < 0.001:
            continue

        # 5. 目标船方位 = 相对运动方向（参考main.py第242行）
        brg = rel_cog

        # 6. 根据brg和dist计算目标船位置（参考main.py第243行）
        tlat, tlon = georef.ll_gc_ll(olat, olon, brg, dist)

        # 7. 计算TCPA和DCPA（参考main.py第244行）
        TCPA, DCPA = calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog)

        # 8. 检查是否满足碰撞危险条件
        # DCPA范围：0-0.5海里（更严格的碰撞危险）
        if TCPA >= 0 and TCPA <= 30.0 and DCPA <= 0.5:
            # 9. 重新计算相对运动参数（参考main.py第246行，正确的参数顺序）
            rel_spd, rel_cog_final = calc_rel_spd_cog(osog, ocog, tsog, tcog)

            # 10. 返回结果（参考main.py第248-264行）
            # 注意：返回的brg是rel_cog（第242行），dist是输入的dist（第257行）
            # 不要重新计算真实方位！
            return {
                'olat': olat,
                'olon': olon,
                'tlat': tlat,
                'tlon': tlon,
                'osog': osog,
                'ocog': ocog,
                'tsog': tsog,
                'tcog': tcog,
                'dist': dist,
                'brg': brg,
                'relbrg': relbrg,
                'rel_cog': rel_cog_final,
                'rel_spd': rel_spd,
                'TCPA': TCPA,
                'DCPA': DCPA,
                'meeting_type': meeting_config['name']
            }

    # 如果所有尝试都失败，返回None
    return None


def gen_tship_no_danger(oship, meeting_config, sog_min, sog_max):
    """
    生成无碰撞危险的目标船
    """
    olat = oship['lat']
    olon = oship['lon']
    osog = oship['sog']
    ocog = oship['cog']

    rel_brg_min = meeting_config['rel_brg_min']
    rel_brg_max = meeting_config['rel_brg_max']
    dist_min = meeting_config['dist_min']
    dist_max = meeting_config['dist_max']

    if rel_brg_max < rel_brg_min:
        if random.random() < 0.5:
            relbrg = random.uniform(rel_brg_min, 360.0)
        else:
            relbrg = random.uniform(0.0, rel_brg_max)
    else:
        relbrg = random.uniform(rel_brg_min, rel_brg_max)
    relbrg = mod360(relbrg)
    dist = random.uniform(dist_min, dist_max)
    tsog = random.uniform(sog_min, sog_max)

    tcog = mod360(ocog + relbrg + 180.0)
    rel_spd, rel_cog = calc_rel_spd_cog(osog, ocog, tsog, tcog)

    # 使用较大的delta值使DCPA较大
    delta = random.uniform(90, 180) * random.choice([-1, 1])

    brg = mod360((rel_cog - 180.0) + delta)
    tlat, tlon = georef.ll_gc_ll(olat, olon, brg, dist)

    TCPA, DCPA = calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog)
    real_dist, real_brg = georef.DistanceBearingMercator(olat, olon, tlat, tlon)

    return {
        'olat': olat,
        'olon': olon,
        'tlat': tlat,
        'tlon': tlon,
        'osog': osog,
        'ocog': ocog,
        'tsog': tsog,
        'tcog': tcog,
        'dist': real_dist,
        'brg': real_brg,
        'relbrg': relbrg,
        'rel_cog': rel_cog,
        'rel_spd': rel_spd,
        'TCPA': TCPA,
        'DCPA': DCPA,
        'meeting_type': meeting_config['name'],
        'delta': delta
    }


def generate_scenario(target_num, osog_min=10.0, osog_max=20.0,
                     tsog_min=10.0, tsog_max=20.0,
                     lat=31.0, lon=123.0,
                     meeting_type_counts=None):
    """
    生成完整场景

    Args:
        target_num: 目标船数量
        osog_min/max: 本船速度范围
        tsog_min/max: 目标船速度范围
        lat/lon: 初始位置
        meeting_type_counts: 指定各会遇类型数量的字典 {meeting_type_id: count}
    """
    oship = gen_oship(lat, lon, osog_min, osog_max)

    target_configs = []

    if meeting_type_counts:
        for meeting_id, count in meeting_type_counts.items():
            meeting_config = next((m for m in meeting_situations if m['id'] == meeting_id), None)
            if meeting_config:
                target_configs.extend([meeting_config] * count)
    else:
        available_types = meeting_situations.copy()
        random.shuffle(available_types)

        for i in range(target_num):
            if i < len(available_types):
                target_configs.append(available_types[i])
            else:
                target_configs.append(random.choice(meeting_situations))

    targets = []
    for i, config in enumerate(target_configs[:target_num]):
        max_attempts = 100
        for attempt in range(max_attempts):
            if config['name'] == '没有危险':
                target = gen_tship_no_danger(oship, config, tsog_min, tsog_max)
            else:
                target = gen_tship(oship, config, tsog_min, tsog_max)

            if target is None:
                continue

            valid = True
            for existing in targets:
                dist_between = georef.DistGreatCircle(
                    target['tlat'], target['tlon'],
                    existing['tlat'], existing['tlon']
                )
                if dist_between < 1.0:
                    valid = False
                    break

            if valid:
                target['id'] = i + 1
                targets.append(target)
                break

        if len(targets) <= i:
            return None

    return {
        'ownship': oship,
        'targets': targets,
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    }


class PolarPlotWidget(QWidget):
    """极坐标绘图控件"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 600)
        self.scenario = None
        # 初始scale会在set_scenario中自动计算
        self.scale = 40.0  # 默认值
        self.offset_x = 0
        self.offset_y = 0
        self.last_mouse_pos = None
        self.setMouseTracking(True)

    def set_scenario(self, scenario):
        self.scenario = scenario
        # 自动计算最大缩放：让6海里的圆填满画布
        # 留出边距：图例、标签等需要空间
        max_range = 6.5  # 6海里 + 0.5海里标签空间
        available_size = min(self.width(), self.height()) - 40  # 减去边距
        self.scale = available_size / (2 * max_range)  # 计算像素/海里
        self.scale = min(self.scale, 100.0)  # 不超过最大限制
        self.offset_x = 0
        self.offset_y = 0
        self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.scale *= 1.1
        else:
            self.scale /= 1.1
        self.scale = max(5.0, min(100.0, self.scale))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton and self.last_mouse_pos:
            delta = event.pos() - self.last_mouse_pos
            self.offset_x += delta.x()
            self.offset_y += delta.y()
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = None

    def paintEvent(self, event):
        if not self.scenario:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center_x = self.width() / 2 + self.offset_x
        center_y = self.height() / 2 + self.offset_y

        painter.fillRect(self.rect(), QColor(240, 248, 255))
        self.draw_polar_grid(painter, center_x, center_y)
        self.draw_ownship(painter, center_x, center_y)

        for target in self.scenario['targets']:
            self.draw_target(painter, center_x, center_y, target)
            self.draw_relative_motion_line(painter, center_x, center_y, target)

        self.draw_legend(painter)

    def draw_polar_grid(self, painter, cx, cy):
        """绘制极坐标网格 - 量程6海里"""
        painter.setPen(QPen(QColor(200, 200, 200), 1))

        # 距离圈：1, 2, 3, 4, 5, 6海里
        distances = [1, 2, 3, 4, 5, 6]
        for dist in distances:
            radius = dist * self.scale
            painter.drawEllipse(QPointF(cx, cy), radius, radius)
            painter.drawText(int(cx + 5), int(cy - radius + 15), f"{dist}NM")

        # 方位线：每30度一条，延伸到6海里
        painter.setPen(QPen(QColor(180, 180, 180), 1))
        max_range = 6  # 最大量程6海里
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x = cx + max_range * self.scale * math.sin(rad)
            y = cy - max_range * self.scale * math.cos(rad)
            painter.drawLine(QPointF(cx, cy), QPointF(x, y))

            # 方位标签放在6.5海里处
            label_x = cx + (max_range + 0.5) * self.scale * math.sin(rad)
            label_y = cy - (max_range + 0.5) * self.scale * math.cos(rad)
            painter.drawText(int(label_x - 15), int(label_y + 5), f"{angle:03d}°")

        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(QPointF(cx - 20, cy), QPointF(cx + 20, cy))
        painter.drawLine(QPointF(cx, cy - 20), QPointF(cx, cy + 20))

    def draw_ownship(self, painter, cx, cy):
        oship = self.scenario['ownship']

        painter.setPen(QPen(QColor(0, 0, 255), 2))
        painter.setBrush(QBrush(QColor(0, 0, 255)))

        cog_rad = math.radians(oship['cog'])
        length = 15
        width = 8

        bow_x = cx + length * math.sin(cog_rad)
        bow_y = cy - length * math.cos(cog_rad)

        port_angle = cog_rad - math.pi / 2
        port_x = cx + width * math.sin(port_angle)
        port_y = cy - width * math.cos(port_angle)

        starboard_angle = cog_rad + math.pi / 2
        starboard_x = cx + width * math.sin(starboard_angle)
        starboard_y = cy - width * math.cos(starboard_angle)

        points = [
            QPointF(bow_x, bow_y),
            QPointF(port_x, port_y),
            QPointF(starboard_x, starboard_y)
        ]
        painter.drawPolygon(QPolygonF(points))

        # 绘制本船真矢量
        # 统一计算方式：航速 * 6分钟 = 航速 * 0.1小时
        vector_dist_nm = oship['sog'] * 6.0 / 60.0  # 6分钟内航行的距离（海里）
        line_length = vector_dist_nm * self.scale  # 转换为像素
        end_x = bow_x + line_length * math.sin(cog_rad)
        end_y = bow_y - line_length * math.cos(cog_rad)
        painter.setPen(QPen(QColor(0, 0, 255), 2, Qt.DashLine))
        painter.drawLine(QPointF(bow_x, bow_y), QPointF(end_x, end_y))

        painter.setPen(QPen(QColor(0, 0, 255), 1))
        painter.drawText(int(cx + 20), int(cy - 20),
                        f"本船\nSOG: {oship['sog']:.1f}kn\nCOG: {oship['cog']:.1f}°")

    def draw_target(self, painter, cx, cy, target):
        dist = target['dist']
        brg = target['brg']

        brg_rad = math.radians(brg)
        tx = cx + dist * self.scale * math.sin(brg_rad)
        ty = cy - dist * self.scale * math.cos(brg_rad)

        if target['TCPA'] >= 0 and target['DCPA'] <= SAFE_DCPA:
            color = QColor(255, 0, 0)
        else:
            color = QColor(0, 200, 0)

        painter.setPen(QPen(color, 2))
        painter.setBrush(QBrush(color))
        painter.drawEllipse(QPointF(tx, ty), 8, 8)

        # 绘制目标船真矢量
        # 统一计算方式：航速 * 6分钟 = 航速 * 0.1小时（与本船相同）
        tcog_rad = math.radians(target['tcog'])
        vector_dist_nm = target['tsog'] * 6.0 / 60.0  # 6分钟内航行的距离（海里）
        line_length = vector_dist_nm * self.scale  # 转换为像素
        tend_x = tx + line_length * math.sin(tcog_rad)
        tend_y = ty - line_length * math.cos(tcog_rad)
        painter.setPen(QPen(color, 2, Qt.DashLine))
        painter.drawLine(QPointF(tx, ty), QPointF(tend_x, tend_y))

        painter.setPen(QPen(color, 1))
        info_text = (f"T{target['id']}: {target['meeting_type']}\n"
                    f"Dist: {dist:.1f}NM\n"
                    f"Brg: {brg:.1f}°\n"
                    f"SOG: {target['tsog']:.1f}kn\n"
                    f"COG: {target['tcog']:.1f}°\n"
                    f"TCPA: {target['TCPA']:.1f}min\n"
                    f"DCPA: {target['DCPA']:.2f}NM")
        painter.drawText(int(tx + 15), int(ty - 10), info_text)

    def draw_relative_motion_line(self, painter, cx, cy, target):
        """绘制相对运动线 - 完全参考main.py第1748-1762行"""
        # 计算目标船在屏幕上的位置
        dist = target['dist']
        brg = target['brg']
        brg_rad = math.radians(brg)
        tx = cx + dist * self.scale * math.sin(brg_rad)
        ty = cy - dist * self.scale * math.cos(brg_rad)

        # 获取相对运动参数
        rel_cog = target['rel_cog']
        rel_spd = target['rel_spd']
        TCPA = target['TCPA']

        # 关键修正：相对运动线长度基于TCPA（参考main.py第1755行）
        # dist2 = rel_spd * abs(TCPA) / 60
        rel_motion_dist = rel_spd * abs(TCPA) / 60.0  # 海里
        line_length = rel_motion_dist * self.scale + 30  # 像素（+30是偏移量）

        # 计算相对运动线终点
        rel_cog_rad = math.radians(rel_cog)
        end_x = tx + line_length * math.sin(rel_cog_rad)
        end_y = ty - line_length * math.cos(rel_cog_rad)

        # 绘制相对运动线（红色实线）
        color = QColor(255, 0, 0)
        line_width = 2

        painter.setPen(QPen(color, line_width, Qt.SolidLine))
        painter.drawLine(QPointF(tx, ty), QPointF(end_x, end_y))

        # 绘制箭头
        arrow_size = 10
        arrow_angle = math.radians(30)

        left_arrow_x = end_x - arrow_size * math.sin(rel_cog_rad - arrow_angle)
        left_arrow_y = end_y + arrow_size * math.cos(rel_cog_rad - arrow_angle)

        right_arrow_x = end_x - arrow_size * math.sin(rel_cog_rad + arrow_angle)
        right_arrow_y = end_y + arrow_size * math.cos(rel_cog_rad + arrow_angle)

        painter.drawLine(QPointF(end_x, end_y), QPointF(left_arrow_x, left_arrow_y))
        painter.drawLine(QPointF(end_x, end_y), QPointF(right_arrow_x, right_arrow_y))

        # 绘制CPA点（最近点）
        if TCPA >= 0:
            # CPA点在相对运动线上，距离目标船 = rel_spd * TCPA / 60
            cpa_dist = TCPA / 60.0 * rel_spd  # 海里
            cpa_x = tx + cpa_dist * self.scale * math.sin(rel_cog_rad)
            cpa_y = ty - cpa_dist * self.scale * math.cos(rel_cog_rad)

            cpa_color = QColor(255, 200, 0)
            painter.setPen(QPen(cpa_color, 2))
            painter.setBrush(QBrush(cpa_color))
            painter.drawEllipse(QPointF(cpa_x, cpa_y), 6, 6)

            painter.setPen(QPen(cpa_color, 1))
            painter.setFont(QFont("Arial", 9, QFont.Bold))
            painter.drawText(int(cpa_x + 10), int(cpa_y - 5), "CPA")

    def draw_legend(self, painter):
        """绘制图例说明"""
        # 绘制图例背景框
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.setBrush(QBrush(QColor(255, 255, 255, 230)))

        legend_x = 10
        legend_y = 10
        legend_width = 150
        legend_height = 110

        painter.drawRect(legend_x, legend_y, legend_width, legend_height)

        # 标题
        y_offset = legend_y + 20
        painter.setFont(QFont("SimHei", 10, QFont.Bold))
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(legend_x + 10, y_offset, "图例说明")

        # 本船 - 蓝色方块
        y_offset += 25
        painter.setPen(QPen(QColor(0, 0, 255), 2))
        painter.setBrush(QBrush(QColor(0, 0, 255)))
        painter.drawRect(legend_x + 10, y_offset - 8, 10, 10)
        painter.setFont(QFont("SimHei", 9))
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(legend_x + 25, y_offset, "本船")

        # 危险目标 - 红色圆点
        y_offset += 20
        painter.setPen(QPen(QColor(255, 0, 0), 2))
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        painter.drawEllipse(QPointF(legend_x + 15, y_offset - 4), 5, 5)
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(legend_x + 25, y_offset, "危险目标")

        # 安全目标 - 绿色圆点
        y_offset += 20
        painter.setPen(QPen(QColor(0, 200, 0), 2))
        painter.setBrush(QBrush(QColor(0, 200, 0)))
        painter.drawEllipse(QPointF(legend_x + 15, y_offset - 4), 5, 5)
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(legend_x + 25, y_offset, "安全目标")

        # 相对运动线 - 虚线
        y_offset += 20
        painter.setPen(QPen(QColor(0, 0, 255), 2, Qt.DashLine))
        painter.drawLine(legend_x + 10, y_offset - 4, legend_x + 20, y_offset - 4)
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(legend_x + 25, y_offset, "相对运动线")


class ScenarioGeneratorMainWindow(QMainWindow):
    """场景生成器主窗口"""

    def __init__(self):
        super().__init__()
        self.current_scenarios = []
        self.current_file_path = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("海上船舶会遇场景生成器 - 专业版 (优化版)")
        self.showMaximized()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)

        splitter = QSplitter(Qt.Horizontal)

        self.table_widget = self.create_table_widget()
        splitter.addWidget(self.table_widget)

        preview_widget = self.create_preview_widget()
        splitter.addWidget(preview_widget)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        main_layout.addWidget(splitter)

        self.create_menu_bar()
        self.statusBar().showMessage("就绪")

    def create_control_panel(self):
        panel = QWidget()
        panel.setMaximumHeight(200)
        main_layout = QHBoxLayout(panel)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(25)

        panel_font = QFont()
        panel_font.setPointSize(11)
        panel.setFont(panel_font)

        basic_group = QGroupBox("基本参数")
        basic_group.setFont(panel_font)
        basic_layout = QGridLayout()
        basic_layout.setSpacing(12)
        basic_layout.setContentsMargins(15, 20, 15, 15)
        basic_layout.setVerticalSpacing(15)

        label1 = QLabel("船舶数量:")
        label1.setFont(panel_font)
        basic_layout.addWidget(label1, 0, 0)
        self.ship_count_combo = QComboBox()
        self.ship_count_combo.addItems(["两船", "三船", "四船", "五船"])
        self.ship_count_combo.setMinimumWidth(100)
        self.ship_count_combo.setMinimumHeight(30)
        self.ship_count_combo.setFont(panel_font)
        basic_layout.addWidget(self.ship_count_combo, 0, 1)

        label2 = QLabel("场景数量:")
        label2.setFont(panel_font)
        basic_layout.addWidget(label2, 1, 0)
        self.scenario_count_spin = QSpinBox()
        self.scenario_count_spin.setRange(1, 1000)
        self.scenario_count_spin.setValue(10)
        self.scenario_count_spin.setMinimumWidth(100)
        self.scenario_count_spin.setMinimumHeight(30)
        self.scenario_count_spin.setFont(panel_font)
        basic_layout.addWidget(self.scenario_count_spin, 1, 1)

        label3 = QLabel("初始位置:")
        label3.setFont(panel_font)
        basic_layout.addWidget(label3, 2, 0)
        pos_layout = QHBoxLayout()
        pos_layout.setSpacing(8)
        self.lat_spin = QDoubleSpinBox()
        self.lat_spin.setRange(-90, 90)
        self.lat_spin.setValue(31.0)
        self.lat_spin.setDecimals(4)
        self.lat_spin.setPrefix("N ")
        self.lat_spin.setMinimumWidth(120)
        self.lat_spin.setMinimumHeight(30)
        self.lat_spin.setFont(panel_font)
        pos_layout.addWidget(self.lat_spin)
        self.lon_spin = QDoubleSpinBox()
        self.lon_spin.setRange(-180, 180)
        self.lon_spin.setValue(123.0)
        self.lon_spin.setDecimals(4)
        self.lon_spin.setPrefix("E ")
        self.lon_spin.setMinimumWidth(120)
        self.lon_spin.setMinimumHeight(30)
        self.lon_spin.setFont(panel_font)
        pos_layout.addWidget(self.lon_spin)
        basic_layout.addLayout(pos_layout, 2, 1)

        basic_group.setLayout(basic_layout)
        main_layout.addWidget(basic_group)

        speed_group = QGroupBox("速度参数(节)")
        speed_group.setFont(panel_font)
        speed_layout = QGridLayout()
        speed_layout.setSpacing(12)
        speed_layout.setContentsMargins(15, 20, 15, 15)
        speed_layout.setVerticalSpacing(15)

        label4 = QLabel("本船:")
        label4.setFont(panel_font)
        speed_layout.addWidget(label4, 0, 0)
        own_speed_layout = QHBoxLayout()
        own_speed_layout.setSpacing(8)
        self.osog_min_spin = QDoubleSpinBox()
        self.osog_min_spin.setRange(0, 30)
        self.osog_min_spin.setValue(10.0)
        self.osog_min_spin.setMinimumWidth(70)
        self.osog_min_spin.setMinimumHeight(30)
        self.osog_min_spin.setFont(panel_font)
        own_speed_layout.addWidget(self.osog_min_spin)
        dash1 = QLabel("-")
        dash1.setFont(panel_font)
        own_speed_layout.addWidget(dash1)
        self.osog_max_spin = QDoubleSpinBox()
        self.osog_max_spin.setRange(0, 30)
        self.osog_max_spin.setValue(20.0)
        self.osog_max_spin.setMinimumWidth(70)
        self.osog_max_spin.setMinimumHeight(30)
        self.osog_max_spin.setFont(panel_font)
        own_speed_layout.addWidget(self.osog_max_spin)
        speed_layout.addLayout(own_speed_layout, 0, 1)

        label5 = QLabel("目标船:")
        label5.setFont(panel_font)
        speed_layout.addWidget(label5, 1, 0)
        target_speed_layout = QHBoxLayout()
        target_speed_layout.setSpacing(8)
        self.tsog_min_spin = QDoubleSpinBox()
        self.tsog_min_spin.setRange(0, 30)
        self.tsog_min_spin.setValue(10.0)
        self.tsog_min_spin.setMinimumWidth(70)
        self.tsog_min_spin.setMinimumHeight(30)
        self.tsog_min_spin.setFont(panel_font)
        target_speed_layout.addWidget(self.tsog_min_spin)
        dash2 = QLabel("-")
        dash2.setFont(panel_font)
        target_speed_layout.addWidget(dash2)
        self.tsog_max_spin = QDoubleSpinBox()
        self.tsog_max_spin.setRange(0, 30)
        self.tsog_max_spin.setValue(20.0)
        self.tsog_max_spin.setMinimumWidth(70)
        self.tsog_max_spin.setMinimumHeight(30)
        self.tsog_max_spin.setFont(panel_font)
        target_speed_layout.addWidget(self.tsog_max_spin)
        speed_layout.addLayout(target_speed_layout, 1, 1)

        speed_group.setLayout(speed_layout)
        main_layout.addWidget(speed_group)

        self.advanced_group = QGroupBox("会遇类型分配")
        self.advanced_group.setFont(panel_font)
        self.advanced_group.setCheckable(True)
        self.advanced_group.setChecked(False)
        advanced_layout = QGridLayout()
        advanced_layout.setContentsMargins(20, 25, 20, 20)
        advanced_layout.setVerticalSpacing(18)
        advanced_layout.setHorizontalSpacing(25)

        self.meeting_type_widgets = {}
        col = 0
        row = 0
        for i, meeting in enumerate(meeting_situations[:8]):
            label = QLabel(f"{meeting['name']}:")
            label.setFont(panel_font)
            label.setMinimumHeight(35)
            spin = QSpinBox()
            spin.setRange(0, 10)
            spin.setValue(0)
            spin.setMinimumWidth(70)
            spin.setMinimumHeight(35)
            spin.setFont(panel_font)
            advanced_layout.addWidget(label, row, col*2)
            advanced_layout.addWidget(spin, row, col*2+1)
            self.meeting_type_widgets[meeting['id']] = spin
            col += 1
            if col >= 2:
                col = 0
                row += 1

        self.advanced_group.setLayout(advanced_layout)
        main_layout.addWidget(self.advanced_group)

        self.gen_button = QPushButton("生成场景")
        self.gen_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                font-size: 16pt;
                padding: 15px 25px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.gen_button.setMinimumWidth(150)
        self.gen_button.setMinimumHeight(60)
        self.gen_button.clicked.connect(self.generate_scenarios)
        main_layout.addWidget(self.gen_button)

        return panel

    def create_table_widget(self):
        table = QTableWidget()
        table.setColumnCount(15)
        table.setHorizontalHeaderLabels([
            "场景ID", "时间戳", "目标船数", "本船纬度", "本船经度",
            "本船SOG", "本船COG", "目标船ID", "会遇类型",
            "目标船纬度", "目标船经度", "目标船SOG", "目标船COG",
            "TCPA(min)", "DCPA(NM)"
        ])
        table.horizontalHeader().setStretchLastSection(True)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.itemSelectionChanged.connect(self.on_table_selection_changed)
        return table

    def create_preview_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        self.polar_plot = PolarPlotWidget()
        layout.addWidget(self.polar_plot, stretch=3)

        info_splitter = QSplitter(Qt.Horizontal)

        left_info = QGroupBox("本船信息")
        left_layout = QVBoxLayout()
        self.ownship_info_text = QTextEdit()
        self.ownship_info_text.setReadOnly(True)
        # 移除最大高度限制，让文本框可以显示更多信息
        left_layout.addWidget(self.ownship_info_text)
        left_info.setLayout(left_layout)
        info_splitter.addWidget(left_info)

        right_info = QGroupBox("目标船信息")
        right_layout = QVBoxLayout()
        self.target_info_text = QTextEdit()
        self.target_info_text.setReadOnly(True)
        # 移除最大高度限制，让文本框可以显示更多信息
        right_layout.addWidget(self.target_info_text)
        right_info.setLayout(right_layout)
        info_splitter.addWidget(right_info)

        layout.addWidget(info_splitter, stretch=1)

        return widget

    def create_menu_bar(self):
        menubar = self.menuBar()

        file_menu = menubar.addMenu("文件")

        new_action = QAction("新建", self)
        new_action.setShortcut("Ctrl+N")
        new_action.triggered.connect(self.new_file)
        file_menu.addAction(new_action)

        open_action = QAction("打开", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self.open_file)
        file_menu.addAction(open_action)

        save_action = QAction("保存", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self.save_file)
        file_menu.addAction(save_action)

        save_as_action = QAction("另存为", self)
        save_as_action.setShortcut("Ctrl+Shift+S")
        save_as_action.triggered.connect(self.save_file_as)
        file_menu.addAction(save_as_action)

        file_menu.addSeparator()

        exit_action = QAction("退出", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        help_menu = menubar.addMenu("帮助")

        about_action = QAction("关于", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

        manual_action = QAction("使用说明", self)
        manual_action.triggered.connect(self.show_manual)
        help_menu.addAction(manual_action)

    def generate_scenarios(self):
        try:
            ship_count = self.ship_count_combo.currentIndex() + 2
            scenario_count = self.scenario_count_spin.value()

            osog_min = self.osog_min_spin.value()
            osog_max = self.osog_max_spin.value()
            tsog_min = self.tsog_min_spin.value()
            tsog_max = self.tsog_max_spin.value()
            lat = self.lat_spin.value()
            lon = self.lon_spin.value()

            meeting_type_counts = None
            if self.advanced_group.isChecked():
                meeting_type_counts = {}
                total_specified = 0
                for meeting_id, spin in self.meeting_type_widgets.items():
                    count = spin.value()
                    if count > 0:
                        meeting_type_counts[meeting_id] = count
                        total_specified += count

                if total_specified > ship_count - 1:
                    QMessageBox.warning(self, "警告",
                                      f"指定的会遇类型总数({total_specified})超过了目标船数量({ship_count - 1})")
                    return

                if total_specified == 0:
                    meeting_type_counts = None

            self.current_scenarios = []
            progress = QProgressDialog("正在生成场景...", "取消", 0, scenario_count, self)
            progress.setWindowModality(Qt.WindowModal)

            success_count = 0
            for i in range(scenario_count):
                if progress.wasCanceled():
                    break

                progress.setValue(i)
                QApplication.processEvents()

                scenario = generate_scenario(
                    ship_count - 1,
                    osog_min, osog_max,
                    tsog_min, tsog_max,
                    lat, lon,
                    meeting_type_counts
                )

                if scenario:
                    scenario['id'] = i + 1
                    self.current_scenarios.append(scenario)
                    success_count += 1

            progress.setValue(scenario_count)

            self.update_table()

            self.statusBar().showMessage(f"成功生成 {success_count}/{scenario_count} 个场景")
            QMessageBox.information(self, "完成", f"成功生成 {success_count} 个场景")

        except Exception as e:
            import traceback
            error_msg = f"生成场景时出错: {str(e)}\n\n详细信息:\n{traceback.format_exc()}"
            print(error_msg)
            QMessageBox.critical(self, "错误", error_msg)

    def update_table(self):
        self.table_widget.setRowCount(0)

        for scenario in self.current_scenarios:
            ownship = scenario['ownship']
            for target in scenario['targets']:
                row = self.table_widget.rowCount()
                self.table_widget.insertRow(row)

                self.table_widget.setItem(row, 0, QTableWidgetItem(str(scenario['id'])))
                self.table_widget.setItem(row, 1, QTableWidgetItem(scenario['timestamp']))
                self.table_widget.setItem(row, 2, QTableWidgetItem(str(len(scenario['targets']))))
                self.table_widget.setItem(row, 3, QTableWidgetItem(f"{ownship['lat']:.6f}"))
                self.table_widget.setItem(row, 4, QTableWidgetItem(f"{ownship['lon']:.6f}"))
                self.table_widget.setItem(row, 5, QTableWidgetItem(f"{ownship['sog']:.2f}"))
                self.table_widget.setItem(row, 6, QTableWidgetItem(f"{ownship['cog']:.2f}"))
                self.table_widget.setItem(row, 7, QTableWidgetItem(str(target['id'])))
                self.table_widget.setItem(row, 8, QTableWidgetItem(target['meeting_type']))
                self.table_widget.setItem(row, 9, QTableWidgetItem(f"{target['tlat']:.6f}"))
                self.table_widget.setItem(row, 10, QTableWidgetItem(f"{target['tlon']:.6f}"))
                self.table_widget.setItem(row, 11, QTableWidgetItem(f"{target['tsog']:.2f}"))
                self.table_widget.setItem(row, 12, QTableWidgetItem(f"{target['tcog']:.2f}"))
                self.table_widget.setItem(row, 13, QTableWidgetItem(f"{target['TCPA']:.2f}"))
                self.table_widget.setItem(row, 14, QTableWidgetItem(f"{target['DCPA']:.2f}"))

        self.table_widget.resizeColumnsToContents()

    def on_table_selection_changed(self):
        selected_rows = self.table_widget.selectedIndexes()
        if not selected_rows:
            return

        row = selected_rows[0].row()
        scenario_id = int(self.table_widget.item(row, 0).text())

        scenario = next((s for s in self.current_scenarios if s['id'] == scenario_id), None)
        if scenario:
            self.polar_plot.set_scenario(scenario)
            self.update_info_panel(scenario)

    def update_info_panel(self, scenario):
        ownship = scenario['ownship']

        own_html = f"""
        <style>
            body {{ font-family: Arial; font-size: 10pt; }}
            h4 {{ color: #2196F3; margin: 5px 0; }}
            p {{ margin: 3px 0; }}
            .label {{ color: #666; }}
        </style>
        <h4>场景 #{scenario['id']}</h4>
        <p><span class="label">时间:</span> {scenario['timestamp']}</p>
        <p><span class="label">目标船数:</span> {len(scenario['targets'])}</p>
        <hr>
        <h4>本船参数</h4>
        <p><span class="label">位置:</span> {ownship['lat']:.4f}°N, {ownship['lon']:.4f}°E</p>
        <p><span class="label">速度:</span> {ownship['sog']:.1f} 节</p>
        <p><span class="label">航向:</span> {ownship['cog']:.1f}°</p>
        """
        self.ownship_info_text.setHtml(own_html)

        target_html = """
        <style>
            body { font-family: Arial; font-size: 9pt; }
            h4 { margin: 3px 0; }
            p { margin: 2px 0; }
            .danger { color: #f44336; font-weight: bold; }
            .safe { color: #4CAF50; }
            .label { color: #666; }
        </style>
        """

        for target in scenario['targets']:
            is_danger = (target['TCPA'] >= 0 and target['DCPA'] <= SAFE_DCPA)
            status_class = "danger" if is_danger else "safe"
            status_text = "⚠ 危险" if is_danger else "✓ 安全"

            target_html += f"""
            <h4 class="{status_class}">目标船 {target['id']} - {target['meeting_type']} {status_text}</h4>
            <p><span class="label">位置:</span> {target['tlat']:.4f}°N, {target['tlon']:.4f}°E</p>
            <p><span class="label">距离/方位:</span> {target['dist']:.2f} NM / {target['brg']:.1f}°</p>
            <p><span class="label">速度/航向:</span> {target['tsog']:.1f} 节 / {target['tcog']:.1f}°</p>
            <p><span class="label">TCPA:</span> {target['TCPA']:.1f} 分钟 | <span class="label">DCPA:</span> {target['DCPA']:.2f} NM</p>
            <p><span class="label">相对运动:</span> {target['rel_spd']:.1f} 节 / {target['rel_cog']:.1f}°</p>
            <hr>
            """

        self.target_info_text.setHtml(target_html)

    def save_file(self):
        if self.current_file_path:
            self.save_to_csv(self.current_file_path)
        else:
            self.save_file_as()

    def save_file_as(self):
        if not self.current_scenarios:
            QMessageBox.warning(self, "警告", "没有可保存的场景数据")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存场景数据", "", "CSV文件 (*.csv)"
        )

        if file_path:
            self.save_to_csv(file_path)
            self.current_file_path = file_path

    def save_to_csv(self, file_path):
        try:
            with open(file_path, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)

                writer.writerow([
                    "场景ID", "时间戳", "目标船数", "本船纬度", "本船经度",
                    "本船SOG(节)", "本船COG(度)", "目标船ID", "会遇类型",
                    "目标船纬度", "目标船经度", "目标船SOG(节)", "目标船COG(度)",
                    "距离(海里)", "方位(度)", "TCPA(分钟)", "DCPA(海里)",
                    "相对速度(节)", "相对航向(度)", "危险状态"
                ])

                for scenario in self.current_scenarios:
                    ownship = scenario['ownship']
                    for target in scenario['targets']:
                        danger = "危险" if (target['TCPA'] >= 0 and target['DCPA'] <= SAFE_DCPA) else "安全"

                        writer.writerow([
                            scenario['id'],
                            scenario['timestamp'],
                            len(scenario['targets']),
                            f"{ownship['lat']:.6f}",
                            f"{ownship['lon']:.6f}",
                            f"{ownship['sog']:.2f}",
                            f"{ownship['cog']:.2f}",
                            target['id'],
                            target['meeting_type'],
                            f"{target['tlat']:.6f}",
                            f"{target['tlon']:.6f}",
                            f"{target['tsog']:.2f}",
                            f"{target['tcog']:.2f}",
                            f"{target['dist']:.2f}",
                            f"{target['brg']:.2f}",
                            f"{target['TCPA']:.2f}",
                            f"{target['DCPA']:.2f}",
                            f"{target['rel_spd']:.2f}",
                            f"{target['rel_cog']:.2f}",
                            danger
                        ])

            self.statusBar().showMessage(f"文件已保存: {file_path}")
            QMessageBox.information(self, "成功", f"场景数据已保存到:\n{file_path}")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存文件时出错: {str(e)}")

    def open_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开场景数据", "", "CSV文件 (*.csv)"
        )

        if file_path:
            self.load_from_csv(file_path)
            self.current_file_path = file_path

    def load_from_csv(self, file_path):
        try:
            self.current_scenarios = []
            scenarios_dict = {}

            with open(file_path, 'r', encoding='utf-8-sig') as f:
                reader = csv.DictReader(f)

                for row in reader:
                    scenario_id = int(row['场景ID'])

                    if scenario_id not in scenarios_dict:
                        scenarios_dict[scenario_id] = {
                            'id': scenario_id,
                            'timestamp': row['时间戳'],
                            'ownship': {
                                'lat': float(row['本船纬度']),
                                'lon': float(row['本船经度']),
                                'sog': float(row['本船SOG(节)']),
                                'cog': float(row['本船COG(度)'])
                            },
                            'targets': []
                        }

                    target = {
                        'id': int(row['目标船ID']),
                        'meeting_type': row['会遇类型'],
                        'tlat': float(row['目标船纬度']),
                        'tlon': float(row['目标船经度']),
                        'tsog': float(row['目标船SOG(节)']),
                        'tcog': float(row['目标船COG(度)']),
                        'dist': float(row['距离(海里)']),
                        'brg': float(row['方位(度)']),
                        'TCPA': float(row['TCPA(分钟)']),
                        'DCPA': float(row['DCPA(海里)']),
                        'rel_spd': float(row['相对速度(节)']),
                        'rel_cog': float(row['相对航向(度)'])
                    }

                    target['olat'] = scenarios_dict[scenario_id]['ownship']['lat']
                    target['olon'] = scenarios_dict[scenario_id]['ownship']['lon']
                    target['osog'] = scenarios_dict[scenario_id]['ownship']['sog']
                    target['ocog'] = scenarios_dict[scenario_id]['ownship']['cog']

                    scenarios_dict[scenario_id]['targets'].append(target)

            self.current_scenarios = list(scenarios_dict.values())
            self.update_table()

            self.statusBar().showMessage(f"已加载 {len(self.current_scenarios)} 个场景")
            QMessageBox.information(self, "成功", f"已从文件加载 {len(self.current_scenarios)} 个场景")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载文件时出错: {str(e)}")

    def new_file(self):
        if self.current_scenarios:
            reply = QMessageBox.question(
                self, "确认", "当前有未保存的数据，是否继续？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.No:
                return

        self.current_scenarios = []
        self.current_file_path = None
        self.table_widget.setRowCount(0)
        self.polar_plot.set_scenario(None)
        self.ownship_info_text.clear()
        self.target_info_text.clear()
        self.statusBar().showMessage("已新建文件")

    def show_about(self):
        QMessageBox.about(self, "关于",
            "海上船舶会遇场景生成器 - 专业版\n\n"
            "版本: 2.0 (优化版)\n"
            "功能: 自动生成具有真实碰撞危险的2-5船会遇场景\n\n"
            "改进特性:\n"
            "- 确保生成的场景具有真实碰撞危险\n"
            "- 自动化会遇类型分配\n"
            "- 专业界面设计\n"
            "- 极坐标可视化\n"
            "- 完整的数据导出功能")

    def show_manual(self):
        QMessageBox.information(self, "使用说明",
            "使用说明:\n\n"
            "1. 选择船舶数量(2-5船)\n"
            "2. 设置场景生成数量\n"
            "3. 配置速度范围和初始位置\n"
            "4. 可选: 高级选项中指定会遇类型\n"
            "5. 点击'生成场景'按钮\n"
            "6. 在表格中选择场景预览\n"
            "7. 使用鼠标滚轮缩放，拖拽移动预览图\n"
            "8. 保存数据到CSV文件")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    font = QFont("Microsoft YaHei", 10)
    app.setFont(font)

    window = ScenarioGeneratorMainWindow()
    window.show()

    sys.exit(app.exec_())
