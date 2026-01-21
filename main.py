from PyQt5.uic import loadUi
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

import math
import random

import logging
import sys
import traceback
import xml.dom.minidom
import json
import georef

SAFE_TCPA = 30.0
SAFE_DCPA = 2.0
TT_DCPA = 1.0

with open('config.json', 'r') as f:
    conf = json.load(f)
    TT_DCPA = conf.get('tt_DCPA', 1.0)

speed_list = [
    (1, '海上全速', 16.6, 17.6),
    (2, '全速', 13.2, 14.5),
    (3, '半速', 10.9, 12.2),
    (4, '慢速', 8.9, 10),
    (5, '微速', 5.7, 6.4),
]

visibility_list = [
    (1, '不良'),
    (2, '良好'),
]

stage_list = [
    # 8,14海里
    (1, '碰撞危险', 8, 14),
    # 2-3海里
    (2, '紧迫局面', 2, 3),
    # 1-2海里
    (3, '紧迫危险', 1, 2),
    (4, '没有危险', 1, 14),
]

ownship_behavior_list = [
    (1, '直航船'),
    (2, '让路船'),
]

meeting_situation_list = [
    (1, '对遇', (356, 6), (8, 14), 1),
    (2, '右舷小角度交叉会遇', (6, 67.5), (8, 14), 1),
    (3, '右舷大角度交叉会遇', (67.5, 112.5), (8, 14), 1),
    (4, '左舷追越', (180, 247.5), (4, 7), 1),
    (5, '右舷追越', (112.5, 180), (4, 7), 1),
    (6, '左舷小角度交叉会遇', (247.5, 292.5), (8, 14), 2),
    (7, '左舷大角度交叉会遇', (292.5, 354), (8, 14), 2),
    (8, '左舷被追越', (112.5, 180), (4, 7), 2),  # 原始方位为 Bt，需改为 Bo
    (9, '右舷被追越', (180, 247.5), (4, 7), 2),  # 原始方位为 Bt，需改为 Bo
    (10, '正后被追越', (180, 180), (4, 7), 2),   # 原始方位为 Bt，需改为 Bo
]

target_behavior_list = [
    (1, '协调避让'),
    (2, '不协调避让'),
]

meeting_situation_table = [
    {
        'name': '对遇',
        'id': 1,
        'deg': (354, 6),
        'dist': (8, 14),
        'plan': '向右转向',
    },
    {
        'name': '右舷小角度交叉会遇',
        'id': 2,
        'deg': (6, 67.5),
        'dist': (8, 14),
        'plan': '向右转向',
    },
    {
        'name': '右舷大角度交叉会遇',
        'id': 3,
        'deg': (67.5, 112.5),
        'dist': (8, 14),
        'plan': '向右转向',
    },
    {
        'name': '左舷追越',
        'id': 4,
        'deg': (180, 247.5),
        'dist': (4, 7),
        'plan': '左/右让',
    },
    {
        'name': '右舷追越',
        'id': 5,
        'deg': (112.5, 180),
        'dist': (4, 7),
        'plan': '左/右让',
    },
    {
        'name': '左舷小角度交叉会遇',
        'id': 6,
        'deg': (247.5, 292.5),
        'dist': (8, 14),
        'plan': '直航',
    },
    {
        'name': '左舷大角度交叉会遇',
        'id': 7,
        'deg': (292.5, 354),
        'dist': (8, 14),
        'plan': '直航',
    },
    {
        'name': '左舷被追越',
        'id': 8,
        'deg': (112.5, 180),
        'dist': (4, 7),
        'plan': '左/右让',
    },
    {
        'name': '右舷被追越',
        'id': 9,
        'deg': (180, 247.5),
        'dist': (4, 7),
        'plan': '左/右让',
},
]


def get_rel_course_north(v_x_o, v_y_o, v_x_t, v_y_t):
    dvx = v_x_t - v_x_o
    dvy = v_y_t - v_y_o
    course = math.degrees(math.atan2(dvx, dvy))
    if course < 0.0:
        course += 360.0

    return course


def calc_rel_spd_cog(osog, ocog, tsog, tcog):
    # 正北分解速度的x和y
    v_x_o = osog * math.sin(math.radians(ocog))
    v_y_o = osog * math.cos(math.radians(ocog))
    v_x_t = tsog * math.sin(math.radians(tcog))
    v_y_t = tsog * math.cos(math.radians(tcog))

    # 计算相对本船的速度
    v_x_rel = v_x_t - v_x_o
    v_y_rel = v_y_t - v_y_o
    rel_spd = math.sqrt(v_x_rel*v_x_rel + v_y_rel*v_y_rel)

    # 得到相对本船的相对速度方向
    rel_course = get_rel_course_north(v_x_o, v_y_o, v_x_t, v_y_t)
    return rel_spd, rel_course

def calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog):
    # 计算2个位置的相对距离和相对方位
    # 得到的结果是(tlat, tlon)点在(olat, olon)的brg方位上
    dist, brg = georef.DistanceBearingMercator(tlat, tlon, olat, olon)

    # 正北分解速度的x和y
    v_x_o = osog * math.sin(math.radians(ocog))
    v_y_o = osog * math.cos(math.radians(ocog))
    v_x_t = tsog * math.sin(math.radians(tcog))
    v_y_t = tsog * math.cos(math.radians(tcog))

    # 计算相对本船的速度
    v_x_rel = v_x_t - v_x_o
    v_y_rel = v_y_t - v_y_o
    rel_spd = math.sqrt(v_x_rel*v_x_rel + v_y_rel*v_y_rel)

    # 得到相对本船的相对速度方向
    rel_course = get_rel_course_north(v_x_o, v_y_o, v_x_t, v_y_t)

    # ???
    delta = rel_course - brg - 180.0
    # if delta < 0.0:
    #     delta = delta + 360.0
    # if delta > 180.0:
    #     delta = delta - 180.0
    TCPA = 99999.0
    if rel_spd > 0.001:
        TCPA = dist * math.cos(math.radians(delta)) / rel_spd * 60.0

    DCPA = abs(dist * math.sin(math.radians(delta)))

    return TCPA, DCPA


def mod360(d):
    v = math.fmod(d, 360)
    if v < 0.0:
        v += 360.0
    return v


def fmt2f(*v):
    for i in v:
        print('%.2f' % i)


OSOG_MIN = 10.0
OSOG_MAX = 20.0
TSOG_MIN = 10.0
TSOG_MAX = 20.0


def gen_oship(lat, lon, sog_min, sog_max):
    sog = random.randint(round(sog_min*10), round(sog_max*10))*0.1
    cog = random.randint(0, 3599)*0.1
    return {
        'lat': lat,
        'lon': lon,
        'sog': sog,
        'cog': cog,
    }


def gen_tship(oship, relbrgMin, relbrgMax, distMin, distMax, sogMin, sogMax):
    if relbrgMax < relbrgMin:
        relbrgMax += 360
    olat = oship['lat']
    olon = oship['lon']
    osog = oship['sog']
    ocog = oship['cog']

    dist = random.randint(round(distMin*10), round(distMax*10))*0.1
    relbrg = random.randint(round(relbrgMin*10), round(relbrgMax*10))*0.1
    relbrg = mod360(relbrg)
    tsog = random.randint(round(sogMin*10), round(sogMax*10))*0.1
    tcog = mod360(ocog + relbrg + 180.0)

    rel_spd, rel_cog = calc_rel_spd_cog(tsog, tcog, osog, ocog)

    brg = rel_cog
    tlat, tlon = georef.ll_gc_ll(olat, olon, brg, dist)
    TCPA, DCPA = calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog)

    rel_spd, rel_cog = calc_rel_spd_cog(osog, ocog, tsog, tcog)

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
        'rel_cog': rel_cog,
        'rel_spd': rel_spd,
        'TCPA': TCPA,
        'DCPA': DCPA,
    }


def gen_tship_no_danger(oship, relbrgMin, relbrgMax,
                        distMin, distMax, sogMin, sogMax):
    if relbrgMax < relbrgMin:
        relbrgMax += 360
    olat = oship['lat']
    olon = oship['lon']
    osog = oship['sog']
    ocog = oship['cog']

    dist = random.randint(round(distMin*10), round(distMax*10))*0.1
    relbrg = random.randint(round(relbrgMin*10), round(relbrgMax*10))*0.1
    relbrg = mod360(relbrg)
    tsog = random.randint(round(sogMin*10), round(sogMax*10))*0.1
    tcog = mod360(ocog + relbrg + 180.0)

    rel_spd, rel_cog = calc_rel_spd_cog(tsog, tcog, osog, ocog)

    brg = mod360(rel_cog + random.randint(150, 3450) * 0.1)
    # brg = mod360(rel_cog + random.randint(135, 225))
    tlat, tlon = georef.ll_gc_ll(olat, olon, brg, dist)
    TCPA, DCPA = calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog)

    if TCPA >= 0.0 and DCPA <= 2.0:
        # 还有危险
        # 直接使TCPA小于0
        brg = mod360(rel_cog + 180)
        tlat, tlon = georef.ll_gc_ll(olat, olon, brg, dist)
        TCPA, DCPA = calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog)

    rel_spd, rel_cog = calc_rel_spd_cog(osog, ocog, tsog, tcog)

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
        'rel_cog': rel_cog,
        'rel_spd': rel_spd,
        'TCPA': TCPA,
        'DCPA': DCPA,
    }


def gen_tship_check(oship, tships, rel_brg_min, rel_brg_max,
                    dist_min, dist_max, sog_min, sog_max):
    tship = gen_tship(oship, rel_brg_min, rel_brg_max,
                      dist_min, dist_max, sog_min, sog_max)
    for t in tships:
        TCPA, DCPA = calc_CPA(
            tship['tlat'], tship['tlon'], t['tlat'], t['tlon'],
            tship['tsog'], t['tsog'], tship['tcog'], t['tcog'])
        if TCPA >= 0.0 and DCPA <= 2.0:
            return None
    return tship


def gen_tship_check_no_danger(oship, tships, rel_brg_min, rel_brg_max,
                              dist_min, dist_max, sog_min, sog_max):
    tship = gen_tship_no_danger(oship, rel_brg_min, rel_brg_max,
                                dist_min, dist_max, sog_min, sog_max)
    for t in tships:
        TCPA, DCPA = calc_CPA(
            tship['tlat'], tship['tlon'], t['tlat'], t['tlon'],
            tship['tsog'], t['tsog'], tship['tcog'], t['tcog'])
        if TCPA >= 0.0 and DCPA <= 2.0:
            return None
    return tship


def make_tship_arg_detail(oship, relbrg, dist, tsog):
    olat = oship['lat']
    olon = oship['lon']
    osog = oship['sog']
    ocog = oship['cog']

    tcog = mod360(ocog + relbrg + 180.0)

    rel_spd, rel_cog = calc_rel_spd_cog(tsog, tcog, osog, ocog)

    # 修正：正确计算相对方位（Bt）
    # Bt = 目标船真方位 - 本船船首向
    # 目标船真方位就是rel_cog（相对航向）
    relbrg_corrected = mod360(rel_cog - ocog)

    # TCPA的计算公式:
    # delta = rel_course - brg - 180.0
    # TCPA = dist * math.cos(math.radians(delta)) / rel_spd * 60.0
    # delta = acos(TCPA/dist*rel_spd/60.0)
    # 根据以上公式,计算让TCPA=0,(0,30], TCPA>30, TCPA<0等几个数值
    # acos函数的参数要求[-1,1],需要进行处理
    # DCPA的计算公式:
    # DCPA = dist * math.sin(math.radians(delta))
    # delta = asin(DCPA / dist)

    # acos参数范围[-1,1], 值范围[180, 0], 单调递减
    # 必定有cosdeg0 < cosdeg1

    cosdeg0 = math.degrees(math.acos(min(30.0 / dist * rel_spd / 60.0, 1.0)))
    cosdeg1 = math.degrees(math.acos(0.0 / dist * rel_spd / 60.0))

    # danger_cos = [cosdeg0, cosdeg1]
    # nodanger_cos = [min(math.floor(cosdeg1+1), 179), 180]

    # 对于danger_delta还需要处理,如果dist大于2时,识别为nodanger

    # asin参数范围[-1,1], 值范围[-90, 90], 单调递增
    # 必定有sindeg在[0, 90]
    # 必定有sindeg0<sindeg1
    sindeg0 = math.degrees(math.asin(0))
    sindeg1 = math.degrees(math.asin(min(2.0/dist, 1.0)))

    # danger_sin = [sindeg0, sindeg1]
    # nodanger_sin = [min(math.floor(sindeg1+1), 89), 90]

    danger_rngs = []
    nodanger_rngs = []
    # 没有相交
    if cosdeg1 < sindeg0 or sindeg1 < cosdeg0:
        pass
    # sin被cos包容
    elif cosdeg0 <= sindeg0 and cosdeg1 >= sindeg1:
        danger_rngs.append([round(sindeg0), round(sindeg1)])
    # cos被sin包容
    elif sindeg0 <= cosdeg0 and sindeg1 >= cosdeg1:
        danger_rngs.append([round(cosdeg0), round(cosdeg1)])
    # 相交
    elif cosdeg1 >= sindeg0:
        danger_rngs.append([round(sindeg0), round(cosdeg0)])
    else:
        danger_rngs.append([round(cosdeg0), round(sindeg0)])

    if danger_rngs:
        val0 = danger_rngs[0][0]
        val1 = danger_rngs[0][1]
        if val0 > 0:
            nodanger_rngs.append([0, val0-1])
        if val1 < 180:
            nodanger_rngs.append([val1+1, 180])
    else:
        nodanger_rngs.append([0, 180])

    danger_rngs = [[0, 0]]
    rel_spd, rel_cog = calc_rel_spd_cog(osog, ocog, tsog, tcog)

    return {
        'olat': olat,
        'olon': olon,
        # 'tlat': tlat,
        # 'tlon': tlon,
        'osog': osog,
        'ocog': ocog,
        'tsog': tsog,
        'tcog': tcog,
        'dist': dist,
        # 'brg': brg,
        'relbrg': relbrg_corrected,  # 使用修正后的相对方位
        'rel_cog': rel_cog,
        'rel_spd': rel_spd,
        'danger': danger_rngs,
        'nodanger': nodanger_rngs
    }


def make_tship_arg(oship, relbrgMin, relbrgMax,
                   distMin, distMax, sogMin, sogMax):
    if relbrgMax < relbrgMin:
        relbrgMax += 360
    olat = oship['lat']
    olon = oship['lon']
    osog = oship['sog']
    ocog = oship['cog']

    dist = random.randint(round(distMin*10), round(distMax*10))*0.1
    relbrg = random.randint(round(relbrgMin*10), round(relbrgMax*10))*0.1
    relbrg = mod360(relbrg)
    tsog = random.randint(round(sogMin*10), round(sogMax*10))*0.1
    tcog = mod360(ocog + relbrg + 180.0)

    rel_spd, rel_cog = calc_rel_spd_cog(tsog, tcog, osog, ocog)

    # 修正：正确计算相对方位（Bt）
    # Bt = 目标船真方位 - 本船船首向
    # 目标船真方位就是rel_cog（相对航向）
    relbrg_corrected = mod360(rel_cog - ocog)

    # TCPA的计算公式:
    # delta = rel_course - brg - 180.0
    # TCPA = dist * math.cos(math.radians(delta)) / rel_spd * 60.0
    # delta = acos(TCPA/dist*rel_spd/60.0)
    # 根据以上公式,计算让TCPA=0,(0,30], TCPA>30, TCPA<0等几个数值
    # acos函数的参数要求[-1,1],需要进行处理
    # DCPA的计算公式:
    # DCPA = dist * math.sin(math.radians(delta))
    # delta = asin(DCPA / dist)

    # acos参数范围[-1,1], 值范围[180, 0], 单调递减
    # 必定有cosdeg0 < cosdeg1

    cosdeg0 = math.degrees(math.acos(min(SAFE_TCPA / dist * rel_spd / 60.0, 1.0)))
    cosdeg1 = math.degrees(math.acos(0.0 / dist * rel_spd / 60.0))

    # danger_cos = [cosdeg0, cosdeg1]
    # nodanger_cos = [min(math.floor(cosdeg1+1), 179), 180]

    # 对于danger_delta还需要处理,如果dist大于2时,识别为nodanger

    # asin参数范围[-1,1], 值范围[-90, 90], 单调递增
    # 必定有sindeg在[0, 90]
    # 必定有sindeg0<sindeg1
    sindeg0 = math.degrees(math.asin(0))
    sindeg1 = math.degrees(math.asin(min(SAFE_DCPA/dist, 1.0)))

    # danger_sin = [sindeg0, sindeg1]
    # nodanger_sin = [min(math.floor(sindeg1+1), 89), 90]

    danger_rngs = []
    nodanger_rngs = []
    # 没有相交
    if cosdeg1 < sindeg0 or sindeg1 < cosdeg0:
        pass
    # sin被cos包容
    elif cosdeg0 <= sindeg0 and cosdeg1 >= sindeg1:
        danger_rngs.append([round(sindeg0), round(sindeg1)])
    # cos被sin包容
    elif sindeg0 <= cosdeg0 and sindeg1 >= cosdeg1:
        danger_rngs.append([round(cosdeg0), round(cosdeg1)])
    # 相交
    elif cosdeg1 >= sindeg0:
        danger_rngs.append([round(sindeg0), round(cosdeg0)])
    else:
        danger_rngs.append([round(cosdeg0), round(sindeg0)])

    if danger_rngs:
        val0 = danger_rngs[0][0]
        val1 = danger_rngs[0][1]
        if val0 > 0:
            nodanger_rngs.append([0, val0-1])
        if val1 < 180:
            nodanger_rngs.append([val1+1, 180])
    else:
        nodanger_rngs.append([0, 180])

    rel_spd, rel_cog = calc_rel_spd_cog(osog, ocog, tsog, tcog)

    return {
        'olat': olat,
        'olon': olon,
        # 'tlat': tlat,
        # 'tlon': tlon,
        'osog': osog,
        'ocog': ocog,
        'tsog': tsog,
        'tcog': tcog,
        'dist': dist,
        # 'brg': brg,
        'relbrg': relbrg_corrected,  # 使用修正后的相对方位
        'rel_cog': rel_cog,
        'rel_spd': rel_spd,
        'danger': danger_rngs,
        'nodanger': nodanger_rngs
    }


def make_tship_by_arg(tship_arg, delta):
    olat = tship_arg['olat']
    olon = tship_arg['olon']
    osog = tship_arg['osog']
    ocog = tship_arg['ocog']
    tsog = tship_arg['tsog']
    tcog = tship_arg['tcog']
    dist = tship_arg['dist']
    rel_cog = tship_arg['rel_cog']
    brg = (rel_cog-180.0) + delta
    brg = mod360(brg)
    tlat, tlon = georef.ll_gc_ll(olat, olon, brg, dist)
    TCPA, DCPA = calc_CPA(olat, olon, tlat, tlon, osog, tsog, ocog, tcog)
    tship = {k: v for k, v in tship_arg.items()}
    tship['tlat'] = tlat
    tship['tlon'] = tlon
    tship['brg'] = brg
    tship['TCPA'] = TCPA
    tship['DCPA'] = DCPA
    return tship


def make_tship_dist_is_ok(tship0, tship1):
    lat0 = tship0['tlat']
    lon0 = tship0['tlon']
    lat1 = tship1['tlat']
    lon1 = tship1['tlon']
    dist, brg = georef.DistanceBearingMercator(lat0, lon0, lat1, lon1)
    if dist < 0.5:
        return False
    return True


def make_tship_is_safe_tt(tship0, tship1):
    lat0 = tship0['tlat']
    lon0 = tship0['tlon']
    sog0 = tship0['tsog']
    cog0 = tship0['tsog']
    lat1 = tship1['tlat']
    lon1 = tship1['tlon']
    sog1 = tship1['tsog']
    cog1 = tship1['tsog']
    TCPA, DCPA = calc_CPA(lat0, lon0, lat1, lon1, sog0, sog1, cog0, cog1)
    if TCPA < 0.0:
        return True
    # if TCPA > 30.0 or DCPA > 2.0:
    # if TCPA > SAFE_TCPA or DCPA > SAFE_DCPA:
    # if TCPA > SAFE_TCPA:
    if DCPA > TT_DCPA:
        return True
    return False


def make_tship_check_tt(tships, tt):
    size = len(tships)
    if size <= 1:
        return True
    danger_list = []
    for i in range(size):
        tship0 = tships[i]
        for j in range(i+1, size):
            tship1 = tships[j]
            # 距离太近
            if not make_tship_dist_is_ok(tship0, tship1):
                return False
            is_safe = make_tship_is_safe_tt(tship0, tship1)
            if tt == 2 and is_safe:
                return False
            if tt == 1 and not is_safe:
                return False
            danger_list.append(1 if not is_safe else 0)
    sum_danger = sum(danger_list)
    if tt == 1 and sum_danger == 0:
        return True
    if tt == 2 and sum_danger == len(danger_list):
        return True
    if tt == 3 and sum_danger > 0:
        return True
    if tt == 4:
        return True
    return False


# ot:
# 1:本船和所有目标船没有危险
# 2:本船和所有目标船有危险
# 3:本船至少和1条目标船有危险
# 4: 随机
# tt:
# 1:所有目标船之间都没有危险
# 2:所有目标船之间都有危险
# 3:所有目标船之间,至少1对有危险
# 4: 随机
def make_tship(oship, tships_condition, ot, tt):
    tship_args = []
    tship_deltas = []
    for idx, cond in enumerate(tships_condition):
        relbrg_min = cond['relbrg_min']
        relbrg_max = cond['relbrg_max']
        dist_min = cond['dist_min']
        dist_max = cond['dist_max']
        sog_min = cond['sog_min']
        sog_max = cond['sog_max']
        targ = None
        while True:
            targ = make_tship_arg(oship, relbrg_min, relbrg_max,
                                  dist_min, dist_max, sog_min, sog_max)
            if targ['danger'] and targ['nodanger']:
                break
            elif not targ['danger'] and ot == 1:
                break
            elif not targ['nodanger'] and ot == 2:
                break
            elif ot == 4:
                break

        tship_args.append(targ)
        dangers = targ['danger']
        nodangers = targ['nodanger']
        delta_list = []
        # 至少一条危险目标船时,ot==3
        # 第一条目标船必定时危险,(ot==2 and idx==0)
        if ot == 1:
            for nodanger in nodangers:
                delta_list.extend(range(nodanger[0], nodanger[1]+1))
        elif ot == 2 or (ot == 3 and idx == 0):
            for danger in dangers:
                delta_list.extend(range(danger[0], danger[1]+1))
        else:
            for danger in dangers:
                delta_list.extend(range(danger[0], danger[1]+1))
            for nodanger in nodangers:
                delta_list.extend(range(nodanger[0], nodanger[1]+1))
        random.shuffle(delta_list)
        tship_deltas.append(delta_list)

    tships = []
    tships_delta_i = {}
    idx = 0
    while True:
        targ = tship_args[idx]
        delta_start = tships_delta_i.get(idx, 0)
        for delta_i in range(delta_start, len(tship_deltas[idx])):
            delta = tship_deltas[idx][delta_i]
            tship = make_tship_by_arg(targ, delta)
            check_list = tships[:]
            check_list.append(tship)
            if make_tship_check_tt(check_list, tt):
                tships.append(tship)
                tships_delta_i[idx] = delta_i + 1
                idx = idx + 1
                break
        else:
            # 没有满足条件的delta
            tships = tships[:-1]
            idx = idx - 1
        if idx >= len(tship_args):
            break
        if idx < 0:
            print('fail')
            break

    return tships


def make_tship_one_detail(oship, idx, relbrg, dist, sog, ot, tt):
    tship_args = []
    tship_deltas = []
    targ = None
    while True:
        targ = make_tship_arg_detail(oship, relbrg, dist, sog)
        if targ['danger'] and targ['nodanger']:
            break
        elif not targ['danger'] and ot == 1:
            break
        elif not targ['nodanger'] and ot == 2:
            break
        elif ot == 4:
            break

    tship_args.append(targ)
    dangers = targ['danger']
    nodangers = targ['nodanger']
    delta_list = []
    # 至少一条危险目标船时,ot==3
    # 第一条目标船必定时危险,(ot==2 and idx==0)
    if ot == 1:
        for nodanger in nodangers:
            delta_list.extend(range(nodanger[0], nodanger[1]+1))
    elif ot == 2 or (ot == 3 and idx == 0):
        for danger in dangers:
            delta_list.extend(range(danger[0], danger[1]+1))
    else:
        for danger in dangers:
            delta_list.extend(range(danger[0], danger[1]+1))
        for nodanger in nodangers:
            delta_list.extend(range(nodanger[0], nodanger[1]+1))
    random.shuffle(delta_list)
    tship_deltas.append(delta_list)

    tships = []
    tships_delta_i = {}
    idx = 0
    while True:
        targ = tship_args[idx]
        delta_start = tships_delta_i.get(idx, 0)
        for delta_i in range(delta_start, len(tship_deltas[idx])):
            delta = tship_deltas[idx][delta_i]
            tship = make_tship_by_arg(targ, delta)
            check_list = tships[:]
            check_list.append(tship)
            if make_tship_check_tt(check_list, tt):
                tships.append(tship)
                tships_delta_i[idx] = delta_i + 1
                idx = idx + 1
                break
        else:
            # 没有满足条件的delta
            tships = tships[:-1]
            idx = idx - 1
        if idx >= len(tship_args):
            break
        if idx < 0:
            print('fail')
            break


class Ship:

    def __init__(self):
        self.lat = None
        self.lon = None
        self.sog = 0.0
        self.cog = 0.0

    def calc_CPA(self, other):
        TCPA, DCPA = calc_CPA(self.lat, self.lon, other.lat, other.lon, self.sog, other.sog, self.cog, other.cog)
        self.TCPA = TCPA
        self.DCPA = DCPA
        return TCPA, DCPA


class MakeTShip:
    def __init__(self, oship, cond): # relbrg_min, relbrg_max, dist_min, dist_max, sog_min, sog_max):
        self.oship = oship
        relbrg_min = cond['relbrg_min']
        relbrg_max = cond['relbrg_max']
        dist_min = cond['dist_min']
        dist_max = cond['dist_max']
        sog_min = cond['sog_min']
        sog_max = cond['sog_max']
        self.relbrg_min = relbrg_min
        self.relbrg_max = relbrg_max
        self.dist_min = dist_min
        self.dist_max = dist_max
        self.sog_min = sog_min
        self.sog_max = sog_max
        step = round((relbrg_max*10 - relbrg_min*10) // 10)
        # 处理step为0的情况
        if step == 0:
            step = 1
        self.relbrg_list = [val*0.1 for val in range(round(relbrg_min*10), round(relbrg_max*10)+step, step)]
        step = round((dist_max*10 - dist_min*10) // 10)
        # 处理step为0的情况
        if step == 0:
            step = 1
        self.dist_list = [val*0.1 for val in range(round(dist_min*10), round(dist_max*10)+step, step)]
        step = round((sog_max*10 - sog_min*10) // 10)
        # 处理step为0的情况
        if step == 0:
            step = 1
        self.sog_list = [val*0.1 for val in range(round(sog_min*10), round(sog_max*10)+step, step)]

        # self.relbrg_list = [val*0.5 for val in range(round(relbrg_min*10)//5, round(relbrg_max*10)//5+1)]
        # self.dist_list = [val*0.5 for val in range(round(dist_min*10)//5, round(dist_max*10)//5+1)]
        # self.sog_list = [val*0.5 for val in range(round(sog_min*10)//5, round(sog_max*10)//5+1)]
        random.shuffle(self.relbrg_list)
        random.shuffle(self.dist_list)
        random.shuffle(self.sog_list)
        self.relbrg_list = self.relbrg_list[:3]
        self.dist_list = self.dist_list[:3]
        self.sog_list = self.sog_list[:3]
        # self.relbrg_list = self.relbrg_list[:min(len(self.relbrg_list)//2, 20)]
        # self.dist_list = self.dist_list[:len(self.dist_list)//2]
        # self.sog_list = self.sog_list[:len(self.sog_list)//2]

    def get(self):
        for relbrg in self.relbrg_list:
            for dist in self.dist_list:
                for sog in self.sog_list:
                    yield relbrg, dist, sog

    def get_arg(self, ot, is_first):
        for relbrg, dist, sog in self.get():
            targ = make_tship_arg_detail(self.oship, relbrg, dist, sog)
            if not targ['danger'] and ot == 2:
                continue
            if not targ['danger'] and ot == 3 and is_first == True:
                continue
            if not targ['nodanger'] and ot == 1:
                continue
            dangers = targ['danger']
            nodangers = targ['nodanger']
            delta_list = []
            # 至少一条危险目标船时,ot==3
            # 第一条目标船必定时危险,(ot==2 and idx==0)
            if ot == 1:
                for nodanger in nodangers:
                    delta_list.extend(range(nodanger[0], nodanger[1]+1))
            elif ot == 2 or (ot == 3 and is_first == True):
                for danger in dangers:
                    delta_list.extend(range(danger[0], danger[1]+1))
            else:
                for danger in dangers:
                    delta_list.extend(range(danger[0], danger[1]+1))
                for nodanger in nodangers:
                    delta_list.extend(range(nodanger[0], nodanger[1]+1))
            random.shuffle(delta_list)
            self.arg = targ
            self.delta_list = delta_list
            for delta in delta_list:
                yield targ, delta


def make_tship_detail(oship, tships_condition, ot, tt):
    mtship_list = [MakeTShip(oship, cond) for cond in tships_condition]
    mtship_dict = {i: mtship for i, mtship in enumerate(mtship_list)}
    mtship0 = mtship_dict[0]
    mtship1 = mtship_dict.get(1, None)
    mtship2 = mtship_dict.get(2, None)
    mtship3 = mtship_dict.get(3, None)
    for targ0, delta0 in mtship0.get_arg(ot, True):
        tship0 = make_tship_by_arg(targ0, delta0)
        if not mtship1:
            tships = [tship0]
            return tships
        for targ1, delta1 in mtship1.get_arg(ot, False):
            tship1 = make_tship_by_arg(targ1, delta1)
            if not mtship2:
                tships = [tship0, tship1]
                if make_tship_check_tt(tships, tt):
                    return tships
                continue
            for targ2, delta2 in mtship2.get_arg(ot, False):
                tship2 = make_tship_by_arg(targ2, delta2)
                if not mtship3:
                    tships = [tship0, tship1, tship2]
                    if make_tship_check_tt(tships, tt):
                        return tships
                    continue
                for targ3, delta3 in mtship3.get_arg(ot, False):
                    tship3 = make_tship_by_arg(targ3, delta3)
                    tships = [tship0, tship1, tship2, tship3]
                    if make_tship_check_tt(tships, tt):
                        return tships
    return []


def gen_situation2(args):
    lat = args['lat']
    lon = args['lon']
    target_num = args['target_num']
    osog_min = args['osog_min']
    osog_max = args['osog_max']
    tsog_min = args['tsog_min']
    tsog_max = args['tsog_max']
    dists = args['dist']
    rel_brgs = args['rel_brg']
    Slist = args['S']

    oship = gen_oship(lat, lon, osog_min, osog_max)
    d_tsog = (tsog_max-tsog_min)/target_num

    tships = []
    for i in range(target_num):
        dist_min = dists[i][0]
        dist_max = dists[i][1]
        rel_brg_min = rel_brgs[i][0]
        rel_brg_max = rel_brgs[i][1]
        if rel_brg_max < rel_brg_min:
            rel_brg_max += 360
        now_tsog_min = tsog_max-d_tsog*(i+1)
        now_tsog_max = tsog_max-d_tsog*i
        count = 0
        cnt = True
        S = Slist[i]
        while cnt:
            count += 1
            if count >= 10:
                # 随机次数太多次了
                # print('STEP')
                return None
            else:
                if S[0] != 4:
                    tship = gen_tship_check(
                        oship, tships, rel_brg_min, rel_brg_max,
                        dist_min, dist_max, now_tsog_min, now_tsog_max)
                else:
                    tship = gen_tship_check_no_danger(
                        oship, tships, rel_brg_min, rel_brg_max,
                        dist_min, dist_max, now_tsog_min, now_tsog_max)
            if tship:
                tships.append(tship)
                break
    print(tships)
    return tships


def gen_situation3(args):
    lat = args['lat']
    lon = args['lon']
    target_num = args['target_num']
    osog_min = args['osog_min']
    osog_max = args['osog_max']
    tsog_min = args['tsog_min']
    tsog_max = args['tsog_max']
    dists = args['dist']
    rel_brgs = args['rel_brg']
    ot = args.get('ot', 2)
    tt = args.get('tt', 4)
    # Slist = args['S']

    oship = gen_oship(lat, lon, osog_min, osog_max)
    d_tsog = (tsog_max-tsog_min)/target_num

    cond_list = []
    for i in range(target_num):
        dist_min = dists[i][0]
        dist_max = dists[i][1]
        rel_brg_min = rel_brgs[i][0]
        rel_brg_max = rel_brgs[i][1]
        if rel_brg_max < rel_brg_min:
            rel_brg_max += 360
        now_tsog_min = tsog_max-d_tsog*(i+1)
        now_tsog_max = tsog_max-d_tsog*i
        cond_list.append({
            'relbrg_min': rel_brg_min,
            'relbrg_max': rel_brg_max,
            'dist_min': dist_min,
            'dist_max': dist_max,
            'sog_min': now_tsog_min,
            'sog_max': now_tsog_max
        })
    tships = make_tship_detail(oship, cond_list, ot, tt)
    print(tships)
    return tships


def find_m(id):
    for obj in meeting_situation_table:
        if obj['id'] == id:
            return obj
    return None


def save_to_csv(filepath, scene):
    headers = [
        'VSOMT',
        'visibility',
        'stage',
        'ownship_behavior',
        'meeting_situation',
        'target_behavior',
        'ownship_lat(dmm)',
        'ownship_lon(dmm)',
        'ownship_lat(ddd)',
        'ownship_lon(ddd)',
        'target_lat(dmm)',
        'target_lon(dmm)',
        'target_lat(ddd)',
        'target_lon(ddd)',
        'ownship_sog(KN)',
        'ownship_cog(deg)',
        'target_sog(KN)',
        'target_cog(deg)',
        'distance(NM)',
        'relative_bearing(deg)',
        'DCPA(KN)',
        'TCPA(min)',
    ]

    lines = []
    for i, tgt in enumerate(scene['target']):
        field_list = []
        # VSOMT = '%d%d%d%d%d' % (
        #     scene['V'][0], scene['S'][i][0], scene['O'][i][0],
        #     scene['M'][i][0], scene['T'][i][0])
        VSOMT = ''.join(
            [str(c) for c in
             [scene['V'][0], scene['S'][i][0], scene['O'][i][0],
              scene['M'][i][0], scene['T'][i][0]]])
        field_list.append(VSOMT)
        field_list.append(str(scene['V'][1]))
        field_list.append(str(scene['S'][i][1]))
        field_list.append(str(scene['O'][i][1]))
        field_list.append(str(scene['M'][i][1]))
        field_list.append(str(scene['T'][i][1]))
        field_list.append(str(to_dmm(tgt['olat'])))
        field_list.append(str(to_dmm(tgt['olon'])))
        field_list.append(str(tgt['olat']))
        field_list.append(str(tgt['olon']))
        field_list.append(str(to_dmm(tgt['tlat'])))
        field_list.append(str(to_dmm(tgt['tlon'])))
        field_list.append(str(tgt['tlat']))
        field_list.append(str(tgt['tlon']))
        field_list.append(str(tgt['osog']))
        field_list.append(str(tgt['ocog']))
        field_list.append(str(tgt['tsog']))
        field_list.append(str(tgt['tcog']))
        field_list.append(str(tgt['dist']))

        # 提取相对方位的纯数值部分，移除括号内的注释
        rel_brg_str = str(tgt['relbrg'])
        # 使用正则表达式提取数字部分
        import re
        rel_brg_clean = re.search(r'(\d+\.?\d*)', rel_brg_str).group(1)
        field_list.append(rel_brg_clean)

        field_list.append(str(tgt['DCPA']))
        field_list.append(str(tgt['TCPA']))
        lines.append(','.join(field_list))

    # 修改：使用UTF-8编码并添加BOM头
    with open(filepath, 'w', encoding='utf-8-sig') as f:
        f.write(','.join(headers))
        f.write('\n')
        for line in lines:
            f.write(line)
            f.write('\n')


def save_to_sce(filepath, scene, envinfo):
    with open(filepath, 'w') as f:
        f.write('env,%s,%.1f,%03d,%.1f,%03d,%.1f,%03d,%d,\n' %
                (scene['name'], envinfo['wind_speed'], envinfo['wind_dir'],
                 envinfo['current_speed'], envinfo['current_dir'],
                 envinfo['wave_height'], envinfo['wave_dir'],
                 envinfo['rain_snow']))

        tgt_list = scene['target']
        tgt0 = tgt_list[0]
        f.write("AIS,111111111,-2,%.8f,%.8f,%.1f,%.1f,0,\n" %
                (tgt0['olon'], tgt0['olat'], tgt0['ocog'], tgt0['osog']))

        for i, tgt in enumerate(tgt_list):
            mmsi = 0
            for j in range(9):
                mmsi += pow(10.0, j) * (i + 2)
            f.write("AIS,%d,-1,%.8f,%.8f,%.1f,%.1f,0,\n" %
                    (mmsi, tgt['tlon'], tgt['tlat'], tgt['tcog'], tgt['tsog']))


def save_to_xml(filepath, scene):
    doc = xml.dom.minidom.Document()
    root = doc.createElement('autocollison')

    def create_text(pnode, name, value):
        node = doc.createElement(name)
        node.appendChild(doc.createTextNode(str(value)))
        pnode.appendChild(node)

    def fmt2d(d):
        return '%.2f' % d

    for i, tgt in enumerate(scene['target']):
        tgtnode = doc.createElement('target')
        # VSOMT = '%d%d%d%d%d' % (
        #     scene['V'][0], scene['S'][i][0], scene['O'][i][0],
        #     scene['M'][i][0], scene['T'][i][0])
        VSOMT = ''.join(
            [str(c) for c in
             [scene['V'][0], scene['S'][i][0], scene['O'][i][0],
              scene['M'][i][0], scene['T'][i][0]]])
        create_text(tgtnode, 'VSOMT', VSOMT)
        create_text(tgtnode, 'visibility', scene['V'][1])
        create_text(tgtnode, 'stage', scene['S'][i][1])
        create_text(tgtnode, 'ownship_behavior', scene['O'][i][1])
        create_text(tgtnode, 'meeting_situation', scene['M'][i][1])
        create_text(tgtnode, 'target_behavior', scene['T'][i][1])

        create_text(tgtnode, 'ownship_lat', tgt['olat'])
        create_text(tgtnode, 'ownship_lon', tgt['olon'])
        create_text(tgtnode, 'ownship_lat_raw', to_dmm(tgt['olat']))
        create_text(tgtnode, 'ownship_lon_raw', to_dmm(tgt['olon']))
        create_text(tgtnode, 'target_lat', tgt['tlat'])
        create_text(tgtnode, 'target_lon', tgt['tlon'])
        create_text(tgtnode, 'target_lat_raw', to_dmm(tgt['tlat']))
        create_text(tgtnode, 'target_lon_raw', to_dmm(tgt['tlon']))

        create_text(tgtnode, 'ownship_sog', fmt2d(tgt['osog']))
        create_text(tgtnode, 'ownship_cog', fmt2d(tgt['ocog']))
        create_text(tgtnode, 'target_sog', fmt2d(tgt['tsog']))
        create_text(tgtnode, 'target_cog', fmt2d(tgt['tcog']))

        create_text(tgtnode, 'distance', fmt2d(tgt['dist']))
        create_text(tgtnode, 'relative_bearing', fmt2d(tgt['relbrg']))

        create_text(tgtnode, 'DCPA', fmt2d(tgt['DCPA']))
        create_text(tgtnode, 'TCPA', fmt2d(tgt['TCPA']))
        root.appendChild(tgtnode)

    doc.appendChild(root)

    with open(filepath, 'w', encoding='utf8') as f:
        doc.writexml(f, addindent='\t', newl='\n', encoding='utf8')

def save_to_nto(filepath, scene):
    nto = {}
    nto['general'] = {'area': 'Open Sea'}

    ship_name = 'Bulk carrier 2 (Dis.76800t) bl.'
    tgt_list = scene['target']
    tgt0 = tgt_list[0]
    obj_list = []
    own_obj = {
        'key': ship_name,
        'name': 'OS 1',
        'pos': {
            'lat': tgt0['olat'],
            'lon': tgt0['olon'],
            'heading': tgt0['ocog']
        },
        'initial_speed': {
            'longitudinal': tgt0['osog'] * 1852 / 3600,
            'lateral': 0.0
        },
        'mode': 'own'
    }
    obj_list.append(own_obj)

    for i, tgt in enumerate(tgt_list):
        tgt_obj = {
            'key': ship_name,
            'name': 'Tgt %d' % (i+1),
            'pos': {
                'lat': tgt['tlat'],
                'lon': tgt['tlon'],
                'heading': tgt['tcog']
            },
            'initial_speed': {
                'longitudinal': tgt['tsog'] * 1852 / 3600,
                'lateral': 0.0
            },
            'mode': '3dof'
        }
        obj_list.append(tgt_obj)

    nto['objects'] = obj_list
    json.dump(nto, open(filepath, 'w', encoding='utf8'), indent='\t')


def deg_to_screen(v):
    return mod360(v+270)


def to_dmm(ddd):
    d = int(ddd)
    m = (ddd-d)*60.0
    return '%d°%.3f′' % (d, m)


def init_VSOMT_table(tb, headers, datas):
    tb.clear()
    tb.setColumnCount(len(headers))
    tb.setHorizontalHeaderLabels(headers)
    tb.setRowCount(len(datas))

    tb.setSelectionMode(QAbstractItemView.SingleSelection)
    tb.setSelectionBehavior(QAbstractItemView.SelectRows)

    for i, d in enumerate(datas):
        item = QTableWidgetItem(d[1])
        item.setCheckState(Qt.Unchecked)
        item.setData(Qt.UserRole, d)
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        tb.setItem(i, 0, item)

    def on_cell_changed(row, col):
        item0 = tb.item(row, col)
        if item0.checkState() != Qt.Checked:
            return

        rowCount = tb.rowCount()
        for i in range(rowCount):
            if i == row:
                continue
            item = tb.item(i, 0)
            item.setCheckState(False)

    tb.cellChanged.connect(on_cell_changed)


class SceneDialog(QDialog):

    def __init__(self, parent=None):
        super().__init__()
        loadUi("./data/ui/scenedlg.ui", self)
        self.pushButtonOK.clicked.connect(self.on_ok)
        self.pushButtonCancel.clicked.connect(self.on_cancel)
        self.init_table_V()
        self.init_speed_combox()

    def init_speed_combox(self):
        comboxO = self.comboBoxOSog
        comboxO.addItem("快速选择")

        comboxT = self.comboBoxTSog
        comboxT.addItem("快速选择")

        for s in speed_list:
            comboxO.addItem(s[1], s)
            comboxT.addItem(s[1], s)

        comboxO.currentIndexChanged.connect(self.on_combox_index_changed)
        comboxT.currentIndexChanged.connect(self.on_combox_index_changed)

    def on_combox_index_changed(self):
        combox = self.sender()
        idx = combox.currentIndex()
        data = combox.itemData(idx)
        if not data:
            return
        sogmin = data[2]
        sogmax = data[3]
        if combox == self.comboBoxOSog:
            self.lineEditOSogMin.setText(str(sogmin))
            self.lineEditOSogMax.setText(str(sogmax))
        else:
            self.lineEditTSogMin.setText(str(sogmin))
            self.lineEditTSogMax.setText(str(sogmax))

    def init_table_V(self):
        headers = [
            '能见度'
        ]

        init_VSOMT_table(self.tableWidgetV, headers, visibility_list)
        self.tableWidgetV.setColumnWidth(0, 100)

    def get_selected_V(self):
        tb = self.tableWidgetV
        row = tb.rowCount()
        for i in range(row):
            item = tb.item(i, 0)
            if item.checkState() == Qt.Checked:
                return {
                    'V': item.data(Qt.UserRole)
                }
        return None

    def on_ok(self):
        if not self.get_selected_V():
            QMessageBox.information(self, '提示', '请选择V')
            return
        self.accept()

    def on_cancel(self):
        self.reject()


class TargetDialog(QDialog):

    def __init__(self, scene, parent=None):
        super().__init__()
        loadUi("./data/ui/targetdlg.ui", self)
        self.pushButtonOK.clicked.connect(self.on_ok)
        self.pushButtonCancel.clicked.connect(self.on_cancel)
        self.scene = scene
        self.init_table_S()
        self.init_table_O()
        self.init_table_M()
        self.init_table_T()
        self.low_visibility = False
        if self.scene['V'][1] == '不良':
            self.low_visibility = True
        if self.low_visibility:
            self.tableWidgetO.setEnabled(False)
        else:
            self.set_M_items_enable([0, 1, 2, 3, 4, 5, 6, 7, 8], False)
            self.tableWidgetO.cellChanged.connect(self.on_O_cell_changed)

    def get_Mcodes_by_S(self, S=None):
        if not S:
            return [m[0] for m in meeting_situation_list], []
        includes = []
        excludes = []
        for m in meeting_situation_list:
            if m[4] == S[0]:
                includes.append(m[0])
            else:
                excludes.append(m[0])
        return includes, excludes

    def clear_M_checked(self):
        tb = self.tableWidgetM
        rowCount = tb.rowCount()
        for i in range(rowCount):
            item = tb.item(i, 0)
            item.setCheckState(False)

    def set_M_items_enable(self, Mcodes, enable):
        tb = self.tableWidgetM
        rows = tb.rowCount()
        for i in range(rows):
            item = tb.item(i, 0)
            data = item.data(Qt.UserRole)
            if data[0] in Mcodes:
                if enable:
                    item.setFlags(item.flags() | Qt.ItemIsEnabled)
                else:
                    item.setFlags(item.flags() & ~Qt.ItemIsEnabled)

    def on_O_cell_changed(self, row, col):
        self.clear_M_checked()
        includes, excludes = self.get_Mcodes_by_S()
        self.set_M_items_enable(includes, False)
        tb = self.tableWidgetO
        item = tb.item(row, col)
        data = item.data(Qt.UserRole)
        # 0: 直航
        # 1: 让路
        if item.checkState() == Qt.Checked:
            includes, excludes = self.get_Mcodes_by_S(data)
            self.set_M_items_enable(includes, True)
            self.set_M_items_enable(excludes, False)

    def init_table_S(self):
        headers = [
            '碰撞局面'
        ]

        init_VSOMT_table(self.tableWidgetS, headers, stage_list)
        self.tableWidgetS.setColumnWidth(0, 100)

    def init_table_O(self):
        headers = [
            '本船行为'
        ]

        init_VSOMT_table(self.tableWidgetO, headers, ownship_behavior_list)
        self.tableWidgetO.setColumnWidth(0, 180)

    def init_table_M(self):
        headers = [
            '船舶态势'
        ]

        init_VSOMT_table(self.tableWidgetM, headers, meeting_situation_list)
        self.tableWidgetM.setColumnWidth(0, 180)

    def init_table_T(self):
        headers = [
            '目标船行为'
        ]

        init_VSOMT_table(self.tableWidgetT, headers, target_behavior_list)
        self.tableWidgetT.setColumnWidth(0, 180);

    def get_selected_SOMT(self):

        def get_data(tb):
            row = tb.rowCount()
            for i in range(row):
                item = tb.item(i, 0)
                if item.checkState() == Qt.Checked:
                    return item.data(Qt.UserRole)

        S = get_data(self.tableWidgetS)
        O = get_data(self.tableWidgetO)
        M = get_data(self.tableWidgetM)
        T = get_data(self.tableWidgetT)

        if not S:
            return None

        # 4是没有危险
        # 有危险的情况哟啊判断M和T
        # 没有危险则不用判断
        if S[0] != 4:
            if not M or not T:
                return None
        # if not S or not M or not T:
        #     return None

            if not self.low_visibility:
                # 能见度良的时候必须选择本船行为
                if not O:
                    return
            else:
                O = (0, '---')
        else:
            O = (0, '---')
            M = (0, '---')
            T = (0, '---')

        return {
            'S': S,
            'O': O,
            'M': M,
            'T': T,
        }

    def on_ok(self):
        if not self.get_selected_SOMT():
            QMessageBox.information(self, '提示', '请选择S,O,M,T')
            return
        self.accept()

    def on_cancel(self):
        self.reject()


class ViewDialog(QDialog):

    MAX_RANGE = 14
    W = 1200  # 将宽度扩大为原来的两倍
    H = 1200  # 将高度扩大为原来的两倍

    def __init__(self, data, parent=None):
        super().__init__(parent)
        self.data = data
        self.resize(ViewDialog.W, ViewDialog.H)
        self.setWindowTitle('预览:' + data['name'])

    def dist_to_pix(self, dist):
        w = self.width()
        h = self.height()
        return round(dist/ViewDialog.MAX_RANGE*min(w/2, h/2))


    def drawCircle(self, painter, cx, cy, r):
        painter.drawEllipse(int(cx-r), int(cy-r), int(r*2), int(r*2))


    def paintEvent(self, event):
        w = self.width()
        h = self.height()

        painter = QPainter(self)

        painter.setRenderHint(QPainter.Antialiasing)

        pen = painter.pen()
        pen.setColor(QColor(0, 0, 0))
        painter.setPen(pen)
        # 自定义的绘画方法
        # self.drawText(event, painter)
        self.drawCoord(painter, w, h)
        self.drawRange(painter, w, h)

        # 添加安全检查
        if 'sog' not in self.data or 'cog' not in self.data:
            return
        
        if 'target' not in self.data or not self.data['target']:
            return

        osog = self.data['sog']
        ocog = self.data['cog']

        # 先绘制本船（红色）
        pen.setWidth(2)
        pen.setColor(QColor(0, 0, 139))  # 深蓝色
        painter.setPen(pen)
        self.drawOwnShip(painter, w/2, h/2, ocog, osog)

        # 再绘制所有目标船（绿色并带编号）
        pen.setWidth(2)
        pen.setColor(QColor(0, 128, 0))  # 绿色
        painter.setPen(pen)
        
        for i, t in enumerate(self.data['target']):
            if all(key in t for key in ['dist', 'brg', 'tcog', 'tsog']):
                self.drawTargetWithNumber(painter, w, h, t['dist'], t['brg'], t['tcog'], t['tsog'], i+1)

        # 绘制避碰参数
        self.drawCollisionAvoidanceParams(painter, w, h)

        # 绘制相对运动矢量
        # 获取最大的TCPA值用于预测轨迹绘制
        TCPA_list = [abs(t['TCPA']) for t in self.data['target'] if 'TCPA' in t]
        
        # 确保TCPA_list非空且包含有效值
        if TCPA_list:
            TCPA = max(TCPA_list)
            pen.setWidth(2)
            pen.setColor(QColor(255, 0, 0))  # 红色
            painter.setPen(pen)

            for t in self.data['target']:
                if all(key in t for key in ['dist', 'brg', 'tcog', 'tsog', 'rel_cog', 'rel_spd', 'TCPA']):
                    self.drawTarget(painter, w, h, t['dist'], t['brg'], t['tcog'],
                                    t['tsog'], t['rel_cog'], t['rel_spd'], t['TCPA'])

    def drawCollisionAvoidanceParams(self, painter, w, h):
        """在右上角绘制避碰参数，每个目标船的参数分行显示"""
        # 保存当前画笔状态
        old_penalty = painter.pen()
        old_font = painter.font()
        
        # 设置文本颜色和字体
        painter.setPen(QColor(0, 0, 0))
        font = QFont()
        font.setPointSize(9)  # 使用合适的字体大小
        painter.setFont(font)
        
        # 在右上角绘制参数
        margin = 10  # 边距
        line_height = 30  # 每行高度，增加到1.5倍行距
        params_per_target = 10  # 每个目标船需要显示的参数行数（标题+9个参数）
        
        # 计算标题和数据所需的总高度
        num_targets = len(self.data.get('target', []))  # 使用get方法确保安全
        title_height = 25  # 标题占用的高度
        total_height = title_height + num_targets * params_per_target * line_height
        
        # 绘制背景矩形
        bg_margin = 8
        bg_width = 250  # 适合分行显示的宽度
        bg_height = total_height + 2 * bg_margin
        
        # 确保背景矩形不会超出显示区域
        actual_bg_height = min(bg_height, h - 2 * margin)
        actual_bg_y = margin
        
        bg_rect = QRect(w - bg_width - margin, actual_bg_y, bg_width, actual_bg_height)
        
        # 设置背景色（浅色半透明）
        bg_color = QColor(245, 245, 245, 220)  # 浅灰色半透明
        painter.fillRect(bg_rect, bg_color)
        
        # 绘制边框
        border_penalty = QPen()
        border_penalty.setColor(QColor(200, 200, 200))
        painter.setPen(border_penalty)
        painter.drawRect(bg_rect)
        
        # 恢复文字绘制用的画笔
        painter.setPen(QColor(0, 0, 0))
        
        # 绘制标题
        title = "避碰参数:"
        title_rect = QRect(w - bg_width - margin + 5, actual_bg_y + 5, bg_width - 10, title_height)
        painter.drawText(title_rect, Qt.AlignLeft, title)
        
        # 绘制每个目标船的参数，分行显示
        current_y = actual_bg_y + title_height + 5
        for i, t in enumerate(self.data.get('target', [])):
            # 获取所有需要的参数，使用get方法确保即使参数缺失也能显示默认值
            tcpa = t.get('TCPA', 0.0)
            dcpa = t.get('DCPA', 0.0)
            tsog = t.get('tsog', 0.0)  # 目标船船速
            tcog = t.get('tcog', 0.0)  # 目标船航向
            rel_spd = t.get('rel_spd', 0.0)  # 相对船速
            rel_cog = t.get('rel_cog', 0.0)  # 相对航向
            dist = t.get('dist', 0.0)  # 距离
            rel_brg = t.get('relbrg', 0.0)  # 相对方位（修改点）

            # 检查是否有足够的空间绘制当前目标船的所有参数
            if current_y + params_per_target * line_height > actual_bg_y + actual_bg_height - 5:
                # 如果空间不够，绘制省略号
                if i < len(self.data.get('target', [])) - 1:  # 不是最后一个
                    omitted_text = f"... 还有 {len(self.data.get('target', [])) - i} 个目标船"
                    omitted_rect = QRect(w - bg_width - margin + 5, current_y, bg_width - 10, line_height - 5)
                    painter.drawText(omitted_rect, Qt.AlignLeft, omitted_text)
                break
            
            # 绘制目标船编号
            target_title = f"目标船{i+1}:"
            title_rect = QRect(w - bg_width - margin + 5, current_y, bg_width - 10, line_height - 5)
            painter.drawText(title_rect, Qt.AlignLeft, target_title)
            current_y += line_height
            
            # 分行绘制所有参数
            params = [
                f"  会遇类型: {self.get_meeting_type_by_deg_range(rel_brg, self.data.get('sog', 0), tsog)}",
                f"  DCPA: {dcpa:.2f} nm",
                f"  TCPA: {tcpa:.2f} min",
                f"  航向Ct: {tcog:.2f}°",
                f"  船速Vt: {tsog:.2f} kn",
                f"  相对航向: {rel_cog:.2f}°",
                f"  相对船速: {rel_spd:.2f} kn",
                f"  距离: {dist:.2f} nm",
                f"  相对方位Bt: {rel_brg:.2f}°"
            ]
            
            for param in params:
                text_rect = QRect(w - bg_width - margin + 5, current_y, bg_width - 10, line_height - 5)
                painter.drawText(text_rect, Qt.AlignLeft, param)
                current_y += line_height
        
        # 恢复原始画笔和字体
        painter.setPen(old_penalty)
        painter.setFont(old_font)

    def get_meeting_type_by_deg_range(self, rel_brg, own_sog=0, target_sog=0):
        """根据目标船相对于本船的方位判断会遇类型"""
        # 计算本船相对于目标船的方位（Bo）
        bo = (rel_brg + 180) % 360
        
        # 正后追越：目标船在本船正后方追越
        if 175 <= rel_brg <= 185 and target_sog > own_sog:
            return "正后追越"
        
        # 判断追越局面（目标船追越本船）
        if target_sog > own_sog and 112.5 <= rel_brg <= 247.5:
            if rel_brg < 180:
                return "右舷追越"
            else:
                return "左舷追越"
        
        # 判断被追越局面（目标船被本船追越）
        elif own_sog > target_sog and 112.5 <= bo <= 247.5:
            if bo < 180:
                return "右舷被追越"
            elif 180 < bo < 247.5:
                return "左舷被追越"
            else:
                return "正后被追越"
        
        # 其他会遇类型判断
        elif (356 <= rel_brg or rel_brg <= 6):
            return "对遇"
        elif (6 < rel_brg <= 67.5):
            return "右舷小角度交叉会遇"
        elif (67.5 < rel_brg <= 112.5):
            return "右舷大角度交叉会遇"
        elif (247.5 < rel_brg < 292.5):
            return "左舷小角度交叉会遇"
        elif (292.5 <= rel_brg < 354):
            return "左舷大角度交叉会遇"
        else:
            return "未知"

    def drawOwnShip(self, qp, x, y, cog, sog):
        """绘制本船（红色三角形）"""
        xy_list = self.get_ship_xy(x, y, cog)

        qp.drawLine(xy_list[0][0], xy_list[0][1], xy_list[1][0], xy_list[1][1])
        qp.drawLine(xy_list[1][0], xy_list[1][1], xy_list[2][0], xy_list[2][1])
        qp.drawLine(xy_list[2][0], xy_list[2][1], xy_list[0][0], xy_list[0][1])

        self.drawCircle(qp, int(x), int(y), 3)

        # 绘制本船真矢量
        # 矢量长度与航速成正比，这里设定比例为每节航速5像素
        vector_length = sog * 5
        theta = math.radians(deg_to_screen(cog))
        vector_end_x = int(x + vector_length * math.cos(theta))
        vector_end_y = int(y + vector_length * math.sin(theta))
        
        # 保存原始画笔
        original_pen = qp.pen()
        
        # 创建新画笔用于绘制真矢量
        vector_pen = QPen()
        vector_pen.setWidth(2)
        vector_pen.setColor(QColor(0, 0, 139))  # 深蓝色表示本船
        vector_pen.setStyle(Qt.SolidLine)
        qp.setPen(vector_pen)
        
        # 绘制真矢量箭头（从船的位置指向前进方向）
        qp.drawLine(int(x), int(y), vector_end_x, vector_end_y)
        
        # 绘制箭头头部
        arrow_size = 8
        # 计算箭头两个翼的点
        angle = math.atan2(vector_end_y - y, vector_end_x - x)
        arrow_angle = math.pi / 6  # 30度的箭头夹角
        
        # 左侧翼
        left_x = vector_end_x - arrow_size * math.cos(angle - arrow_angle)
        left_y = vector_end_y - arrow_size * math.sin(angle - arrow_angle)
        qp.drawLine(vector_end_x, vector_end_y, int(left_x), int(left_y))
        
        # 右侧翼
        right_x = vector_end_x - arrow_size * math.cos(angle + arrow_angle)
        right_y = vector_end_y - arrow_size * math.sin(angle + arrow_angle)
        qp.drawLine(vector_end_x, vector_end_y, int(right_x), int(right_y))
        
        # 恢复原始画笔
        qp.setPen(original_pen)

    def drawTarget(self, qp, w, h, dist, brg, cog, sog, rel_cog, rel_spd, TCPA):
        # 极坐标处理
        cx = int(w/2)
        cy = int(h/2)
        # r = round(dist/ViewDialog.MAX_RANGE*min(cx, cy))
        r = self.dist_to_pix(dist)
        theta = math.radians(deg_to_screen(brg))
        xt = int(cx + r*math.cos(theta))
        yt = int(cy + r*math.sin(theta))
        # self.drawShip(qp, xt, yt, cog,8, TCPA)

        x = xt
        y = yt
        xy_list = self.get_ship_xy(x, y, cog)

        qp.drawLine(xy_list[0][0], xy_list[0][1], xy_list[1][0], xy_list[1][1])
        qp.drawLine(xy_list[1][0], xy_list[1][1], xy_list[2][0], xy_list[2][1])
        qp.drawLine(xy_list[2][0], xy_list[2][1], xy_list[0][0], xy_list[0][1])

        self.drawCircle(qp, x, y, 3)

        dist1 = sog * abs(TCPA) / 60
        theta1 = math.radians(deg_to_screen(cog))
        r1 = self.dist_to_pix(dist1) + 10
        x1 = int(x + r1*math.cos(theta1))
        y1 = int(y + r1*math.sin(theta1))
        qp.drawLine(x, y, x1, y1)

        oldpen = qp.pen()
        pen = QPen()
        pen.setWidth(3)
        pen.setColor(QColor(0, 0, 255))  # 蓝色
        pen.setStyle(Qt.DashLine)
        qp.setPen(pen)

        dist2 = rel_spd * abs(TCPA) / 60
        theta2 = math.radians(deg_to_screen(rel_cog))
        r2 = self.dist_to_pix(dist2) + 30
        x2 = int(x + r2*math.cos(theta2))
        y2 = int(y + r2*math.sin(theta2))
        qp.drawLine(x, y, x2, y2)

        qp.setPen(oldpen)

    def drawTargetWithNumber(self, qp, w, h, dist, brg, cog, sog, number):
        """绘制目标船（绿色三角形）并标注编号，同时绘制相对运动方位线"""
        # 极坐标处理
        cx = int(w/2)
        cy = int(h/2)
        # r = round(dist/ViewDialog.MAX_RANGE*min(cx, cy))
        r = self.dist_to_pix(dist)
        theta = math.radians(deg_to_screen(brg))
        xt = int(cx + r*math.cos(theta))
        yt = int(cy + r*math.sin(theta))

        x = xt
        y = yt
        xy_list = self.get_ship_xy(x, y, cog)

        qp.drawLine(xy_list[0][0], xy_list[0][1], xy_list[1][0], xy_list[1][1])
        qp.drawLine(xy_list[1][0], xy_list[1][1], xy_list[2][0], xy_list[2][1])
        qp.drawLine(xy_list[2][0], xy_list[2][1], xy_list[0][0], xy_list[0][1])

        self.drawCircle(qp, x, y, 3)

        # 绘制目标船编号
        old_pen = qp.pen()
        old_brush = qp.brush()
        
        # 设置文本颜色和字体
        qp.setPen(QColor(0, 0, 0))
        font = qp.font()
        font.setPointSize(10)
        qp.setFont(font)
        
        # 在目标船旁边绘制编号
        text_rect = QRect(x + 5, y - 15, 20, 20)
        qp.drawText(text_rect, Qt.AlignLeft, str(number))
        
        # 恢复原来的画笔
        qp.setPen(old_pen)
        qp.setFont(font)
        
        # 绘制相对运动方位线（从本船中心到目标船）
        oldpen = qp.pen()
        pen = QPen()
        pen.setWidth(3)
        pen.setColor(QColor(255, 165, 0))  # 橙色
        pen.setStyle(Qt.DashLine)
        qp.setPen(pen)
        
        # 从屏幕中心（本船位置）到目标船位置画线
        qp.drawLine(int(w/2), int(h/2), x, y)
        
        # 恢复原始画笔
        qp.setPen(old_pen)
        
        # 绘制目标船真矢量
        # 矢量长度与航速成正比，这里设定比例为每节航速5像素
        vector_length = sog * 5
        theta = math.radians(deg_to_screen(cog))
        vector_end_x = int(x + vector_length * math.cos(theta))
        vector_end_y = int(y + vector_length * math.sin(theta))
        
        # 创建新画笔用于绘制真矢量
        vector_pen = QPen()
        vector_pen.setWidth(2)
        vector_pen.setColor(QColor(0, 128, 0))  # 绿色表示目标船
        vector_pen.setStyle(Qt.SolidLine)
        qp.setPen(vector_pen)
        
        # 绘制真矢量箭头（从船的位置指向前进方向）
        qp.drawLine(int(x), int(y), vector_end_x, vector_end_y)
        
        # 绘制箭头头部
        arrow_size = 8
        # 计算箭头两个翼的点
        angle = math.atan2(vector_end_y - y, vector_end_x - x)
        arrow_angle = math.pi / 6  # 30度的箭头夹角
        
        # 左侧翼
        left_x = vector_end_x - arrow_size * math.cos(angle - arrow_angle)
        left_y = vector_end_y - arrow_size * math.sin(angle - arrow_angle)
        qp.drawLine(vector_end_x, vector_end_y, int(left_x), int(left_y))
        
        # 右侧翼
        right_x = vector_end_x - arrow_size * math.cos(angle + arrow_angle)
        right_y = vector_end_y - arrow_size * math.sin(angle + arrow_angle)
        qp.drawLine(vector_end_x, vector_end_y, int(right_x), int(right_y))
        
        # 恢复原始画笔
        qp.setPen(old_pen)

    def get_ship_xy(self, x, y, cog):
        tri_xy = [
            (0, -20),
            (-8, 8),
            (8, 8),
        ]
        theta = math.radians(cog)

        xy_list = []
        for xy in tri_xy:
            xy_list.append((
                int(xy[0]*math.cos(theta)-xy[1]*math.sin(theta) + x),
                int(xy[0]*math.sin(theta)+xy[1]*math.cos(theta) + y),
            ))
        return xy_list

    def drawCoord(self, qp, w, h):
        cx = int(w/2)
        cy = int(h/2)
        qp.drawLine(0, cy, int(w), cy)
        qp.drawLine(cx, 0, cx, int(h))

    def drawRange(self, qp, w, h):
        # 2海里一个圈
        # 半径14海里
        cx = int(w/2)
        cy = int(h/2)
        radius = min(int(w/2), int(h/2))
        n = round(ViewDialog.MAX_RANGE/2)
        step = radius / n
        for i in range(n):
            r = int(step * (i+1))
            self.drawCircle(qp, cx, cy, r)

    def drawText(self, event, qp):
        # 设置笔的颜色
        qp.setPen(QColor(3, 20, 160))
        # 设置字体
        qp.setFont(QFont('SimSun', 20))
        # 画出文本
        qp.drawText(event.rect(), Qt.AlignCenter, self.text)

class MainWindow(QMainWindow):

    UNNAME_SCENE_ID = 1

    def __init__(self):
        super().__init__()
        loadUi("./data/ui/main.ui", self)
        self.pushButtonExportXML.clicked.connect(self.on_export_xml)
        self.pushButtonExportCSV.clicked.connect(self.on_export_csv)
        self.pushButtonExportSCE.clicked.connect(self.on_export_sce)
        self.pushButtonExportNTO.clicked.connect(self.on_export_nto)
        self.pushButtonViewScene.clicked.connect(self.on_view)
        self.pushButtonCreateScene.clicked.connect(self.on_create)
        self.pushButtonGenScene.clicked.connect(self.on_gen)
        self.pushButtonAddTarget.clicked.connect(self.on_add_target)
        self.tableWidgetScene.cellChanged.connect(self.on_cell_changed)

        self.comboBoxOT.addItem('全部没有碰撞危险')
        self.comboBoxOT.addItem('全部有碰撞危险')
        self.comboBoxOT.addItem('至少和一条目标船有碰撞危险')
        self.comboBoxOT.addItem('随机')

        self.comboBoxTT.addItem('全部没有碰撞危险')
        self.comboBoxTT.addItem('全部有碰撞危险')
        self.comboBoxTT.addItem('至少和一条目标船有碰撞危险')
        self.comboBoxTT.addItem('随机')

        self.comboBoxOT.setCurrentIndex(1)
        self.comboBoxTT.setCurrentIndex(3)

        self.scenes = []
        # test
        test_scene = {
            'name': 'unname_001',
            'target_num': 2,
            'ok': True,
            'lat': 31.003333333333334,
            'lon': 123.0,
            'osog_min': 10.0,
            'osog_max': 20.0,
            'tsog_min': 10.0,
            'tsog_max': 20.0,
            'V': (2, '良好'),
            'S': [
                (1, '碰撞危险', 8, 14),
                (1, '碰撞危险', 8, 14)
            ],
            'O': [
                (1, '直航船'),
                (1, '直航船')
            ],
            'M': [
                (1, '对遇'),
                (2, '右舷小角度交叉会遇')
            ],
            'T': [
                (1, '正常避让，实施舵点坐标()'),
                (1, '正常避让，实施舵点坐标()')
            ],
            'dist': [(8, 14), (8, 14)],
            'rel_brg': [
                (354, 6),
                (6, 67.5)
            ],
            'sog': 14.5,
            'cog': 342.8,
            'target': [
                {
                    'olat': 31.003333333333334,
                    'olon': 123.0,
                    'tlat': 31.19217711282364,
                    'tlon': 122.92788555576831,
                    'osog': 14.5,
                    'ocog': 342.8,
                    'tsog': 15.200000000000001,
                    'tcog': 160.9000000000001,
                    'dist': 11.9,
                    'brg': 341.82760737667263,
                    'relbrg': 358.1,
                    'TCPA': 24.097673200693205,
                    'DCPA': 0.0038718716217898187
                },
                {
                    'olat': 31.003333333333334,
                    'olon': 123.0,
                    'tlat': 31.220242296080226,
                    'tlon': 122.96643551704175,
                    'osog': 14.5,
                    'ocog': 342.8,
                    'tsog': 12.4,
                    'tcog': 183.70000000000005,
                    'dist': 13.100000000000001,
                    'brg': 352.42508988892934,
                    'relbrg': 20.900000000000002,
                    'TCPA': 29.775687981286303,
                    'DCPA': 0.0019842597930527866
                }
            ]
        }
        # self.scenes.append(test_scene)
        self.init_table_scene()


    def init_table_scene(self):
        tb = self.tableWidgetScene

        selected = self.get_selected_scene()

        headers = [
            '场景名称',
            '能见度(V)',
            '目标船数量',
            '是否已生成',
            '本船航速(节)',
            '本船航向(deg)',
            '本船纬度(度分)',
            '本船经度(度分)',
        ]
        tb.clear()
        tb.setColumnCount(len(headers))
        tb.setSelectionMode(QAbstractItemView.SingleSelection)
        tb.setSelectionBehavior(QAbstractItemView.SelectRows)
        tb.setHorizontalHeaderLabels(headers)
        tb.setRowCount(len(self.scenes))
        for i, scene in enumerate(self.scenes):
            item = QTableWidgetItem(scene['name'])
            item.setCheckState(Qt.Unchecked)
            tb.setItem(i, 0, item)

            tb.setItem(i, 1, QTableWidgetItem(scene['V'][1]))
            tb.setItem(i, 2, QTableWidgetItem(str(scene['target_num'])))
            s = '是' if scene['ok'] else '否'
            tb.setItem(i, 3, QTableWidgetItem(s))
            s = '%.2f' % scene['sog'] if 'sog' in scene else '--.--'
            tb.setItem(i, 4, QTableWidgetItem(s))
            s = '%.2f' % scene['cog'] if 'cog' in scene else '--.--'
            tb.setItem(i, 5, QTableWidgetItem(s))
            tb.setItem(i, 6, QTableWidgetItem(to_dmm(scene['lat'])))
            tb.setItem(i, 7, QTableWidgetItem(to_dmm(scene['lon'])))

        rowCount = tb.rowCount()
        colCount = tb.columnCount()
        for i in range(rowCount):
            for j in range(colCount):
                item = tb.item(i, j)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)

        if selected:
            idx = self.scenes.index(selected)
            item = tb.item(idx, 0)
            item.setCheckState(Qt.Checked)

    def on_cell_changed(self, row, col):
        tb = self.tableWidgetScene
        item0 = tb.item(row, col)
        if item0.checkState() != Qt.Checked:
            return

        rowCount = tb.rowCount()
        for i in range(rowCount):
            if i == row:
                continue
            item = tb.item(i, 0)
            item.setCheckState(False)

    def get_selected_scene(self):
        tb = self.tableWidgetScene
        row = tb.rowCount()
        for i in range(row):
            item = tb.item(i, 0)
            if item.checkState() == Qt.Checked:
                return self.scenes[i]
        return None

    def on_gen(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if scene['target_num'] == 0:
            r = QMessageBox.warning(self, '提示', '请先添加目标船')
            return

        if scene['ok']:
            r = QMessageBox.question(
                self, '提示', '该场景已生成,是否重新生成?')
            if r != QMessageBox.Yes:
                return

        count = 0
        scene['ot'] = self.comboBoxOT.currentIndex() + 1
        scene['tt'] = self.comboBoxTT.currentIndex() + 1
        print(scene)
        tgt_list = gen_situation3(scene)
        if not tgt_list:
            scene['ok'] = False
            QMessageBox.information(self, '提示', '生成失败')
            return

        # while True:
        #     if count > 10:
        #         scene['ok'] = False
        #         QMessageBox.information(self, '提示', '生成失败')
        #         return

        #     tgt_list = gen_situation3(scene)
        #     count = count + 1
        #     if tgt_list:
        #         break

        scene['sog'] = tgt_list[0]['osog']
        scene['cog'] = tgt_list[0]['ocog']
        scene['target'] = tgt_list
        scene['ok'] = True
        QMessageBox.information(self, '提示', '生成成功')
        self.init_table_scene()

    def on_add_target(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if scene['target_num'] >= 4:
            QMessageBox.warning(self, '提示', '目标船数量已达到4!')
            return

        dlg = TargetDialog(scene, self)
        rcode = dlg.exec()
        if rcode != QDialog.Accepted:
            return

        scene['target_num'] += 1
        SOMT = dlg.get_selected_SOMT()
        S = SOMT['S']
        O = SOMT['O']
        M = SOMT['M']
        T = SOMT['T']
        scene['S'].append(S)
        scene['O'].append(O)
        scene['M'].append(M)
        scene['T'].append(T)

        if S[0] != 4:
            if S[2] > M[3][0]:
                scene['dist'].append(M[3])
            else:
                scene['dist'].append((S[2], S[3]))
            scene['rel_brg'].append(M[2])
        else:
            scene['dist'].append((S[2], S[3]))
            # 没有危险时,rel_brg随机
            # scene['rel_brg'].append((-359.9, -0.1))
            scene['rel_brg'].append((0, 359.9))

        self.init_table_scene()


    def on_create(self):
        dlg = SceneDialog(self)
        dlg.lineEditName.setText('unname_%03d' % self.UNNAME_SCENE_ID)

        rcode = dlg.exec()
        if rcode != QDialog.Accepted:
            return

        self.UNNAME_SCENE_ID += 1

        name = dlg.lineEditName.text()
        latDeg = int(dlg.lineEditOLatDeg.text())
        lonDeg = int(dlg.lineEditOLonDeg.text())
        latMin = float(dlg.lineEditOLatMin.text())
        lonMin = float(dlg.lineEditOLonMin.text())

        lat = latDeg + latMin / 60.0
        lon = lonDeg + lonMin / 60.0

        osog_min = float(dlg.lineEditOSogMin.text())
        osog_max = float(dlg.lineEditOSogMax.text())

        tsog_min = float(dlg.lineEditTSogMin.text())
        tsog_max = float(dlg.lineEditTSogMax.text())

        V = dlg.get_selected_V()['V']

        scene = {
            'name': name,
            'target_num': 0,
            'ok': False,
            'lat': lat,
            'lon': lon,
            'osog_min': osog_min,
            'osog_max': osog_max,
            'tsog_min': tsog_min,
            'tsog_max': tsog_max,
            'V': V,
            'S': [],
            'O': [],
            'M': [],
            'T': [],
            'dist': [],
            'rel_brg': [],
        }

        self.scenes.append(scene)
        self.init_table_scene()

    def on_export_csv(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if scene['target_num'] == 0:
            QMessageBox.warning(self, '提示', '请先添加目标船')
            return

        if not scene['ok']:
            QMessageBox.warning(self, '提示', '该场景还未生成')
            return

        filename = QFileDialog.getSaveFileName(
            self, '输入文件名', '.', 'CSV File(*.csv)')

        print(scene)
        save_to_csv(filename[0], scene)


    def on_export_xml(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if scene['target_num'] == 0:
            QMessageBox.warning(self, '提示', '请先添加目标船')
            return

        if not scene['ok']:
            QMessageBox.warning(self, '提示', '该场景还未生成')
            return

        filename = QFileDialog.getSaveFileName(
            self, '输入文件名', '.', 'XML File(*.xml)')

        save_to_xml(filename[0], scene)

    def on_export_sce(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if scene['target_num'] == 0:
            QMessageBox.warning(self, '提示', '请先添加目标船')
            return

        if not scene['ok']:
            QMessageBox.warning(self, '提示', '该场景还未生成')
            return

        dlg = SCEEnvInfoDlg(self)

        rcode = dlg.exec()
        if rcode != QDialog.Accepted:
            return

        filename = QFileDialog.getSaveFileName(
            self, '输入文件名', '.', 'SCE File(*.sce)')
        save_to_sce(filename[0], scene, dlg.info)

    def on_export_nto(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if scene['target_num'] == 0:
            QMessageBox.warning(self, '提示', '请先添加目标船')
            return

        if not scene['ok']:
            QMessageBox.warning(self, '提示', '该场景还未生成')
            return

        filename = QFileDialog.getSaveFileName(
            self, '输入文件名', '.', 'NTO File(*.nto)')
        save_to_nto(filename[0], scene)

    def on_view(self):
        scene = self.get_selected_scene()
        if not scene:
            QMessageBox.warning(self, '提示', '请先选择一种场景')
            return

        if not scene['ok']:
            QMessageBox.warning(self, '提示', '该场景还未生成')
            return

        view = ViewDialog(scene, self)
        view.show()


def unhandler_hook(t, val, tb):
    logging.warning(traceback.print_exception(t, val, tb))


def read_csv(filename):
    title = []
    datas = []
    try:
        f = open(filename, 'r')
        text = f.read()
        f.close()
        lines = text.split('\n')
        title = lines[0].split(',')
        lines = lines[1:]
        for line in lines:
            if line:
                datas.append(line.split(','))
    except:
        pass

    return title, datas


def init_csv_data():
    global visibility_list
    global stage_list
    global ownship_behavior_list
    global meeting_situation_list
    global target_behavior_list
    global speed_list

    filename = './data/visibility.csv'
    title, datas = read_csv(filename)
    if datas:
        visibility_list = [(int(d[0]), d[1]) for d in datas]

    filename = './data/stage.csv'
    title, datas = read_csv(filename)
    if datas:
        stage_list = [(int(d[0]), d[1], float(d[2]), float(d[3]))
                      for d in datas]

    filename = './data/ownship_behavior.csv'
    title, datas = read_csv(filename)
    if datas:
        ownship_behavior_list = [(int(d[0]), d[1]) for d in datas]

    filename = './data/meeting_situation.csv'
    title, datas = read_csv(filename)
    if datas:
        meeting_situation_list = [(int(d[0]), d[1],
                                   (float(d[2]), float(d[3])),
                                   (float(d[4]), float(d[5])), int(d[6]))
                                  for d in datas]

    filename = './data/target_behavior.csv'
    title, datas = read_csv(filename)
    if datas:
        target_behavior_list = [(int(d[0]), d[1])for d in datas]

    filename = './data/speed.csv'
    title, datas = read_csv(filename)
    if datas:
        speed_list = [(int(d[0]), d[1], float(d[2]), float(d[3])) for d in datas]


class SCEEnvInfoDlg(QDialog):

    def __init__(self, parent=None):
        super().__init__()
        loadUi("./data/ui/sceenvinfodlg.ui", self)
        self.pushButtonOK.clicked.connect(self.on_ok)
        self.pushButtonCancel.clicked.connect(self.on_cancel)

    def on_ok(self):
        wind_speed = self.doubleSpinBoxWindSpeed.value()
        wind_dir = self.doubleSpinBoxWindDir.value()
        current_speed = self.doubleSpinBoxCurrentSpeed.value()
        current_dir = self.doubleSpinBoxCurrentDir.value()
        wave_dir = self.doubleSpinBoxWaveDir.value()
        wave_height = self.doubleSpinBoxWaveHeight.value()
        rain_snow = self.spinBoxRainSnow.value()
        self.info = {
            'wind_speed': wind_speed,
            'wind_dir': wind_dir,
            'current_speed': current_speed,
            'current_dir': current_dir,
            'wave_dir': wave_dir,
            'wave_height': wave_height,
            'rain_snow': rain_snow
        }
        self.accept()

    def on_cancel(self):
        self.reject()


t, d = calc_CPA(30.89484334577408, 122.90946683827808, 30.960132311570135, 122.94083760311402, 18, 10.5, 38.2, 199.5)


if __name__ == "__main__":
    from logging.handlers import RotatingFileHandler

    Rthandler = RotatingFileHandler(
        'myapp.log', maxBytes=10*1024*1024, backupCount=5)
    Rthandler.setLevel(logging.WARNING)
    formatter = logging.Formatter(
        '%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s')
    Rthandler.setFormatter(formatter)
    logging.getLogger('').addHandler(Rthandler)

    init_csv_data()

    sys.excepthook = unhandler_hook
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    app.exec()

    sys.exit(0)