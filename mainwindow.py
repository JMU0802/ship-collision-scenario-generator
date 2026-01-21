def get_encounter_type(self, relbrg):
    """
    根据相对方位Bt判断会遇类型
    """
    # 将角度归一化到[0, 360)范围
    relbrg = self.mod360(relbrg)
    
    # 正后追越：目标船在本船正后方且满足追越条件
    if 175 <= relbrg <= 185 and self.target_speed > self.own_speed:
        return "正后追越"
    
    # 正后被追越：目标船在本船正前方且满足被追越条件
    elif 355 <= relbrg < 360 or 0 <= relbrg <= 5 and self.target_speed < self.own_speed:
        return "正后被追越"
    
    # 左舷追越：目标船在本船左舷且满足追越条件
    elif 112.5 <= relbrg <= 247.5 and self.target_speed > self.own_speed:
        return "左舷追越"
    
    # 右舷追越：目标船在本船右舷且满足追越条件
    elif 112.5 <= relbrg <= 247.5 and self.target_speed < self.own_speed:
        return "右舷追越"
    
    # 对遇：目标船在本船正前方或正后方
    elif 356 <= relbrg <= 360 or 0 <= relbrg <= 6:
        return "对遇"
    
    # 右舷小角度交叉
    elif 6 < relbrg <= 67.5:
        return "右舷小角度交叉"
    
    # 右舷大角度交叉
    elif 67.5 < relbrg <= 112.5:
        return "右舷大角度交叉"
    
    # 左舷小角度交叉
    elif 247.5 < relbrg <= 292.5:
        return "左舷小角度交叉"
    
    # 左舷大角度交叉
    elif 292.5 < relbrg <= 354:
        return "左舷大角度交叉"
    
    # 默认情况
    return "未知"