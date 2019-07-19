import math


def log(*args):
    print('[chaeyk-log]', *args)


debug_enabled = False


def debug(*args):
    if debug_enabled:
        print('[chaeyk-debug]', *args)


simulator = False


def is_simulator():
    return simulator


def set_simulator():
    global simulator, debug_enabled
    simulator = True
    debug_enabled = True


# simulator를 위해 필요하다
speeds = []


def set_speeds(l):
    global speeds
    speeds = l
    debug('speeds', speeds)


rebuilt_waypoints = None
optimal_waypoints = None
waypoint_hash = 0


def fix_params_error(params):
    global rebuilt_waypoints, optimal_waypoints, reverse_printed, waypoint_hash

    # Read input variables
    waypoints = params['waypoints'].copy()
    is_reversed = params['is_reversed']
    track_width = params['track_width']

    hash = waypoints[0][0] + waypoints[1][0] + waypoints[2][0]
    if hash != waypoint_hash:
        fixed_waypoints = fix_waypoints(waypoints, is_reversed)
        rebuilt_waypoints = build_waypoints(fixed_waypoints, track_width)
        optimal_waypoints = build_optimal_waypoints(rebuilt_waypoints, track_width)
        waypoint_hash = hash

    params['waypoints'] = rebuilt_waypoints
    params['x_optimal_waypoints'] = optimal_waypoints

    # AWS document 에는 closest_waypoints가 순서 상관없이 가장 가까운 2개라고 되어 있다.
    # waypoints의 간격은 일정하지 않으므로, 이러면 뒤쪽의 2개나 앞쪽의 2개가 될 수도 있다.
    # 정말로 그런지 확인은 못해봤는데 어쨌든 찜찜하니 다시 계산해서 세팅한다.
    pt = (params['x'], params['y'])
    closest_waypoints = find_closest_waypoint_index(pt, rebuilt_waypoints)
    old_closest_waypoints = params['closest_waypoints']
    if closest_waypoints[0] != old_closest_waypoints[0] or closest_waypoints[1] != old_closest_waypoints[1]:
        log('closest_waypoints is different with mine: {} != {} (pt={})'
            .format(old_closest_waypoints, closest_waypoints, pt))
    params['closest_waypoints'] = closest_waypoints

    distance_from_center = get_distance_to_waypoint(
        pt, rebuilt_waypoints, closest_waypoints)
    is_left_of_center = (distance_from_center <= 0)
    distance_from_center = abs(distance_from_center)

    params['distance_from_center'] = distance_from_center
    params['is_left_of_center'] = is_left_of_center

    optimal_index = find_closest_waypoint_index(pt, optimal_waypoints)
    params['x_optimal_index'] = optimal_index


reverse_printed = False


def fix_waypoints(waypoints, is_reversed):
    global reverse_printed
    if is_reversed:
        if not reverse_printed:
            reverse_printed = True
            log('REVERSED')
        # waypoints 순서를 뒤집어야 한다.
        if waypoints[0][0] < waypoints[1][0]:
            log('reversing waypoints')
            waypoints.reverse()

        # reverse이면 waypoint 25, 26의 순서가 바뀌어있다.
        if waypoints[25][0] > waypoints[26][0]:
            log('swapping waypoints 25, 26:', waypoints[25], waypoints[26])
            (waypoints[25], waypoints[26]) = (waypoints[26], waypoints[25])
    else:
        # waypoint 44, 45의 순서가 바뀌어있다.
        if waypoints[44][0] < waypoints[45][0]:
            log('swapping waypoints 44, 45:', waypoints[44], waypoints[45])
            (waypoints[44], waypoints[45]) = (waypoints[45], waypoints[44])

    return waypoints


def reward_function(params):
    fix_params_error(params)

    p_reward = position_reward(params)
    s_reward = speed_reward(params)
    d_reward = direction_reward(params)
    
    progress = params['progress']
    steps = params['steps']

    if steps == 0:
        pace = 0.5
    else:
        pace = progress * 1.5 / steps

    reward = 10 * p_reward * s_reward * d_reward * pace
    return reward


# 시뮬레이터가 주는 waypoints는 간격도 제멋대로 엉망이라
# 여기서 다시 촘촘하게 일정한 간격으로 만든다.
def build_waypoints(waypoints, track_width):
    global debug_enabled

    # 주석 풀려있으면 waypoint 다시 만들지 않음
    return waypoints

    new_waypoints = []

    start_wp = waypoints[0]
    distance = 0
    before_wp = start_wp
    delta = 0.1

    _de = debug_enabled
    debug_enabled = False

    i = 0
    while distance < 1 or get_distance(before_wp, start_wp) >= delta:
        current_wp = get_point_on_waypoint(waypoints, 0, distance, start_wp)

        wp_dist = get_distance(current_wp, before_wp)
        if i > 0 and (wp_dist < delta * 0.9 or wp_dist > delta * 1.1):
            log('too close or far waypoints:', before_wp, current_wp, "i=", i, "distance=", distance, "dist=", wp_dist)

        new_waypoints.append(current_wp)

        before_wp = current_wp
        distance += delta
        i += 1

    new_waypoints.append(new_waypoints[0])

    debug_enabled = _de
    return new_waypoints

def build_optimal_waypoints(waypoints, track_width):
    global debug_enabled

    # 주석 풀려있으면 waypoint 다시 만들지 않음
    #return waypoints

    optimal_waypoints = []

    start_wp = waypoints[0]
    distance = 0
    before_wp = start_wp
    delta = 0.3

    _de = debug_enabled
    debug_enabled = False

    i = 0
    while distance < 1 or get_distance(before_wp, start_wp) >= delta:
        current_wp = get_point_on_waypoint(waypoints, 0, distance, start_wp)

        wp_dist = get_distance(current_wp, before_wp)
        if i > 0 and (wp_dist < delta * 0.9 or wp_dist > delta * 1.1):
            log('too close or far waypoints:', before_wp, current_wp, "i=", i, "distance=", distance, "dist=", wp_dist)

        # optimal_wp = current_wp
        optimal_wp = get_optimal_position(waypoints, track_width, current_wp)
        optimal_waypoints.append(optimal_wp)

        if len(optimal_waypoints) >= 2:
            owp_dist = get_distance(optimal_waypoints[-2], optimal_waypoints[-1])
            if owp_dist < delta * 0.5 or owp_dist > delta * 1.5:
                log('too close or far optimal waypoints:',
                    optimal_waypoints[-2], optimal_waypoints[-1], "i=", i, "distance=", distance, "dist=", owp_dist)

        before_wp = current_wp
        distance += delta
        i += 1

    optimal_waypoints.append(optimal_waypoints[0])

    debug_enabled = _de
    return optimal_waypoints

# 현재 내 위치가 pt일 때, 여기에 대응하는 레코드 라인의 위치를 찾는다
def get_optimal_position(waypoints, track_width, pt):
    closest_waypoints = find_closest_waypoint_index(pt, waypoints)

    # 이전부터 앞으로 track 방향이 어떻게 되는지 살펴본다.
    angle_b2 = get_waypoint_direction(waypoints, closest_waypoints, pt, -1.4)
    angle_b1 = get_waypoint_direction(waypoints, closest_waypoints, pt, -0.7)
    angle_0 = get_waypoint_direction(waypoints, closest_waypoints, pt)
    angle_f1 = get_waypoint_direction(waypoints, closest_waypoints, pt, 0.7)
    angle_f2 = get_waypoint_direction(waypoints, closest_waypoints, pt, 1.4)

    # debug('angles', angle_b2, angle_b1, angle_0, angle_f1, angle_f2)

    # 이전 위치와의 각도 차이를 구함
    # 왼쪽으로 구부러지면 마이너스, 반대쪽은 플러스
    angle_b2_b1 = get_angle_diff(angle_b2, angle_b1)
    angle_b1_0 = get_angle_diff(angle_b1, angle_0)
    angle_0_f1 = get_angle_diff(angle_0, angle_f1)
    angle_f1_f2 = get_angle_diff(angle_f1, angle_f2)

    # debug("angle diffs: %.2f %.2f 0 %.2f %.2f"%(angle_b2_b1, angle_b1_0, angle_0_f1, angle_f1_f2))

    # 트랙의 방향을 보고 현재 내가 있어야 할 위치(중심으로부터의 거리)를  찾는다.
    max_angle = 25
    rr1 = 0.4
    rr2 = 1
    rr3 = 1.4
    rr4 = 0.6
    r1 = convert_range(-max_angle, max_angle, 1, -1, angle_b2_b1) * rr1
    r2 = convert_range(-max_angle, max_angle, -1, 1, angle_b1_0) * rr2
    r3 = convert_range(-max_angle, max_angle, -1, 1, angle_0_f1) * rr3
    r4 = convert_range(-max_angle, max_angle, 1, -1, angle_f1_f2) * rr4
    # 중심으로부터 떨어진 거리. -는 왼쪽, +는 오른쪽.
    r = (r1 + r2 + r3 + r4) / (rr1 + rr2 + rr3 + rr4) * track_width * 0.4

    # debug("r: %.2f <= r1: %.2f r2: %.2f r3: %.2f r4: %.2f"%(r, r1, r2, r3, r4))

    # optimal position
    center = get_closest_point_from_line(pt, waypoints[closest_waypoints[0]], waypoints[closest_waypoints[1]])
    # debug('center:', center, 'closest:', closest_waypoints)
    op = rotate_point(center, (center[0], center[1] - r), angle_0)

    return op

def position_reward(params):
    pt = (params['x'], params['y'])
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    optimal_waypoints = params['x_optimal_waypoints']
    optimal_index = params['x_optimal_index']
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']

    marker3 = track_width * 0.48
    if distance_from_center > marker3:
        return 1e-3

    prev_owp = optimal_waypoints[optimal_index[0]]
    next_owp = optimal_waypoints[optimal_index[1]]
    owp = get_closest_point_from_line(pt, prev_owp, next_owp)

    dist_center_owp = get_distance_to_waypoint(owp, waypoints, closest_waypoints)
    left_margin = max(0, track_width / 2 + dist_center_owp)
    right_margin = max(0, track_width / 2 - dist_center_owp)
    if left_margin == 0 or right_margin == 0:
        log('too small margin. pt:', pt, 'left_margin:', left_margin, 'right_margin:', right_margin)

    dist_owp_pt = get_distance_to_waypoint(pt, optimal_waypoints, optimal_index)

    if dist_owp_pt <= 0: # pt가 optimal의 왼쪽에 있다
        reward = convert_range(0, left_margin * left_margin, 1, 0.1, dist_owp_pt * dist_owp_pt)
    else:
        reward = convert_range(0, right_margin * right_margin, 1, 0.1, dist_owp_pt * dist_owp_pt)
    
    if reward <= 0.1:
        reward = 0.001
    
    return reward

def fill_speeds(params):
    global speeds

    speed = params['speed']
    if speed == 0:
        log('ZEROSPEED')
        return 0

    if not speed in speeds:
        speeds.append(speed)
        speeds.sort()

    if len(speeds) < 3:
        debug('MAXSPEED is', speeds[-1])
    
    return speed

def speed_reward(params):
    global speeds

    speed = fill_speeds(params)

    if len(speeds) == 0:
        return 0.001

    if is_close_enough(speeds[0], speed):
        return 1
    else:
        return 0.001

def direction_reward(params):
    return 1

############################################################################
# DMath

# angle1, angle2의 각도 차이를 리턴한다.
# 리턴값은 -180 < r <= 180 사이에 있다.
def get_angle_diff(angle1, angle2):
    return get_180_angle(angle1 - angle2)

def get_mid_angle(angle1, angle2):
    return angle1 + get_angle_diff(angle2, angle1) / 2

# angle을 0~359.999... 의 각도로 변환한다
def get_360_angle(angle):
    while angle >= 360:
        angle -= 360
    while angle < 0:
        angle += 360
    return angle

# angle을 -179.999... ~ 180 의 각도로 변환한다
def get_180_angle(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle

# pt를 center를 기준으로 angle도만큼 반시계 방향으로 회전시킨다
def rotate_point(center, pt, angle):
    radians = math.radians(angle)
    sin = math.sin(radians)
    cos = math.cos(radians)

    # Translate point back to origin
    cpt = (pt[0] - center[0], pt[1] - center[1])

    # Rotate point
    xnew = cpt[0] * cos - cpt[1] * sin
    ynew = cpt[0] * sin + cpt[1] * cos

    # Translate point back
    return (xnew + center[0], ynew + center[1])

# lpt1에서 lpt2방향으로 distance만큼 이동한 후의 위치를 구한다.
def get_point_of_distance(lpt1, lpt2, distance):
    angle = get_angle(lpt1, lpt2)
    dpt = (lpt1[0] + distance, lpt1[1])
    return rotate_point(lpt1, dpt, angle)

def get_middle(pt1, pt2):
    return ((pt1[0] + pt2[0]) / 2.0, (pt1[1] + pt2[1]) / 2.0)

# waypoints에서 pt에 가장 가까운 2개의 점을 리턴한다.
# 보통은 pt의 앞뒤로 하나씩 나오니 (n, n+1) 이 리턴된다.
# 그런데 위치가 애매한 경우 (n, n) 이 나올 수도 있다.
def find_closest_waypoint_index(pt, waypoints):
    closestIndex = (-1, -1)
    centeredIndex = (-1, -1)

    oldDist = 10000.0
    oldCenterDist = 10000.0

    for i in range(0, len(waypoints) - 1):
        prev_wp = waypoints[i]
        next_wp = waypoints[i + 1]

        angle = get_angle(prev_wp, next_wp)

        rotated_next_wp = rotate_point(prev_wp, next_wp, -angle)
        rotated_pt = rotate_point(prev_wp, pt, -angle)
        if (rotated_pt[0] <= rotated_next_wp[0]):
            cross = get_closest_point_from_line(pt, prev_wp, next_wp)
            middle = get_middle(prev_wp, next_wp)
            newDist = min(get_distance(prev_wp, pt), get_distance(next_wp, pt))

            if closestIndex[1] < 0 or newDist < oldDist:
                oldDist = newDist
                if prev_wp[0] <= rotated_pt[0]:
                    closestIndex = (i, i + 1)
                else:
                    closestIndex = (i, i)

            newCenterDist = get_distance(cross, middle)
            if prev_wp[0] <= rotated_pt[0] and newCenterDist < oldCenterDist and newDist < 0.7:
                centeredIndex = (i, i + 1)
                oldCenterDist = newCenterDist

    if closestIndex[0] == -1:
        raise Exception('cannot find closest waypoint of {}'.format(pt))
    
    if centeredIndex[0] >= 0:
        return centeredIndex
    else:
        return closestIndex

# pt에서 index에 해당하는 waypoint 라인까지의 거리
# pt가 waypoint 진행방향의 왼쪽에 있으면 마이너스 값을 리턴한다.
# index = (prev, next)
def get_distance_to_waypoint(pt, waypoints, index):
    if index[0] == index[1]:
        wp = waypoints[index[0]]
        distance = get_distance(pt, wp)
        trackDirection = get_waypoint_direction(waypoints, index, pt)
        rotated_pt = (pt[0] - wp[0], pt[1] - wp[1])
        rotated_pt = rotate_point((0, 0), rotated_pt, -trackDirection)
        isLeft = rotated_pt[1] >= 0
        return -distance if isLeft else distance
    else:
        return get_distance_to_line(pt, waypoints[index[0]], waypoints[index[1]])

def next_waypoint_index(waypoints, wpindex):
    wpindex += 1
    if len(waypoints) == wpindex:
        wpindex = 1
    return wpindex

def next_waypoint(waypoints, wpindex):
    return waypoints[next_waypoint_index(waypoints, wpindex)]

def prev_waypoint_index(waypoints, wpindex):
    wpindex -= 1
    if wpindex < 0:
        wpindex = len(waypoints) - 2
    return wpindex

def prev_waypoint(waypoints, wpindex):
    return waypoints[prev_waypoint_index(waypoints, wpindex)]

# waypoints 상에서 pt에서 가장 가까운 부분의 방향.
# index는 pt에서 가장 가까운 앞뒤 waypoint
# distance가 0이 아니면 앞/뒤로 distance 이동한 다음에 방향을 구한다.
def get_waypoint_direction(waypoints, index, pt, distance = 0):
    if index[0] == index[1]:
        cross = waypoints[index[0]]
    else:
        cross = get_closest_point_from_line(pt, waypoints[index[0]], waypoints[index[1]])

    if distance != 0:
        cross = get_point_on_waypoint(waypoints, index[1 if distance > 0 else 0], distance, cross)
        index = find_closest_waypoint_index(cross, waypoints)

    pt_prev = get_point_on_waypoint(waypoints, index[0], -0.15, cross)
    pt_next = get_point_on_waypoint(waypoints, index[1], 0.15, cross)

    return get_angle(pt_prev, pt_next)

# waypoints 상의 pt에서 시작해 앞/뒤로 distance 떨어진 곳의 점을 구한다.
# distance > 0 이면 앞으로, 아니면 뒤쪽 방향으로 이동.
# pt는 반드시 waypoint 를 잇는 라인 위에 있어야 한다.
def get_point_on_waypoint(waypoints, index, distance, pt):
    if distance == 0:
        return pt

    absDistance = abs(distance)

    distanceSum = 0
    while True:
        currentDistance = get_distance(pt, waypoints[index])
        if distanceSum + currentDistance >= absDistance:
            break
        
        distanceSum += currentDistance
        pt = waypoints[index]

        if distance > 0:
            index = next_waypoint_index(waypoints, index)
        else:
            index = prev_waypoint_index(waypoints, index)

    # log('get_point_on_waypoint', pt, waypoints[index], absDistance - distanceSum)
    return get_point_of_distance(pt, waypoints[index], absDistance - distanceSum)

# DMath
############################################################################

############################################################################
# RPoint

# pt1, pt2 사이의 거리
def get_distance(pt1, pt2):
    return math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))

# lpt1 - lpt2 를 지나는 선과 pt 와의 거리
# pt가 lpt1-lpt2의 왼쪽이면 마이너스, 오른쪽이면 플러스 값이 리턴된다.
def get_distance_to_line(pt, lpt1, lpt2):
    if lpt1[0] == lpt2[0] and lpt1[1] == lpt2[1]:
        return get_distance(pt, lpt1)
        
    px = lpt2[0] - lpt1[0]
    py = lpt2[1] - lpt1[1]
    dab = px * px + py * py
    return (py * pt[0] - px * pt[1] + lpt2[0] * lpt1[1] - lpt2[1] * lpt1[0]) / math.sqrt(dab)

# pt1, pt2 점 2개를 잇는 선과 x축 사이의 각도를 리턴한다.
# 리턴값은 -180 < r <= 180 사이에 있다.
def get_angle(pt1, pt2):
    track_direction = math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0]) 
    track_direction = math.degrees(track_direction)
    return get_180_angle(track_direction)

# lpt1 - lpt2 를 지나는 선에서 pt로부터 제일 가까운 점을 찾는다
def get_closest_point_from_line(pt, lpt1, lpt2):
    if lpt1[0] == lpt2[0] and lpt1[1] == lpt2[1]:
        return lpt1

    px = lpt2[0] - lpt1[0]
    py = lpt2[1] - lpt1[1]
    dab = px * px + py * py
    u = ((pt[0] - lpt1[0]) * px + (pt[1] - lpt1[1]) * py) / dab
    return (lpt1[0] + u * px, lpt1[1] + u * py)

# RPoint
############################################################################

# s1 ~ e1 에 해당하는 range를 s2 ~ e2 가 되도록 하는 변환에 v를 대입한다
def convert_range(s1, e1, s2, e2, v):
    v = s1 if v < s1 else v
    v = e1 if v > e1 else v
    d1 = e1 - s1
    d2 = e2 - s2
    if d1 == 0:
        return (s2 + e2) / 2.0
    else:
        return (v - s1) * d2 / float(d1) + s2

def get_closest_from_list(values, target):
    min_distance = 100000000
    closest_value = 0
    for v in values:
        distance = abs(v - target)
        if (distance <= min_distance):
            min_distance = distance
            closest_value = v

    return closest_value

# v1과 v2가 충분히 가까우면 true 리턴
# 나눗셈 등으로 발생한 실수 값들은 오차가 있을 수 있으므로 이걸 사용해 비교한다.
def is_close_enough(v1, v2):
    return abs(v1 - v2) < 0.1
