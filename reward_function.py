import math

def log(*args):
	print('[chaeyk]', *args)

def debug(*args):
	if is_simulator():
		print('[chaeyk]', *args)

simulator = False
def is_simulator():
	return simulator

def set_simulator():
	global simulator
	simulator = True

# simulator를 위해 필요하다
speeds = []
def set_speeds(l):
	global speeds
	speeds = l
	debug('speeds', speeds)

def reward_function(params):
	fix_waypoints_error(params)

	p_reward = position_reward(params)
	s_reward = speed_reward(params)
	d_reward = direction_reward(params)

	#return min(p_reward, s_reward, d_reward)
	return max(0.001, p_reward * s_reward * d_reward)

reverse_printed = False

def fix_waypoints_error(params):
	global reverse_printed

	# Read input variables
	waypoints = params['waypoints'].copy()
	params['waypoints'] = waypoints
	is_reversed = params['is_reversed']

	if is_reversed:
		if not reverse_printed:
			reverse_printed = True
			log('REVERSED')
		# waypoints 순서를 뒤집어야 한다.
		if waypoints[0][0] < waypoints[1][0]:
			waypoints.reverse()
		
		# reverse이면 waypoint 25, 26의 순서가 바뀌어있다.
		if waypoints[25][0] > waypoints[26][0]:
			(waypoints[25], waypoints[26]) = (waypoints[26], waypoints[25])
	else:
		# waypoint 44, 45의 순서가 바뀌어있다.
		if waypoints[44][0] < waypoints[45][0]:
			(waypoints[44], waypoints[45]) = (waypoints[45], waypoints[44])

	# AWS document 에는 closest_waypoints가 순서 상관없이 가장 가까운 2개라고 되어 있다.
	# waypoints의 간격은 일정하지 않으므로, 이러면 뒤쪽의 2개나 앞쪽의 2개가 될 수도 있다.
	# 정말로 그런지 확인은 못해봤는데 어쨌든 찜찜하니 다시 계산해서 세팅한다.
	pt = (params['x'], params['y'])
	closest_waypoints = get_closest_waypoints(waypoints, pt)
	old_closest_waypoints = params['closest_waypoints']
	if closest_waypoints[0] != old_closest_waypoints[0] or closest_waypoints[1] != old_closest_waypoints[1]:
		log('closest_waypoints is different with mime: {} != {} (pt={})'
			.format(old_closest_waypoints, closest_waypoints, pt))
	params['closest_waypoints'] = closest_waypoints

	distance_from_center = get_line_distance(pt, waypoints[closest_waypoints[0]], waypoints[closest_waypoints[1]])
	is_left_of_center = (distance_from_center <= 0)
	distance_from_center = abs(distance_from_center)

	params['distance_from_center'] = distance_from_center
	params['is_left_of_center'] = is_left_of_center

def get_closest_waypoints(waypoints, pt):
	index = -1
	for i in range(0, len(waypoints) - 1):
		prev_wp = waypoints[i]
		next_wp = waypoints[i + 1]

		angle = get_angle(prev_wp, next_wp)

		rotated_next_wp = rotate_point(prev_wp, next_wp, -angle)
		rotated_pt = rotate_point(prev_wp, pt, -angle)
		if (rotated_pt[0] <= rotated_next_wp[0]):
			if index < 0 or get_distance(pt, next_wp) < get_distance(pt, waypoints[index + 1]):
				index = i

	if index == -1:
		raise Exception('cannot find closest waypoint of {}'.format(pt))
	
	return (index, index + 1)

def build_optimal_waypoints(waypoints, track_width):
	optimal_waypoints = []
	for wp in waypoints:
		optimal_waypoints.append(get_optimal_position(waypoints, track_width, wp))
	return optimal_waypoints

# 현재 내 위치가 pt일 때, 여기에 대응하는 레코드 라인의 위치를 찾는다
def get_optimal_position(waypoints, track_width, pt):
	closest_waypoints = get_closest_waypoints(waypoints, pt)

	# 이전부터 앞으로 track 방향이 어떻게 되는지 살펴본다.
	angle_b2 = get_track_direction(pt, waypoints, closest_waypoints, 1.4, False)
	angle_b1 = get_track_direction(pt, waypoints, closest_waypoints, 0.7, False)
	angle_0 = get_track_direction(pt, waypoints, closest_waypoints, 0, True)
	angle_f1 = get_track_direction(pt, waypoints, closest_waypoints, 0.7, True)
	angle_f2 = get_track_direction(pt, waypoints, closest_waypoints, 1.4, True)

	# 이전 위치와의 각도 차이를 구함
	# 왼쪽으로 구부러지면 마이너스, 반대쪽은 플러스
	angle_b2_b1 = get_angle_diff(angle_b2, angle_b1)
	angle_b1_0 = get_angle_diff(angle_b1, angle_0)
	angle_0_f1 = get_angle_diff(angle_0, angle_f1)
	angle_f1_f2 = get_angle_diff(angle_f1, angle_f2)

	#debug("angle diffs: %.2f %.2f 0 %.2f %.2f"%(angle_b2_b1, angle_b1_0, angle_0_f1, angle_f1_f2))

    # 트랙의 방향을 보고 현재 내가 있어야 할 위치(중심으로부터의 거리)를  찾는다.
	max_angle = 25
	r1 = convert_range(-max_angle, max_angle, 1, -1, angle_b2_b1) * 0.8
	r2 = convert_range(-max_angle, max_angle, -1, 1, angle_b1_0)
	r3 = convert_range(-max_angle, max_angle, -1, 1, angle_0_f1)
	r4 = convert_range(-max_angle, max_angle, 1, -1, angle_f1_f2) * 0.8

	#debug("r1: %.2f r2: %.2f r3: %.2f r4: %.2f"%(r1, r2, r3, r4))

	# 중심으로부터 떨어진 거리. -는 왼쪽, +는 오른쪽.
	r = (r1 + r2 + r3 + r4) / 3.6 * track_width * 0.4

	# optimal position
	center = get_closest_point_from_line(pt, waypoints[closest_waypoints[0]], waypoints[closest_waypoints[1]])
	op = rotate_point(center, (center[0], center[1] - r), angle_0)

	return op

def position_reward(params):
	pt = (params['x'], params['y'])
	waypoints = params['waypoints']
	closest_waypoints = params['closest_waypoints']
	track_width = params['track_width']
	distance_from_center = params['distance_from_center']
	is_left_of_center = params['is_left_of_center']

	marker3 = track_width * 0.48
	if distance_from_center > marker3:
		return 1e-3

	op = get_optimal_position(waypoints, track_width, pt)

	# 중심으로부터 떨어진 거리. -는 왼쪽, +는 오른쪽. r은 원하는 위치, p는 현재 위치
	r = get_line_distance(op, waypoints[closest_waypoints[0]], waypoints[closest_waypoints[1]])
	p = -distance_from_center if is_left_of_center else distance_from_center
	d = abs(p - r)

	debug("r: %.2f p: %.2f d: %.2f"%(r, p, d))

	marker1 = track_width * 0.08
	marker2 = track_width * 0.25

	if d < marker1:
		reward = 1.0
	elif d < marker2:
		reward = convert_range(marker1, marker2, 1.0, 0.3, d)
	else:
		reward = convert_range(marker2, marker3, 0.3, 0.05, d)
	
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
		log('MAXSPEED is', speeds[-1])
	
	return speed

def speed_reward(params):
	global speeds

	speed = fill_speeds(params)
	if len(speeds) <= 0 or speed == 0:
	    return 0.001

	max_speed = speeds[-1]
	min_speed = speeds[0]
	
	pt = (params['x'], params['y'])
	waypoints = params['waypoints']
	closest_waypoints = params['closest_waypoints']
	track_width = params['track_width']
	heading = params['heading']

	# pt가 가는 방향의 연장선
	pt2 = (math.cos(math.radians(heading)) + pt[0], math.sin(math.radians(heading)) + pt[1])

	# 현재 위치에서 직진할 때 어느 waypoint부터 트랙을 벗어나는지 조사한다.
	max_joint_to_pt = 1.8 # 너무 멀리 볼 필요는 없어서...
	min_joint_to_pt = 0.2
	wpindex = closest_waypoints[1]
	joint_to_waypoint = 0
	joint_to_pt = 0
	while joint_to_waypoint < track_width / 2 and joint_to_pt < max_joint_to_pt:
		wp = waypoints[wpindex]

		joint = get_closest_point_from_line(wp, pt, pt2)
		joint_to_waypoint = get_distance(wp, joint)
		joint_to_pt = get_distance(pt, joint)

		wpindex = next_waypoint_index(waypoints, wpindex)

	if joint_to_pt < min_joint_to_pt:
		target_speed = min_speed
	else:
		target_speed = convert_range(math.sqrt(min_joint_to_pt), math.sqrt(max_joint_to_pt), min_speed, max_speed, math.sqrt(joint_to_pt))

	closest_speed = target_speed
	#closest_speed = get_closest_from_list(speeds, target_speed)

	debug('joint_to_pt: %.2f target_speed: %.2f closest_speed: %.2f'%(joint_to_pt, target_speed, closest_speed))

	if is_close_enough(speed, closest_speed):
		reward = 1.0
	elif speed < closest_speed:
		reward = max(0.001, convert_range(0, closest_speed, -0.3, 1, speed))
	else:
		reward = convert_range(closest_speed, closest_speed * 2.2, 1, 0, speed)

	return max(0.001, reward)
def direction_reward(params):
	pt = (params['x'], params['y'])
	waypoints = params['waypoints']
	closest_waypoints = params['closest_waypoints']
	heading = params['heading']

	angle_b1 = get_track_direction(pt, waypoints, closest_waypoints, 0.7, False)
	angle_0 = get_track_direction(pt, waypoints, closest_waypoints, 0, True)
	angle_f1 = get_track_direction(pt, waypoints, closest_waypoints, 0.7, True)

	angle_b1_0 = get_angle_diff(angle_b1, angle_0)
	angle_0_f1 = get_angle_diff(angle_0, angle_f1)

	angle_heading_0 = get_angle_diff(heading, angle_0)

	straight = abs(angle_b1_0) < 5 and abs(angle_0_f1) < 5

	#debug('pt:', pt, closest_waypoints)
	#debug('straight: %.2f, angle_heading_0: %.2f'%(straight, angle_heading_0))

	if straight:
		reward = convert_range(5, 25, 1, 0.001, abs(angle_heading_0))
	else:
		reward = convert_range(20, 40, 1, 0.001, abs(angle_heading_0))

	return reward

# pt1, pt2 점 2개를 잇는 선과 x축 사이의 각도를 리턴한다.
# 리턴값은 -180 < r <= 180 사이에 있다.
def get_angle(pt1, pt2):
	track_direction = math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0]) 
	track_direction = math.degrees(track_direction)
	return get_angle_diff(track_direction, 0)


# angle1, angle2의 각도 차이를 리턴한다.
# 리턴값은 -180 < r <= 180 사이에 있다.
def get_angle_diff(angle1, angle2):
	diff = angle1 - angle2
	if diff > 180:
		diff -= 360
	elif diff <= -180:
		diff += 360
	return diff

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

# pt1, pt2 사이의 거리
def get_distance(pt1, pt2):
	return math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))

# lpt1 - lpt2 를 지나는 선과 pt 와의 거리
# pt가 lpt1-lpt2의 왼쪽이면 마이너스, 오른쪽이면 플러스 값이 리턴된다.
def get_line_distance(pt, lpt1, lpt2):
	px = lpt2[0] - lpt1[0]
	py = lpt2[1] - lpt1[1]
	dab = px * px + py * py
	return (py * pt[0] - px * pt[1] + lpt2[0] * lpt1[1] - lpt2[1] * lpt1[0]) / math.sqrt(dab)

# lpt1 - lpt2 를 지나는 선에서 pt로부터 제일 가까운 점을 찾는다
def get_closest_point_from_line(pt, lpt1, lpt2):
	px = lpt2[0] - lpt1[0]
	py = lpt2[1] - lpt1[1]
	dab = px * px + py * py
	u = ((pt[0] - lpt1[0]) * px + (pt[1] - lpt1[1]) * py) / dab
	return (lpt1[0] + u * px, lpt1[1] + u * py)

# pt에서 부터 forward 방향으로 distance 떨어진 지점에서의 트랙 방향
def get_track_direction(pt, waypoints, closest_waypoints, distance, forward):
	wpindex = closest_waypoints[1 if forward else 0]

	prev_wp = None
	next_wp = waypoints[wpindex]

	# 첫번째 waypoint와 pt의 거리는 pt위치에 대응하는 prev/next waypoint 중간 지점과 waypoint간의 거리다.
	cpt = get_closest_point_from_line(pt, waypoints[closest_waypoints[0]], waypoints[closest_waypoints[1]])
	distsum = get_distance(cpt, next_wp)
	while distsum < distance:
		prev_wp = next_wp
		wpindex = next_waypoint_index(waypoints, wpindex) if forward else prev_waypoint_index(waypoints, wpindex)
		next_wp = waypoints[wpindex]
		distsum += get_distance(prev_wp, next_wp)
		if distsum > distance:
			break

	if forward:
		if not prev_wp:
			prev_wp = prev_waypoint(waypoints, wpindex)
		return get_angle(prev_wp, next_wp)
	else:
		if not prev_wp:
			prev_wp = next_waypoint(waypoints, wpindex)
		return get_angle(next_wp, prev_wp)

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
