import math

reverse_printed = False

def reward_function(params):
	p_reward = position_reward(params)
	s_reward = speed_reward(params)

	#return min(p_reward, s_reward)
	return max(0.001, p_reward * s_reward)

def fix_waypoints_error(params):
	# Read input variables
	waypoints = params['waypoints']
	#closest_waypoints = params['closest_waypoints']
	is_reversed = params['is_reversed']

	if is_reversed:
		# waypoints 순서를 뒤집어야 한다.
		if waypoints[0][0] < waypoints[1][0]:
			waypoints.reverse()
		
		# reverse이면 waypoint 25, 26의 순서가 바뀌어있다.
		if waypoints[25][0] > waypoints[26][0]:
			(waypoints[25], waypoints[26]) = (waypoints[26], waypoints[25])
		
		#if closest_waypoints[0] > closest_waypoints[1]:
		#	closest_waypoints.reverse()
	else:
		# waypoint 44, 45의 순서가 바뀌어있다.
		if waypoints[44][0] < waypoints[45][0]:
			(waypoints[44], waypoints[45]) = (waypoints[45], waypoints[44])

	# AWS document 에는 closest_waypoints가 순서 상관없이 가장 가까운 2개라고 되어 있다.
	# waypoints의 간격은 일정하지 않으므로, 이러면 뒤쪽의 2개나 앞쪽의 2개가 될 수도 있다.
	# 정말로 그런지 확인은 못해봤는데 어쨌든 찜찜하니 다시 계산해서 세팅한다.
	pt = (params['x'], params['y'])
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
	
	closest_waypoints = (index, index + 1)
	old_closest_waypoints = params['closest_waypoints']
	if closest_waypoints[0] != old_closest_waypoints[0] or closest_waypoints[1] != old_closest_waypoints[1]:
		print('closest_waypoints is different with mime: {} != {}'.format(old_closest_waypoints, closest_waypoints))
	params['closest_waypoints'] = closest_waypoints

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

	# 이전부터 앞으로 track 방향이 어떻게 되는지 살펴본다.
	angle_b2 = get_track_direction(pt, waypoints, closest_waypoints, 1.4, False)
	angle_b1 = get_track_direction(pt, waypoints, closest_waypoints, 0.7, False)
	angle_0 = get_track_direction(pt, waypoints, closest_waypoints, 0, True)
	angle_f1 = get_track_direction(pt, waypoints, closest_waypoints, 0.7, True)
	angle_f2 = get_track_direction(pt, waypoints, closest_waypoints, 1.4, True)

	# 이전 위치와의 각도 차이를 구함
	# 후방의 왼쪽 턴은 마이너스, 오른쪽 턴은 플러스
	# 전방의 왼쪽 턴은 플러스, 오른쪽 턴은 마이너스
	angle_b2_b1 = get_angle_diff(angle_b2, angle_b1)
	angle_b1_0 = get_angle_diff(angle_b1, angle_0)
	angle_0_f1 = get_angle_diff(angle_0, angle_f1)
	angle_f1_f2 = get_angle_diff(angle_f1, angle_f2)

	#print("angle diffs: %.2f %.2f 0 %.2f %.2f"%(angle_b2_d, angle_b1_d, angle_f1_d, angle_f2_d))

    # 트랙의 방향을 보고 현재 내가 있어야 할 위치(중심으로부터의 거리)를  찾는다.
	max_angle = 25
	r1 = convert_range(-max_angle, max_angle, 1, -1, angle_b2_b1) * 0.8
	r2 = convert_range(-max_angle, max_angle, -1, 1, angle_b1_0)
	r3 = convert_range(-max_angle, max_angle, -1, 1, angle_0_f1)
	r4 = convert_range(-max_angle, max_angle, 1, -1, angle_f1_f2) * 0.8

	#print("r1: %.2f r2: %.2f r3: %.2f r4: %.2f"%(r1, r2, r3, r4))

	# 중심으로부터 떨어진 거리. -는 왼쪽, +는 오른쪽. r은 원하는 위치, p는 현재 위치
	r = (r1 + r2 + r3 + r4) / 3.6 * track_width * 0.4
	p = -distance_from_center if is_left_of_center else distance_from_center
	d = abs(p - r)

	#print("r: %.2f p: %.2f d: %.2f"%(r, p, d))

	marker1 = track_width * 0.05
	marker2 = track_width * 0.3

	if d < marker1:
		return 1.0
	elif d < marker2:
		return convert_range(marker1, marker2, 1.0, 0.3, d)
	else:
		return convert_range(marker2, marker3, 0.3, 0.05, d)

max_speed = 2 # 이건 자동 세팅된다.
min_speed = 2

def speed_reward(params):
	global max_speed, min_speed

	if max_speed < 5:
		print('MAXSPEED is', max_speed)

	pt = (params['x'], params['y'])
	waypoints = params['waypoints']
	closest_waypoints = params['closest_waypoints']
	track_width = params['track_width']
	speed = params['speed']
	heading = params['heading']

	if speed > max_speed:
		max_speed = speed


	# pt가 가는 방향의 연장선
	pt2 = (math.cos(math.radians(heading)) + pt[0], math.sin(math.radians(heading)) + pt[1])

	# 현재 위치에서 직진할 때 어느 waypoint부터 트랙을 벗어나는지 조사한다.
	max_joint_to_pt = 2 # 너무 멀리 볼 필요는 없어서...
	wpindex = closest_waypoints[1]
	joint_to_waypoint = 0
	joint_to_pt = 0
	while joint_to_waypoint < track_width / 2 and joint_to_pt < max_joint_to_pt:
		wp = waypoints[wpindex]

		joint = get_closest_point_from_line(wp, pt, pt2)
		joint_to_waypoint = get_distance(wp, joint)
		joint_to_pt = get_distance(pt, joint)

		wpindex = next_waypoint_index(waypoints, wpindex)

	if joint_to_pt < 0.1:
		target_speed = min_speed
	else:
		target_speed = convert_range(0.1, max_joint_to_pt, min_speed, max_speed, joint_to_pt)

	#print(f'joint_to_pt: {joint_to_pt:.2f} target_speed: {target_speed:.2f}  -- {joint}')

	error = 0.2
	max_speed_diff = 3
	min_score = 0.05
	max_score = convert_range(min_speed, max_speed, min_score, 1, target_speed)
	

	if abs(speed - target_speed) < error:
		return max_score

	if speed < target_speed:
		return convert_range(target_speed - max_speed_diff, target_speed - error, min_score, max_score, speed)
	else:
		return convert_range(target_speed + error, target_speed + max_speed_diff, max_score, min_score, speed)


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
def get_line_distance(pt, lpt1, lpt2):
	px = lpt2[0] - lpt1[0]
	py = lpt2[1] - lpt1[1]
	dab = px * px + py * py
	return abs(py * pt[0] - px * pt[1] + lpt2[0] * lpt1[1] - lpt2[1] * lpt1[0] / math.sqrt(dab))

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
