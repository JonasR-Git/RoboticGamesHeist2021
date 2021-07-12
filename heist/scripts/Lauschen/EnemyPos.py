import matplotlib.pyplot as plt
import math
import random
import numpy as np

ROBOTER_SPEED = 0.22
TIME_INTERVAL = 2
MAX_TRAVEL_DISTANCE_BETWEEN_INTERVAL = ROBOTER_SPEED * TIME_INTERVAL
MAX_NOISE = 0.44


def calculate_distance_between_2_points(x1, x2, y1, y2):
	return math.sqrt(math.pow(abs(x1 - x2), 2) + math.pow(abs(y1 - y2), 2))


def get_slope_between_to_points(x1, x2, y1, y2):
	return (y1 - y2) / (x1 - x2)


def get_gradient_angle(slope):
	return math.atan(slope)


# x1 ist hintere Punkte in der Liste, also der i-te Punkt, x2 ist der i-1-te Punkt
def get_points_closer_together(x1, x2, y1, y2):
	distance = calculate_distance_between_2_points(x1, x2, y1, y2)
	while distance > MAX_TRAVEL_DISTANCE_BETWEEN_INTERVAL:
		half_extra_distance = (distance - MAX_TRAVEL_DISTANCE_BETWEEN_INTERVAL) / 2
		slope = abs(get_slope_between_to_points(x1, x2, y1, y2))
		gradient_angle = get_gradient_angle(slope)
		y_extra = abs(half_extra_distance * math.sin(gradient_angle)) + 0.005  # Ansonsten gibt es eine Dauerschleife,
		# da Lim von Distance gegen Max_Travel geht
		x_extra = abs(half_extra_distance * math.cos(gradient_angle)) + 0.005  # Ansonsten gibt es eine Dauerschleife,
		# da Lim von Distance gegen Max_Travel geht
		if x1 > x2:
			x1 = x1 - x_extra
			x2 = x2 + x_extra
		else:
			x1 = x1 + x_extra
			x2 = x2 - x_extra

		if y1 > y2:
			y1 = y1 - y_extra
			y2 = y2 + y_extra
		else:
			y1 = y1 + y_extra
			y2 = y2 - y_extra

		distance = calculate_distance_between_2_points(x1, x2, y1, y2)
	return x1, x2, y1, y2


class AdjustEnemyPosition:

	def __init__(self, enemy_pos_input):
		for pos in enemy_pos_input:
			self.enemy_pos_original.append(pos)

	enemy_pos_original = []
	enemy_pos_noise = []
	enemy_pos_refactor = []

	enemy_pos_x = []
	enemy_pos_y = []

	enemy_pos_noise_x = []
	enemy_pos_noise_y = []

	enemy_pos_refactor_x_idea1 = []
	enemy_pos_refactor_y_idea1 = []

	enemy_pos_refactor_x_idea2 = []
	enemy_pos_refactor_y_idea2 = []

	def set_noise_on_enemy_pos(self):
		i = 0
		while i < len(self.enemy_pos_original):
			random_x = 10
			random_y = 10
			while MAX_NOISE < pow(random_x, 2) + pow(random_y, 2) > MAX_NOISE/2:
				if random.uniform(0, 1) > 0:
					random_x = - random.uniform(-MAX_NOISE, MAX_NOISE)
				else:
					random_x = random.uniform(-MAX_NOISE, MAX_NOISE)

				if random.uniform(0, 1) > 0:
					random_y = - random.uniform(-MAX_NOISE, MAX_NOISE)
				else:
					random_y = random.uniform(-MAX_NOISE, MAX_NOISE)

			self.enemy_pos_noise.append(self.enemy_pos_original[i] + random_x)
			self.enemy_pos_noise.append(self.enemy_pos_original[i + 1] + random_y)
			i = i + 2

	def split_in_x_y(self):
		i = 0
		while i < len(self.enemy_pos_original):
			if i % 2 == 0:
				self.enemy_pos_x.append(self.enemy_pos_original[i])
				self.enemy_pos_noise_x.append(self.enemy_pos_noise[i])
			else:
				self.enemy_pos_y.append(self.enemy_pos_original[i])
				self.enemy_pos_noise_y.append(self.enemy_pos_noise[i])
			i = i + 1

	def idea_get_points_closer_together(self):
		self.enemy_pos_refactor_x_idea2 = np.copy(self.enemy_pos_noise_x)
		self.enemy_pos_refactor_y_idea2 = np.copy(self.enemy_pos_noise_y)
		i = len(self.enemy_pos_refactor_x_idea2) - 1
		while i > 0:
			self.enemy_pos_refactor_x_idea2[i], self.enemy_pos_refactor_x_idea2[i - 1], self.enemy_pos_refactor_y_idea2[
				i], self.enemy_pos_refactor_y_idea2[i - 1] = get_points_closer_together(
				self.enemy_pos_refactor_x_idea2[i], self.enemy_pos_refactor_x_idea2[i - 1],
				self.enemy_pos_refactor_y_idea2[i], self.enemy_pos_refactor_y_idea2[i - 1])
			i = i - 1
	"""
	def idea_get_points_closer_together(self):
		i = len(self.enemy_pos_noise_x)-1
		while i > 0:
			distance = calculate_distance_between_2_points(self.enemy_pos_noise_x[i], self.enemy_pos_noise_x[i - 1],
															self.enemy_pos_noise_y[i], self.enemy_pos_noise_y[i - 1])
			point_before_x = self.enemy_pos_noise_x[i]
			point_x = self.enemy_pos_noise_x[i - 1]
			point_before_y = self.enemy_pos_noise_y[i]
			point_y = self.enemy_pos_noise_y[i - 1]

			while distance > ROBOTER_SPEED * TIME_INTERVAL:
				if point_before_x > point_x:
					if point_before_y > point_y:
						extra_distance = (distance - ROBOTER_SPEED * TIME_INTERVAL)
						half_extra_distance = extra_distance/2
						slope = slope_between_to_points(point_before_x, point_x, point_before_y, point_y)
						point_before_x = - 1 / slope * half_extra_distance + point_before_x
						point_x = 1 / slope * half_extra_distance + point_x
						point_before_y = - slope * half_extra_distance + point_before_y
						point_y = slope * half_extra_distance + point_y
						distance = calculate_distance_between_2_points(point_before_x, point_x, point_before_y, point_y)

						print(1, point_before_x, point_x, point_before_y, point_y, slope, distance,  distance - 
						ROBOTER_SPEED * TIME_INTERVAL) else: slope = slope_between_to_points(point_before_x, point_x, 
						point_before_y, point_y) point_before_x = - 1 / slope * half_extra_distance + point_before_x 
						point_x = 1 / slope * half_extra_distance + point_x point_before_y = slope * 
						half_extra_distance + point_before_y point_y = - slope * half_extra_distance + point_y 
						distance = calculate_distance_between_2_points(point_before_x, point_x, point_before_y, 
						point_y) 

						print(2, point_before_x, point_x, point_before_y, point_y, slope, distance, distance - 
						ROBOTER_SPEED * TIME_INTERVAL) else: if point_before_y > point_y: slope = abs(
						slope_between_to_points(point_before_x, point_x, point_before_y, point_y)) point_before_x = 1 
						/ slope * half_extra_distance + point_before_x point_x = - 1 / slope * half_extra_distance + 
						point_x point_before_y = - slope * half_extra_distance + point_before_y point_y = slope * 
						half_extra_distance + point_y distance = calculate_distance_between_2_points(point_before_x, 
						point_x, point_before_y, point_y) 

						print(3, point_before_x, point_x, point_before_y, point_y, slope, distance, distance - 
						ROBOTER_SPEED * TIME_INTERVAL) else: slope = abs(slope_between_to_points(point_before_x, 
						point_x, point_before_y, point_y)) point_before_x = 1 / slope * half_extra_distance + 
						point_before_x point_x = - 1 / slope * half_extra_distance + point_x point_before_y = slope * 
						half_extra_distance + point_before_y point_y = - slope * half_extra_distance + point_y 
						distance = calculate_distance_between_2_points(point_before_x, point_x, point_before_y, 
						point_y) 

						print(4, point_before_x, point_x, point_before_y, point_y, slope, distance, distance - 
						ROBOTER_SPEED * TIME_INTERVAL) 

			print("Distanz okay")
			self.enemy_pos_refactor_y_idea1.append(point_x)
			self.enemy_pos_refactor_x_idea1.append(point_y)
			i = i -1
	"""

	def idea_average_points(self):
		number_of_pathpoints = len(self.enemy_pos_x)-1
		i = 0
		while i < number_of_pathpoints:
			self.enemy_pos_refactor_x_idea1.append((self.enemy_pos_noise_x[i] + self.enemy_pos_noise_x[i + 1]) / 2)
			self.enemy_pos_refactor_y_idea1.append((self.enemy_pos_noise_y[i] + self.enemy_pos_noise_y[i + 1]) / 2)
			i = i + 1

	def plot_the_result(self):
		figure_1, ax_1 = plt.subplots()
		figure_2, ax_2 = plt.subplots()
		figure_3, ax_3 = plt.subplots()
		figure_4, ax_4 = plt.subplots()
		ax_1.plot(self.enemy_pos_x, self.enemy_pos_y, 'g')
		ax_2.plot(self.enemy_pos_noise_x, self.enemy_pos_noise_y, 'r')
		ax_3.plot(self.enemy_pos_refactor_x_idea1, self.enemy_pos_refactor_y_idea1, 'b')
		ax_4.plot(self.enemy_pos_refactor_x_idea2, self.enemy_pos_refactor_y_idea2, 'k')
		plt.show()

	def simulate(self):
		self.set_noise_on_enemy_pos()
		self.split_in_x_y()
		self.idea_average_points()
		self.idea_get_points_closer_together()


enemy_pos_data = [
	1.2, 0.4,
	1.3, 0.7,
	1.5, 0.9,
	1.5, 1.3,
	1.5, 1.7,
	1.1, 1.7,
	0.7, 1.7,
	0.7, 2.1,
	0.7, 2.3,
	1.0, 2.4,
	1.4, 2.4,
	1.4, 2.4,
	1.7, 2.4,
	1.8, 2.4,
	1.8, 2.7,
	1.8, 3.0,
	1.8, 3.4,
	1.7, 3.7,
	1.8, 4.0,
	1.8, 4.3,
	1.5, 4.3,
	1.2, 4.3,
	0.8, 4.3,
	0.4, 4.3,
	0.05, 4.3,
	0.05, 3.9,
	0.05, 3.5,
	0.05, 3.1,
	0.05, 2.7,
	0.05, 3.1,
	0.05, 3.5,
	0.05, 3.9,
	0.05, 4.3,
	0.05, 4.7,
	0.05, 4.8,
	0.05, 4.9,
	0.15, 5.2,
	0.3, 5.4,
	0.6, 5.5,
	0.6, 5.8,
	0.6, 6.1,
	0.6, 6.4,
	0.6, 6.7,
	0.6, 7.1,
	0.7, 7.2,
	1.05, 7.1,
	1.3, 7.0,
	1.6, 7.0,
	1.9, 7.0,
	2.3, 7.0,
	2.7, 7.0,
	3.0, 7.0,
	3.3, 7.0,
	3.5, 6.8,
	3.7, 6.6,
	3.9, 6.4,
	3.7, 6.2,
	3.5, 6.0,
	3.2, 5.9,
	2.9, 5.8,
	2.6, 5.8,
	2.3, 5.7,
	2.0, 5.6,
]

test = AdjustEnemyPosition(enemy_pos_data)
test.simulate()
distance_original_noise = calculate_distance_between_2_points(test.enemy_pos_x[0], test.enemy_pos_noise_x[0], test.enemy_pos_y[0], test.enemy_pos_noise_y[0])
distance_original_idea1 = calculate_distance_between_2_points(test.enemy_pos_x[0], test.enemy_pos_refactor_x_idea1[0], test.enemy_pos_y[0], test.enemy_pos_refactor_y_idea1[0])
distance_original_idea2 = calculate_distance_between_2_points(test.enemy_pos_x[0], test.enemy_pos_refactor_x_idea2[0], test.enemy_pos_y[0], test.enemy_pos_refactor_y_idea2[0])
print(distance_original_noise)
print(distance_original_idea1)
print(distance_original_idea2)
test.plot_the_result()
