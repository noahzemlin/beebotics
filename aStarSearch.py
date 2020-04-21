class Node:

	def __init__(self, curr_x, curr_y, goal_x, goal_y, step_cost = None,
	 previous_cost = None, previous = None):
		self.contains_goal = False
		self.fn = -1
		self.x = curr_x
		self.y = curr_y
		self.name = str(curr_x) + ',' + str(curr_y)

		if (curr_x == goal_x) and (curr_y == goal_x):
			self.contains_goal = True

class aStar():

	def __init__(self, map):
		self.map = map
		self.curr_x = None
		self.curr_y = None
		self.goal_x = None
		self.goal_y = None
		self.left_bound = None
		self.right_bound = None
		self.upper_bound = None
		self.lower_bound = None

		(xs, ys) = np.nonzero(self.map)
		for i, x in enumeate(xs):
			y = ys[i]
			if (self.map[x, y] == 2):
				self.curr_y = y
				self.curr_x = x
			elif (self.map[x,y] == 3):
				self.goal_y = y
				self.goal_x = x

	def findChildren(self, parent):
		x  = parent.x
		y = parent.y
		gn = parent.gn

		children = []
		if((x-1) >= selfleft_bound) and (self.map[x-1,y] != 1) and 
		(self.map[x-1,y]
