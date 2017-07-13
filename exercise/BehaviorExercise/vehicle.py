import math
class StateTransition(object):
	def __init__(self, ego):
		self.ego = ego
		return
	def successor_states(self, current_fsm_state):
		if current_fsm_state == "CS" or current_fsm_state == "KL":
			return ["KL", "PLCL","PLCR"]
		if current_fsm_state == "PLCL":
			return ["PLCL", "LCL"]
		if current_fsm_state == "PLCR":
			return ["PLCR", "LCR"]
		if current_fsm_state == "LCL":
			return ["KL", "LCL"]
		if current_fsm_state == "LCR":
			return ["KL", "LCR"]
		raise Exception("unexpected sate")
	def generate_trajectory(self, state, current_pose, predictions):
		if state == "KL":
			#roughly assuem we will have the same speed as the vehicle in front of us
			
			target_lane = current_pose.lane
			
		if state == "PLCL" or state == "LCL":
			target_lane = current_pose.lane + 1
		if state == "PLCR" or state == "LCR":
			target_lane = current_pose.lane - 1
			
		in_front = [v for (v_id, v) in predictions.items() if v[0]['lane'] == target_lane and v[0]['s'] > current_pose.s ]
		if len(in_front) > 0: 
			leading = min(in_front, key=lambda v: v[0]['s'] - current_pose.s)
			target_v = leading[1]['s'] - leading[0]['s']
		else:
			target_v = self.ego.target_speed
		trajectory_for_state = []
		horizon = 10
# 		s = current_pose.s
		for i in range(horizon):
			s = current_pose.s + target_v*i
			trajectory_for_state.append({'s':s, 'lane': target_lane})
			
		return trajectory_for_state
	def valid_lane_cost(self,state, trajectory_for_state, predictions):
		cost = 0
		lane = trajectory_for_state[0]['lane']
		if lane < 0 or lane >= 4:
			cost = 1
		print("valide lane cost: {}".format(cost))
			
		return cost
	def prepare_lc_cost(self,state, trajectory_for_state, predictions):
		cost = 0;
		if (self.ego.state in ["PLCL", "PLCR"]) and (state in ["PLCL", "PLCR"]):
			#give teh vehicle a bit push, so that it has the urge to go from prepare lane change to lane change
			cost = 0.1
		print("preapre lc cost: {}".format(cost))
		return cost
	def lane_choice_cost(self,state, trajectory_for_state, predictions):
		lane = trajectory_for_state[0]['lane']
		delta_lane = abs(lane - self.ego.goal_lane)
		
		delta_s = self.ego.goal_s - self.ego.s
		
		if  abs(delta_s) > 1:
			cost = 1- math.exp(-delta_lane/float(delta_s))
		else:
			cost = (delta_lane)
		print("lane choice cost: {}".format(cost))
		return cost
	def target_speed_cost(self,state, trajectory_for_state, predictions):
		estimated_spped = trajectory_for_state[1]['s'] - trajectory_for_state[0]['s']
		cost = abs(estimated_spped - self.ego.target_speed)/float(self.ego.target_speed)
		print("target speed cost: {}".format(cost))
		return cost
	def collision_cost(self,state, trajectory_for_state, predictions):
		cost = 0
		L = 1
		if state in ["LCL", "LCR"]:
			
			s = trajectory_for_state[0]['s']
			target_lane = trajectory_for_state[0]['lane']
			
			collided_vehicles = [v for (v_id, v) in predictions.items() if v[0]['lane'] == target_lane and abs(v[0]['s'] - s)<L ]
			if len(collided_vehicles)>0:
				cost = 1;
		
		print("collision cost: {}".format(cost))
		return cost
		
	def choose_next_state(self,predictions, current_fsm_state, current_pose):
		cost_functions = [self.valid_lane_cost, self.collision_cost, self.lane_choice_cost, self.collision_cost, self.prepare_lc_cost, self.target_speed_cost]
		weights = [1000, 1000, 10, 1, 1, 1]
		next_state = self.transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
		return next_state
	def transition_function(self, predictions, current_fsm_state, current_pose, cost_functions, weights):
		# only consider states which can be reached from current FSM state.
		possible_successor_states = self.successor_states(current_fsm_state)
		
	
		# keep track of the total cost of each state.
		costs = []
		for state in possible_successor_states:
			# generate a rough idea of what trajectory we would
			# follow IF we chose this state.
			trajectory_for_state = self.generate_trajectory(state, current_pose, predictions)
	
			# calculate the "cost" associated with that trajectory.
			cost_for_state = 0
			for i in range(len(cost_functions)) :
				# apply each cost function to the generated trajectory
				cost_function = cost_functions[i]
				cost_for_cost_function = cost_function(state, trajectory_for_state, predictions)
	
				# multiply the cost by the associated weight
				weight = weights[i]
				cost_for_state += weight * cost_for_cost_function
			costs.append(cost_for_state)
			print("**state = {}, cost={}".format(state, cost_for_state))

	
		# Find the minimum cost state.
		best_next_state = None
		min_cost = 9999999
		for i in range(len(possible_successor_states)):
			state = possible_successor_states[i]
			cost  = costs[i]
			
			if cost < min_cost:
				min_cost = cost
				best_next_state = state 
	
		print("current state={}, lane=[{}, speed={}, s={}".format(self.ego.state, self.ego.lane, self.ego.v, self.ego.s))
		print("###Best state = {}".format(best_next_state))
		return best_next_state 


class Vehicle(object):
	L = 1
	preferred_buffer = 6 # impacts "keep lane" behavior.

	def __init__(self, lane, s, v, a):
		self.lane = lane
		self.s = s 
		self.v = v
		self.a = a
		self.state = "CS"
		self.max_acceleration = None

	# TODO - Implement this method.
	def update_state(self, predictions):
		"""
		Updates the "state" of the vehicle by assigning one of the
		following values to 'self.state':

		"KL" - Keep Lane
		 - The vehicle will attempt to drive its target speed, unless there is 
			 traffic in front of it, in which case it will slow down.

		"LCL" or "LCR" - Lane Change Left / Right
		 - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
			 behavior for the "KL" state in the new lane.

		"PLCL" or "PLCR" - Prepare for Lane Change Left / Right
		 - The vehicle will find the nearest vehicle in the adjacent lane which is
			 BEHIND itself and will adjust speed to try to get behind that vehicle.

		INPUTS
		- predictions 
		A dictionary. The keys are ids of other vehicles and the values are arrays
		where each entry corresponds to the vehicle's predicted location at the 
		corresponding timestep. The FIRST element in the array gives the vehicle's
		current position. Example (showing a car with id 3 moving at 2 m/s):

		{
			3 : [
				{"s" : 4, "lane": 0},
				{"s" : 6, "lane": 0},
				{"s" : 8, "lane": 0},
				{"s" : 10, "lane": 0},
			]
		}

		"""
		#self.state = "KL" # this is an example of how you change state.
		trainsition_agent = StateTransition(self)
		self.state = trainsition_agent.choose_next_state(predictions, self.state, self)
	
	def configure(self, road_data):
		"""
		Called by simulator before simulation begins. Sets various
		parameters which will impact the ego vehicle. 
		"""
		self.target_speed = road_data['speed_limit']
		self.lanes_available = road_data["num_lanes"]
		self.max_acceleration = road_data['max_acceleration']
		goal = road_data['goal']
		self.goal_lane = goal[1]
		self.goal_s = goal[0]
					
	def __repr__(self):
		s = "s:		{}\n".format(self.s)
		s +="lane: {}\n".format(self.lane)
		s +="v:		{}\n".format(self.v)
		s +="a:		{}\n".format(self.a)
		return s
			
	def increment(self, dt=1):
		self.s += self.v * dt
		self.v += self.a * dt
			
	def state_at(self, t):
		"""
		Predicts state of vehicle in t seconds (assuming constant acceleration)
		"""
		s = self.s + self.v * t + self.a * t * t / 2
		v = self.v + self.a * t
		return self.lane, s, v, self.a

	def collides_with(self, other, at_time=0):
		"""
		Simple collision detection.
		"""
		l,	 s,	 v,	 a = self.state_at(at_time)
		l_o, s_o, v_o, a_o = other.state_at(at_time)
		return l == l_o and abs(s-s_o) <= L 

	def will_collide_with(self, other, timesteps):
		for t in range(timesteps+1):
			if self.collides_with(other, t):
				return True, t
		return False, None

	def realize_state(self, predictions):
		"""
		Given a state, realize it by adjusting acceleration and lane.
		Note - lane changes happen instantaneously.
		"""
		state = self.state
		if	 state == "CS"	: self.realize_constant_speed()
		elif state == "KL"	: self.realize_keep_lane(predictions)
		elif state == "LCL" : self.realize_lane_change(predictions, "L")
		elif state == "LCR" : self.realize_lane_change(predictions, "R")
		elif state == "PLCL": self.realize_prep_lane_change(predictions, "L")
		elif state == "PLCR": self.realize_prep_lane_change(predictions, "R")

	def realize_constant_speed(self):
		self.a = 0

	def _max_accel_for_lane(self, predictions, lane, s):
		delta_v_til_target = self.target_speed - self.v
		max_acc = min(self.max_acceleration, delta_v_til_target)
		in_front = [v for (v_id, v) in predictions.items() if v[0]['lane'] == lane and v[0]['s'] > s ]
		if len(in_front) > 0:
			leading = min(in_front, key=lambda v: v[0]['s'] - s)
			next_pos = leading[1]['s']
			my_next = s + self.v
			separation_next = next_pos - my_next
			available_room = separation_next - self.preferred_buffer
			max_acc = min(max_acc, available_room)
		return max_acc

	def realize_keep_lane(self, predictions):
		self.a = self._max_accel_for_lane(predictions, self.lane, self.s)

	def realize_lane_change(self, predictions, direction):
		delta = -1
		if direction == "L": delta = 1
		self.lane += delta
		self.a = self._max_accel_for_lane(predictions, self.lane, self.s)

	def realize_prep_lane_change(self, predictions, direction):
		delta = -1
		if direction == "L": delta = 1
		lane = self.lane + delta
		ids_and_vehicles = [(v_id, v) for (v_id, v) in predictions.items() if v[0]['lane'] == lane and v[0]['s'] <= self.s]
		if len(ids_and_vehicles) > 0:
			vehicles = [v[1] for v in ids_and_vehicles]

			nearest_behind = max(ids_and_vehicles, key=lambda v: v[1][0]['s'])

			print("nearest behind : {}".format(nearest_behind))
			nearest_behind = nearest_behind[1]
			target_vel = nearest_behind[1]['s'] - nearest_behind[0]['s']
			delta_v = self.v - target_vel
			delta_s = self.s - nearest_behind[0]['s']
			if delta_v != 0:
				print ("delta_v {}".format(delta_v))
				print( "delta_s {}".format(delta_s))
				time = -2 * delta_s / delta_v
				if time == 0:
					a = self.a
				else:
					#approximately the acceleration/time it takes to adjust the speed to target speed
					a = delta_v / time
				print ("raw a is {}".format(a))
				if a > self.max_acceleration: a = self.max_acceleration
				if a < -self.max_acceleration: a = -self.max_acceleration
				self.a = a
				print ("time : {}".format(time))
				print ("a: {}".format(self.a))
			else :
				min_acc = max(-self.max_acceleration, -delta_s)
				self.a = min_acc

	def generate_predictions(self, horizon=10):
		predictions = []
		for i in range(horizon):
			lane, s, v, a = self.state_at(i)
			predictions.append({'s':s, 'lane': lane})
		return predictions