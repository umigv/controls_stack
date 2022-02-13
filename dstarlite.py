import numpy as np
from map import Map
from utils import dist, distNode
from plot_utils import plot_points, destroy_points
from queue import PriorityQueue, Queue
from custompq import CustomPQ

km = 0
start_node = None

class Node:
    def __init__(self, state, id_in, g_in, rhs_in):
        # Parameters
        self.state = state
        self.id = id_in
        self.g = g_in
        self.rhs = rhs_in
        self.neighbors = []
        self.k1 = None
        self.k2 = None
        self.isObs = False
        self.neighbors_generated = False
        # if pred_id is not None:
        #     self.predecessors.append(pred_id)
        #self.successors = []
        self.calculate_key()

    def calculate_key(self):
        if globals()['start_node'] is None:
            self.k1 = 0
            self.k2 = 0
        else:
            self.k1 = min(self.g, self.rhs) + distNode(self, globals()['start_node']) + globals()['km']
            self.k2 = min(self.g, self.rhs)
        return [self.k1, self.k2]

    # Returns true if self is lesser priority than other
    def __lt__(self, other):
        # my_key = self.calculate_key()
        # other_key = other.calculate_key()
        if self.k1 == other.k1:
            return self.k2 < other.k2
        return self.k1 < other.k1

    def __gt__(self, other):
        if self.k1 == other.k1:
            return self.k2 > other.k2
        return self.k1 > other.k1

    def __eq__(self, other):
        if self.id == other.id:
            return True
        return False


    ########### Getters ##########
    def get_rhs(self):
        return self.rhs

    def get_id(self):
        return self.id

    def get_neighbors(self):
        return self.neighbors

    def get_state(self):
        return self.state

    def get_g(self):
        return self.g

    def get_key(self):
        return [self.k1, self.k2]

    def get_new_key(self):
        return [min(self.g, self.rhs) + distNode(self, globals()['start_node']) + globals()['km'], min(self.g, self.rhs)]

    # def get_predecessors(self):
    #     return self.predecessors

    ############# Setting values #########3
    def set_rhs(self, rhs_in):
        self.rhs = rhs_in

    def set_g(self, g_in):
        self.g = g_in

    def add_to_neigh(self, unique_id):
        self.neighbors.append(unique_id)

    # def add_to_succ(self, unique_id):
    #     self.successors.append(unique_id)



class Dstarlite:
    def __init__(self, map, robot):
        # Map and Robot parameters
        self.myrobot = robot
        self.global_map = map
        self.trans_step_size = self.myrobot.get_trans_step_size()
        self.rot_step_size = self.myrobot.get_rot_step_size()

        # Metrics
        self.total_distance = 0

        # start and goal states
        self.goal_state = None  # set by Robot in setting goal state
        self.start_state = None  # set by Robot when loading world

        # debugging
        self.debug = False

        # D* lite parameters
        self.goal_node = None
        self.pq = CustomPQ()
        self.unique_id = 0
        self.nodeSet = {}
        self.openSet = {}
        # start is a global variable
        # km is a global variable
        self.edge_check_depth = 1


    def set_start_state(self, state):
        self.start_state = state

        # create start state
        globals()['start_node'] = Node(self.start_state, self.unique_id, np.inf, np.inf)

        # calculate its key and generate its neighbors
        globals()['start_node'].calculate_key()

        # add itself to the open and node set
        self.openSet[tuple(self.start_state)] = self.unique_id
        self.nodeSet[self.unique_id] = globals()['start_node']
        self.unique_id += 1

    def updateGoal(self):
        self.goal_state = self.myrobot.get_goal_state()
        self.openSet[tuple(self.goal_state)] = -1
        self.goal_node = Node(self.goal_state, -1, np.inf, 0)
        self.nodeSet[-1] = self.goal_node

    # returns true if key1 is numerically lower than key2
    def compare_keys(self, key1, key2):
        if key1[0] == key2[0]:
            return key1[1] < key2[1]
        return key1[0] < key2[0]

    def run(self): #the "MAIN" from pseduocode
        ######### Intitialize parameters #############
        obs_marker_ids = []
        path_marker_ids = []

        # get first view of map
        new_obs_points = self.myrobot.update_map(globals()['start_node'].get_state())
        if self.debug:
            obs_marker_ids.extend(plot_points(new_obs_points))

        self.goal_node.calculate_key()
        self.pq.put(self.goal_node)


        self.compute_shortest_path()
        path = []
        path_idx = 0
        if self.debug:
            path = self.generate_path()
            path_marker_ids = plot_points(path, color=(0, 1, 0, 1))

        while not self.is_goal(globals()['start_node']):
            if globals()['start_node'].g == np.inf:
                print("No solution")
                return

            # find successor to move towards
            # generate successors in case there are none
            self.generate_and_add_neighbors(globals()['start_node'])
            temp_path = [globals()['start_node'].get_state()]
            new_start_node = None
            min_val = np.inf
            for neighbor_id in globals()['start_node'].neighbors:
                if distNode(globals()['start_node'], self.nodeSet[neighbor_id]) + self.nodeSet[neighbor_id].get_g() < min_val:
                    min_val = distNode(globals()['start_node'], self.nodeSet[neighbor_id]) + self.nodeSet[neighbor_id].get_g()
                    new_start_node = self.nodeSet[neighbor_id]
            temp_path.append(new_start_node.get_state())

            distance_moved = dist(temp_path[0], temp_path[1])

            self.myrobot.move_in_path(temp_path, 0)
            self.total_distance += distance_moved

            # move start node forward
            globals()['start_node'] = new_start_node

            #scan area and update map
            new_obs_points = self.myrobot.update_map(globals()['start_node'].get_state())
            if self.debug:
                obs_marker_ids.extend(plot_points(new_obs_points))

            redrawPath = False
            if self.debug and self.myrobot.does_conflict_with_path(new_obs_points, path, path_idx):
                redrawPath = True

            hasUpdatedKm = False
            generateNewPath = False
            # Perform a breadth first search for each obstacle point to see if the new obstacles is a node or if its neighbors are nodes
            # if a neighbor encountered is in openSet
            # 1.) Change the nodes g to be infinity
            # 2.) Update all of its predecessors
            for new_obs_point in new_obs_points:
                bq = Queue()
                exploredSet = {}

                # put first elements into the queue -- (point, depth)
                for i in range(0, 4):
                    angle = (np.pi/2) * i
                    if angle > np.pi:
                        angle = (-np.pi + (angle - np.pi))
                    state_to_add = self.global_map.roundPointToCell([new_obs_point[0], new_obs_point[1], angle])
                    exploredSet[tuple(state_to_add)] = True
                    bq.put((state_to_add, 0))

                while not bq.empty():
                    top_pt = bq.get()

                    if top_pt[1] < self.edge_check_depth: # depth to search to
                        # generate and add children
                        r = self.global_map.roundPointToCell([top_pt[0][0] + self.global_map.resolution, top_pt[0][1], top_pt[0][2]])
                        if exploredSet.get(tuple(r)) is None:
                            bq.put((r, top_pt[1] + 1))
                            exploredSet[tuple(r)] = True
                        l = self.global_map.roundPointToCell([top_pt[0][0] - self.global_map.resolution, top_pt[0][1], top_pt[0][2]])
                        if exploredSet.get(tuple(l)) is None:
                            bq.put((l, top_pt[1] + 1))
                            exploredSet[tuple(l)] = True
                        t = self.global_map.roundPointToCell([top_pt[0][0], top_pt[0][1] + self.global_map.resolution, top_pt[0][2]])
                        if exploredSet.get(tuple(t)) is None:
                            bq.put((t, top_pt[1] + 1))
                            exploredSet[tuple(t)] = True
                        b = self.global_map.roundPointToCell([top_pt[0][0], top_pt[0][1] - self.global_map.resolution, top_pt[0][2]])
                        if exploredSet.get(tuple(b)) is None:
                            bq.put((b, top_pt[1] + 1))
                            exploredSet[tuple(b)] = True

                    # handle the current child
                    if self.openSet.get(tuple(self.global_map.roundPointToCell(top_pt[0]))) is not None:
                        obs_node = self.nodeSet[self.openSet[tuple(top_pt[0])]]
                        if not hasUpdatedKm:
                            hasUpdatedKm = True
                            globals()['km'] = globals()['km'] + distance_moved
                        generateNewPath = True
                        # if it is a node then label the node as an obstacle
                        obs_node.isObs = True
                        # obs_node.g = np.inf
                        # obs_node.rhs = np.inf

                        # neighbors also become obstacles
                        # update the neighbors and the original obs
                        self.generate_and_add_neighbors(obs_node)
                        for neighbor_id in obs_node.neighbors:
                            self.nodeSet[neighbor_id].isObs = True
                            # self.nodeSet[neighbor_id].g = np.inf
                            # self.nodeSet[neighbor_id].rhs = np.inf
                            self.generate_and_add_neighbors(self.nodeSet[neighbor_id])
                        self.update_node(obs_node)

                        for neighbor_id in obs_node.neighbors:
                            self.update_node(self.nodeSet[neighbor_id])
                            for n_sq_id in self.nodeSet[neighbor_id].neighbors:
                                self.generate_and_add_neighbors(self.nodeSet[n_sq_id])
                                self.update_node(self.nodeSet[n_sq_id])
                    else:
                        pass #print("WATCH OUT")

            if generateNewPath:
                self.compute_shortest_path()
                if redrawPath:
                    destroy_points(path_marker_ids)
                    path_marker_ids.clear()
                    path = self.generate_path()
                    path_marker_ids = plot_points(path, color=(0, 1, 0, 1))
                    path_idx = 0


            ##### END OF COMPUTING CHANGES IN EDGE COSTS
        ########### END OF START TO GOAL WHILE


    def compute_shortest_path(self):
        while self.compare_keys(self.pq.queue[0].get_key(), globals()['start_node'].get_new_key()) or globals()['start_node'].get_rhs() != globals()['start_node'].get_g():
            #key_old = [self.pq.queue[0].k1, self.pq.queue[0].k2]
            u = self.pq.get()
            key_old = u.get_key()
            # print(str(distNode(u, globals()['start_node'])))
            if self.compare_keys(key_old, u.get_new_key()):
                u.calculate_key()
                self.pq.put(u)
            elif u.get_g() > u.get_rhs():
                u.g = u.get_rhs()
                self.generate_and_add_neighbors(u)
                for neighbor_id in u.neighbors:
                    self.update_node(self.nodeSet[neighbor_id])
            else:
                u.g = np.inf
                self.generate_and_add_neighbors(u)
                for neighbor_id in u.neighbors:
                    self.update_node(self.nodeSet[neighbor_id])
                self.update_node(u)

    def update_node(self, node):
        # if node.isObs:
        #     return
        if not self.is_goal(node):
            # rhs correlated to successors, therefore generate and add any new successors
            self.generate_and_add_neighbors(node)
            rhs_min = np.inf
            for neighbor_id in node.neighbors:
                rhs_min = min(rhs_min, distNode(node, self.nodeSet[neighbor_id]) + self.nodeSet[neighbor_id].g)
            node.rhs = rhs_min

        # check to see if node is contained within priority queue and remove if it is
        i = 0
        while i < self.pq.qsize():
            if self.pq.queue[i].id == node.id:
                self.pq.queue.pop(i)
                break
            i += 1

        if node.g != node.rhs:
            node.calculate_key()
            self.pq.put(node)


    def generate_and_add_neighbors(self, node, iter=0):
        # This node has already generated its neighbors
        if node.neighbors_generated or iter == 2:
            return
        node.neighbors_generated = True
        # Generate and check 4 connected successors for being
        # 1.) already created
        # 2.) an obstacle

        # Create #1
        new_state = [x for x in node.get_state()]
        new_state[0] += self.trans_step_size
        new_state = self.global_map.roundPointToCell(new_state)
        # Check #1 for existing already or for being an obstacle
        #if not self.global_map.isObstacle(new_state):
        if self.openSet.get(tuple(new_state)) is None:
            # Doesn't exist, so let's create it
            new_node = Node(new_state, self.unique_id,
                            np.inf, np.inf)
            self.openSet[tuple(new_state)] = self.unique_id
            self.nodeSet[self.unique_id] = new_node
            self.unique_id += 1

            if self.global_map.isObstacle(new_state):
                new_node.isObs = True

            # exchange numbers
            node.add_to_neigh(new_node.id)
            new_node.add_to_neigh(node.id)
        else:
            if not node.id in self.nodeSet[self.openSet[tuple(new_state)]].neighbors:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[tuple(new_state)]]
                node_that_exists.add_to_neigh(node.id)

                if not node_that_exists.id in node.neighbors:
                    node.add_to_neigh(node_that_exists.id)


        # Create #2
        new_state2 = [x for x in node.get_state()]
        new_state2[0] -= self.trans_step_size
        new_state2 = self.global_map.roundPointToCell(new_state2)
        # Check #2 for existing already or for being an obstacle
        #if not self.global_map.isObstacle(new_state2):
        if self.openSet.get(tuple(new_state2)) is None:
            # Doesn't exist, so let's create it
            new_node2 = Node(new_state2, self.unique_id,
                            np.inf, np.inf)
            self.openSet[tuple(new_state2)] = self.unique_id
            self.nodeSet[self.unique_id] = new_node2
            self.unique_id += 1

            if self.global_map.isObstacle(new_state2):
                new_node2.isObs = True

            # exchange numbers
            node.add_to_neigh(new_node2.id)
            new_node2.add_to_neigh(node.id)
        else:
            if not node.get_id() in self.nodeSet[self.openSet[tuple(new_state2)]].neighbors:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[tuple(new_state2)]]
                node_that_exists.add_to_neigh(node.id)

                if not node_that_exists.id in node.neighbors:
                    node.add_to_neigh(node_that_exists.id)


        # Create #3
        new_state3 = [x for x in node.get_state()]
        new_state3[1] += self.trans_step_size
        new_state3 = self.global_map.roundPointToCell(new_state3)
        # Check #3 for existing already or for being an obstacle
        #if not self.global_map.isObstacle(new_state3):
        if self.openSet.get(tuple(new_state3)) is None:
            # Doesn't exist, so let's create it
            new_node3 = Node(new_state3, self.unique_id,
                            np.inf, np.inf)
            self.openSet[tuple(new_state3)] = self.unique_id
            self.nodeSet[self.unique_id] = new_node3
            self.unique_id += 1

            if self.global_map.isObstacle(new_state3):
                new_node3.isObs = True

            # exchange numbers
            node.add_to_neigh(new_node3.id)
            new_node3.add_to_neigh(node.id)
        else:
            if not node.get_id() in self.nodeSet[self.openSet[tuple(new_state3)]].neighbors:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[tuple(new_state3)]]
                node_that_exists.add_to_neigh(node.id)

                if not node_that_exists.id in node.neighbors:
                    node.add_to_neigh(node_that_exists.id)


        # Create #4
        new_state4 = [x for x in node.get_state()]
        new_state4[1] -= self.trans_step_size
        new_state4 = self.global_map.roundPointToCell(new_state4)
        # Check #1 for existing already or for being an obstacle
        #if not self.global_map.isObstacle(new_state4):
        if self.openSet.get(tuple(new_state4)) is None:
            # Doesn't exist, so let's create it
            new_node4 = Node(new_state4, self.unique_id,
                            np.inf, np.inf)
            self.openSet[tuple(new_state4)] = self.unique_id
            self.nodeSet[self.unique_id] = new_node4
            self.unique_id += 1

            if self.global_map.isObstacle(new_state4):
                new_node4.isObs = True

            #exchange numbers
            node.add_to_neigh(new_node4.id)
            new_node4.add_to_neigh(node.id)
        else:
            if not node.get_id() in self.nodeSet[self.openSet[tuple(new_state4)]].neighbors:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[tuple(new_state4)]]
                node_that_exists.add_to_neigh(node.id)

                if not node_that_exists.id in node.neighbors:
                    node.add_to_neigh(node_that_exists.id)


        # Create #5
        new_state5 = [x for x in node.get_state()]
        new_state5[2] += self.rot_step_size
        if new_state5[2] > np.pi:
            new_state5[2] = (-np.pi + (new_state5[2] - np.pi))
        new_state5 = self.global_map.roundPointToCell(new_state5)
        # Check #5 for existing already or for being an obstacle
        #if not self.global_map.isObstacle(new_state5):
        if self.openSet.get(tuple(new_state5)) is None:
            # Doesn't exist, so let's create it
            new_node5 = Node(new_state5, self.unique_id,
                            np.inf, np.inf)
            self.openSet[tuple(new_state5)] = self.unique_id
            self.nodeSet[self.unique_id] = new_node5
            self.unique_id += 1

            if self.global_map.isObstacle(new_state5):
                new_node5.isObs = True

            # exchange numbers
            node.add_to_neigh(new_node5.id)
            new_node5.add_to_neigh(node.id)
        else:
            if not node.get_id() in self.nodeSet[self.openSet[tuple(new_state5)]].neighbors:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[tuple(new_state5)]]
                node_that_exists.add_to_neigh(node.id)

                if not node_that_exists.id in node.neighbors:
                    node.add_to_neigh(node_that_exists.id)


        # Create #6
        new_state6 = [x for x in node.get_state()]
        new_state6[2] -= self.rot_step_size
        if new_state6[2] < -np.pi:
            new_state6[2] = (np.pi - (np.abs(new_state6[2] - np.pi)))
        new_state6 = self.global_map.roundPointToCell(new_state6)
        # Check #6 for existing already or for being an obstacle
        #if not self.global_map.isObstacle(new_state6):
        if self.openSet.get(tuple(new_state6)) is None:
            # Doesn't exist, so let's create it
            new_node6 = Node(new_state6, self.unique_id,
                            np.inf, np.inf)
            self.openSet[tuple(new_state6)] = self.unique_id
            self.nodeSet[self.unique_id] = new_node6
            self.unique_id += 1

            if self.global_map.isObstacle(new_state6):
                new_node6.isObs = True

            # exchange numbers
            node.add_to_neigh(new_node6.id)
            new_node6.add_to_neigh(node.id)
        else:
            if not node.get_id() in self.nodeSet[self.openSet[tuple(new_state6)]].neighbors:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[tuple(new_state6)]]
                node_that_exists.add_to_neigh(node.id)

                if not node_that_exists.id in node.neighbors:
                    node.add_to_neigh(node_that_exists.id)

        for neighbor_id in node.neighbors:
            self.generate_and_add_neighbors(self.nodeSet[neighbor_id], iter + 1)

    def is_goal(self, node):
        if distNode(node, self.goal_node) <= self.myrobot.goal_threshold:
            return True
        return False

    def get_total_distance(self):
        return self.total_distance

    def generate_path(self):
        curr_node = globals()['start_node']
        path = []
        while not curr_node == self.goal_node:
            path.append(curr_node.get_state())
            # next node is the minimum of it's predecessors
            min_node = None
            min_score = np.inf
            for neighbor_id in curr_node.neighbors:
                if distNode(self.nodeSet[neighbor_id], curr_node) + self.nodeSet[neighbor_id].get_g() < min_score:
                    min_score = distNode(self.nodeSet[neighbor_id], curr_node) + self.nodeSet[neighbor_id].get_g()
                    min_node = self.nodeSet[neighbor_id]
            if min_node is None:
                return path
            curr_node = min_node
        return path

    def set_debug(self, debug):
        self.debug = debug

