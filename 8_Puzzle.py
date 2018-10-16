import numpy as np
import time

class Apply_A_Star():
    def __init__(self, state, immed_parent, movement, depth, step_cost, path_cost, heuristic_cost):
        self.state = state
        self.immed_parent = immed_parent  # parent node
        self.movement = movement  # tiles can move up, left, down, right
        self.depth = depth  # depth of the node 
        self.step_cost = step_cost  # g(n), step-cost, In our case : 1
        self.path_cost = path_cost  # Overall g(n)
        self.heuristic_cost = heuristic_cost  # h(n), cost to reach goal state from the current node

        ''' For deriving child nodes'''
        self.move_tile_up = None
        self.move_tile_left = None
        self.move_tile_down = None
        self.move_tile_right = None

    # check if moving down is valid
    def try_move_tile_down(self):
        # index of the empty tile
        empty_tile_index = [i[0] for i in np.where(self.state == 0)]
        if empty_tile_index[0] == 0:
            return False
        else:
            value_above = self.state[empty_tile_index[0] - 1, empty_tile_index[1]]  # value of the upper tile
            new_state = self.state.copy()
            new_state[empty_tile_index[0], empty_tile_index[1]] = value_above
            new_state[empty_tile_index[0] - 1, empty_tile_index[1]] = 0
            return new_state, value_above

    # check if moving right is valid
    def try_move_tile_right(self):
        empty_tile_index = [i[0] for i in np.where(self.state == 0)]
        if empty_tile_index[1] == 0:
            return False
        else:
            value_left = self.state[empty_tile_index[0], empty_tile_index[1] - 1]  # value of the left tile
            new_state = self.state.copy()
            new_state[empty_tile_index[0], empty_tile_index[1]] = value_left
            new_state[empty_tile_index[0], empty_tile_index[1] - 1] = 0
            return new_state, value_left

    # check if moving up is valid
    def try_move_tile_up(self):
        empty_tile_index = [i[0] for i in np.where(self.state == 0)]
        if empty_tile_index[0] == 2:
            return False
        else:
            value_below = self.state[empty_tile_index[0] + 1, empty_tile_index[1]]  # value of the lower tile
            new_state = self.state.copy()
            new_state[empty_tile_index[0], empty_tile_index[1]] = value_below
            new_state[empty_tile_index[0] + 1, empty_tile_index[1]] = 0
            return new_state, value_below

    # check if moving left is valid
    def try_move_tile_left(self):
        empty_tile_index = [i[0] for i in np.where(self.state == 0)]
        if empty_tile_index[1] == 2:
            return False
        else:
            value_right = self.state[empty_tile_index[0], empty_tile_index[1] + 1]  # value of the right tile
            new_state = self.state.copy()
            new_state[empty_tile_index[0], empty_tile_index[1]] = value_right
            new_state[empty_tile_index[0], empty_tile_index[1] + 1] = 0
            return new_state, value_right

    # return user specified heuristic cost
    def get_heuristic_cost(self, new_state, goal_state, heuristic_function, path_cost, depth):
        if heuristic_function == 'num_misplaced':
            return self.heuristic_misplaced(new_state, goal_state)
        elif heuristic_function == 'manhattan':
            return self.heuristic_manhattan(new_state, goal_state) - path_cost + depth

    # return heuristic cost: number of misplaced tiles
    def heuristic_misplaced(self, new_state, goal_state):
        cost = np.sum(new_state != goal_state) - 1  # minus 1 to exclude the empty tile
        if cost > 0:
            return cost
        else:
            return 0  # when all tiles matches

    # return heuristic cost: sum of Manhattan distance to reach the goal state
    def heuristic_manhattan(self, new_state, goal_state):
        current = new_state
        # digit and coordinates they are supposed to be
        goal_position_dic = {1: (0, 0), 2: (0, 1), 3: (0, 2), 8: (1, 0), 0: (1, 1), 4: (1, 2), 7: (2, 0), 6: (2, 1),
                             5: (2, 2)}
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i, j] != 0:
                    sum_manhattan += sum(abs(a - b) for a, b in zip((i, j), goal_position_dic[current[i, j]]))
        return sum_manhattan

    # once the goal node is met, rewind back to the root node and print out the path
    def print_path(self):
        print "\nGoal has been found!!!!"
        # Establish stacks to generate output hierarchy 
        state_Anc = [self.state]
        movement_Anc = [self.movement]
        depth_Anc = [self.depth]
        # add node information as rewinding back up to root
        while self.immed_parent:
            self = self.immed_parent
            state_Anc.append(self.state)
            movement_Anc.append(self.movement)
            depth_Anc.append(self.depth)
        # print the path
        step_counter = 0
        while state_Anc:
            print 'step', step_counter
            print state_Anc.pop()
            print 'movement=', movement_Anc.pop(), ', depth=', str(depth_Anc.pop()), '\n'
            step_counter += 1

    # search based on path cost + heuristic cost
    def a_star_search(self, goal_state, heuristic_function):
        start = time.time()

        queue = [
            (self, 0)]  # queue of (found but unvisited nodes)
        queue_num_nodes_popped = 0  # number of nodes popped off the queue
        queue_exp = 1  # checking expanding nodes

        depth_queue = [(0, 0)]  # queue of node depth, (depth, path_cost+heuristic cost)
        path_cost_queue = [(0, 0)]  # queue for path cost, (path_cost, path_cost+heuristic cost)
        visited = set([])  # record visited states

        while queue:
            # sort queue in ascending order
            queue = sorted(queue, key=lambda x: x[1])
            depth_queue = sorted(depth_queue, key=lambda x: x[1])
            path_cost_queue = sorted(path_cost_queue, key=lambda x: x[1])

            # update maximum length of the queue
            if len(queue) > queue_exp:
                queue_exp = len(queue)

            current_node = queue.pop(0)[0]  # select and remove the first node in the queue
            queue_num_nodes_popped += 1
            current_depth = depth_queue.pop(0)[0]  # select and remove the depth for current node
            current_path_cost = path_cost_queue.pop(0)[0]  # # select and remove the path cost for reaching current node
            visited.add(
                tuple(current_node.state.reshape(1, 9)[0]))  # avoid repeated state, which is represented as a tuple
            #print visited
            # when the goal state is found, rewind back to the root node and print out the path
            if np.array_equal(current_node.state, goal_state):
                current_node.print_path()
                print 'Generated', str(queue_num_nodes_popped)
                print 'Expanded:', str(queue_exp)
                print 'Time spent: %0.2fs' % (time.time() - start)
                return True

            else:
                # check if moving upper tile down is a valid move
                if current_node.try_move_tile_down():
                    new_state, value_above = current_node.try_move_tile_down()
                    # check if  already visited
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + value_above
                        depth = current_depth + 1
                        # get heuristic cost
                        h_cost = self.get_heuristic_cost(new_state, goal_state, heuristic_function, path_cost, depth)
                        # Establish a new child node
                        total_cost = path_cost + h_cost
                        current_node.move_tile_down = Apply_A_Star(state=new_state, immed_parent=current_node, movement='down', depth=depth,step_cost=1, path_cost=path_cost, heuristic_cost=h_cost)
                        queue.append((current_node.move_tile_down, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))

                # check if moving left tile to the right is a valid move
                if current_node.try_move_tile_right():
                    new_state, value_left = current_node.try_move_tile_right()
                    # check if  already visited
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + value_left
                        depth = current_depth + 1
                        # get heuristic cost
                        h_cost = self.get_heuristic_cost(new_state, goal_state, heuristic_function, path_cost, depth)
                        # Establish a new child node
                        total_cost = path_cost + h_cost
                        current_node.move_tile_right = Apply_A_Star(state=new_state, immed_parent=current_node, movement='right',
                                                       depth=depth, \
                                                       step_cost=1, path_cost=path_cost, heuristic_cost=h_cost)
                        queue.append((current_node.move_tile_right, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))

                # check if moving lower tile up is a valid move
                if current_node.try_move_tile_up():
                    new_state, value_below = current_node.try_move_tile_up()
                    # check if  already visited
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + value_below
                        depth = current_depth + 1
                        # get heuristic cost
                        h_cost = self.get_heuristic_cost(new_state, goal_state, heuristic_function, path_cost, depth)
                        # Establish a new child node
                        total_cost = path_cost + h_cost
                        current_node.move_tile_up = Apply_A_Star(state=new_state, immed_parent=current_node, movement='up', depth=depth,step_cost=1, path_cost=path_cost, heuristic_cost=h_cost)
                        queue.append((current_node.move_tile_up, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))

                # check if moving right tile to the left is a valid move
                if current_node.try_move_tile_left():
                    new_state, value_right = current_node.try_move_tile_left()
                    # check if  already visited
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + value_right
                        depth = current_depth + 1
                        # get heuristic cost
                        h_cost = self.get_heuristic_cost(new_state, goal_state, heuristic_function, path_cost, depth)
                        # Establish a new child node
                        total_cost = path_cost + h_cost
                        current_node.move_tile_left = Apply_A_Star(state=new_state, immed_parent=current_node, movement='left', depth=depth, step_cost=1, path_cost=path_cost, heuristic_cost=h_cost)
                        queue.append((current_node.move_tile_left, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))

initial_ar = [int(x) for x in raw_input("Enter Start State, numbers split by space(ex- 1 2 3 4 5 7 6 8 0):").split()]
#initial_state=np.array([1,2,3,7,4,5,6,8,0]).reshape(3,3)
initial_state=np.array([initial_ar]).reshape(3,3)
print initial_state
goal_ar = [int(x) for x in raw_input("Enter goal State, numbers split by space(ex- 1 2 3 4 5 6 7 8 0):").split()]
#goal_state = np.array([1,2,3,8,6,4,7,5,0]).reshape(3,3)
goal_state=np.array([goal_ar]).reshape(3,3)
print goal_state
print "Misplaced Tiles :"
print 'Wait. Calculating........................\n'
root = Apply_A_Star(state=initial_state,immed_parent=None,movement=None,depth=0,step_cost=0, path_cost=0,heuristic_cost=0)
root.a_star_search(goal_state,heuristic_function = 'num_misplaced')
print "\nManhattan Heuristic:"
root.a_star_search(goal_state,heuristic_function = 'manhattan')