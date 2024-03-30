import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # filter constraints for the given agent
    agent_constraints = [c for c in constraints if c['agent'] == agent]

    # sort constraints by time step
    agent_constraints.sort(key=lambda x: x['timestep'])

    return agent_constraints



def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    for constraint in constraint_table:
        if constraint['timestep'] == next_time:
            # edge constraint
            if len(constraint['loc']) == 2:
                if curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                    return True
            # vertex constraint
            else:
                if next_loc == constraint['loc'][0]:
                    return True

        # case for 2.3, constraints for all future time steps
        # if this constraint is a goal constraint (prevents the agent from colliding with others already at the goal)
        elif next_time > constraint['timestep']:
            if constraint.get('at_goal', False) and next_loc == constraint['loc'][0]:
                return True

    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node['time_step'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    # fix ties
    if n1['g_val'] + n1['h_val'] == n2['g_val'] + n2['h_val']:
        # prefer higher g-val
        return n1['g_val'] < n2['g_val']
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def compute_max_path_length(my_map):
    return len(my_map) * len(my_map[0])


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()

    # the earliest timestep the agent can reach the goal
    # 1.4 if there is a goal constraint, earliest_goal_timestep is the timestep of the constraint
    # if any([(c['loc'][0] == goal_loc) for c in constraints]):
    #     earliest_goal_timestep = min([c['timestep'] for c in constraints if c['loc'][0] == goal_loc])
    # else:
    #     earliest_goal_timestep = 0

    earliest_goal_timestep = 0
    for constraint in constraints:
        if constraint['agent'] == agent and constraint['loc'][0] == goal_loc:
            earliest_goal_timestep = constraint['timestep']
            break


    # 2.4 upper bound on for path length
    max_path_length = compute_max_path_length(my_map)

    h_value = h_values[start_loc]

    constraint_table = build_constraint_table(constraints, agent)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], 0)] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['time_step'] >= earliest_goal_timestep: #
            return get_path(curr)

        # 2.4 terminate the search if the path length exceeds the upper bound
        if curr['g_val'] > max_path_length:
            return None

        # iterate over all possible moves
        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            # print(child_loc)
            #
            # print(my_map[child_loc[0]][child_loc[1]])

            # check if the move is within the map
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue

            if my_map[child_loc[0]][child_loc[1]]:
                continue

            # check if the move violates any constraint
            if is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, constraint_table):
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step': curr['time_step'] + 1}

            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)

        # case for waiting in current cell
        # child = {'loc': curr['loc'],
        #         'g_val': curr['g_val'] + 1,
        #         'h_val': h_values[curr['loc']],
        #         'parent': curr,
        #         'time_step': curr['time_step'] + 1}

    return None  # Failed to find solutions
