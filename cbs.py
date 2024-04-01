import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

DEBUG = False

def paths_violate_constraint(constraint, paths):
    # compute the list of agents that violates the positive constraints
    if len(constraint['loc']) == 1:
        return vertex_check(constraint, paths)
    else:
        return edge_check(constraint, paths)

def vertex_check(constraint, paths):
    agents_violate = []
    for agent in range(len(paths)):
        if constraint['loc'][0] == get_location(paths[agent], constraint['timestep']):
            agents_violate.append(agent)
    return agents_violate

def edge_check(constraint, paths):
    agents_violate = []
    for agent in range(len(paths)):
        loc = [get_location(paths[agent], constraint['timestep'] - 1), get_location(paths[agent], constraint['timestep'])]
        if loc == constraint['loc'] or constraint['loc'][0] == loc[0] or constraint['loc'][1] == loc[1]:
            agents_violate.append(agent)
    return agents_violate

def is_equal_constraint(constraint1, constraint2):
    """Check if two constraints are equal."""
    return (constraint1['agent'] == constraint2['agent'] and
            constraint1['loc'] == constraint2['loc'] and
            constraint1['timestep'] == constraint2['timestep']) and (constraint1['positive'] == constraint2['positive'])

def add_unique_constraint(constraints, new_constraint):
    """Add a new constraint to the list if it is unique."""
    for existing_constraint in constraints:
        if is_equal_constraint(existing_constraint, new_constraint):
            # The new constraint is not unique, so we don't add it.
            return constraints
    # If we get here, the new constraint is unique, and we can add it.
    constraints.append(new_constraint)
    return constraints

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    total_steps = max(len(path1), len(path2))
    for t in range(total_steps):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)

        # vertex collision
        if loc1 == loc2:
            return {'loc': [loc1], 'timestep': t}
            # return [{'loc': [loc1], 'timestep': t}]

        # edge collision
        if t < total_steps - 1:
            next_loc1 = get_location(path1, t + 1)
            next_loc2 = get_location(path2, t + 1)

            # check if the robots swap their location -> edge conflict
            if loc1 == next_loc2 and loc2 == next_loc1:
                return {'loc': [next_loc2, next_loc1], 'timestep': t + 1}
                # return [{'loc': [loc1, next_loc1], 'timestep': t + 1}, {'loc': [loc2, next_loc2], 'timestep': t + 1}]


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    # format: [{'a1': 0, 'a2': 1, 'loc': [(1, 4)], 'timestep': 3}]
    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            collision = detect_collision(paths[i], paths[j])

            if collision:
                collision['a1'] = i
                collision['a2'] = j

                collisions.append(collision)
                # collisions += collision

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constraints = []
    # vertex collision
    if len(collision['loc']) == 1:
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    # edge collision
    else:
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        rev = list(reversed(collision['loc']))
        constraints.append({'agent': collision['a2'], 'loc': rev, 'timestep': collision['timestep']})

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    constraints = []
    random_agent = random.choice([collision['a1'], collision['a2']])
    # vertex collision
    if len(collision['loc']) == 1:
        # positive constraint
        constraints.append({'agent': random_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True, 'is_goal': False})
        # negative constraint
        constraints.append({'agent': random_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False, 'is_goal': False})

    # edge collision
    else:
        # positive constraint
        constraints.append({'agent': random_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True, 'is_goal': False})
        # negative constraint
        constraints.append({'agent': random_agent, 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False, 'is_goal': False})

    return constraints

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        # if id > 8:
        #     print("No solution found")
        #     return None
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])
        #
        # # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            curr = self.pop_node()

            print(len(curr['constraints']))
            print(curr['paths'])
            print('Cost: ' + str(curr['cost']))
            print(curr['collisions'])
            print(curr['constraints'])

            if not curr['collisions']:
                self.print_results(curr)
                return curr['paths'] # curr is a goal node

            collision = curr['collisions'][0]
            #constraints = standard_splitting(collision)
            constraints = disjoint_splitting(collision)
            print('generated constraints' + str(constraints))

            print('=====================')

            for constraint in constraints:
                skip = False
                new_constraints = curr['constraints'].copy()
                new_constraints = add_unique_constraint(new_constraints, constraint)
                child = {'cost': 0,
                         'constraints': new_constraints,
                         'paths': curr['paths'].copy(),
                         'collisions': []}

                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                              agent, child['constraints'])

                if path:
                    child['paths'][agent] = path

                    if constraint['positive']:
                        violating_agents = paths_violate_constraint(constraint, child['paths'])
                        for a in violating_agents:
                            constraint_new = constraint.copy()
                            constraint_new['agent'] = a
                            constraint_new['positive'] = False
                            child['constraints'] = add_unique_constraint(child['constraints'], constraint_new)
                            agent_path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a],
                                                a, child['constraints'])

                            if not agent_path:
                                skip = True
                                break
                            else:
                                child['paths'][a] = agent_path

                    if not skip:
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                        self.push_node(child)
                else:
                    raise BaseException('No solutions')




    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
