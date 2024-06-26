import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        # # 1.2
        # constraints.append({'agent': 0, 'loc': [(1, 5)], 'timestep': 4})
        # # 1.3
        # constraints.append({'agent': 1, 'loc': [(1, 2), (1, 3)], 'timestep': 1})
        # # 1.4
        # constraints.append({'agent': 0, 'loc': [(1, 5)], 'timestep': 10})
        #test
        #constraints.append({'agent': 1, 'loc': [(1, 4)], 'timestep': 3})
        # 1.5 set of constraints to find collision-free paths
        # constraints.append({'agent': 1, 'loc': [(1, 3), (1, 2)], 'timestep': 2})
        # constraints.append({'agent': 1, 'loc': [(1, 3), (1, 4)], 'timestep': 2})
        # constraints.append({'agent': 1, 'loc': [(1, 3)], 'timestep': 2})
        # constraints.append({'agent': 1, 'loc': [(1, 2)], 'timestep': 2})


        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            # 2.1 itreates over the path of the current agent
            for j in range(len(path)):
                # add constraints for all future agents
                for k in range(i + 1, self.num_of_agents):

                    # add a permenant constraint if the current agent i is at the goal location
                    if path[j] == self.goals[i]:
                        constraints.append({'agent': k, 'loc': [path[j]], 'timestep': j, 'at_goal': True})
                    else:
                        # normal vertex constraints
                        constraints.append({'agent': k, 'loc': [path[j]], 'timestep': j})

                    # for all adjacent locations (edge) in the path
                    if j < len(path) - 1:
                        # edge constraints for both directions
                        constraints.append({'agent': k, 'loc': [path[j], path[j + 1]], 'timestep': j + 1})
                        constraints.append({'agent': k, 'loc': [path[j + 1], path[j]], 'timestep': j + 1})


            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
