import numpy as np
from SMP.motion_planner.node import PriorityNode

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import GreedyBestFirstSearch


class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Motion planner implementation by students.
    Note that you may inherit from any given motion planner as you wish, or come up with your own planner.
    Here as an example, the planner is inherited from the GreedyBestFirstSearch planner.
    """

    def __init__(self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automata,
                         plot_config=plot_config)
        """
        if plot_config.SAVE_FIG:
            self.path_fig = '../figures/gbfs/'
        else:
            self.path_fig = None
            """


    def evaluation_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own evaluation function here.                   #
        ########################################################################
        
        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority


    def heuristic_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own heuristic cost calculation here.            #
        # Hint:                                                                #
        #   Use the State of the current node and the information from the     #
        #   planning problem, as well as from the scenario.                    #
        #   Some helper functions for your convenience can be found in         #
        #   ./search_algorithms/base_class.py                             #
        ########################################################################
        
        current_where = node_current.list_paths[-1]
        
        if self.reached_goal(current_where):
            return 0.0
        
        ###much slower###
        #if self.position_desired is None:   
        #    return self.time_desired.start - current_where[-1].time_step
        
        #if self.calc_heuristic_distance(current_where[0]) < self.calc_heuristic_distance(current_where[-1]):
        #    return np.inf
       
        
        else:
            velocity = current_where[-1].velocity

            if np.isclose(velocity, 0):
                return np.inf
            
            else:
                #distance from goal
                distance_from_goal = self.calc_heuristic_distance(state=current_where[-1],distance_type=1)
                
                #angle to goal
                goal_orientation = self.calc_angle_to_goal(current_where[-1])
                orientation_diff = self.calc_orientation_diff(goal_orientation,current_where[-1].orientation)
                
                #time_cost = self.calc_time_cost(node_current.list_paths[-1])    it's all the same
                
                
                ####much slower####
                
                cost_lanelet, end_lanelet_id, start_lanelet_id = self.calc_heuristic_lanelet(current_where)
                
                factor = 1
                
                if self.is_collision_free(current_where):
                    if self.is_goal_in_lane(end_lanelet_id[0]):
                        factor = 0.1
                    else:
                    factor = 1
                 
                """
                num_obstacles = self.num_obstacles_in_lanelet_at_time_step(current_state[-1].time_step, end_lanelet_id[0])
                 
                self.is_goal_in_lane(end_lanelet_id[0])
                
                dist_to_obstacle = self.calc_dist_to_closest_obstacle(end_lanelet_id[0], current_state[-1].x, current_state[-1].time_step)
                """
                
                if cost_lanelet is None:
                    cost = distance_from_goal + (abs(orientation_diff))*0.01 
                else:
                    cost = distance_from_goal + (abs(orientation_diff))*0.01 + cost_lanelet

                
               # print(cost)   only for debug

                return cost*factor