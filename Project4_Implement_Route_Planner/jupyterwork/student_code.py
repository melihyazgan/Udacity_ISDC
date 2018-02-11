import math
from math import sqrt
import pdb
"""Pseudo Code --> http://mat.uab.cat/~alseda/MasterOpt/AStar-Algorithm.pdf,https://en.wikipedia.org/wiki/A*_search_algorithm"""
class AStarRun:
    def __init__(self,map):
        self.map = map
        self.frontier = set() 
        self.expanded = set()
        self.path= {}
    
    def heuristic(self,Node1,Node2):
        """ Returns ecludian distance between given two nodes"""
        Node1 = self.map.intersections[Node1]
        Node2 = self.map.intersections[Node2]
        return math.sqrt((Node1[0]-Node2[0])**2 + (Node1[1]-Node2[1])**2)
    
    def low_costNode(self):    
        """ No more frontier left"""
        if len(self.frontier) == 0:
            return False
        #pdb.set_trace()
        nodes_cost =[]
        nodes_index=[]
        for node in self.frontier:
            nodes_index.append(node)
            node_cost = self.path[node]['h']
            nodes_cost.append(node_cost)
            """Minimizing h helps us keep focused on finding the goal, so search the minimum heuristic, like Exercise Faragas has lower h but at first another node epanded and then return to Faragas."""
        lowcost_node = nodes_index[nodes_cost.index(min(nodes_cost))]
        return lowcost_node
    
    def expanding(self, node):
        """ Expand the connections"""
        for successor in self.map.roads[node]:
            successor_g = self.path[node]['g'] + self.heuristic(node,successor)
            successor_f = successor_g + self.heuristic(successor, self.end)
            """
            1) what if the possible node not in path, so it would be added in frontier and of course in Path
            2) Estimated and current cost comporison to check whether found shorter way or not
            """
            if successor not in self.path or successor_f < self.path[successor]['h']:
                self.frontier.add(successor)
                self.path[successor] ={'g':successor_g, 'h': successor_f, 'come_from':node}
        """ Do not Forget to remove this node from Frontier"""
        #pdb.set_trace()
        self.frontier.remove(node)

    def path_finding(self,start,end):
        self.start = start
        self.end = end
        # Initialize
        self.frontier = set([start])
        """ initialize start"""
        h_start = self.heuristic(start,end)        
        self.path[start] = {'g':0 ,'h':h_start, 'come_from':'noWhere'}
        self.expanding(start)
        
        while len(self.frontier)>0:
            """ Taking the one, who has low cost."""
            low_cost_node = self.low_costNode()
            
            if low_cost_node == 'noWhere':
                raise MyError("No Route Found, path came to start point")
            
            """Come to end"""
            if low_cost_node == end:
                break
            
            """ Frontier is not empty and reached not to end, go through and expand it"""
            if low_cost_node!= 'noWhere':
                self.expanding(low_cost_node)
                
        #pdb.set_trace()
        return self.reconstruct_path(self.path,end)

    def reconstruct_path(self,path,end):
        #pdb.set_trace()
        current_node = end
        return_path = []
        while current_node != 'noWhere':
            return_path.append(current_node)
            come_from=self.path[current_node]['come_from']
            current_node = come_from
        return_path.reverse()
        return return_path
        
def shortest_path(M,start,goal):
    print("shortest path called")
    Routing_plan = AStarRun(M)
    result = Routing_plan.path_finding(start,goal)
    return result
