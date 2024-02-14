import numpy as np
from scipy import spatial
import random
import sys
sys.setrecursionlimit(3500)

import env, plotting, utils, queue  

class tree_node:
    def __init__(self, x, y, parent_node=object(), child_node=object, cost=0):
        self.x = x
        self.y = y
        self.child = child_node
        self.parent = parent_node
        self.axis_val = np.array([x, y])  # storing coordinates as a numpy array
        self.cost = cost

class RRTstarFN:
    def initializer(self, x, y):
        y = np.random.uniform(x, y)
        return y

    def norm_func(self, x):
        return np.sqrt(x.dot(x))

    def RRTstarFN_algo(self, root_node, goal_node, max_distance, max_nodes, max_iter, obstacles):
        Vertex_list = [root_node]
        Node_cord_list = [root_node.axis_val]
        Edge_list = []
        radius = 0.5
        flag = 0
        count = 0

        # Include the plotting and utils modules
        plotting_utils = plotting.Plotting(root_node.axis_val, goal_node[0])
        self.utils = utils.Utils()

        for random_node in range(max_iter - 1):
            count += 1
            random_node_sampling = tree_node(self.initializer(0, 10), self.initializer(0, 10))
            nearest_node = self.nearest_neighbour(random_node_sampling, Vertex_list)
            connection = self.distance_specified_point(random_node_sampling, nearest_node, max_distance)

            if not self.is_collide(connection, obstacles):
                newest_node = connection[len(connection) - 1]
                Vertex_list.append(newest_node)
                Node_cord_list.append(newest_node.axis_val)
                newest_node.parent = nearest_node
                nearest_node.child = newest_node
                newest_node.cost += self.distance_func(nearest_node, newest_node)
                newest_node_neighbours = self.nodes_within_radius(newest_node, Vertex_list, Node_cord_list, radius)
                nearest_node = self.choose_parent(newest_node, nearest_node, newest_node_neighbours, max_distance,
                                                obstacles)
                newest_node.parent = nearest_node
                nearest_node.child = newest_node
                newest_node.cost += self.distance_func(nearest_node, newest_node)
                Edge_list.append((newest_node.parent, newest_node))
                Edge_list = self.rewire(newest_node_neighbours, Edge_list, newest_node, max_distance, obstacles)

                # Plotting for each iteration
                plotting_utils.animation(Vertex_list, Edge_list, f"RRT* FN Iteration {count}", False)

                if count > max_nodes:
                    Vertex_list = self.Forced_removal(Vertex_list, newest_node, goal_node)

                if self.is_collide([newest_node], goal_node):
                    flag = 1

        if flag == 1:
            print('Goal Found')
        else:
            print('Goal Not Found')

        # Plotting the final path
        plotting_utils.animation(Vertex_list, Edge_list, f"RRT* FN Iteration {count}", True)
        return Vertex_list, Edge_list

#Calculates the Euclidean distance between two nodes.
    def distance_func(self, nearest_node, newest_node):
        return self.norm_func(nearest_node.axis_val - newest_node.axis_val)

#Checks for collision between a point and an obstacle.
    def collide_iter(self, x_value, y_value, obstacle_x, obstacle_y, width, height):
        if (x_value <= obstacle_x + width and x_value >= obstacle_x) and (y_value <= obstacle_y + height and y_value >= obstacle_y):
            return True

#Finds the nearest node in the tree to a given random node.
    def nearest_neighbour(self, random_node, vertex_list):
        nearest_node = (object(), 1000)
        for selected_point in vertex_list:
            x_cordinate, y_cordinate = selected_point.x, selected_point.y
            x_cordinate1, y_cordinate1 = random_node.x, random_node.y
            if x_cordinate1 == x_cordinate and y_cordinate1 == y_cordinate: continue
            space = np.linalg.norm(random_node.axis_val - selected_point.axis_val)
            if space < nearest_node[1]:
                nearest_node = (selected_point,space)
        return nearest_node[0]

# Checks if the distance between two nodes is within a specified maximum distance.
    def distance_specified_point(self, random_node, nearest_node, max_distance):
        if self.distance_func(random_node, nearest_node) <= max_distance:
            return [random_node]
        cord_values = [random_node.x - nearest_node.x, random_node.y - nearest_node.y]
        normalised_value = cord_values / np.linalg.norm(cord_values)
        obstacle = True
        if obstacle:
            added_node = nearest_node.axis_val + max_distance * normalised_value
            return [tree_node(added_node[0], added_node[1])]

#Checks if a set of points (nodes) collide with obstacles.
    def is_collide(self, new_points, obstacles):
        for ((obstacle_x, obstacle_y), width, height) in obstacles:
            for new_point in new_points:
                x_value, y_value = new_point.x, new_point.y
                if (self.collide_iter(x_value ,y_value, obstacle_x, obstacle_y, width, height)):
                    return True
        return False

#Finds nodes within a specified radius from a given node.    
    def nodes_within_radius(self, newest_node,Vertex_list,Node_cord_list, radius):
        tree = spatial.KDTree(Node_cord_list)
        query_point = newest_node.axis_val
        required_nodes = tree.query_ball_point(query_point, radius)
        for node in required_nodes:
            return [Vertex_list[node]]

#Chooses the parent for a new node based on cost and collision checking.    
    def choose_parent(self, newest_node, nearest_node, newest_node_neighbours, max_distance, obstacles):
        for node in newest_node_neighbours:
            connection = self.distance_specified_point(newest_node, node, max_distance)
            if not self.is_collide(connection,obstacles):
                if node.cost + self.distance_func(node,newest_node) < newest_node.cost:
                    newest_node.cost = node.cost + self.distance_func(node,newest_node)
                    nearest_node = node
        return nearest_node
#Rewires the tree to optimize the path
    def rewire(self,newest_node_neighbours,Edge_list,newest_node,max_distance,obstacles):
        for node in newest_node_neighbours:
            connection = self.distance_specified_point(newest_node, node, max_distance)
            if not self.is_collide(connection,obstacles):
                if newest_node.cost + self.distance_func(node,newest_node) < node.cost:
                    parent = node.parent
                    node.parent = newest_node
                    node.cost = newest_node.cost + self.distance_func(node,newest_node)
                    Edge_list.remove((parent,node))
                    Edge_list.append((newest_node,node))
        return Edge_list
#Removes nodes to maintain a maximum number of nodes.    
    def Forced_removal(self, Vertex_list, newest_node, goal_node):
        childfree = []
        childfree = self.childlessnodess(Vertex_list)
        best_path = self.BestPathLastNode(Vertex_list, goal_node)
        for newest_node in best_path:
            childfree.remove(newest_node)
            Premove = random.randrange(len([childfree]))
            premove = childfree[Premove]
            for node in premove:
                Vertex_list = Vertex_list.remove(node)
        return Vertex_list
    #Finds nodes without children
    def childlessnodess(self, Vertex_list):
        childlessnodes=[]
        for node in Vertex_list:
            if node.child == None:
                childlessnodes.append(node)
        return childlessnodes
#Traces the path from a node to the root.                
    def path_tracing(self, node,path):
        if node.cost == 0:
            path.append((node,node))
            return path
        else:
            parent = node.parent
            path.append((parent,node))
            return self.path_tracing(node.parent,path)
    
    def path_cost(self, path):
        cost = 0
        for edge in path:
            cost += self.distance_func(edge[0],edge[1])
        return cost
#inds the best path to the goal node among all paths in the tree.    
    def BestPathLastNode(self, Vertex_list, goal_node):
        path_to_goal = []
        best_path = []
        cost = 9999
        for node in Vertex_list:
            if self.is_collide([node],goal_node):
                path_to_goal = self.path_tracing(node,[])
                goal_cost = self.path_cost(path_to_goal)
                if goal_cost <= cost:
                    best_path = path_to_goal
        return (best_path)

if __name__ == "__main__":
    max_nodes = 1500
    max_iter = 1500
    obstacles = [((2,2),2,1),((5,2),2,1),((7,4),1,4),((2,4),1,4),((3.3,8),3,1),((5,3),1,4.8),((7,8),3,1),((0,8),3,1)]
    root_node = tree_node(1, 1)
    goal_node = [((9,9), 0.4, 0.4)]
    max_distance = 1
    rrt = RRTstarFN()
    rrt.RRTstarFN_algo(root_node, goal_node, max_distance, max_nodes, max_iter, obstacles)