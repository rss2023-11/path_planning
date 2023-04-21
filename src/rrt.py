# Import required libraries
import numpy as np
import matplotlib.pyplot as plt

# Define class for RRT node
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT_Connect:
    def __init__(self, start, goal, map):
        # Initialize tree for start and goal
        self.start_node = Node(start[0], start[1])
        self.goal_node = Node(goal[0], goal[1])
        self.start_tree = [self.start_node]
        self.goal_tree = [self.goal_node]
        self.x_range = (0, len(map[0]))
        self.y_range = (0, len(map))
        self.map = map

        self.path = None


    def is_occupied(self, x, y):
        """
        Check that the position on the map is occupied
        """
        return self.map[int(round(y))][int(round(x))]

    def get_random_point(self):
        rand_x = np.random.uniform(*self.x_range)
        rand_y = np.random.uniform(*self.y_range)

        while self.is_occupied(rand_x, rand_y):
            rand_x = np.random.uniform(*self.x_range)
            rand_y = np.random.uniform(*self.y_range)

        return rand_x, rand_y

    def extend_start_tree(self, delta=0.1):
        """
        Try to extend the start tree. Could be unsuccessful if the new node collides with an object.
        """
        rand_x, rand_y = self.get_random_point()

        # Find nearest node in start tree
        nearest_start_node = min(self.start_tree, key=lambda node: (rand_x - node.x) ** 2 + (rand_y - node.y) ** 2)
        dist = ((rand_x - nearest_start_node.x) ** 2 + (rand_y - nearest_start_node.y) ** 2) ** 0.5

        # Extend start tree towards random point
        new_start_node = Node(nearest_start_node.x + delta*(rand_x - nearest_start_node.x)/dist,
                              nearest_start_node.y + delta*(rand_y - nearest_start_node.y)/dist)
        if self.is_occupied(new_start_node.x, new_start_node.y):
            return
        
        new_start_node.parent = nearest_start_node
        self.start_tree.append(new_start_node)

        # Check if new node in start tree is close to any node in goal tree
        for node in self.goal_tree:
            dist = np.sqrt((new_start_node.x - node.x)**2 + (new_start_node.y - node.y)**2)
            if dist < delta:
                # Connect trees and return path
                path = []
                node1 = new_start_node
                node2 = node
                while node1 is not None:
                    path.append([node1.x, node1.y])
                    node1 = node1.parent
                path.reverse()
                while node2 is not None:
                    path.append([node2.x, node2.y])
                    node2 = node2.parent
                self.path = path
                return
                              
    def extend_goal_tree(self, delta=0.1):
        """
        Try to extend the goal tree. Could be unsuccessful if the new node collides with an object.
        """      
        rand_x, rand_y = self.get_random_point()
        # Find nearest node in goal tree
        nearest_goal_node = min(self.goal_tree, key=lambda node: (rand_x - node.x) ** 2 + (rand_y - node.y) ** 2)
        dist = ((rand_x - nearest_goal_node.x) ** 2 + (rand_y - nearest_goal_node.y) ** 2) ** 0.5

        # Extend start tree towards random point
        new_goal_node = Node(nearest_goal_node.x + delta*(rand_x - nearest_goal_node.x)/dist,
                              nearest_goal_node.y + delta*(rand_y - nearest_goal_node.y)/dist)
        
        if self.is_occupied(new_goal_node.x, new_goal_node.y):
            return
        
        new_goal_node.parent = nearest_goal_node
        self.goal_tree.append(new_goal_node)
    
        # Check if new node in goal tree is close to any node in start tree
        for node in self.start_tree:
            dist = np.sqrt((new_goal_node.x - node.x)**2 + (new_goal_node.y - node.y)**2)
            if dist < delta:
                # Connect trees and return path
                path = []
                node1 = node
                node2 = new_goal_node
                while node1 is not None:
                    path.append([node1.x, node1.y])
                    node1 = node1.parent
                path.reverse()
                while node2 is not None:
                    path.append([node2.x, node2.y])
                    node2 = node2.parent
                self.path = path
                return
            
    def get_path(self, max_iter=10000, delta=10):
        for i in range(max_iter):
            if self.path is not None:
                return self.path
            if i%2 == 0:
                self.extend_start_tree(delta)
            else:
                self.extend_goal_tree(delta)
        print("PATH", self.path)
        return self.path


def get_path(start_point, end_point, map):
    rrt = RRT_Connect(start_point, end_point, map)
    return rrt.get_path()
