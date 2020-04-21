import sys
import os
import queue
import math
import numpy as np
import sys
import math

# Adopted from Elliott and De Los Angeles AI Project 1 

def euclDist(curr_x,curr_y, goal_x, goal_y):
    '''Calcualte the euclidian distance for our admissible and consistent heuristic
    Arguments:
        curr_x: current x val
        curr_y: current y val
        goal_x: goal x val
        goal_y: goal y val

    Returns:
        int: Euclidian distance value to goal
    '''
    # sqrt(A^2 + B^2) where A is delta x and b is delta y
    return math.sqrt(((goal_x-curr_x)**2)+((goal_y-curr_y))**2)

def returnPath(node):
    '''Returns a path tracing the node back through all its parents'''
    path=[]
    while (node.parent != None):
        path.append([node.x,node.y])
        node = node.parent
    return path

class Node:
    '''A* search node'''
    def __init__(self, curr_x, curr_y, goal_x, goal_y, step_cost = None, previous_cost = None, previous = None):
        '''A* search node
        Arguments:
            curr_x: The location of this node map x
            curr_y: The location of this node map y
            goal_x: The map x of the goal
            goal_y: The map y of the goal
            step_cost: The step cost to this node
            previous_cost: The cost to get to the node's parent
            previous: The parent of this node
        '''
        #self.children = []
        self.contains_goal = False
        self.fn = -1
        self.x = curr_x
        self.y = curr_y
        self.name = str(curr_x)+','+str(curr_y)

        if (curr_x == goal_x) and (curr_y == goal_y):
            self.contains_goal = True
        self.hn = euclDist(curr_x,curr_y,goal_x,goal_y) #H(n)
        self.gn = previous_cost + step_cost
        self.fn = self.hn + self.gn
        self.parent = previous

class aStar():
    """
    Initialize an A* path planning object
    """
    # Create necessary class variables
    def __init__(self,map):
        self.map = map
        self.curr_x = None
        self.curr_y = None
        self.goal_x = None
        self.goal_y = None
        self.left_bound = None
        self.right_bound = None
        self.upper_bound = None
        self.lower_bound = None
        # Set important starting variables based upon the map
        (xs, ys) = np.nonzero(self.map)
        for i, x in enumerate(xs):
            y = ys[i]
            if (self.map[x, y] == 2):
                self.curr_y = y
                self.curr_x = x
            elif (self.map[x, y] == 3):
                self.goal_y = y
                self.goal_x = x

    def findChildren(self,parent):
        """
        Find the valid childrent of the parent

        Parameters:
            node: Parent to find the childrent of
        Returns:
            list of nodes: List of all the parents valid children
        """
        x = parent.x
        y = parent.y
        gn = parent.gn
        # Find the children of the parent node
        children = []
        if((x - 1) >= self.left_bound) and (self.map[x-1,y] != 1) and (self.map[x-1,y] != 5):
            children.append(Node(x-1, y, self.goal_x, self.goal_y,1,gn, parent))

        if((x + 1) <= self.right_bound) and (self.map[x+1,y] != 1) and (self.map[x+1,y] != 5):
            children.append(Node(x+1, y, self.goal_x, self.goal_y,1,gn, parent))

        if((y - 1) >= self.lower_bound) and (self.map[x,y-1] != 1) and (self.map[x,y-1] != 5):
            children.append(Node(x, y-1, self.goal_x, self.goal_y,1,gn, parent))

        if((y + 1) <= self.upper_bound) and (self.map[x,y+1] != 1) and (self.map[x,y+1] != 5):
            children.append(Node(x, y+1, self.goal_x, self.goal_y,1,gn, parent))
        return children

    def grid_astar(self):
        """
        Find the grided A* path to the goal defined in the map from the current x and y values
        The x and y values are set default as the start state and can be changed by replanning with a diffrent x and y

        Returns:
            list of [direction,x,y]: Represents the direction and xy pairs to get to the goal at each step
        """
        closed = []
        # Set starting node
        curr_node = Node(self.curr_x, self.curr_y, self.goal_x, self.goal_y,0,0)
        # Empty list that will be sorted every time new item is added
        fringe = [] 
        max_steps = 1000
        step_count = 0
        # Set map boundaries
        self.upper_bound = self.map.shape[1] -1
        self.lower_bound = 0
        self.right_bound = self.map.shape[0] -1
        self.left_bound = 0
        # Add starting node to fringe
        fringe.append(curr_node)
        # Begin searching
        # Timeout condition
        while step_count <= max_steps:
            if not fringe:
                return False
            if step_count == max_steps:
                return False
            # Get next node from fringe
            next_node = fringe.pop()
            # Has the goal been found?
            if next_node.contains_goal:
                path = returnPath(next_node)
                return path
            # Has the next node been visited?
            if next_node not in closed:
                closed.append(next_node)
                children = self.findChildren(next_node)
                for child in children:
                    # Is there a node with the same x,y already in the fringe?
                    matching_nodes = [nodes for nodes in fringe if nodes.name == child.name]
                    if len(matching_nodes) > 1:
                        raise Exception("AStar shouldn't have multiple nodes with same coordinates")
                    if not any(matching_nodes):
                        # The node isn't in the fringe so add it
                        fringe.append(child)
                    else:
                        # The node is already in the fringe
                        same_node = matching_nodes[0]
                        # Only keep the node with the best fn value
                        if child.fn < same_node.fn:
                            fringe.remove(same_node)
                            fringe.append(child)
            # Sort nodes based on fn values
            fringe.sort(reverse=True, key=lambda node: node.fn)
            step_count+=1

    def replan(self, map, curr_x, curr_y):
        """
        Recalculates the A* path based on a new x and y

        Parameters:
            map: numpy array with following the droneMapGUI format
            int: current x value
            int: current y value

        Returns:
            list of [direction,x,y]: Represents the direction and xy pairs to get to the goal at each step
        """
        # Replan our A* with a new x,y start
        self.map = map
        self.curr_x = curr_x
        self.curr_y = curr_y
        return self.grid_astar()

