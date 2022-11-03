# Daniela Filipa Pinto Dias
# nMec 98039

from typing import final
from tree_search import *
from cidades import *

class MyNode(SearchNode):
    def __init__(self,state,parent,cost,heuristic,evaluation=None):
        
        # Variable to save cost
        self.cost = cost
        
        # Variable to save heuristic
        self.heuristic = heuristic
        
        # Variable to save children
        self.children = []
        
        # Variable to save value of the A* evaluation function
        #self.eval = evaluation
        self.eval = round(evaluation)

        super().__init__(state,parent)

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0): 
        super().__init__(problem,strategy,seed)
        
        # Change root to MyNode and save it
        heuristic = self.problem.domain.heuristic(problem.initial, self.problem.goal)
        root = MyNode(problem.initial, None, 0, heuristic, heuristic)
        self.all_nodes = [root]
        self.open_nodes = [0]
        
        # Variable to save closed nodes
        self.closed_nodes = []
        
        # Variable to save solution tree
        self.solution_tree = None
        
        # Variable to save shortcuts used in make shortcuts()
        self.used_shortcuts = []

    def astar_add_to_open(self,lnewnodes):
        # Note: lnewnodes and open_nodes work with nodeIDs
        self.open_nodes += lnewnodes
        self.open_nodes.sort(key = lambda nodeID: self.all_nodes[nodeID].cost + self.all_nodes[nodeID].heuristic)

    def propagate_eval_upwards(self,node):
        
        # If node is root
        if node.parent == None:
            return
        
        # Obtain parent node
        parent = self.all_nodes[node.parent]
        
        # Update parent eval
        parent.eval = node.eval

        self.propagate_eval_upwards(parent)

    def search2(self,atmostonce=False):
    
        while self.open_nodes != []:
            
            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID]
            
            # Graph search
            if atmostonce:
                self.closed_nodes.append(nodeID)
            
            # If found solution
            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                
                # Obtain lowest eval of all terminals
                min_terminal = None
                for terminalId in (self.open_nodes + [nodeID]):
                    node_terminal = self.all_nodes[terminalId]
                    
                    if not min_terminal or min_terminal.eval > node_terminal.eval:
                        min_terminal = node_terminal
                
                # Propagate this update to all ancestors
                node.eval = min_terminal.eval
                self.propagate_eval_upwards(node)
                
                return self.get_path(node)
            
            lnewnodes = []
            self.non_terminals += 1
            
            for a in self.problem.domain.actions(node.state):
                
                newstate = self.problem.domain.result(node.state,a)
                
                # Calculate cost of newnode
                cost = self.problem.domain.cost(node.state, a) + node.cost
                
                # Calculate heuristic of newnode
                heuristic = self.problem.domain.heuristic(newstate, self.problem.goal)
                
                if newstate not in self.get_path(node):
                    
                    # Create node with cost and heuristic
                    newnode = MyNode(newstate, nodeID, cost, heuristic, cost + heuristic)
                    
                    # Graph search
                    if atmostonce:
                        
                        # Note: If node belongs to CV n OPEN or CV n OPEN, it is already in all_nodes
                        open_states = [self.all_nodes[n].state for n in self.open_nodes]
                        closed_states = [self.all_nodes[n].state for n in self.closed_nodes]
                        
                        # Node belongs to CV n OPEN
                        if newstate in open_states:
                        
                            # Iterate through all open nodes 
                            for this_id in self.open_nodes:
                                
                                this_node = self.all_nodes[this_id]
                                
                                if newstate == this_node.state:
                                    
                                    # If new node provides a better path in terms of cost
                                    if newnode.cost < this_node.cost:
                                        self.all_nodes[this_id] = newnode
                         
                        # Node belongs to CV n CLOSED               
                        elif newstate in closed_states:
                            
                            # Iterate through all closed nodes 
                            for this_id in self.closed_nodes:
                                
                                this_node = self.all_nodes[this_id]
                                
                                if newstate == this_node.state:
                                    
                                    # If new node provides a better path in terms of cost
                                    if newnode.cost < this_node.cost:
                                        self.all_nodes[this_id] = newnode
                                          
                        # Node is new                  
                        else: 

                            self.all_nodes.append(newnode)
                            self.all_nodes[nodeID].children.append(len(self.all_nodes)-1)
                            
                            lnewnodes.append(len(self.all_nodes)-1)
                            
                    else:        
                        
                        self.all_nodes.append(newnode)
                        self.all_nodes[nodeID].children.append(len(self.all_nodes)-1)
                        
                        lnewnodes.append(len(self.all_nodes)-1)

            self.add_to_open(lnewnodes)
            
            # If node isn't root
            if nodeID != 0:
                
                # If node is terminal (leaf)
                if not node.children:
                    self.propagate_eval_upwards(node)
                
                else:                        
                    
                    # Get leaves under node
                    leaves = self.get_leaves(node)
                    
                    # Obtain lowest eval of all terminals
                    leaf = min(leaves, key = lambda leaf: leaf.eval)
                    
                    # Updates eval attribute and propagate this update to ancestors
                    node.eval = leaf.eval
                    self.propagate_eval_upwards(node)
            
        return None

    def get_leaves(self, node):
        
        # Default value
        leaves = []
        
        # If node is terminal
        if not node.children:
            return [node]
        
        for childId in node.children:
            child = self.all_nodes[childId]
            leaves = leaves + self.get_leaves(child)
        
        return leaves

    def repeated_random_depth(self,numattempts=3,atmostonce=False):
        
        # Default value
        best_solution = None
        
        for attempt in range(1, numattempts):
            
            # Create new tree with seed = attempt
            solution_tree = MyTree(self.problem, 'rand_depth', attempt)
            solution = solution_tree.search2()
            
            if not best_solution or self.solution_tree.solution.cost > solution_tree.solution.cost:
                best_solution = solution
                self.solution_tree = solution_tree
                
        return best_solution
            

    def make_shortcuts(self):

        path = self.get_path(self.solution)
        final_path = path.copy()
        
        #In each iteration, walk the path from left to right trying to detect states Si and Sj, where j − i > 1
        for i in range(len(path)):
            for j in range(len(path)):
            
                if j - i > 1:
                    
                    city_one = path[i]
                    city_two = path[j]
                    
                    if city_one not in final_path:
                        # Continue to next starting point
                        break
                    
                    if city_two not in final_path:
                        # Continue to next ending point
                        continue
                    
                    for connection in self.problem.domain.connections:

                        # If there is a direct transition from Si to Sj
                        if city_one in connection and city_two in connection:
                            
                            # Update final path
                            for idx in range(i+1, j):
                                final_path[idx] = ''
                            
                            # Update shortcuts
                            self.used_shortcuts.append((city_one, city_two))
                            
        return [city for city in final_path if city != '']
                            

class MyCities(Cidades):

    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        
        avg_neighbours = 0
        
        for city in self.coordinates:
            # Check neighbours of city
            for connection in self.connections:
                if city in connection:
                    avg_neighbours += 1
        
        # Calculate average neighbours per state
        avg_neighbours = avg_neighbours / len(self.coordinates)
        
        # Calculate maximum tree size
        maximum_tree_size = 0
        for level in range(0, depth+1):
            maximum_tree_size += avg_neighbours**level
            
        return maximum_tree_size
    


