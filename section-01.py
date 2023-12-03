#-------BFS------
graph = {
        
       's': ['a', 'b', 'd'],
        'a': ['c'],
        'b': ['d'],
        'c': ['g', 'd'],
        'd': ['g']
        }

def bfs(graph, start, goal):
    visited=[]    
    queue = [[start]]
    
    while queue:
       
        path = queue.pop(0)
        
        node = path[-1]
        
        if node in visited:
            continue
        visited.append(node)
        if node == goal:
            return path
        else:
            adjacent_nodes=graph.get(node,[])
            for node2 in adjacent_nodes:
                new_path = path.copy()
                new_path.append(node2)
                queue.append(new_path)
        
solution=bfs(graph, 's', 'g')
print("Solution is ", solution)        
            

#----------------------DFS-----------------------------
graph = {
        
       's': ['a', 'b', 'd'],
        'a': ['c'],
        'b': ['d'],
        'c': ['g', 'd'],
        'd': ['g']
        }

def dfs(graph, start, goal):
    visited=[]    
    stack = [[start]]
    
    while stack:
       
        path = stack.pop()
        
        node = path[-1]
        
        if node in visited:
            continue
        visited.append(node)
        if node == goal:
            return path
        else:
            adjacent_nodes=graph.get(node,[])
            for node2 in adjacent_nodes:
                new_path = path.copy()
                new_path.append(node2)
                stack.append(new_path)
        
solution=bfs(graph, 's', 'g')
print("Solution is ", solution)        

#-----------------------------USC-------------------------
graph = {
    'S': [('A', 2), ('B', 3),('D',5)],
    'A': [('C', 4)],
    'B': [('D', 4)],
    'C': [('D', 1), ('G', 2)],
    'D':[('G',5)],
    #'G':[]
}
def path_cost(path):
    total_cost=0
    for(node,cost) in path:
        total_cost +=cost
    return total_cost,path[-1][0]  
def ucs(graph, start, goal):
    visited = []
    queue = [[(start, 0)]]

    while queue:
        queue.sort(key= path_cost)  # sorting by cost
        path = queue.pop(0)  # choosing least cost node path[0]

        node= path[-1][0]

        if node in visited:
            continue

        visited.append(node)

        if node == goal:
            return path

        else:
            adjacent_nodes = graph.get(node, [])
            for (node2, cost) in adjacent_nodes:
                new_path = path.copy()
                new_path.append((node2, cost))
                queue.append(new_path)



# Example usage:

solution = ucs(graph, 'S', 'G')
print('Solution is', solution)
print('Cost of solution is', path_cost(solution)[0])


#-----------------------A*----------------------
graph = {
    'S': [('A', 1), ('B', 4)],
    'A': [('B', 2),('C',5),('G',12)],
    'B': [('C', 2)],
    'C': [ ('G', 3)],
    
    #'G':[]
}

H_table = {
    'S' :7,
    'A':6,
    'B':4,
    'C':2,
    'G':0  
    }

def path_f_cost(path):
    g_cost=0
    for(node,cost) in path:
        g_cost +=cost
        last_node=path[-1][0]
        h_cost=H_table[last_node]
        f_cost=g_cost+h_cost
    return f_cost,last_node


def a_star_search(graph, start, goal):
    visited = []
    queue = [[(start, 0)]]

    while queue:
        queue.sort(key= path_f_cost)  # sorting by cost
        path = queue.pop(0)  # choosing least cost node path[0]

        node= path[-1][0]

        if node in visited:
            continue

        visited.append(node)

        if node == goal:
            return path

        else:
            adjacent_nodes = graph.get(node, [])
            for (node2, cost) in adjacent_nodes:
                new_path = path.copy()
                new_path.append((node2, cost))
                queue.append(new_path)



solution = a_star_search(graph, 'S', 'G')
print('Solution is', solution)
print('Cost of solution is', path_f_cost(solution)[0])


#------------------------------------Greedy--------------
graph = {
    'S': [('A', 1), ('B', 4)],
    'A': [('B', 2),('C',5),('G',12)],
    'B': [('C', 2)],
    'C': [ ('G', 3)],
    
    #'G':[]
}

H_table = {
    'S' :7,
    'A':6,
    'B':4,
    'C':2,
    'G':0  
    }

def path_h_cost(path):
    g_cost=0
    for(node,cost) in path:
        g_cost +=cost
        last_node=path[-1][0]
        h_cost=H_table[last_node]
       # f_cost=g_cost+h_cost
    return h_cost,last_node


def Greedy_best_search(graph, start, goal):
    visited = []
    queue = [[(start, 0)]]

    while queue:
        queue.sort(key= path_h_cost)  # sorting by cost
        path = queue.pop(0)  # choosing least cost node path[0]

        node= path[-1][0]

        if node in visited:
            continue

        visited.append(node)

        if node == goal:
            return path

        else:
            adjacent_nodes = graph.get(node, [])
            for (node2, cost) in adjacent_nodes:
                new_path = path.copy()
                new_path.append((node2, cost))
                queue.append(new_path)



solution = Greedy_best_search(graph, 'S', 'G')
print('Solution is', solution)
print('Cost of solution is', path_h_cost(solution)[0])


















