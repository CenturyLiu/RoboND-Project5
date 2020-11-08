# -*- coding: utf-8 -*-
"""
Created on Sat Nov  7 11:33:22 2020

@author: centuryliu
"""

# map_split_detection.py
# check whether the graph has possibly reachable yet undetected part

import numpy as np
import random

def generate_graph(point_coordinate):
    '''
    generate 4-connected graph based on given coordinate
    ---
    Input : 
    
    point_coordinate : list of (float,float)
        [(x,y),(x,y)...]    
    
    Output : 
    number_of_graphs : int
        the number of graphs detected
    
    graphs : list of list of (float,float)
    
    '''
    # initialize return value
    graphs = []
    
    # create the first graph with the first coordinate
    graphs.append([point_coordinate.pop(0)])
    
    # loop through all the points and create graph
    for pt in point_coordinate:
        # get all possible connected coordinates
        up_pt = (pt[0]-1,pt[1]) # same column, smaller row
        down_pt = (pt[0]+1,pt[1]) # same column, larger row
        left_pt = (pt[0],pt[1]-1) # same row, smaller column
        right_pt = (pt[0],pt[1]+1) # same row, larger column
        
        # check the graphs connected by this graph
        pt_connected_graphs = []
        
        # loop through all possible graphs
        for ii in range(len(graphs)):
            if up_pt in graphs[ii] or down_pt in graphs[ii] or left_pt in graphs[ii] or right_pt in graphs[ii]: # if this pt is connected with  a point in the graph
                pt_connected_graphs.append(ii) # record the graph index
        
        len_pt_connected_graphs = len(pt_connected_graphs)
        # if no graph is connected
        if len_pt_connected_graphs == 0:
            graphs.append([pt]) # create a graph with pt as its initial point
        elif len_pt_connected_graphs == 1:
            graphs[pt_connected_graphs[0]].append(pt)
        else: # more than one graph is connected
            temp_graph = []
            for jj in range(len(pt_connected_graphs)-1,-1,-1): # traverse the graph index in reverse order
                temp_graph += graphs.pop(pt_connected_graphs[jj]) # get and delete a specific graph, combine those graph together
            temp_graph.append(pt)
            graphs.append(temp_graph)
    
    # the graph has been generated, return
    return len(graphs), graphs
        
    

def check_reacheable_unexplored_part(occupancy_map, min_unoccupied = 1, epsilon = 0.5):
    '''
    check whether a possibly reacheable yet undetected part exists in the given map
    ---
    Input :
    
    occupancy_map : np.array()
        0   - unoccupied
        0.5 - unexplored
        1.0 - occupied
    
    ---
    Output : 
    
    reacheable : bool
        whether a possibly reacheable yet undetected part exists in the given map

    
    '''
    # initialize return value
    reachable = False
    
    # initialize consts
    unoccupied = 0.0
    unexplored = -1.0
    occupied = 1.0
    unoccupied_center = []
    
    # check whether any unoccupied point exists
    if not unoccupied in occupancy_map:
        return reachable, unoccupied_center # no unoccupied point exists, unreacheable
        
    if len(occupancy_map[occupancy_map == unoccupied]) < min_unoccupied:
        return reachable, unoccupied_center
        
    if not unexplored in occupancy_map:
        return reachable, unoccupied_center
        
    # there exists unoccupied point
    # find out unvisited points, connect all connectable pts into graph
    # also, create graphs for unoccupied points
    unexplored_point_coordinate = []
    unoccupied_point_coordinate = []
    for ii in range(occupancy_map.shape[0]):
        for jj in range(occupancy_map.shape[1]):
            if occupancy_map[ii][jj] == unexplored:
                unexplored_point_coordinate.append((ii,jj))
            if occupancy_map[ii][jj] == unoccupied:
                unoccupied_point_coordinate.append((ii,jj))
    
    
    number_of_unexplored_graphs, unexplored_graphs = generate_graph(unexplored_point_coordinate)
    '''
    print("number_of_unexplored_graphs == %d"%(number_of_unexplored_graphs))
    for kk in range(number_of_unexplored_graphs):
        print("number of elements in unexplored graph %d is %d"%(kk,len(unexplored_graphs[kk])))
    '''    
    number_of_unoccupied_graphs, unoccupied_graphs = generate_graph(unoccupied_point_coordinate)
    '''
    print("number_of_unoccupied_graphs == %d"%(number_of_unoccupied_graphs))
    for kk in range(number_of_unoccupied_graphs):
        print("number of elements in unoccupied graph %d is %d"%(kk,len(unoccupied_graphs[kk])))
    '''
    
    # check whether the unoccupied points can be connected to the unexplored points
    
    for unoccupied_graph in unoccupied_graphs:
        for unoccupied_pt in unoccupied_graph:
            up_pt = (unoccupied_pt[0]-1,unoccupied_pt[1]) # same column, smaller row
            down_pt = (unoccupied_pt[0]+1,unoccupied_pt[1]) # same column, larger row
            left_pt = (unoccupied_pt[0],unoccupied_pt[1]-1) # same row, smaller column
            right_pt = (unoccupied_pt[0],unoccupied_pt[1]+1) # same row, larger column
            
            for unexplored_graph in unexplored_graphs:
                if up_pt in unexplored_graph or down_pt in unexplored_graph or left_pt in unexplored_graph or right_pt in unexplored_graph:
                    reachable = True
                    # generate a random number to decide whether return mean or random unoccupied point
                    choice = np.random.rand()
                    if choice < epsilon:
                        unoccupied_center = np.mean(unoccupied_graph, 0)
                    else:
                        unoccupied_center = random.choice(unoccupied_graph)
                    
                    
                    return reachable, unoccupied_center

    return reachable, unoccupied_center
        
    
    
    
if __name__ == "__main__":
    '''
    occupancy_map = np.array([[1.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [0.5,0.5,0.5,0.5,1.0,0.5,0.5],
                              [0.5,0.5,0.5,1.0,0.5,1.0,0.5],
                              [0.5,0.5,0.5,0.5,1.0,0.5,0.5],
                              [0.0,0.0,0.5,0.5,1.0,1.0,1.0],
                              [0.0,0.0,1.0,1.0,1.0,0.0,0.0],
                              [0.0,0.0,1.0,1.0,1.0,0.0,0.0]])
    '''
    '''
    occupancy_map = np.array([[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [1.0,0.5,0.5,0.5,1.0,0.0,0.0,1.0,0.5,0.5],
                              [1.0,0.5,0.5,0.5,1.0,0.0,0.0,0.0,1.0,0.5],
                              [1.0,1.0,1.0,1.0,1.0,0.0,0.0,1.0,0.5,0.5],
                              [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [0.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0],
                              [0.0,0.0,0.0,1.0,0.0,0.0,1.0,1.0,1.0,0.0],
                              [0.0,0.0,0.0,1.0,1.0,0.0,1.0,1.0,1.0,1.0],
                              [0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]])
    '''
    
    occupancy_map = np.array([[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [1.0,0.5,0.5,0.5,0.0,0.0,0.0,1.0,0.5,0.5],
                              [1.0,0.5,0.5,0.5,1.0,0.0,0.0,0.0,1.0,0.5],
                              [1.0,1.0,1.0,1.0,1.0,0.0,0.0,1.0,0.5,0.5],
                              [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [0.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0],
                              [0.0,0.0,0.0,1.0,0.0,0.0,1.0,1.0,1.0,0.0],
                              [0.0,0.0,0.0,1.0,1.0,0.0,1.0,1.0,1.0,1.0],
                              [0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,1.0],
                              [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]])
                              
    for ii in range(occupancy_map.shape[0]):  
        for jj in range(occupancy_map.shape[1]):
            if occupancy_map[ii][jj] == 0.5:
                occupancy_map[ii][jj] = -1
                              
    reachable, unoccupied_center = check_reacheable_unexplored_part(occupancy_map,1)
    print(reachable)
    print(unoccupied_center)
