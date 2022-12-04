import sys
import numpy as np
import matplotlib.pyplot as plt
import igraph as ig
import random
import math

def gen_weighted_adj_matrix(rows, cols, min_dist, max_dist):
    n = rows * cols
    M = np.zeros((n, n))
    for r in list(range(rows)):
        for c in list(range(cols)):
            i = r * cols + c
            # Two inner diagonals
            if c > 0: M[i - 1, i] = M[i, i - 1] = random.randint(1,10)
            # Two outer diagonals
            if r > 0: M[i - cols, i] = M[i, i - cols] = random.randint(1,10)
    return M

def dijkstra(start_vertex, final_vertex, g):
    # initialization
    dist = {}
    prev = {}
    vertices = ig.VertexSeq(g).indices
    # initialization: set all min distances to be infinity, and set the distance of the start node to 0
    for vertex in vertices:
        dist[vertex] = float('inf')
    dist[start_node] = 0
    # traverse through unvisited vertices
    while vertices:
        # set the minimum
        min_dist_vertex = vertices[0]
        vertices.remove(min_dist_vertex)
        for vertex in g.neighbors(min_dist_vertex):
            if dist[vertex] < dist[min_dist_vertex]:
                min_dist_vertex = vertex
        for vertex in g.neighbors(min_dist_vertex):
            # get the weight through igraph's select
            # indexing 0 because otherwise it will return a list
            cost = dist[min_dist_vertex] + g.es.select(_source=min_dist_vertex, _target=vertex)['weight'][0]
            if cost < dist[vertex]:
                dist[vertex] = cost
                prev[vertex] = min_dist_vertex

    shortest_path = []
    vertex = final_vertex
    while vertex != start_vertex:
        shortest_path.append(vertex)
        vertex = prev[vertex]

    shortest_path.append(start_vertex)
    return shortest_path

def delete_node_on_path(graph, path):
    node_index = random.choice(path[1:len(path)-1])
    edges = graph.vs[node_index].all_edges()
    graph.delete_edges(edges)
    return node_index

def delete_node_probability(graph, connectivity_matrix, p, cols):
    deleted_nodes = []
    for i in range(len(connectivity_matrix)):
        for j in range(len(connectivity_matrix[0])):
            if(p < connectivity_matrix[cols - 1 - i, j]):
                graph.delete_edges(graph.vs[(cols*i) + j].all_edges())
                deleted_nodes.append((cols*i) + j)
    return deleted_nodes

def node_connectivity(rows, cols):
    connectivity_matrix = np.zeros((rows,cols))
    for row in range(rows):
        for col in range(cols):
            connectivity_matrix[row, col] = np.random.random()
    return connectivity_matrix


# min and max distances in the graph
min_dist = 1
max_dist = 10

# number of iterations where one node along the shortest path is deleted
iterations = 5

# rows and columns must be equal
rows = 10
cols = rows

connectivity_matrix = node_connectivity(rows, cols)

# defining the search nodes
start_node = 0
target_node = 99

# this is kept track of for coloring purposes
disconnected_nodes = []

# generate the adjacency matrix used to create the graph
adj_matrix = gen_weighted_adj_matrix(rows, cols, min_dist, max_dist)

for i in np.arange(0.5, 1, .025):
    # instantiate an igraph graph through our adjacency matrix
    g = ig.Graph.Weighted_Adjacency(adj_matrix, "max")
    disconnected_nodes = delete_node_probability(g, connectivity_matrix, i, cols)
    g.vs["color"] = "black"
    # get the shortest node path from our start_node to our target_node
    try:
        # call dijkstra's to generate the shortest path
        path = dijkstra(start_node, target_node, g)
        if(not previous_nodes == {} and not shortest_path == {}):
            path = get_shortest_path(previous_nodes, shortest_path, start_node=start_node, target_node=target_node)
        else:
            print("No possible path to the target node")
            path = []
    except:
        pass
    # color each node in the path
    for node in path:
        g.vs[node]["color"] = "green"
    for node in disconnected_nodes:
        g.vs[node]["color"] = "red"

    fig, ax = plt.subplots(figsize=(10, 10))
    ig.plot(
        g,
        target=ax,
        layout="grid",
        #layout="fruchterman_reingold",  # print nodes in a circular layout
        edge_label=g.es["weight"],
        vertex_size=.5,
        vertex_shape='rectangle',
        vertex_frame_width=1.0,
        vertex_frame_color="white",
        vertex_label_size=7.0,
        bbox=(1024, 1024),
        margin=10
    )
    plt.show()
    g.vs["color"] = "black"