import sys
import numpy as np
import matplotlib.pyplot as plt
import igraph as ig
import random

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

def delete_vertex_on_path(graph, path):
    node_index = random.choice(path[1:len(path)-1])
    edges = graph.vs[node_index].all_edges()
    graph.delete_edges(edges)
    return node_index

def delete_edge_bernoulli(graph, p):
    edges_to_delete = graph.es.select(weight_lt=p)
    graph.delete_edges(edges_to_delete)
    return edges_to_delete


# min and max distances in the graph
min_dist = 1
max_dist = 10

# number of iterations where one node along the shortest path is deleted
iterations = 25

# rows and columns must be equal
rows = 10
cols = rows

# defining the search nodes
start_node = 0
target_node = 99

# this is kept track of for coloring purposes
disconnected_nodes = []

# generate the adjacency matrix used to create the graph
adj_matrix = gen_weighted_adj_matrix(rows, cols, min_dist, max_dist)
# instantiate an igraph graph through our adjacency matrix
# g_1 = ig.Graph.Weighted_Adjacency(adj_matrix, "min")
#
# for i in range(iterations):
#     g_1.vs["color"] = "blue"
#     # get the shortest node path from our start_node to our target_node
#     try:
#         # call dijkstra's to generate the shortest path
#         path = dijkstra(start_node, target_node, g_1)
#     except:
#         print("No possible path to the target node")
#         break
#     print(path)
#     # color each node in the path
#     for node in path:
#         g_1.vs[node]["color"] = "green"
#     for node in disconnected_nodes:
#         g_1.vs[node]["color"] = "red"
#
#     fig, ax = plt.subplots(figsize=(10, 10))
#     ig.plot(
#         g_1,
#         target=ax,
#         layout="grid",
#         # layout="fruchterman_reingold",  # print nodes in a circular layout
#         edge_label=g_1.es["weight"],
#         vertex_size=.5,
#         vertex_shape='rectangle',
#         vertex_frame_width=1.0,
#         vertex_frame_color="white",
#         vertex_label_size=7.0,
#         bbox=(1024, 1024),
#         margin=10
#     )
#     plt.show()
#     fig_name = str(i) + ".png"
#     plt.savefig(fig_name)
#     disconnected_nodes.append(delete_vertex_on_path(g_1, path))
#     g_1.vs["color"] = "blue"

g_2 = ig.Graph.Weighted_Adjacency(adj_matrix, "min")

for i in range(iterations):
    p = (max_dist/iterations) * i
    g_2.vs["color"] = "blue"
    # get the shortest node path from our start_node to our target_node
    try:
        # call dijkstra's to generate the shortest path
        path = dijkstra(start_node, target_node, g_2)
    except:
        print("No possible path to the target node")
        break
    print(path)
    # color each node in the path
    for node in path:
        g_2.vs[node]["color"] = "green"
    for node in disconnected_nodes:
        g_2.vs[node]["color"] = "red"

    fig, ax = plt.subplots(figsize=(10, 10))
    ig.plot(
        g_2,
        target=ax,
        layout="grid",
        # layout="fruchterman_reingold",  # print nodes in a circular layout
        edge_label=g_2.es["weight"],
        vertex_size=.5,
        vertex_shape='rectangle',
        vertex_frame_width=1.0,
        vertex_frame_color="white",
        vertex_label_size=7.0,
        bbox=(1024, 1024),
        margin=10
    )
    plt.show()
    fig_name = str(i) + ".png"
    plt.savefig(fig_name)
    disconnected_vertices = delete_edge_bernoulli(g_2, p)
    disconnected_nodes.append(disconnected_vertices)
    g_2.vs["color"] = "blue"


