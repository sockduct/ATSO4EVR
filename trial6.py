#! /usr/bin/env python3.12


# Import of necessary packages
import networkx as nx
import matplotlib.pyplot as plt


if __name__ == '__main__':
    # Creation of an empty undirected graph
    G = nx.Graph()

    '''
    # Adding nodes
    G.add_node(1)
    G.add_nodes_from(range(2, 13))

    # Adding edges
    G.add_edge(1, 4)
    G.add_edges_from(
        [(2, 5),
         (3, 4), (4, 5), (5, 6),
         (4, 8), (5, 9),
         (7, 8), (8, 9), (9, 10),
         (8, 11), (9, 12)]
    )
    '''
    G.add_nodes_from(range(1, 13))
    G.add_weighted_edges_from(
        [(1, 4, 1.0), (2, 5, 1.0),
         (3, 4, 1.0), (4, 5, 1.0), (5, 6, 1.0),
         (4, 8, 1.0), (5, 9, 1.0),
         (7, 8, 1.0), (8, 9, 1.0), (9, 10, 1.0),
         (8, 11, 1.0), (9, 12, 1.0)]
    )

    # graph visualization
    nx.draw_networkx(G, with_labels=True)
    plt.show()
