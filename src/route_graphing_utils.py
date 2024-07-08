from matplotlib import pyplot as plt, image as um
import os,sys,ast 
import networkx as nx
import numpy as np


def rand_color():
    color_vectors=[np.array([float(i==k) for k in range(3)]) for i in range(3)]
    v=np.array(color_vectors)@np.random.random(3)
    return v/np.linalg.norm(v)

def graph_route(targets,
                depots,
                tours,
                tour_labels=None,
                with_labels=False,
                color_list=None,
                rotation=None,
                depot_color="red",
                target_color="#00b4d9",
                node_size=7,
                edge_width=1.5,
                random_seed=None,
                ax=None,
                ):
    """
    Args:
        targets: (T,2) numpy array of target positions
        depots: (D,2) numpy array of depot positions
        tours: list of tours
            each tours is a list of target/depot indices
            assumes targets are indexed starting from 1, 
                depots indexed starting from |targets|+1
        tour_labels: |tours| size iterable of names for each tour, in order
        with_labels: whether to put depot labels on them
        color_list: list of colors to use for tour edges
            should be length of |depots|
            if None, randomizes
        rotation: (2,2) numpy matrix 
            for each point, will plot point@rotation
            useful when fitting images
        depot_color: pyplot color of depot
        target_color: pyplot color of target
        node_size: size of node to plot
        edge_width: width of edge
        random_seed: seed to set randomness
    """
    if random_seed is not None:
        np.random.seed(random_seed)
    if color_list is None:
        color_list=[]
    if rotation is None:
        rotation=np.identity(2)
    
    color_list+=[rand_color() for _ in range(len(depots))]

    buildings=np.concatenate((targets,depots),axis=0)
    G = nx.DiGraph()

    for i in range(len(targets)):
        G.add_node(i+1, pos=rotation@buildings[i], color=target_color)
    for i in range(len(targets), len(buildings)):
        G.add_node(i+1, pos=rotation@buildings[i], color=depot_color)
    
    for tour_no,tour in enumerate(tours):
        color=color_list[tour_no]
        for i in range(len(tour)-1):
            if i==0 and tour_labels is not None:
                label=tour_labels[tour_no]
                ax.plot([0],[0],
                color=color,
                label=label,
                linewidth=edge_width,
                )
            G.add_edge(tour[i],tour[i+1],color=color,width=edge_width)
    
    pos=(nx.get_node_attributes(G,"pos"))
    node_color = list(nx.get_node_attributes(G,'color').values())
    edge_color = list(nx.get_edge_attributes(G,'color').values())
    edge_widths = list(nx.get_edge_attributes(G,'width').values())

    if ax is None:
        ax=plt.gca()
    nx.draw_networkx(G,
        pos=pos,
        with_labels=with_labels, # whether to label targets
        font_weight='bold', 
        node_color=node_color,
        edge_color=edge_color,
        node_size=node_size,
        width=edge_widths,
        ax=ax,
        )