#! /usr/bin/env python3.12


import simpy
import networkx as nx
import random


class TrafficLight:
    def __init__(self, env, cycle_time=10):
        self.env = env
        self.state = 'NS'  # North-South green
        self.action = env.process(self.run(cycle_time))

    def run(self, cycle_time):
        while True:
            self.state = 'NS'
            yield self.env.timeout(cycle_time)
            self.state = 'EW'
            yield self.env.timeout(cycle_time)


class Vehicle:
    def __init__(self, env, name, graph, traffic_lights):
        self.env = env
        self.name = name
        self.graph = graph
        self.traffic_lights = traffic_lights
        self.action = env.process(self.run())

    def run(self):
        path = list(nx.shortest_path(self.graph, (0,0), (1,1)))
        for i in range(len(path)-1):
            u, v = path[i], path[i+1]
            if v in self.traffic_lights:
                light = self.traffic_lights[v]
                while light.state not in direction_allowed(u, v):
                    yield self.env.timeout(1)
            print(f"{self.name} moving from {u} to {v} at time {self.env.now}")
            yield self.env.timeout(self.graph[u][v]['length'])


def direction_allowed(u, v):
    dx, dy = v[0] - u[0], v[1] - u[1]
    if dx != 0:
        return ['NS']
    else:
        return ['EW']


def setup(env, graph):
    traffic_lights = {(1,0): TrafficLight(env), (0,1): TrafficLight(env)}
    for i in range(5):
        yield env.timeout(random.randint(1, 3))
        Vehicle(env, f"Car_{i}", graph, traffic_lights)


if __name__ == '__main__':
    # Create a small 2x2 grid road network
    G = nx.grid_2d_graph(2, 2, create_using=nx.DiGraph)
    for u, v in G.edges():
        G[u][v]['length'] = 1  # time to traverse edge

    env = simpy.Environment()
    env.process(setup(env, G))
    env.run(until=30)
