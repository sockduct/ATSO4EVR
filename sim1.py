#! /usr/bin/env python3.12
'''
Traffic Grid Simulation:
* Use time units in seconds
* X x X Traffic Grid represented as a graph with NetworkX
* Intersections typically have a traffic light
* Vehicles enter the grid from all directions
* Vehicles leave the grid in all directions
'''


# Standard Library:
from collections.abc import Generator
from datetime import datetime
from enum import Enum
from itertools import pairwise
from math import inf
import random
import statistics
import sys
from typing import Any, Self

# Third-Party:
import networkx  # type: ignore
import simpy  # type: ignore
import simpy.events  # type: ignore


# Global Constants:
# Randomized interval in seconds (1 - #) between new car arrivals:
ARRIVAL_INTERVAL = 10
LIGHT_LENGTH = 30  # How long each traffic light is
GRID_SIZE = 'large'  # small | medium | large
#
# Global lists to track times
VEHICLE_WAIT_TIMES = []
VEHICLE_TOTAL_TIMES = []


class Counter:
    '''
    Singleton to track the number of vehicles through the traffic grid
    '''
    _instance = None
    _count = 0

    def __new__(cls) -> Self:
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __iadd__(self, value: int) -> Self:
        if not isinstance(value, int):
            raise TypeError('Only integers can be added to the counter')
        self._count += value
        return self

    def __repr__(self) -> str:
        return str(self._count)


class TLCState(Enum):
    '''
    Traffic Light Color States
    '''
    RED = 0
    GREEN = 1
    YELLOW = -1

    def __repr__(self) -> str:
        return self.name


class TrafficLight:
    '''
    Represent an intersection traffic light with two directions:
    * North - South
    * East - West

    Traffic light states:
    * Transitions from GREEN-NS/RED-EW to RED-NS/GREEN-EW after LIGHT_LENGTH
      seconds
    * Transitions from RED-NS/GREEN-EW to GREEN-NS/RED-EW after LIGHT_LENGTH
      seconds
    * (repeat...)

    Note:  Omitting yellow state for now for simplicity
    '''
    def __init__(self, env: simpy.Environment, vertex: tuple[int, int],
                 light_length: int=LIGHT_LENGTH, debug: bool=False) -> None:
        self.env = env
        self.vertex = vertex
        self.light_length = light_length
        self.debug = debug
        self.ns_state = TLCState.GREEN
        self.ew_state = TLCState.RED
        self.allowed = 'NS'
        self.start = self.env.now
        self.init = True
        self.change_event = env.event()
        env.process(self.run())

    def __repr__(self) -> str:
        return f'TrafficLight({self.vertex}, {self.allowed=})'

    def run(self) -> Generator[simpy.events.Event, None, None]:
        '''
        Manage traffic light state/transitions while simulation running
        '''
        while True:
            if self.init and self.debug:
                print(f'{self.env.now:05.1f}s: Traffic Light {self.vertex} Initialized '
                      f'({self.ns_state=}, {self.ew_state=}, {self.allowed=})')
                self.init = False
            yield self.env.timeout(1)

            if (self.ns_state == TLCState.GREEN and self.ew_state == TLCState.RED
                    and self.env.now - self.start >= self.light_length):
                self.change_event.succeed()
                self.change_event = self.env.event()
                self.ns_state = TLCState.RED
                self.ew_state = TLCState.GREEN
                self.allowed = 'EW'
                self.start = self.env.now
                if self.debug:
                    print(f'{self.env.now:05.1f}s: Traffic Light {self.vertex} Transition '
                          f'({self.ns_state=}, {self.ew_state=}, {self.allowed=})')
            elif (self.ns_state == TLCState.RED and self.ew_state == TLCState.GREEN
                    and self.env.now - self.start >= self.light_length):
                self.ns_state = TLCState.GREEN
                self.ew_state = TLCState.RED
                self.allowed = 'NS'
                self.start = self.env.now
                if self.debug:
                    print(f'{self.env.now:05.1f}s: Traffic Light {self.vertex} Transition '
                          f'({self.ns_state=}, {self.ew_state=}, {self.allowed=})')


class Vehicle:
    '''
    Represent vehicles in the traffic grid

    Track vehicle location in grid, speed, destination, entry and exit times and
    more
    '''
    def __init__(self, env: simpy.Environment, name: str, origin: tuple[str, tuple[int, int]],
                 destination: tuple[int, int], traffic_grid: networkx.Graph, speed: int=30,
                 emergency: bool=False) -> None:
        self.env = env
        self.name = name
        self.emergency = emergency
        self.origin = origin
        self.destination = destination
        self.path = get_shortest_path(traffic_grid, origin[1], destination)
        # self.path_check = networkx.shortest_path(traffic_grid, origin[1], destination)
        self.path_check = networkx.bellman_ford_path(traffic_grid, origin[1], destination)
        # Debug:
        if (
            self.path != self.path_check and
            (
                self.path[0] != self.path_check[0] and
                self.path[-1] != self.path_check[-1] and
                len(self.path) != len(self.path_check)
            )
        ):
            raise RuntimeError(f'Path mismatch:\n      {self.path=}\n{self.path_check=}')

        self.previous_node = None
        self.current_node = origin[1]
        self.next_index = 1
        self.next_node = self.path[self.next_index]
        self.traffic_grid = traffic_grid
        self.speed = speed
        self.wait_time = 0
        # When Vehicle enters grid (e.g., self.env.now)
        self.entry_time = None
        # When Vehicle leaves grid:
        self.exit_time = None
        env.process(self.run())

    def __repr__(self) -> str:
        return f'Vehicle({self.name} at {self.current_node})'

    def run(self) -> Generator[simpy.events.Event, None, None]:
        '''
        Manage vehicle state/movement while simulation running
        '''
        self.entry_time = self.env.now
        print(f'{self.entry_time:05.1f}s: {self.name} arrives from {self.origin[0]} '
              f'at {self.origin[1]} heading to {self.destination} {self.path}')

        if self.emergency:
            print(f'{self.env.now:05.1f}s: {self.name} is an emergency vehicle - setting '
                  'up traffic light preemption...')
            self.preempt_traffic_lights()

        while True:
            # Check if current node is intersection with traffic light:
            if light := self.traffic_grid.nodes[self.current_node].get('tl'):
                if get_direction(self.previous_node, self.current_node) != light.allowed:
                    red_arrival = self.env.now
                    print(f'{self.env.now:05.1f}s: {self.name} waits at red')

                    yield light.change_event

                    self.wait_time = self.env.now - red_arrival
                    print(f'{self.env.now:05.1f}s: {self.name} waited {self.wait_time:.1f}s')

                print(f'{self.env.now:05.1f}s: {self.name} crosses intersection with traffic light')

            # Travel time to next node in path:
            duration = (
                (self.traffic_grid.edges[self.current_node, self.next_node]['length'] / self.speed)
                * 3600
            )

            yield self.env.timeout(duration)

            self.previous_node = self.current_node
            self.current_node = self.next_node
            next_node = (
                self.path[self.next_index + 1]
                if self.next_index + 1 < len(self.path) else 'Exit'
            )

            direction = get_direction(self.previous_node, self.current_node)
            print(f'{self.env.now:05.1f}s: {self.name} reaches {self.current_node} from '
                  f'{self.previous_node} going to {next_node}, {direction=}')

            # Check if exit node:
            if self.current_node == self.destination:
                break

            self.next_index += 1
            self.next_node = self.path[self.next_index]

        VEHICLE_WAIT_TIMES.append(self.wait_time)

        self.exit_time = self.env.now
        total_time = self.exit_time - self.entry_time
        VEHICLE_TOTAL_TIMES.append(total_time)
        print(f'{self.exit_time:05.1f}s: {self.name} exits simulation after {total_time:.1f}s')

    def preempt_traffic_lights(self) -> None:
        '''
        Handle traffic light preemption to ensure Emergency Vehicles (e.g.,
        Ambulance) have a green light at each intersection they cross through

        The outer function schedules the preemption for each intersection
        '''
        arrival_time = 0
        for current_node, next_node in pairwise(self.path):
            edge_length_miles = self.traffic_grid.edges[self.current_node, self.next_node]['length']
            arrival_time += (edge_length_miles / self.speed) * 3600

            if light := self.traffic_grid.nodes[next_node].get('tl'):
                direction = get_direction(current_node, next_node)

                def preempt(light: TrafficLight=light, arrival_time: int=arrival_time,
                            direction: str=direction, env: simpy.Environment=self.env
                            ) -> Generator[simpy.events.Event, None, None]:
                    '''
                    The inner function handles actual traffic light preemption
                    if needed
                    '''
                    preempt_buffer = 5  # seconds before arrival to preempt
                    restore_buffer = 2  # seconds after passing to restore

                    # Wait until just before EV arrives
                    print(f'{self.env.now:05.1f}s: Scheduling TL preemption at {light.vertex} in '
                          f'{arrival_time - preempt_buffer} seconds for Ambulance for {direction=}')
                    # yield env.timeout(max(arrival_time - env.now - preempt_buffer, 0))
                    yield env.timeout(arrival_time - preempt_buffer)

                    print(f'{self.env.now:05.1f}s: TL preemption for {light.vertex} awoke '
                          f'for Ambulance for {direction=}')

                    light_future_state = self.get_light_state(light, direction)

                    # Light will change - extend:
                    if light_future_state == TLCState.YELLOW:
                        light.start += preempt_buffer + restore_buffer
                        print(f'{env.now:05.1f}s: Extended TL at {light.vertex} for '
                              f'Ambulance for {direction=}')
                    # Light will be red - preempt:
                    elif light_future_state == TLCState.RED:
                        light.ns_state = TLCState.GREEN if direction == 'NS' else TLCState.RED
                        light.ew_state = TLCState.GREEN if direction == 'EW' else TLCState.RED
                        light.allowed = direction
                        light.start = env.now
                        light.change_event.succeed()
                        light.change_event = env.event()
                        print(f'{env.now:05.1f}s: Preempted TL at {light.vertex} for '
                              f'Ambulance for {direction=}')

                env.process(preempt())

    def get_light_state(self, light: TrafficLight, direction: str) -> TLCState:
        '''
        Ambulance will cross this light in 5 seconds - need to determine light state:
        1) Light is and will be green - no action needed
        2) Light is green but turning to red - need to extend
        3) Light is red but turning to green - no action needed
        4) Light is and will be red - need to preempt
        '''
        # Light color for direction:
        current_color = TLCState.GREEN if light.allowed == direction else TLCState.RED

        # Will light change state?
        changing = self.env.now + 5 - light.start >= light.light_length

        if current_color == TLCState.GREEN and not changing:
            return TLCState.GREEN
        elif current_color == TLCState.GREEN and changing:
            return TLCState.YELLOW
        elif current_color == TLCState.RED and changing:
            return TLCState.GREEN
        # Must be:  current_color == TLCState.RED and not changing:
        else:
            return TLCState.RED


def get_shortest_path(graph: networkx.Graph, start_node: tuple[int, int],
                      end_node: tuple[int, int]) -> list[tuple[int, int]]:
    '''
    Computes the shortest path from start_node to end_node using the
    Bellman-Ford algorithm.  Suitable for graphs with no negative cycles.
    Complexity is O(VE) where V is the number of vertices and E is the
    number of edges.

    Args:
    * graph: A NetworkX graph representing the traffic grid
    * start_node: The starting node (tuple of coordinates)
    * end_node: The destination node (tuple of coordinates)

    Returns:
    * A list of nodes representing the shortest path from start_node to end_node
    '''

    # Initialize distances and predecessors
    distances = {node: inf for node in graph.nodes}
    predecessors = {node: None for node in graph.nodes}
    distances[start_node] = 0

    # Bellman-Ford algorithm
    for _ in range(len(graph.nodes) - 1):
        for u, v, data in graph.edges(data=True):
            weight = data.get('length', 1)  # Default to 1 if 'length' not present
            if distances[u] != inf and distances[u] + weight < distances[v]:
                distances[v] = distances[u] + weight
                predecessors[v] = u
            if distances[v] != inf and distances[v] + weight < distances[u]:
                distances[u] = distances[v] + weight
                predecessors[u] = v

    # Check for negative cycles (optional, but good practice)
    for u, v, data in graph.edges(data=True):
        weight = data.get('length', 1)
        if distances[u] != inf and distances[u] + weight < distances[v]:
            raise ValueError('Graph contains a negative cycle')
        if distances[v] != inf and distances[v] + weight < distances[u]:
            raise ValueError('Graph contains a negative cycle')

    # Reconstruct the path
    path: list[tuple[int, int]] = []
    current = end_node
    while current is not None:
        path.insert(0, current)
        current = predecessors[current]

    return path


def vehicle_generator(env: simpy.Environment, traffic_grid: networkx.Graph,
                      origins: list[tuple[str, tuple[int, int]]],
                      destinations: set[tuple[int, int]],
                      epath: list[tuple[str, tuple[int, int]]|tuple[int, int]],
                      arrival_interval: int) -> Generator[simpy.events.Event, None, None]:
    '''
    Continuously injects vehicles into the traffic grid simulation
    '''
    ambulance_in = False
    count = Counter()

    while True:
        yield env.timeout(RNG.randint(1, arrival_interval))
        if env.now > 90 and not ambulance_in:
            Vehicle(env, 'Ambulance', epath[0], epath[1], traffic_grid, emergency=True)
            count += 1
            ambulance_in = True
        else:
            origin = RNG.choice(origins)
            destination = RNG.choice(list(destinations - {origin[1]}))
            Vehicle(env, f'Car-{count}', origin, destination, traffic_grid)
            count += 1


def report_results() -> None:
    '''
    Basic reporting at conclusion of simulation
    '''
    count = Counter()

    print('\nVehicle Wait Time Summary')
    print(f'Total Vehicles entering Traffic Grid: {count}')
    print(f'Total Vehicles through Traffic Grid: {len(VEHICLE_WAIT_TIMES)}')
    print('Vehicles through Traffic Grid that waited: '
          f'{sum(1 for w in VEHICLE_WAIT_TIMES if w > 0)}')
    print('Average Wait Time for Vehicles through Traffic Grid: '
          f'{statistics.mean(VEHICLE_WAIT_TIMES):.2f}s')
    print('Max Wait Time for Vehicles through Traffic Grid: '
          f'{max(VEHICLE_WAIT_TIMES):.2f}s')

    print('\nVehicle Total Time Summary')
    print('Average Total Time for Vehicles through Traffic Grid: '
          f'{statistics.mean(VEHICLE_TOTAL_TIMES):.2f}s')
    print(f'Max Total Time for Vehicles through Traffic Grid: {max(VEHICLE_TOTAL_TIMES):.2f}s')
    print(f'Min Total Time for Vehicles through Traffic Grid: {min(VEHICLE_TOTAL_TIMES):.2f}s')


def get_direction(current_node: tuple[int, int], next_node: tuple[int, int]) -> str:
    '''
    Simple helper function to determine vehicle direction
    '''
    return 'NS' if current_node[1] == next_node[1] else 'EW'


def setup_grid(env: simpy.Environment, vertices: list[tuple[int, int]],
               edges: list[tuple[tuple[int, int], tuple[int, int]]],
               lights: list[tuple[int, int]]) -> networkx.Graph:
    '''
    Build the traffic grid for simulation
    '''
    G = networkx.Graph()
    G.add_nodes_from(vertices)
    # Note:  Using weight as distance in miles:
    G.add_edges_from(edges)
    for edge in G.edges:
        G.edges[edge]['length'] = 1  # Length in miles
        G.edges[edge]['weight'] = 1  # For future use...
    for vertex in lights:
        G.nodes[vertex]['tl'] = TrafficLight(env, vertex=vertex, debug=True)

    return G


def get_environment(size: str) -> dict[str, Any]:
    '''
    Convenience function to select a traffic grid environment
    '''
    # Small and simple 4x4 traffic grid:
    small = {
        'vertices': [
                    (0, 1), (0, 2),
            (1, 0), (1, 1), (1, 2), (1, 3),
            (2, 0), (2, 1), (2, 2), (2, 3),
                    (3, 1), (3, 2),
        ],
        'edges': [
                    ((0, 1), (1, 1)), ((0, 2), (1, 2)),
            ((1, 0), (1, 1)), ((1, 1), (1, 2)), ((1, 2), (1, 3)),
                    ((1, 1), (2, 1)), ((1, 2), (2, 2)),
            ((2, 0), (2, 1)), ((2, 1), (2, 2)), ((2, 2), (2, 3)),
                    ((2, 1), (3, 1)), ((2, 2), (3, 2)),
        ],
        'lights': [
            (1, 1), (1, 2), (2, 1), (2, 2)
        ],
        'origins': [
            ('N', (0, 1)), ('N', (0, 2)),
            ('W', (1, 0)), ('W', (2, 0)),
            ('E', (1, 3)), ('E', (2, 3)),
            ('S', (3, 1)), ('S', (3, 2)),
        ],
        'destinations': {
            (0, 1), (0, 2),
            (1, 0), (2, 0),
            (1, 3), (2, 3),
            (3, 1), (3, 2)
        },
        'emergency_path': [('S', (3, 1)), (1, 3)],
        'duration': 600
    }

    # Medium 6x6 traffic grid:
    medium = {
        'vertices': [
                    (0, 1), (0, 2), (0, 3), (0, 4),
            (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5),
            (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5),
            (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5),
            (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
                    (5, 1), (5, 2), (5, 3), (5, 4),
        ],
        'edges': [
                    ((0, 1), (1, 1)), ((0, 2), (1, 2)), ((0, 3), (1, 3)), ((0, 4), (1, 4)),
            ((1, 0), (1, 1)), ((1, 1), (1, 2)), ((1, 2), (1, 3)), ((1, 3), (1, 4)),
                ((1, 4), (1, 5)),
                    ((1, 1), (2, 1)), ((1, 2), (2, 2)), ((1, 3), (2, 3)), ((1, 4), (2, 4)),
            ((2, 0), (2, 1)), ((2, 1), (2, 2)), ((2, 2), (2, 3)), ((2, 3), (2, 4)),
                ((2, 4), (2, 5)),
                    ((2, 1), (3, 1)), ((2, 2), (3, 2)), ((2, 3), (3, 3)), ((2, 4), (3, 4)),
            ((3, 0), (3, 1)), ((3, 1), (3, 2)), ((3, 2), (3, 3)), ((3, 3), (3, 4)),
                ((3, 4), (3, 5)),
                    ((3, 1), (4, 1)), ((3, 2), (4, 2)), ((3, 3), (4, 3)), ((3, 4), (4, 4)),
            ((4, 0), (4, 1)), ((4, 1), (4, 2)), ((4, 2), (4, 3)), ((4, 3), (4, 4)),
                ((4, 4), (4, 5)),
                    ((4, 1), (5, 1)), ((4, 2), (5, 2)), ((4, 3), (5, 3)), ((4, 4), (5, 4)),
        ],
        'lights': [
            (1, 1), (1, 2), (1, 3), (1, 4),
            (2, 1), (2, 2), (2, 3), (2, 4),
            (3, 1), (3, 2), (3, 3), (3, 4),
            (4, 1), (4, 2), (4, 3), (4, 4),
        ],
        'origins': [
            ('N', (0, 1)), ('N', (0, 2)), ('N', (0, 3)), ('N', (0, 4)),
            ('W', (1, 0)), ('W', (2, 0)), ('W', (3, 0)), ('W', (4, 0)),
            ('E', (1, 5)), ('E', (2, 5)), ('E', (3, 5)), ('E', (4, 5)),
            ('S', (5, 1)), ('S', (5, 2)), ('S', (5, 3)), ('S', (5, 4)),
        ],
        'destinations': {
            (0, 1), (0, 2), (0, 3), (0, 4),
            (1, 0), (2, 0), (3, 0), (4, 0),
            (1, 5), (2, 5), (3, 5), (4, 5),
            (5, 1), (5, 2), (5, 3), (5, 4),
        },
        'emergency_path': [('S', (5, 1)), (1, 5)],
        'duration': 1080
    }

    # Large and complex 8x8 traffic grid:
    large = {
        'vertices': [
                    (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
            (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 7),
            (2, 0), (2, 1),         (2, 3), (2, 4),         (2, 6), (2, 7),
            (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7),
            (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6), (4, 7),
            (5, 0), (5, 1),         (5, 3), (5, 4),         (5, 6), (5, 7),
            (6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7),
                    (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (7, 6),
        ],
        'edges': [
                    ((0, 1), (1, 1)), ((0, 2), (1, 2)), ((0, 3), (1, 3)), ((0, 4), (1, 4)),
                        ((0, 5), (1, 5)), ((0, 6), (1, 6)),
            ((1, 0), (1, 1)), ((1, 1), (1, 2)), ((1, 2), (1, 3)), ((1, 3), (1, 4)),
                ((1, 4), (1, 5)), ((1, 5), (1, 6)), ((1, 6), (1, 7)),
                    ((1, 1), (2, 1)), ((1, 3), (2, 3)), ((1, 4), (2, 4)), ((1, 6), (2, 6)),
            ((2, 0), (2, 1)), ((2, 3), (2, 4)), ((2, 6), (2, 7)),
                    ((2, 1), (3, 1)), ((2, 3), (3, 3)), ((2, 4), (3, 4)), ((2, 6), (3, 6)),
            ((3, 0), (3, 1)), ((3, 1), (3, 2)), ((3, 2), (3, 3)), ((3, 3), (3, 4)),
                ((3, 4), (3, 5)), ((3, 5), (3, 6)), ((3, 6), (3, 7)),
                    ((3, 1), (4, 1)), ((3, 2), (4, 2)), ((3, 3), (4, 3)), ((3, 4), (4, 4)),
                        ((3, 5), (4, 5)), ((3, 6), (4, 6)),
            ((4, 0), (4, 1)), ((4, 1), (4, 2)), ((4, 2), (4, 3)), ((4, 3), (4, 4)),
                ((4, 4), (4, 5)), ((4, 5), (4, 6)), ((4, 6), (4, 7)),
                    ((4, 1), (5, 1)), ((4, 3), (5, 3)), ((4, 4), (5, 4)), ((4, 6), (5, 6)),
            ((5, 0), (5, 1)), ((5, 3), (5, 4)), ((5, 6), (5, 7)),
                ((5, 1), (6, 1)), ((5, 3), (6, 3)), ((5, 4), (6, 4)), ((5, 6), (6, 6)),
            ((6, 0), (6, 1)), ((6, 1), (6, 2)), ((6, 2), (6, 3)), ((6, 3), (6, 4)),
                ((6, 4), (6, 5)), ((6, 5), (6, 6)), ((6, 6), (6, 7)),
                    ((6, 1), (7, 1)), ((6, 2), (7, 2)), ((6, 3), (7, 3)), ((6, 4), (7, 4)),
                    ((6, 5), (7, 5)), ((6, 6), (7, 6)),
        ],
        'lights': [
            (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6),
            (2, 1),         (2, 3), (2, 4),         (2, 6),
            (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6),
            (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
            (5, 1),         (5, 3), (5, 4),         (5, 6),
            (6, 1), (6, 2), (6, 3), (6, 4), (6, 5), (6, 6),
        ],
        'origins': [
            ('N', (0, 1)), ('N', (0, 2)), ('N', (0, 3)), ('N', (0, 4)), ('N', (0, 5)),
                ('N', (0, 6)),
            ('W', (1, 0)), ('W', (2, 0)), ('W', (3, 0)), ('W', (4, 0)), ('W' , (5, 0)),
                ('W', (6, 0)),
            ('E', (1, 7)), ('E', (2, 7)), ('E', (3, 7)), ('E', (4, 7)), ('E', (5, 7)),
                ('E', (6, 7)),
            ('S', (7, 1)), ('S', (7, 2)), ('S', (7, 3)), ('S', (7, 4)), ('S', (7, 5)),
                ('S', (7, 6)),
        ],
        'destinations': {
            (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
            (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0),
            (1, 7), (2, 7), (3, 7), (4, 7), (5, 7), (6, 7),
            (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (7, 6),
        },
        'emergency_path': [('S', (7, 1)), (2, 7)],
        'duration': 1440
    }

    match size:
        case 'small':
            return small
        case 'medium':
            return medium
        case 'large':
            return large
        case _:
            raise ValueError(f'Expected traffic grid size of small, medium, or large, got: {size}')


if __name__ == '__main__':
    # Change output encoding from Windows default of cp1252 to UTF-8:
    sys.stdout.reconfigure(encoding='utf-8')

    RNG = random.SystemRandom()
    grid_env = get_environment(GRID_SIZE)
    env = simpy.Environment()

    traffic_grid = setup_grid(env, grid_env['vertices'], grid_env['edges'], grid_env['lights'])
    sim_start = env.now

    env.process(
        vehicle_generator(
            env, traffic_grid, grid_env['origins'], grid_env['destinations'],
            grid_env['emergency_path'], arrival_interval=ARRIVAL_INTERVAL
        )
    )

    # Run simulation:
    print(f'Starting traffic grid simulation - {GRID_SIZE} size - at '
          f'{datetime.now():%Y-%m-%d %H:%M:%S} for {grid_env["duration"]} seconds')
    env.run(until=grid_env['duration'])
    report_results()
