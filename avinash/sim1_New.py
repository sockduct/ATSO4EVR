#! /usr/bin/env python3.12
'''
Traffic Grid Simulation:
* Use time units in seconds
* X x X Traffic Grid represented as a graph with NetworkX
* Each intersection has a traffic light
    * Intersection probably uses queue for vehicles
* Vehicles enter the grid from all directions
* Vehicles leave the grid in all directions
* Each vehicle has a direction (which can change at intersections) and speed
* When a vehicle enters an intersection, it randomly goes left, right, or
  straight
* Later need to add traffic lights to intersections
'''


# Standard Library:
from enum import Enum
from itertools import pairwise
import random
import statistics
import sys

# Third-Party:
import networkx
import simpy


# Global Constants:
LIGHT_LENGTH = 30
# Global lists to track times
VEHICLE_WAIT_TIMES = []
VEHICLE_TOTAL_TIMES = []


class TLCState(Enum):
    RED = 0
    GREEN = 1
    YELLOW = -1

    def __repr__(self):
        return self.name


class TrafficLight:
    '''
    Traffic light states:
    * Transitions from GREEN-NS/RED-EW to RED-NS/GREEN-EW after 60 seconds
    * Transitions from RED-NS/GREEN-EW to GREEN-NS/RED-EW after 60 seconds
    * (...)

    Note:  Omitting yellow state for now
    '''
    def __init__(self, env: simpy.Environment, vertex, light_length: int=LIGHT_LENGTH,
                 debug: bool=False) -> None:
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

    def __repr__(self):
        return f'TrafficLight({self.vertex}, {self.allowed=})'

    def run(self):
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
    Track vehicle location in grid, speed, direction, entry and exit times
    * For simplicity, start with Vehicles going only in one direction
    '''
    def __init__(self, env: simpy.Environment, name: str, origin: tuple[str, tuple[int, int]],
                 destination: tuple[int, int], traffic_grid, speed: int=30,
                 emergency: bool=False) -> None:
        self.env = env
        self.name = name
        self.emergency = emergency
        self.origin = origin
        self.destination = destination
        self.path = networkx.shortest_path(traffic_grid, origin[1], destination)
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

    def __repr__(self):
        return f'Vehicle({self.name} at {self.current_node})'

    def run(self):
        self.entry_time = self.env.now
        print(f'{self.entry_time:05.1f}s: {self.name} arrives from {self.origin[0]} '
              f'at {self.origin[1]} heading to {self.destination} {self.path}')

        if self.emergency:
            print(f'{self.env.now:05.1f}s: {self.name} is an emergency vehicle - setting '
                  'up traffic light preemption...')
            self.preempt_traffic_lights()
            self.path = dp_shortest_path(traffic_grid, self.origin[1], self.destination)
            print(f"DEBUG: Regular vehicle {name} using Dijkstra's path: {self.path}")

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

    def preempt_traffic_lights(self):
        arrival_time = 0
        for current_node, next_node in pairwise(self.path):
            edge_length_miles = self.traffic_grid.edges[self.current_node, self.next_node]['length']
            arrival_time += (edge_length_miles / self.speed) * 3600

            if light := self.traffic_grid.nodes[next_node].get('tl'):
                direction = get_direction(current_node, next_node)

                def preempt(light=light, arrival_time=arrival_time,
                                        direction=direction, env=self.env):
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
                        print(f'{env.now:05.1f}s: ⚠️ Extended TL at {light.vertex} for '
                              f'Ambulance for {direction=}')
                    # Light will be red - preempt:
                    elif light_future_state == TLCState.RED:
                        light.ns_state = TLCState.GREEN if direction == 'NS' else TLCState.RED
                        light.ew_state = TLCState.GREEN if direction == 'EW' else TLCState.RED
                        light.allowed = direction
                        light.start = env.now
                        light.change_event.succeed()
                        light.change_event = env.event()
                        print(f'{env.now:05.1f}s: ⚠️ Preempted TL at {light.vertex} for '
                              f'Ambulance for {direction=}')
                env.process(preempt())

    def get_light_state(self, light, direction):
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
        elif current_color == TLCState.RED and not changing:
            return TLCState.RED


def dp_shortest_path(graph: networkx.Graph, start_node, end_node):
    """
    Computes the shortest path from start_node to end_node using the Bellman-Ford algorithm.
    Suitable for graphs with no negative cycles.

    Args:
        graph: A NetworkX graph representing the traffic grid.
        start_node: The starting node (tuple of coordinates).
        end_node: The destination node (tuple of coordinates).

    Returns:
        A list of nodes representing the shortest path from start_node to end_node.
    """

    # Initialize distances and predecessors
    distances = {node: float('inf') for node in graph.nodes}
    predecessors = {node: None for node in graph.nodes}
    distances[start_node] = 0

    # Bellman-Ford algorithm
    for _ in range(len(graph.nodes) - 1):
        for u, v, data in graph.edges(data=True):
            weight = data.get('length', 1)  # Default to 1 if 'length' not present
            if distances[u] != float('inf') and distances[u] + weight < distances[v]:
                distances[v] = distances[u] + weight
                predecessors[v] = u
            if distances[v] != float('inf') and distances[v] + weight < distances[u]:
                distances[u] = distances[v] + weight
                predecessors[u] = v

    # Check for negative cycles (optional, but good practice)
    for u, v, data in graph.edges(data=True):
        weight = data.get('length', 1)
        if distances[u] != float('inf') and distances[u] + weight < distances[v]:
            raise ValueError("Graph contains a negative cycle")
        if distances[v] != float('inf') and distances[v] + weight < distances[u]:
            raise ValueError("Graph contains a negative cycle")

    # Reconstruct the path
    path = []
    current = end_node
    while current is not None:
        path.insert(0, current)
        current = predecessors[current]

    return path


def vehicle_generator(env, traffic_grid, arrival_interval):
    ambulance_in = False
    count = 0
    roads = [
        ('N', (1, 2)), ('N', (1, 3)),
        ('W', (2, 1)), ('W', (3, 1)),
        ('E', (2, 4)), ('E', (3, 4)),
        ('S', (4, 2)), ('S', (4, 3)),
    ]
    destinations = {
        (1, 2), (1, 3),
        (2, 1), (3, 1),
        (2, 4), (3, 4),
        (4, 2), (4, 3)
    }

    while True:
        yield env.timeout(RNG.randint(1, arrival_interval))
        if env.now > 90 and not ambulance_in:
            Vehicle(env, 'Ambulance', ('S', (4, 2)), (2, 4), traffic_grid, emergency=True)
            ambulance_in = True
        else:
            origin = RNG.choice(roads)
            destination = RNG.choice(list(destinations - {origin[1]}))
            Vehicle(env, f'Car-{count}', origin, destination, traffic_grid)
            count += 1


def report_results():
    print("\n🚦 Vehicle Wait Time Summary 🚦")
    print(f"Total Vehicles: {len(VEHICLE_WAIT_TIMES)}")
    print(f"Vehicles that waited: {sum(1 for w in VEHICLE_WAIT_TIMES if w > 0)}")
    print(f"Average Wait Time: {statistics.mean(VEHICLE_WAIT_TIMES):.2f}s")
    print(f"Max Wait Time: {max(VEHICLE_WAIT_TIMES):.2f}s")

    print("\n🕒 Vehicle Total Time Summary 🕒")
    print(f"Average Total Time: {statistics.mean(VEHICLE_TOTAL_TIMES):.2f}s")
    print(f"Max Total Time: {max(VEHICLE_TOTAL_TIMES):.2f}s")
    print(f"Min Total Time: {min(VEHICLE_TOTAL_TIMES):.2f}s")


def get_direction(current_node, next_node):
    return 'NS' if current_node[1] == next_node[1] else 'EW'


def setup_grid(env: simpy.Environment) -> networkx.Graph:
    G = networkx.Graph()
    G.add_nodes_from([
                 (1, 2), (1, 3),
         (2, 1), (2, 2), (2, 3), (2, 4),
         (3, 1), (3, 2), (3, 3), (3, 4),
                 (4, 2), (4, 3),
    ])
    # Note:  Using weight as distance in miles:
    G.add_edges_from([
                ((1, 2), (2, 2)), ((1, 3), (2, 3)),
        ((2, 1), (2, 2)), ((2, 2), (2, 3)), ((2, 3), (2, 4)),
                ((2, 2), (3, 2)), ((2, 3), (3, 3)),
        ((3, 1), (3, 2)), ((3, 2), (3, 3)), ((3, 3), (3, 4)),
                ((3, 2), (4, 2)), ((3, 3), (4, 3)),
    ])
    for edge in G.edges:
        G.edges[edge]['length'] = 1  # Length in miles
        G.edges[edge]['weight'] = 1  # For future use...
    for vertex in ((2, 2), (2, 3), (3, 2), (3, 3)):
        G.nodes[vertex]['tl'] = TrafficLight(env, vertex=vertex, debug=True)

    return G


if __name__ == '__main__':
    # Change output encoding from Windows default of cp1252 to UTF-8:
    sys.stdout.reconfigure(encoding='utf-8')

    RNG = random.SystemRandom()
    env = simpy.Environment()
    traffic_grid = setup_grid(env)
    sim_start = env.now
    env.process(vehicle_generator(env, traffic_grid, arrival_interval=10))

    # Run simulation for 10 minutes:
    env.run(until=600)
    report_results()
