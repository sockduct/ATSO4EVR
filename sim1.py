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
from math import inf, sqrt
import random
import statistics
import sys

# Third-Party:
import networkx
import numpy as np
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
        # self.path = dynamic_routing(traffic_grid, origin[1], destination)
        # print(f'{self.origin=}, {self.destination=}, {self.path=}')
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

        ### Debugging:
        # print(f'Original path:  {self.old_path}\nNew path:  {self.path}')

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


def dynamic_routing(traffic_grid, start, destination):
    # Grid length or number of vertices must be a power of 2:
    # grid = list(traffic_grid.nodes)
    grid = [
        (0, 0), (0, 1), (0, 2), (0, 3),
        (1, 0), (1, 1), (1, 2), (1, 3),
        (2, 0), (2, 1), (2, 2), (2, 3),
        (3, 0), (3, 1), (3, 2), (3, 3),
    ]
    edges = list(traffic_grid.edges)

    # grid = np.reshape(grid, (4, -1))
    grid = np.array(grid)
    cols = 4
    grid = [grid[i:i + cols] for i in range(0, len(grid), cols)]
    n, m = len(grid), len(grid[0])
    dp = [[inf] * m for _ in range(n)]
    parent = [[None for _ in range(m)] for _ in range(n)]  # To track path

    # Debugging:
    print(f'{start=}, {destination=}\n')

    dp[start[0]][start[1]] = 0

    for i in range(n):
        for j in range(m):
            if dp[i][j] == inf:
                continue
            for ni, nj in list(traffic_grid.edges((i, j))):
                print(f'{ni=}, {nj=}')
                # immediate_cost = traffic_grid.edges[(i, j), (ni, nj)]['weight']
                immediate_cost = traffic_grid.edges[ni, nj]['weight']
                new_cost = dp[ni[0]][ni[1]] + immediate_cost
                if new_cost < dp[nj[0]][nj[1]]:
                    dp[nj[0]][nj[1]] = new_cost
                    parent[nj[0]][nj[1]] = ni  # Track where we came from

    print(f'{dp=}\n{parent=}')

    # Reconstruct path from destination to start
    path = []
    curr = destination
    while curr is not None:
        path.append(curr)
        curr = parent[curr[0]][curr[1]]
    path.reverse()  # Reverse the path to get start -> destination

    print(f'dynamic_routing:  cost is {dp[destination[0]][destination[1]]} and {path=}')
    return path


def vehicle_generator(env, traffic_grid, origins, destinations, epath, arrival_interval):
    ambulance_in = False
    count = 0

    while True:
        yield env.timeout(RNG.randint(1, arrival_interval))
        if env.now > 90 and not ambulance_in:
            Vehicle(env, 'Ambulance', epath[0], epath[1], traffic_grid, emergency=True)
            ambulance_in = True
        else:
            origin = RNG.choice(origins)
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


def setup_grid(env: simpy.Environment, vertices, edges, lights) -> networkx.Graph:
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


if __name__ == '__main__':
    # Change output encoding from Windows default of cp1252 to UTF-8:
    sys.stdout.reconfigure(encoding='utf-8')

    RNG = random.SystemRandom()
    env = simpy.Environment()

    '''
    vertices = [
                (0, 1), (0, 2),
        (1, 0), (1, 1), (1, 2), (1, 3),
        (2, 0), (2, 1), (2, 2), (2, 3),
                (3, 1), (3, 2),
    ]
    edges = [
                ((0, 1), (1, 1)), ((0, 2), (1, 2)),
        ((1, 0), (1, 1)), ((1, 1), (1, 2)), ((1, 2), (1, 3)),
                ((1, 1), (2, 1)), ((1, 2), (2, 2)),
        ((2, 0), (2, 1)), ((2, 1), (2, 2)), ((2, 2), (2, 3)),
                ((2, 1), (3, 1)), ((2, 2), (3, 2)),
    ]
    lights = [
        (1, 1), (1, 2), (2, 1), (2, 2)
    ]
    origins = [
        ('N', (0, 1)), ('N', (0, 2)),
        ('W', (1, 0)), ('W', (2, 0)),
        ('E', (1, 3)), ('E', (2, 3)),
        ('S', (3, 1)), ('S', (3, 2)),
    ]
    destinations = {
        (0, 1), (0, 2),
        (1, 0), (2, 0),
        (1, 3), (2, 3),
        (3, 1), (3, 2)
    }
    emergency_path = [('S', (3, 1)), (1, 3)]
    duration = 600
    '''
    vertices = [
                (0, 1), (0, 2), (0, 3), (0, 4),
        (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5),
        (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5),
        (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5),
        (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
                (5, 1), (5, 2), (5, 3), (5, 4),
    ]
    edges = [
                ((0, 1), (1, 1)), ((0, 2), (1, 2)), ((0, 3), (1, 3)), ((0, 4), (1, 4)),
        ((1, 0), (1, 1)), ((1, 1), (1, 2)), ((1, 2), (1, 3)), ((1, 3), (1, 4)), ((1, 4), (1, 5)),
                ((1, 1), (2, 1)), ((1, 2), (2, 2)), ((1, 3), (2, 3)), ((1, 4), (2, 4)),
        ((2, 0), (2, 1)), ((2, 1), (2, 2)), ((2, 2), (2, 3)), ((2, 3), (2, 4)), ((2, 4), (2, 5)),
                ((2, 1), (3, 1)), ((2, 2), (3, 2)), ((2, 3), (3, 3)), ((2, 4), (3, 4)),
        ((3, 0), (3, 1)), ((3, 1), (3, 2)), ((3, 2), (3, 3)), ((3, 3), (3, 4)), ((3, 4), (3, 5)),
                ((3, 1), (4, 1)), ((3, 2), (4, 2)), ((3, 3), (4, 3)), ((3, 4), (4, 4)),
        ((4, 0), (4, 1)), ((4, 1), (4, 2)), ((4, 2), (4, 3)), ((4, 3), (4, 4)), ((4, 4), (4, 5)),
                ((4, 1), (5, 1)), ((4, 2), (5, 2)), ((4, 3), (5, 3)), ((4, 4), (5, 4)),
    ]
    lights = [
        (1, 1), (1, 2), (1, 3), (1, 4),
        (2, 1), (2, 2), (2, 3), (2, 4),
        (3, 1), (3, 2), (3, 3), (3, 4),
        (4, 1), (4, 2), (4, 3), (4, 4),
    ]
    origins = [
        ('N', (0, 1)), ('N', (0, 2)), ('N', (0, 3)), ('N', (0, 4)),
        ('W', (1, 0)), ('W', (2, 0)), ('W', (3, 0)), ('W', (4, 0)),
        ('E', (1, 5)), ('E', (2, 5)), ('E', (3, 5)), ('E', (4, 5)),
        ('S', (5, 1)), ('S', (5, 2)), ('S', (5, 3)), ('S', (5, 4)),
    ]
    destinations = {
        (0, 1), (0, 2), (0, 3), (0, 4),
        (1, 0), (2, 0), (3, 0), (4, 0),
        (1, 5), (2, 5), (3, 5), (4, 5),
        (5, 1), (5, 2), (5, 3), (5, 4),
    }
    emergency_path = [('S', (5, 1)), (1, 5)]
    duration = 1080

    traffic_grid = setup_grid(env, vertices, edges, lights)
    sim_start = env.now

    # Debug:
    # Vehicle(env, f'Car-0', ('E', (1, 1)), (2, 0), traffic_grid)

    env.process(
        vehicle_generator(
            env, traffic_grid, origins, destinations, emergency_path, arrival_interval=10
        )
    )

    # Run simulation for 10 minutes:
    env.run(until=duration)
    report_results()
