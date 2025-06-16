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
import random
import statistics

# Third-Party:
import networkx
import simpy


# Global Constants:
LIGHT_LENGTH = 60
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
    def __init__(self, env: simpy.Environment, vertex, debug: bool=False) -> None:
        self.env = env
        self.vertex = vertex
        self.debug = debug
        self.ns_state = TLCState.GREEN
        self.ew_state = TLCState.RED
        self.allowed = 'NS'
        self.start = self.env.now
        self.init = True
        self.change_event = env.event()
        env.process(self.run())

    def run(self):
        while True:
            if self.init and self.debug:
                print(f'{self.env.now:05.1f}s: Traffic Light {self.vertex} Initialized '
                      f'({self.ns_state=}, {self.ew_state=}, {self.allowed=})')
                self.init = False
            yield self.env.timeout(1)

            if (self.ns_state == TLCState.GREEN and self.ew_state == TLCState.RED
                    and self.env.now - self.start >= LIGHT_LENGTH):
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
                    and self.env.now - self.start >= LIGHT_LENGTH):
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
                 destination: tuple[int, int], traffic_grid, speed: int=30) -> None:
        self.env = env
        self.name = name
        self.origin = origin
        self.destination = destination
        self.path = networkx.shortest_path(traffic_grid, origin[1], destination)
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

    def run(self):
        self.entry_time = self.env.now
        print(f'{self.entry_time:05.1f}s: {self.name} arrives from {self.origin[0]} '
              f'at {self.origin[1]} heading to {self.destination}')

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
            print(f'{self.env.now:05.1f}s: {self.name} reaches {self.current_node}')

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


def vehicle_generator(env, traffic_grid, arrival_interval):
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
        origin = RNG.choice(roads)
        destination = RNG.choice(list(destinations - {origin[1]}))
        Vehicle(env, f'Car{count}', origin, destination, traffic_grid)
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


def opposite_direction(direction: str) -> str:
    match direction:
        case 'N': return 'S'
        case 'S': return 'N'
        case 'E': return 'W'
        case 'W': return 'E'
        case _: raise ValueError(f'Expected direction to be N, S, E or W, got {direction}')


def get_ellapsed_time(env: simpy.Environment) -> str:
    seconds = env.now - sim_start
    minutes, seconds = divmod(seconds, 60)
    minutestr = 'minute' if minutes == 1 else 'minutes'
    secondstr = 'second' if seconds == 1 else 'seconds'
    return f'{minutes:.1f} {minutestr}, {seconds:.1f} {secondstr}'


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
    RNG = random.SystemRandom()
    env = simpy.Environment()
    traffic_grid = setup_grid(env)
    sim_start = env.now
    env.process(vehicle_generator(env, traffic_grid, arrival_interval=120))

    # Run simulation for 10 minutes:
    env.run(until=600)
    report_results()
