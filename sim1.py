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
    def __init__(self, env: simpy.Environment, debug: bool=False) -> None:
        self.env = env
        self.debug = debug
        self.ns_state = TLCState.GREEN
        self.ew_state = TLCState.RED
        self.start = self.env.now
        self.init = True
        self.change_event = env.event()
        env.process(self.run())

    def run(self):
        while True:
            if self.init and self.debug:
                print(f'{self.env.now:05.1f}s: Traffic Light Initialized ({self.ns_state=}, '
                      f'{self.ew_state=})')
                self.init = False
            yield self.env.timeout(1)

            if (self.ns_state == TLCState.GREEN and self.ew_state == TLCState.RED
                    and self.env.now - self.start >= LIGHT_LENGTH):
                self.change_event.succeed()
                self.change_event = self.env.event()
                self.ns_state = TLCState.RED
                self.ew_state = TLCState.GREEN
                self.start = self.env.now
                if self.debug:
                    print(f'{self.env.now:05.1f}s: Traffic Light Transition ({self.ns_state=}, '
                          f'{self.ew_state=})')
            elif (self.ns_state == TLCState.RED and self.ew_state == TLCState.GREEN
                    and self.env.now - self.start >= LIGHT_LENGTH):
                self.ns_state = TLCState.GREEN
                self.ew_state = TLCState.RED
                self.start = self.env.now
                if self.debug:
                    print(f'{self.env.now:05.1f}s: Traffic Light Transition ({self.ns_state=}, '
                          f'{self.ew_state=})')


class Vehicle:
    '''
    Track vehicle location in grid, speed, direction, entry and exit times
    * For simplicity, start with Vehicles going only in one direction
    '''
    def __init__(self, env: simpy.Environment, name: str, origin: str, intersection,
                 road_length: int, speed: int=30) -> None:
        self.env = env
        self.name = name
        self.origin = origin
        self.direction = opposite_direction(origin)
        self.intersection = intersection
        self.road_length = road_length
        self.speed = speed
        self.wait_time = 0
        # When Vehicle enters grid (e.g., self.env.now)
        self.entry_time = None
        # When Vehicle leaves grid:
        self.exit_time = None
        env.process(self.run())

    def run(self):
        self.entry_time = self.env.now
        print(f'{self.entry_time:05.1f}s: {self.name} arrives from {self.origin}')

        if (
            (self.direction in ('N', 'S') and self.intersection.light.ns_state == TLCState.RED) or
            (self.direction in ('E', 'W') and self.intersection.light.ew_state == TLCState.RED)
        ):
            red_arrival = self.env.now
            print(f'{self.env.now:05.1f}s: {self.name} waits at red')
            yield self.intersection.light.change_event
            self.wait_time = self.env.now - red_arrival
            print(f'{self.env.now:05.1f}s: {self.name} waited {self.wait_time:.1f}s')

        VEHICLE_WAIT_TIMES.append(self.wait_time)

        print(f'{self.env.now:05.1f}s: {self.name} crosses intersection')
        yield self.env.timeout(self.road_length)

        self.exit_time = self.env.now
        total_time = self.exit_time - self.entry_time
        VEHICLE_TOTAL_TIMES.append(total_time)
        print(f'{self.exit_time:05.1f}s: {self.name} exits simulation after {total_time:.1f}s')


class Intersection:
    def __init__(self, env, road_length, debug: bool=False) -> None:
        self.env = env
        self.light = TrafficLight(env, debug)
        self.road_length = road_length


def vehicle_generator(env, intersection, arrival_interval):
    count = 0
    roads = ['N', 'S', 'E', 'W']
    while True:
        yield env.timeout(random.randint(1, arrival_interval))
        origin = random.choice(roads)
        Vehicle(env, f'Car{count}', origin, intersection, intersection.road_length)
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


if __name__ == '__main__':
    rng = random.SystemRandom()
    env = simpy.Environment()
    sim_start = env.now
    intersection = Intersection(env, road_length=20, debug=True)
    env.process(vehicle_generator(env, intersection, arrival_interval=10))

    # Run simulation for 10 minutes:
    env.run(until=100)
    report_results()
