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

# Third-Party:
import simpy


class TrafficGrid:
    def __init__(self, env, intersections):
        self.env = env
        self.intersections = intersections

    def transit_intersection(self, vehicle):
        yield self.env.timeout(random.randint(1, 3))


class TLCState(Enum):
    RED = 0
    GREEN = 1
    YELLOW = -1

    def __repr__(self):
        return self.name


class TrafficLight:
    '''
    Traffic light states:
    * Transitions from GREEN-NS/RED-EW to YELLOW-NS/RED-EW after 55 seconds
    * Transitions from YELLOW-NS/RED-EW to RED-NS/GREEN-EW after 5 seconds
    * Transitions from RED-NS/GREEN-EW to RED-NS/YELLOW-EW after 55 seconds
    * Transitions from RED-NS/YELLOW-EW to GREEN-NS/RED-EW after 5 seconds
    * (...)
    '''
    def __init__(self, env: simpy.Environment, debug: bool=False) -> None:
        self.env = env
        self.debug = debug
        self.ns_state = TLCState.GREEN
        self.ew_state = TLCState.RED
        self.start = self.env.now
        self.init = True

    def run(self):
        while True:
            if self.init and self.debug:
                print(f'Traffic Light activated at time: {get_ellapsed_time(self.env)}')
                print(f'NS light is RED, EW light is GREEN '
                        f'({self.ns_state=}, {self.ew_state=})')
                self.init = False
            yield self.env.timeout(1)

            if (self.ns_state == TLCState.GREEN and self.ew_state == TLCState.RED
                    and self.env.now - self.start >= 55):
                self.ns_state = TLCState.YELLOW
                self.start = self.env.now
                if self.debug:
                    print(f'\nEllapsed time: {get_ellapsed_time(self.env)}')
                    print(f'Transition NS light from GREEN to YELLOW ({self.ns_state=},'
                          f' {self.ew_state=})')
            elif (self.ns_state == TLCState.YELLOW and self.ew_state == TLCState.RED
                    and self.env.now - self.start >= 5):
                self.ns_state = TLCState.RED
                self.ew_state = TLCState.GREEN
                self.start = self.env.now
                if self.debug:
                    print(f'\nEllapsed time: {get_ellapsed_time(self.env)}')
                    print(f'Transition NS light from YELLOW to RED, EW light from RED to GREEN '
                          f'({self.ns_state=}, {self.ew_state=})')
            elif (self.ns_state == TLCState.RED and self.ew_state == TLCState.GREEN
                    and self.env.now - self.start >= 55):
                self.ew_state = TLCState.YELLOW
                self.start = self.env.now
                if self.debug:
                    print(f'\nEllapsed time: {get_ellapsed_time(self.env)}')
                    print(f'Transition EW light from GREEN to YELLOW ({self.ns_state=}, '
                          f'{self.ew_state=})')
            elif (self.ns_state == TLCState.RED and self.ew_state == TLCState.YELLOW
                    and self.env.now - self.start >= 5):
                self.ns_state = TLCState.GREEN
                self.ew_state = TLCState.RED
                self.start = self.env.now
                if self.debug:
                    print(f'\nEllapsed time: {get_ellapsed_time(self.env)}')
                    print(f'Transition NS light from RED to GREEN, EW light from YELLOW to RED '
                          f'({self.ns_state=}, {self.ew_state=})')
            # Would need more logic to implement this:
            # else:
            #     raise ValueError(f'Invalid traffic light state: {self.ns_state=}, {self.ew_state=}')


class Vehicle:
    '''
    Track vehicle location in grid, speed, direction, entry and exit times
    * For simplicity, start with Vehicles going only in one direction
    '''
    def __init__(self, env: simpy.Environment, speed: int=30) -> None:
        self.env = env
        self.speed = speed
        # When Vehicle enters grid (e.g., self.env.now)
        self.entry_time = None
        # When Vehicle leaves grid:
        self.exit_time = None


def get_ellapsed_time(env: simpy.Environment) -> str:
    seconds = env.now - sim_start
    minutes, seconds = divmod(seconds, 60)
    minutestr = 'minute' if minutes == 1 else 'minutes'
    secondstr = 'second' if seconds == 1 else 'seconds'
    return f'{minutes:.1f} {minutestr}, {seconds:.1f} {secondstr}'


def run_grid(env, grid, vehicles):
    rng = random.SystemRandom()
    active_vehicles = 0

    # Car enters the grid every 1 - 60 seconds
    while True:
        yield env.timeout(rng.randint(1, 60)/60)
        if active_vehicles < 25:
            vehicle = Vehicle()
            active_vehicles += 1
        env.process(grid.transit_intersection(vehicle))


if __name__ == '__main__':
    env = simpy.Environment()
    light = TrafficLight(env, debug=True)
    sim_start = env.now
    env.process(light.run())
    # Run simulation for 10 minutes:
    env.run(until=600)
