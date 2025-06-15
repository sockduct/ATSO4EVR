#! /usr/bin/env python3.12


import simpy
import random


class TrafficLight:
    """
    Alternates between allowing NS traffic (north/south) and EW traffic (east/west).
    """
    def __init__(self, env, green_duration, red_duration):
        self.env = env
        self.green_duration = green_duration
        self.red_duration = red_duration
        self.allow_ns = True  # start with NS green
        self.change_event = env.event()
        env.process(self.run())

    def run(self):
        while True:
            # NS green, EW red
            self.allow_ns = True
            self.change_event.succeed()          # wake waiting vehicles
            self.change_event = self.env.event() # reset for next phase
            yield self.env.timeout(self.green_duration)

            # EW green, NS red
            self.allow_ns = False
            self.change_event.succeed()
            self.change_event = self.env.event()
            yield self.env.timeout(self.red_duration)


class Vehicle:
    """
    A vehicle arrives on one of four roads, waits for green, crosses, then leaves.
    """
    def __init__(self, env, name, origin, intersection, road_length):
        self.env = env
        self.name = name
        self.origin = origin        # one of 'N','S','E','W'
        self.intersection = intersection
        self.road_length = road_length
        env.process(self.run())

    def run(self):
        arrive = self.env.now
        print(f'{arrive:.1f}s: {self.name} arrives from {self.origin}')

        # Wait until light is green for our direction
        heading_ns = self.origin in ('N', 'S')
        if heading_ns != self.intersection.light.allow_ns:
            print(f'{self.env.now:.1f}s: {self.name} waits at red')
            yield self.intersection.light.change_event

        # Once green
        print(f'{self.env.now:.1f}s: {self.name} crosses intersection')
        # Simulate travel to end of road
        yield self.env.timeout(self.road_length)
        print(f'{self.env.now:.1f}s: {self.name} exits simulation')


class Intersection:
    """
    Holds the traffic light and road lengths.
    """
    def __init__(self, env, green_duration, red_duration, road_length):
        self.env = env
        self.light = TrafficLight(env, green_duration, red_duration)
        self.road_length = road_length


def vehicle_generator(env, intersection, arrival_interval):
    """Spawns vehicles every 1-arrival_interval seconds on a random road."""
    count = 0
    roads = ['N', 'S', 'E', 'W']
    while True:
        yield env.timeout(random.randint(1, arrival_interval))
        origin = random.choice(roads)
        Vehicle(env,
                name=f'Car{count}',
                origin=origin,
                intersection=intersection,
                road_length=intersection.road_length)
        count += 1


def main(sim_time=100):
    random.seed(42)
    env = simpy.Environment()
    # traffic light: 30s green, 30s red; road length beyond intersection is 20s travel
    intersection = Intersection(env, green_duration=30, red_duration=30, road_length=20)
    env.process(vehicle_generator(env, intersection, arrival_interval=10))
    env.run(until=sim_time)


if __name__ == '__main__':
    main()
