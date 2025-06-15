#! /usr/bin/env python3.12


import simpy
import random


class TrafficLight:
    def __init__(self, env, green_ns, green_ew):
        self.env = env
        self.green_ns = green_ns
        self.green_ew = green_ew
        self.direction = 'NS'  # Start with NS green
        self.event_ns = env.event()
        self.event_ew = env.event()
        env.process(self.run())

    def run(self):
        while True:
            # NS green, EW red
            self.direction = 'NS'
            print(f"{self.env.now:.1f}: Light is GREEN for NS")
            self.event_ns.succeed()
            self.event_ns = self.env.event()
            yield self.env.timeout(self.green_ns)

            # EW green, NS red
            self.direction = 'EW'
            print(f"{self.env.now:.1f}: Light is GREEN for EW")
            self.event_ew.succeed()
            self.event_ew = self.env.event()
            yield self.env.timeout(self.green_ew)

    def can_proceed(self, direction):
        """Return event the vehicle should wait on if light is red"""
        if (direction in ['N', 'S'] and self.direction == 'NS') or \
           (direction in ['E', 'W'] and self.direction == 'EW'):
            return None  # green, proceed
        return self.event_ns if direction in ['N', 'S'] else self.event_ew


class Vehicle:
    def __init__(self, env, name, direction, traffic_light):
        self.env = env
        self.name = name
        self.direction = direction
        self.traffic_light = traffic_light
        env.process(self.run())

    def run(self):
        print(f"{self.env.now:.1f}: {self.name} approaches from {self.direction}")
        wait_event = self.traffic_light.can_proceed(self.direction)
        if wait_event:
            print(f"{self.env.now:.1f}: {self.name} waits at red light from {self.direction}")
            yield wait_event
        print(f"{self.env.now:.1f}: {self.name} passes through intersection from {self.direction}")
        travel_time = random.uniform(3, 6)
        yield self.env.timeout(travel_time)
        print(f"{self.env.now:.1f}: {self.name} exits the simulation from {self.direction}")


def vehicle_generator(env, traffic_light):
    vehicle_count = 0
    directions = ['N', 'S', 'E', 'W']
    while True:
        yield env.timeout(random.randint(1, 10))
        direction = random.choice(directions)
        vehicle = Vehicle(env, f"Vehicle {vehicle_count}", direction, traffic_light)
        vehicle_count += 1


def main():
    random.seed(42)
    env = simpy.Environment()
    traffic_light = TrafficLight(env, green_ns=15, green_ew=10)
    env.process(vehicle_generator(env, traffic_light))
    env.run(until=60)


if __name__ == '__main__':
    main()
