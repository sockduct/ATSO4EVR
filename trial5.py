#! /usr/bin/env python3.12


import simpy
import random
import statistics

# Global lists to track times
vehicle_wait_times = []
vehicle_total_times = []


class TrafficLight:
    def __init__(self, env, green_duration, red_duration):
        self.env = env
        self.green_duration = green_duration
        self.red_duration = red_duration
        self.allow_ns = True
        self.change_event = env.event()
        env.process(self.run())

    def run(self):
        while True:
            self.allow_ns = True
            self.change_event.succeed()
            self.change_event = self.env.event()
            yield self.env.timeout(self.green_duration)

            self.allow_ns = False
            self.change_event.succeed()
            self.change_event = self.env.event()
            yield self.env.timeout(self.red_duration)


class Vehicle:
    def __init__(self, env, name, origin, intersection, road_length):
        self.env = env
        self.name = name
        self.origin = origin
        self.intersection = intersection
        self.road_length = road_length
        env.process(self.run())

    def run(self):
        enter_time = self.env.now
        print(f'{enter_time:.1f}s: {self.name} arrives from {self.origin}')

        heading_ns = self.origin in ('N', 'S')
        wait_time = 0

        if heading_ns != self.intersection.light.allow_ns:
            red_arrival = self.env.now
            print(f'{self.env.now:.1f}s: {self.name} waits at red')
            yield self.intersection.light.change_event
            wait_time = self.env.now - red_arrival
            print(f'{self.env.now:.1f}s: {self.name} waited {wait_time:.1f}s')

        vehicle_wait_times.append(wait_time)

        print(f'{self.env.now:.1f}s: {self.name} crosses intersection')
        yield self.env.timeout(self.road_length)

        exit_time = self.env.now
        total_time = exit_time - enter_time
        vehicle_total_times.append(total_time)
        print(f'{exit_time:.1f}s: {self.name} exits simulation after {total_time:.1f}s')


class Intersection:
    def __init__(self, env, green_duration, red_duration, road_length):
        self.env = env
        self.light = TrafficLight(env, green_duration, red_duration)
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
    print(f"Total Vehicles: {len(vehicle_wait_times)}")
    print(f"Vehicles that waited: {sum(1 for w in vehicle_wait_times if w > 0)}")
    print(f"Average Wait Time: {statistics.mean(vehicle_wait_times):.2f}s")
    print(f"Max Wait Time: {max(vehicle_wait_times):.2f}s")

    print("\n🕒 Vehicle Total Time Summary 🕒")
    print(f"Average Total Time: {statistics.mean(vehicle_total_times):.2f}s")
    print(f"Max Total Time: {max(vehicle_total_times):.2f}s")
    print(f"Min Total Time: {min(vehicle_total_times):.2f}s")


def main(sim_time=100):
    random.seed(42)
    env = simpy.Environment()
    intersection = Intersection(env, green_duration=30, red_duration=30, road_length=20)
    env.process(vehicle_generator(env, intersection, arrival_interval=10))
    env.run(until=sim_time)
    report_results()


if __name__ == '__main__':
    main()
