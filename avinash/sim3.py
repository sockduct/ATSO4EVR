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
import math
import random
import statistics
import sys
import heapq # For Dijkstra's priority queue

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
                 emergency: bool=False, floyd_paths=None) -> None:
        self.env = env
        self.name = name
        self.emergency = emergency
        self.origin = origin
        self.destination = destination
        self.traffic_grid = traffic_grid
        self.speed = speed
        self.wait_time = 0
        # When Vehicle enters grid (e.g., self.env.now)
        self.entry_time = None
        # When Vehicle leaves grid:
        self.exit_time = None

        if self.emergency and floyd_paths:
            # Use pre-computed Floyd-Warshall path for emergency vehicles
            self.path = reconstruct_path_floyd(
                floyd_paths, origin[1], destination
            )
            print(f"DEBUG: Emergency vehicle {name} using Floyd-Warshall path: {self.path}")
        else:
            # Use Dijkstra's algorithm for regular vehicles
            self.path = dijkstra_shortest_path(traffic_grid, origin[1], destination)
            print(f"DEBUG: Regular vehicle {name} using Dijkstra's path: {self.path}")
        
        self.previous_node = None
        self.current_node = origin[1]
        self.next_index = 1
        # Ensure path is not empty before accessing next_index
        if len(self.path) > 1:
            self.next_node = self.path[self.next_index]
        else:
            self.next_node = self.destination # If path has only origin and destination, next_node is destination

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

        while True:
            # Check if current node is intersection with traffic light:
            if light := self.traffic_grid.nodes[self.current_node].get('tl'):
                # Ensure previous_node is set before calling get_direction for the first intersection
                if self.previous_node is None:
                    # For the first intersection, assume it's entered from a "virtual" previous node
                    # aligned with the path's initial direction.
                    # This might need refinement based on exact entry points for a real simulation.
                    # For now, we'll assume the light check only happens after moving from the first segment.
                    pass
                else:
                    direction_of_approach = get_direction(self.previous_node, self.current_node)
                    if direction_of_approach != light.allowed:
                        red_arrival = self.env.now
                        print(f'{self.env.now:05.1f}s: {self.name} waits at red light {light.vertex}')

                        yield light.change_event

                        self.wait_time += (self.env.now - red_arrival) # Accumulate wait time
                        print(f'{self.env.now:05.1f}s: {self.name} waited {self.env.now - red_arrival:.1f}s at {light.vertex}')

                print(f'{self.env.now:05.1f}s: {self.name} crosses intersection with traffic light {light.vertex}')

            # Travel time to next node in path:
            # Check if self.next_node is within the path bounds before accessing it
            if self.next_index >= len(self.path):
                # This means we've reached the destination and there's no next node in the path.
                # This 'break' should ideally happen before trying to calculate duration for a non-existent next_node.
                break # Exit the loop if destination is reached
            
            duration = (
                (self.traffic_grid.edges[self.current_node, self.next_node]['length'] / self.speed)
                * 3600
            )

            yield self.env.timeout(duration)

            self.previous_node = self.current_node
            self.current_node = self.next_node
            
            # Check if destination reached before updating next_node
            if self.current_node == self.destination:
                break # Exit the loop if destination is reached

            self.next_index += 1
            
            # Check if next_index is within bounds before accessing self.path
            next_node_display = (
                self.path[self.next_index]
                if self.next_index < len(self.path) else 'Exit'
            )
            
            direction = get_direction(self.previous_node, self.current_node)
            print(f'{self.env.now:05.1f}s: {self.name} reaches {self.current_node} from '
                  f'{self.previous_node} going to {next_node_display}, {direction=}')

            self.next_node = self.path[self.next_index]


        VEHICLE_WAIT_TIMES.append(self.wait_time)

        self.exit_time = self.env.now
        total_time = self.exit_time - self.entry_time
        VEHICLE_TOTAL_TIMES.append(total_time)
        print(f'{self.exit_time:05.1f}s: {self.name} exits simulation after {total_time:.1f}s')

    def preempt_traffic_lights(self):
        # Calculate cumulative travel time to each intersection on the path
        # from the vehicle's current position (entry point).
        cumulative_travel_time = 0
        # Start from the current node's next segment
        path_segments = list(pairwise(self.path))

        for current_node_on_path, next_node_on_path in path_segments:
            # Only consider segments where current_node_on_path is the current_node of the vehicle
            # or it's a future segment
            # Note: self.current_node might change during simulation, so this check needs to be dynamic or based on initial path segments
            # For simplicity, we'll iterate through all path segments and schedule preemption
            # based on calculated arrival times *from the start of the vehicle's journey*.
            
            edge_length_miles = self.traffic_grid.edges[current_node_on_path, next_node_on_path]['length']
            segment_travel_time_seconds = (edge_length_miles / self.speed) * 3600
            cumulative_travel_time += segment_travel_time_seconds

            if light := self.traffic_grid.nodes[next_node_on_path].get('tl'):
                direction = get_direction(current_node_on_path, next_node_on_path)

                def preempt(light=light, arrival_time_from_entry=cumulative_travel_time,
                            direction=direction, env=self.env, vehicle_name=self.name):
                    preempt_buffer = 5  # seconds before arrival to preempt
                    restore_buffer = 2  # seconds after passing to restore

                    # Calculate the absolute simulation time for preemption
                    preempt_sim_time = self.entry_time + arrival_time_from_entry - preempt_buffer
                    # Ensure we don't try to go back in time
                    yield env.timeout(max(0, preempt_sim_time - env.now))

                    print(f'{env.now:05.1f}s: TL preemption for {light.vertex} awoke for {vehicle_name} for {direction=}')

                    # Recalculate light state *at the time of preemption*
                    # rather than using get_light_state from initial scheduling
                    current_light_color = TLCState.GREEN if light.allowed == direction else TLCState.RED
                    time_elapsed_in_cycle = env.now - light.start
                    will_change_soon = light.light_length - time_elapsed_in_cycle <= preempt_buffer + restore_buffer # A heuristic

                    preempted = False
                    if current_light_color == TLCState.GREEN and not will_change_soon:
                        # Light is green and will remain green, no preemption needed
                        pass
                    elif current_light_color == TLCState.GREEN and will_change_soon:
                        # Light is green but turning red, extend the green phase
                        # Extend the current green phase to ensure EV passes and restore buffer
                        # Calculate needed extension: how long until EV passes current light from now + restore buffer
                        needed_extension = (self.entry_time + arrival_time_from_entry - env.now) + restore_buffer
                        light.light_length = light.light_length + needed_extension # Extend the cycle length
                        print(f'{env.now:05.1f}s: ⚠️ Extended TL at {light.vertex} for {vehicle_name} for {direction=} to new length {light.light_length}s')
                        preempted = True
                    elif current_light_color == TLCState.RED and will_change_soon:
                        # Light is red but will turn green naturally, no preemption needed
                        pass
                    elif current_light_color == TLCState.RED and not will_change_soon:
                        # Light is red and will remain red, force to green
                        # Save original state for restoration
                        original_ns_state = light.ns_state
                        original_ew_state = light.ew_state
                        original_allowed = light.allowed
                        original_start_time = light.start
                        # Calculate elapsed time within the current light cycle for restoration
                        elapsed_since_start = env.now - light.start

                        light.ns_state = TLCState.GREEN if direction == 'NS' else TLCState.RED
                        light.ew_state = TLCState.GREEN if direction == 'EW' else TLCState.RED
                        light.allowed = direction
                        light.start = env.now
                        light.change_event.succeed()
                        light.change_event = env.event()
                        print(f'{env.now:05.1f}s: ⚠️ Preempted TL at {light.vertex} for {vehicle_name} for {direction=}')
                        preempted = True

                        # Schedule restoration if preemption occurred
                        # Wait until after EV passes plus restore buffer
                        # Calculate when the vehicle is expected to pass this light
                        passage_sim_time = self.entry_time + arrival_time_from_entry + restore_buffer
                        yield env.timeout(max(0, passage_sim_time - env.now))

                        # Restore original state if it changed due to preemption
                        if (light.ns_state, light.ew_state, light.allowed) != (
                            original_ns_state, original_ew_state, original_allowed
                        ):
                            light.ns_state = original_ns_state
                            light.ew_state = original_ew_state
                            light.allowed = original_allowed
                            # Restore original start time relative to current time to maintain cycle phase
                            light.start = env.now - elapsed_since_start
                            print(f'{env.now:05.1f}s: ✅ Restored TL at {light.vertex} to {light.allowed=} after {vehicle_name}')
                            # Trigger a change event if the state was truly reverted, so waiting cars react
                            light.change_event.succeed()
                            light.change_event = env.event()
                    
                self.env.process(preempt())

    def get_light_state(self, light, direction):
        '''
        Ambulance will cross this light in 5 seconds - need to determine light state:
        1) Light is and will be green - no action needed
        2) Light is green but turning to red - need to extend
        3) Light is red but turning to green - no action needed (it will become green naturally)
        4) Light is and will be red - need to preempt
        '''
        # The time from the light's last state change until the vehicle arrives
        time_until_arrival_at_light_check = light.start + light.light_length - self.env.now

        # Light color for direction at arrival time:
        if light.allowed == direction: # Currently green for vehicle's direction
            if time_until_arrival_at_light_check > 5: # Still green when vehicle arrives in 5s
                return TLCState.GREEN
            else: # Turns red within 5 seconds of arrival
                return TLCState.YELLOW # Represents "green but turning red"
        else: # Currently red for vehicle's direction
            if time_until_arrival_at_light_check <= 5: # Turns green within 5 seconds of arrival (it's the opposite light's turn to go red)
                return TLCState.GREEN # Represents "red but turning green" (will be green when EV arrives)
            else: # Still red when vehicle arrives
                return TLCState.RED


def floyd_warshall_shortest_paths(graph: networkx.Graph):
    """
    Implements the Floyd-Warshall algorithm to find all-pairs shortest paths
    and their next hops.

    Args:
        graph: A NetworkX graph where edges have a 'length' attribute.

    Returns:
        A tuple (distances, next_hops) where:
        distances: dict of dicts, distances[u][v] is the shortest distance from u to v.
        next_hops: dict of dicts, next_hops[u][v] is the next node on the shortest
                   path from u to v.
    """
    nodes = list(graph.nodes)
    num_nodes = len(nodes)
    # No need for node_to_idx and idx_to_node mappings if using node objects directly as keys.

    # Initialize distance matrix
    # distances[u][v] will store the shortest distance from node u to node v
    distances = {u: {v: float('inf') for v in nodes} for u in nodes}
    # next_hops[u][v] will store the next node on the shortest path from node u to node v
    next_hops = {u: {v: None for v in nodes} for u in nodes}

    # Initialize distances and next_hops with direct edge weights
    for u in nodes:
        distances[u][u] = 0  # Distance from a node to itself is 0

    for u, v, data in graph.edges(data=True):
        weight = data.get('length', 1)  # Default to 1 if 'length' not present
        distances[u][v] = weight
        distances[v][u] = weight  # Assuming undirected graph for symmetric distances
        next_hops[u][v] = v
        next_hops[v][u] = u

    # Apply the Floyd-Warshall algorithm
    for k_node in nodes: # Intermediate node
        for i_node in nodes: # Source node
            for j_node in nodes: # Destination node
                if distances[i_node][j_node] > distances[i_node][k_node] + distances[k_node][j_node]:
                    distances[i_node][j_node] = distances[i_node][k_node] + distances[k_node][j_node]
                    next_hops[i_node][j_node] = next_hops[i_node][k_node]
    
    return distances, next_hops


def reconstruct_path_floyd(next_hops, start_node, end_node):
    """
    Reconstructs the shortest path from start_node to end_node using the
    next_hops matrix from Floyd-Warshall.

    Args:
        next_hops: The next_hops dictionary generated by floyd_warshall_shortest_paths.
        start_node: The starting node of the path.
        end_node: The ending node of the path.

    Returns:
        A list of nodes representing the shortest path, or None if no path exists.
    """
    if start_node == end_node:
        return [start_node]
    if next_hops[start_node][end_node] is None:
        return None  # No path exists

    path = [start_node]
    current = start_node
    while current != end_node:
        current = next_hops[current][end_node]
        if current is None: # This should not happen if a path exists and next_hops is correctly built
            return None
        path.append(current)
    return path


def dijkstra_shortest_path(graph: networkx.Graph, start_node, end_node):
    """
    Implements Dijkstra's algorithm to find the shortest path from a start_node
    to an end_node in a graph with non-negative edge weights.

    Args:
        graph: A NetworkX graph where edges have a 'length' attribute representing weight.
        start_node: The starting node for the path.
        end_node: The destination node for the path.

    Returns:
        A list of nodes representing the shortest path, or None if no path exists.
    """
    # Priority queue to store (distance, node) tuples
    priority_queue = [(0, start_node)]
    
    # Dictionary to store the shortest distance from start_node to each node found so far
    distances = {node: float('inf') for node in graph.nodes}
    distances[start_node] = 0
    
    # Dictionary to store the predecessor of each node in the shortest path
    predecessors = {node: None for node in graph.nodes}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # If we've already found a shorter path to current_node, skip
        if current_distance > distances[current_node]:
            continue
            
        # If we reached the end node, reconstruct the path
        if current_node == end_node:
            path = []
            while current_node is not None:
                path.insert(0, current_node)
                current_node = predecessors[current_node]
            return path
            
        # Explore neighbors
        for neighbor in graph.neighbors(current_node):
            weight = graph.edges[current_node, neighbor].get('length', 1)
            distance = current_distance + weight
            
            # If a shorter path to neighbor is found
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))
                
    return None # No path found


def vehicle_generator(env, traffic_grid, arrival_interval, floyd_paths_data):
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
            # Pass the pre-computed Floyd-Warshall paths to the emergency vehicle
            Vehicle(env, 'Ambulance', ('S', (4, 2)), (2, 4), traffic_grid, emergency=True, floyd_paths=floyd_paths_data[1])
            ambulance_in = True
        else:
            origin = RNG.choice(roads)
            destination = RNG.choice(list(destinations - {origin[1]}))
            # Regular vehicles now use Dijkstra's algorithm
            Vehicle(env, f'Car-{count}', origin, destination, traffic_grid)
            count += 1


def report_results():
    print("\n🚦 Vehicle Wait Time Summary 🚦")
    print(f"Total Vehicles: {len(VEHICLE_WAIT_TIMES)}")
    print(f"Vehicles that waited: {sum(1 for w in VEHICLE_WAIT_TIMES if w > 0)}")
    # Handle case where no vehicles waited to avoid statistics.mean error
    if VEHICLE_WAIT_TIMES:
        print(f"Average Wait Time: {statistics.mean(VEHICLE_WAIT_TIMES):.2f}s")
        print(f"Max Wait Time: {max(VEHICLE_WAIT_TIMES):.2f}s")
    else:
        print("No wait times recorded.")

    print("\n🕒 Vehicle Total Time Summary 🕒")
    if VEHICLE_TOTAL_TIMES:
        print(f"Average Total Time: {statistics.mean(VEHICLE_TOTAL_TIMES):.2f}s")
        print(f"Max Total Time: {max(VEHICLE_TOTAL_TIMES):.2f}s")
        print(f"Min Total Time: {min(VEHICLE_TOTAL_TIMES):.2f}s")
    else:
        print("No total times recorded.")


def get_direction(current_node, next_node):
    if current_node is None: # This handles the very first segment where previous_node is None
        # This function should typically be called when both current and next nodes are valid graph nodes.
        # For initial entry, it implies movement from an 'outside' point to the first node in path.
        # This case might need specific handling if direction at entry impacts initial traffic light interaction.
        # For current simulation logic, it's mostly for printing and is handled by the initial path segments.
        pass

    # Determine if movement is primarily North-South or East-West
    if current_node[0] == next_node[0]:  # X-coordinate (row) is same, means EW movement
        return 'EW'
    elif current_node[1] == next_node[1]: # Y-coordinate (column) is same, means NS movement
        return 'NS'
    else:
        # This case should ideally not be reached with typical grid movements (only orthogonal).
        # For diagonal moves, it's ambiguous. In your grid, moves are orthogonal.
        # If the `get_direction` is called with invalid `current_node` or `next_node` (e.g., if path is just one node),
        # this `ValueError` might be triggered. Ensure `get_direction` is called only for valid segments of a path.
        raise ValueError(f"Invalid move between {current_node} and {next_node}. Expected orthogonal movement.")


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

    # Pre-compute all-pairs shortest paths using Floyd-Warshall
    print("Pre-computing all-pairs shortest paths with Floyd-Warshall (for emergency vehicles)...")
    floyd_paths_data = floyd_warshall_shortest_paths(traffic_grid)
    print("Floyd-Warshall pre-computation complete.")

    sim_start = env.now
    env.process(vehicle_generator(env, traffic_grid, arrival_interval=10, floyd_paths_data=floyd_paths_data))

    # Run simulation for 10 minutes:
    env.run(until=600)
    report_results()
