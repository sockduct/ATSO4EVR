import networkx as nx
import simpy
import random
import heapq # Used for the priority queue in Dijkstra's algorithm

# --- Simulation Setup Parameters ---
# Seed for random number generation to ensure reproducibility of results
RANDOM_SEED = 42
# Total duration for which the simulation will run in minutes
SIM_DURATION = 100 # 10 # minutes 100
# Number of emergency vehicles to be spawned during the simulation
NUM_EMERGENCY_VEHICLES = 5 # 3 # 5
# Time (in minutes) it takes for a traffic signal to switch phases for an EV
SIGNAL_SWITCH_TIME = 2 #0.5 # minutes - 2

# --- Graph Definition ---
# List of central intersection nodes in our network. These will have traffic signals.
CENTRAL_NODES = ['C1', 'C2', 'C3', 'C4']
# List of entry/exit points for emergency vehicles
ENTRY_EXIT_NODES = ['E1', 'E2', 'E3', 'E4', 'E5', 'E6', 'E7', 'E8']

# Mapping of central intersections to their connected entry/exit points
NODE_CONNECTIONS = {
    'C1': ['E1', 'E2'],
    'C2': ['E3', 'E4'],
    'C3': ['E5', 'E6'],
    'C4': ['E7', 'E8']
}

# Global dictionary to hold TrafficSignal objects for each central node
# This allows EmergencyVehicle processes to access and interact with signals
GLOBAL_TRAFFIC_SIGNALS = {}

def create_road_network():
    """
    Creates a NetworkX graph representing the road network for emergency vehicles.
    Nodes are intersections/entry-exit points, and edges are roads with travel times.
    Travel times (weights) are randomly assigned within specified ranges.
    """
    G = nx.Graph() # Initialize an undirected graph

    # Add all central intersection nodes to the graph, tagging them as 'central'
    G.add_nodes_from(CENTRAL_NODES, type='central')
    # Add all entry/exit nodes, tagging them as 'entry_exit'
    G.add_nodes_from(ENTRY_EXIT_NODES, type='entry_exit')

    # Add edges between central intersections to form a grid-like structure
    # and some diagonal connections for more routing options.
    # Weights represent travel time in minutes.
    G.add_edge('C1', 'C2', weight=random.uniform(5, 15)) # Edge between C1 and C2
    G.add_edge('C2', 'C3', weight=random.uniform(5, 15)) # Edge between C2 and C3
    G.add_edge('C3', 'C4', weight=random.uniform(5, 15)) # Edge between C3 and C4
    G.add_edge('C4', 'C1', weight=random.uniform(5, 15)) # Edge between C4 and C1
    G.add_edge('E1', 'E8', weight=random.uniform(5, 20)) # Edge between E1-E8
    G.add_edge('E2', 'E3', weight=random.uniform(5, 20)) # Edge between E2-E3
    G.add_edge('E4', 'E5', weight=random.uniform(5, 20)) # Edge between E4-E5
    G.add_edge('E6', 'E7', weight=random.uniform(5, 20)) # Edge between E6-E7

    # Add edges connecting entry/exit nodes to their respective central intersections.
    # Each central node is connected to two distinct entry/exit nodes.
    for central, entries in NODE_CONNECTIONS.items():
        for entry in entries:
            G.add_edge(central, entry, weight=random.uniform(3, 10)) # Connects a central node to an entry/exit node

    print("--- Road Network Created ---")
    print(f"Nodes in the network: {G.nodes()}")
    print(f"Edges in the network (with weights): {G.edges(data=True)}")
    return G

# --- Greedy Algorithm for Shortest Path (Dijkstra's) ---
def greedy_shortest_path(graph, start_node, end_node):
    """
    Implements Dijkstra's algorithm to find the shortest path (minimum travel time)
    between two nodes in the graph. This algorithm is "greedy" as it always
    explores the unvisited node with the smallest known distance from the source.
    This pathfinding is based purely on the static edge weights (road travel times)
    and does not inherently account for dynamic signal delays. Those are handled
    during the simulation traversal.

    Args:
        graph (nx.Graph): The network graph with 'weight' attributes on edges.
        start_node (str): The starting node for the pathfinding.
        end_node (str): The destination node for the pathfinding.

    Returns:
        tuple: A tuple containing:
            - path (list): A list of nodes representing the shortest path from start_node to end_node.
                           Returns None if no path exists.
            - total_time (float): The cumulative weight (travel time) of the shortest path.
                                  Returns float('inf') if no path exists.
    """
    # Initialize distances to all nodes as infinity, except the start_node which is 0
    distances = {node: float('inf') for node in graph.nodes()}
    distances[start_node] = 0

    # Stores the predecessor of each node in the shortest path found so far
    previous_nodes = {node: None for node in graph.nodes()}

    # Priority queue (min-heap) to store (distance, node).
    # It allows efficient retrieval of the node with the smallest distance.
    priority_queue = [(0, start_node)]

    print(f"\n  [Greedy Pathfinding] Initiating path search from '{start_node}' to '{end_node}'...")

    while priority_queue:
        # Extract the node with the smallest known distance from the priority queue
        current_distance, current_node = heapq.heappop(priority_queue)

        # If we've already processed this node with a shorter path, skip it
        if current_distance > distances[current_node]:
            continue

        # If the destination node has been reached, we can stop the search
        if current_node == end_node:
            break

        # Explore neighbors of the current node
        for neighbor in graph.neighbors(current_node):
            # Get the weight (travel time) of the edge between current_node and neighbor
            # Accessing graph[u][v] directly is efficient for undirected graphs
            weight = graph[current_node][neighbor]['weight']
            # Calculate the new distance to the neighbor through the current_node
            new_distance = current_distance + weight

            # If a shorter path to the neighbor is found
            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance # Update the shortest distance
                previous_nodes[neighbor] = current_node # Update the predecessor
                heapq.heappush(priority_queue, (new_distance, neighbor)) # Add/update neighbor in priority queue

    path = []
    total_time = distances[end_node] # The shortest time to the end_node

    # If the total_time is still infinity, it means no path was found
    if total_time == float('inf'):
        print(f"  [Greedy Pathfinding] No path could be found from '{start_node}' to '{end_node}'.")
        return None, float('inf')

    # Reconstruct the shortest path by backtracking from the end_node using previous_nodes
    current = end_node
    while current is not None:
        path.insert(0, current) # Insert at the beginning to get the path in correct order
        current = previous_nodes[current]

    print(f"  [Greedy Pathfinding] Path found: {path}, Estimated Topology-based Time: {total_time:.2f} minutes.")
    return path, total_time

# --- Adaptive Traffic Signal Optimization (Greedy Algorithm) ---
class TrafficSignal:
    """
    Represents an adaptive traffic signal at a central intersection.
    It implements a greedy algorithm to prioritize emergency vehicles by
    immediately setting the signal to green for the EV's specific path
    (incoming to outgoing segment), incurring a switch time delay only
    if the signal phase needs to change. This implicitly addresses
    all four directions" by handling any valid incoming-outgoing pair.
    """
    def __init__(self, env, graph, node_id, switch_time):
        """
        The switch time delay (SIGNAL_SWITCH_TIME = 0.5 minutes) is included for several reasons:
        Physical Reality:
            Traffic signals can't change instantly
            Need time for:
            Current traffic to clear
            Yellow light phase
            Cross traffic to stop
            Safety Considerations:
        Resource Contention:
            If multiple EVs arrive simultaneously
            First EV gets priority (greedy)
            Others wait for:
            Signal availability
            Switch time completion
        This implementation balances:
            Emergency response speed
            Traffic safety
            Physical constraints of traffic infrastructure
        """
        self.env = env
        self.graph = graph # Reference to the graph to understand node connections
        self.node_id = node_id
        self.switch_time = switch_time
        # Use a SimPy Resource to model the signal's occupancy/control.
        # An EV acquires this resource to control the signal's state.
        self.signal_resource = simpy.Resource(env, capacity=1)
        # Represents the (from_node, to_node) tuple that currently has a green light.
        # Initialize to None, meaning no specific direction/route is prioritized.
        self._current_active_route = None

    def request_clear_path(self, ev_id, incoming_node, outgoing_node):
        """
        An Emergency Vehicle calls this method when it arrives at a signalized intersection.
        The signal's internal "greedy algorithm" immediately adapts its state
        to clear the path for the EV from `incoming_node` to `outgoing_node`.
        A `switch_time` delay is incurred only if the signal was not already
        configured for this specific route.
        """
        print(f'Time {self.env.now:.2f}: EV {ev_id} at "{self.node_id}" requesting clear path from "{incoming_node}" to "{outgoing_node}".')

        # Acquire exclusive control over this signal. Other EVs (or simulated traffic)
        # will have to wait here if the signal is currently being adapted for another EV.
        with self.signal_resource.request() as request:
            yield request # Wait for the signal to be available for adaptation

            # Define the required route for the EV.
            required_route = (incoming_node, outgoing_node)

            # For undirected graphs, (A,B) is the same as (B,A) for an edge.
            # We canonicalize the route tuple for comparison to ensure symmetry.
            required_route_canonical = tuple(sorted(required_route))
            
            is_aligned = False
            if self._current_active_route:
                current_route_canonical = tuple(sorted(self._current_active_route))
                if current_route_canonical == required_route_canonical:
                    is_aligned = True

            # Greedy Algorithm for Signal Optimization:
            # If the signal is not currently aligned for the EV's required route,
            # it immediately switches to prioritize it, incurring the switch_time.
            if not is_aligned:
                print(f'Time {self.env.now:.2f}: Signal at "{self.node_id}" is adapting for EV {ev_id} to clear path from "{incoming_node}" to "{outgoing_node}" (delay: {self.switch_time:.2f} min).')
                yield self.env.timeout(self.switch_time) # Simulate the time it takes for the signal to change
                self._current_active_route = required_route # Update the signal's active route to the *non-canonical* route
                print(f'Time {self.env.now:.2f}: Signal at "{self.node_id}" is clear for EV {ev_id}.')
            else:
                print(f'Time {self.env.now:.2f}: Signal at "{self.node_id}" is already aligned for EV {ev_id}. No additional delay.')

# --- SimPy Emergency Vehicle Process ---
class EmergencyVehicle:
    """
    Represents an emergency vehicle (EV) that moves through the road network.
    It uses the greedy_shortest_path algorithm to determine its route and
    simulates its movement using SimPy's `timeout` events, incorporating
    delays caused by adaptive traffic signals based on its specific incoming
    and outgoing directions at intersections. This setup directly aims to
    minimize the ambulance delay from traffic lights by implementing a greedy
    signal preemption strategy.
    """
    def __init__(self, env, graph, start_node, destination_node, ev_id):
        self.env = env # SimPy environment
        self.graph = graph # NetworkX graph
        self.start_node = start_node # Starting point of the EV
        self.destination_node = destination_node # Destination of the EV
        self.ev_id = ev_id # Unique ID for the EV
        self.path = [] # Stores the calculated shortest path (nodes)
        self.total_travel_time_topology = 0 # Stores the initial topology-based travel time

    def run(self):
        """
        This is the main SimPy process method for the EmergencyVehicle.
        It first calculates the shortest path using Dijkstra's and then
        simulates the EV traversing it, including interactions with
        adaptive traffic signals at central intersections.
        """
        print(f'Time {self.env.now:.2f}: EV {self.ev_id} spawned at "{self.start_node}". Destination: "{self.destination_node}".')

        # Step 1: Use the greedy algorithm (Dijkstra's) to find the shortest path based on topology
        self.path, self.total_travel_time_topology = greedy_shortest_path(
            self.graph, self.start_node, self.destination_node
        )

        # Check if a path was successfully found
        if not self.path:
            print(f'Time {self.env.now:.2f}: EV {self.ev_id} could not find a valid path from "{self.start_node}" to "{self.destination_node}". Aborting journey.')
            return # Terminate this EV's process

        # Step 2: Simulate traversing the found path, accounting for traffic signals
        # Iterate through the path, segment by segment (node by node)
        for i in range(len(self.path) - 1):
            current_node = self.path[i]
            next_node = self.path[i + 1]

            # Simulate travel time along the road segment
            try:
                travel_time = self.graph[current_node][next_node]['weight']
            except KeyError:
                print(f"Error: No edge found between '{current_node}' and '{next_node}' for EV {self.ev_id}. Path: {self.path}. Aborting journey.")
                break

            print(f'Time {self.env.now:.2f}: EV {self.ev_id} traveling from "{current_node}" to "{next_node}" (road time: {travel_time:.2f} min).')
            yield self.env.timeout(travel_time)
            print(f'Time {self.env.now:.2f}: EV {self.ev_id} arrived at "{next_node}".')

            # Check if the arrived node (`next_node`) is a central intersection
            # AND if it's not the very last node in the path (i.e., the EV needs to proceed through it).
            if next_node in CENTRAL_NODES and (i + 1) < (len(self.path) - 1):
                # The EV is now AT an intersection and needs to proceed THROUGH it.
                # It needs the signal to clear its path from `current_node` (where it came from)
                # to `self.path[i + 2]` (the next node AFTER the current intersection in its path).
                incoming_node_for_signal = current_node
                outgoing_node_for_signal = self.path[i + 2]

                signal_controller = GLOBAL_TRAFFIC_SIGNALS[next_node]
                # EV requests the signal to clear its path for the specific route (incoming->outgoing)
                yield self.env.process(signal_controller.request_clear_path(
                    self.ev_id, incoming_node_for_signal, outgoing_node_for_signal
                ))

        # Step 3: Log the completion of the journey
        print(f'Time {self.env.now:.2f}: EV {self.ev_id} arrived at its final destination "{self.destination_node}".')
        # Note: total_travel_time_topology is the time based purely on road segments.
        # The true total time for the EV is the simulation time when it finishes its run().
        print(f'EV {self.ev_id} journey details: Path taken: {self.path}.')


# --- SimPy Source for Emergency Vehicles ---
def source_emergency_vehicles(env, graph, num_vehicles):
    """
    This SimPy process is responsible for periodically generating new
    EmergencyVehicle processes into the simulation.
    """
    for i in range(1, num_vehicles + 1):
        # Randomly choose a starting point from the entry/exit nodes
        start_node = random.choice(ENTRY_EXIT_NODES)

                # Create list of valid destination nodes
        valid_destinations = [
            node for node in ENTRY_EXIT_NODES 
            if node != start_node  # Not the same as start
            and not graph.has_edge(start_node, node)  # Not adjacent
            and node not in NODE_CONNECTIONS[next(  # Not connected to same central node
                central for central, entries in NODE_CONNECTIONS.items() 
                if start_node in entries
            )]
        ]

        # Randomly choose a destnmation from the valid options.
        destination_node = random.choice(valid_destinations)

        # Simulate a random inter-arrival time for vehicles (e.g., exponential distribution)
        # This makes vehicle arrivals more realistic and less simultaneous.
        # Average 1 minutes between vehicles. It can be sometime much shorter or longer # 10
        inter_arrival_time = random.expovariate(1.0 / 10) 
        print(f'Time {env.now:.2f}: Next EV ({i}) will spawn in {inter_arrival_time:.2f} minutes.')
        # env.timeout() - Creates a time delay in the SimPy simulation
        """ yield - Makes the function a Python generator
            Pauses the current process (vehicle spawning)
            Lets other simulation processes run
            Resumes when the timeout completes
        """
        yield env.timeout(inter_arrival_time) # Wait for the inter-arrival time

        # Start a new EmergencyVehicle process in the environment
        env.process(EmergencyVehicle(env, graph, start_node, destination_node, i).run())

# --- Main Simulation Execution ---
def run_simulation():
    """
    Main function to set up and run the entire emergency vehicle simulation,
    including the road network, emergency vehicles, and adaptive traffic signals.
    """
    # Set the random seed for reproducible results across multiple runs
    random.seed(RANDOM_SEED)

    # 1. Create the road network graph
    road_network_graph = create_road_network()

    # 2. Initialize the SimPy environment
    env = simpy.Environment()

    # 3. Initialize TrafficSignal objects for each central node and store them globally.
    # Each signal needs a reference to the graph to correctly identify routes/directions.
    for node_id in CENTRAL_NODES:
        GLOBAL_TRAFFIC_SIGNALS[node_id] = TrafficSignal(env, road_network_graph, node_id, SIGNAL_SWITCH_TIME)
    print("\n--- Traffic Signals Initialized ---")
    print(f"Signals at: {list(GLOBAL_TRAFFIC_SIGNALS.keys())}, each with a switch time of {SIGNAL_SWITCH_TIME} minutes.")

    # 4. Start the process that sources (generates) emergency vehicles
    env.process(source_emergency_vehicles(env, road_network_graph, NUM_EMERGENCY_VEHICLES))

    print(f"\n--- Starting Emergency Vehicle Simulation for a maximum of {SIM_DURATION} minutes ---")
    # 5. Run the simulation until the specified duration
    env.run(until=SIM_DURATION)
    print("\n--- Simulation Ended ---")

# Execute the simulation when the script is run directly
if __name__ == '__main__':
    run_simulation()
