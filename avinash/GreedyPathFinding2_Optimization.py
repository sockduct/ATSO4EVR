import networkx as nx
import simpy
import random
import heapq # Used for the priority queue in Dijkstra's algorithm

# --- Simulation Setup Parameters ---
# Seed for random number generation to ensure reproducibility of results
RANDOM_SEED = 42
# Total duration for which the simulation will run in minutes
SIM_DURATION = 10 # minutes
# Number of emergency vehicles to be spawned during the simulation
NUM_EMERGENCY_VEHICLES = 2

# --- Graph Definition ---
# List of central intersection nodes in our network
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
    G.add_edge('C1', 'C3', weight=random.uniform(7, 20)) # Diagonal edge C1-C3
    G.add_edge('C2', 'C4', weight=random.uniform(7, 20)) # Diagonal edge C2-C4

    # Add edges connecting entry/exit nodes to their respective central intersections.
    # Each central node is connected to two distinct entry/exit nodes.
    for central, entries in NODE_CONNECTIONS.items():
        for entry in entries:
            G.add_edge(central, entry, weight=random.uniform(3, 10)) # Connects a central node to an entry/exit node

    print("--- Road Network Created ---")
    print(f"Nodes in the network: {G.nodes()}")
    print(f"Edges in the network (with weights): {G.edges(data=True)}")
    return G

# --- Greedy Algorithm (Dijkstra's Shortest Path) ---
def greedy_shortest_path(graph, start_node, end_node):
    """
    Implements Dijkstra's algorithm to find the shortest path (minimum travel time)
    between two nodes in the graph. This algorithm is "greedy" as it always
    explores the unvisited node with the smallest known distance from the source.

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

    print(f"\n  [Greedy Algorithm] Initiating path search from '{start_node}' to '{end_node}'...")

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
        print(f"  [Greedy Algorithm] No path could be found from '{start_node}' to '{end_node}'.")
        return None, float('inf')

    # Reconstruct the shortest path by backtracking from the end_node using previous_nodes
    current = end_node
    while current is not None:
        path.insert(0, current) # Insert at the beginning to get the path in correct order
        current = previous_nodes[current]

    print(f"  [Greedy Algorithm] Path found: {path}, Estimated Total Travel Time: {total_time:.2f} minutes.")
    return path, total_time

# --- SimPy Emergency Vehicle Process ---
class EmergencyVehicle:
    """
    Represents an emergency vehicle (EV) that moves through the road network.
    It uses the greedy_shortest_path algorithm to determine its route and
    simulates its movement using SimPy's `timeout` events.
    """
    def __init__(self, env, graph, start_node, destination_node, ev_id):
        self.env = env # SimPy environment
        self.graph = graph # NetworkX graph
        self.start_node = start_node # Starting point of the EV
        self.destination_node = destination_node # Destination of the EV
        self.ev_id = ev_id # Unique ID for the EV
        self.path = [] # Stores the calculated shortest path
        self.total_travel_time = 0 # Stores the calculated total time for the path

    def run(self):
        """
        This is the main SimPy process method for the EmergencyVehicle.
        It calculates the path and then simulates the EV traversing it.
        """
        print(f'Time {self.env.now:.2f}: EV {self.ev_id} spawned at "{self.start_node}". Destination: "{self.destination_node}".')

        # Step 1: Use the greedy algorithm (Dijkstra's) to find the shortest path
        self.path, self.total_travel_time = greedy_shortest_path(
            self.graph, self.start_node, self.destination_node
        )

        # Check if a path was successfully found
        if not self.path:
            print(f'Time {self.env.now:.2f}: EV {self.ev_id} could not find a valid path from "{self.start_node}" to "{self.destination_node}". Aborting journey.')
            return # Terminate this EV's process

        # Step 2: Simulate traversing the found path
        current_node_index = 0
        current_node = self.path[current_node_index]

        # Iterate through the path, segment by segment
        while current_node_index < len(self.path) - 1:
            next_node = self.path[current_node_index + 1]
            try:
                # Get the travel time (weight) for the current road segment
                travel_time = self.graph[current_node][next_node]['weight']
            except KeyError:
                # This should ideally not happen if greedy_shortest_path works correctly,
                # but it's a safeguard for malformed paths or graph issues.
                print(f"Error: No edge found between '{current_node}' and '{next_node}' for EV {self.ev_id}. Path: {self.path}. Aborting journey.")
                break # Exit the loop if an invalid edge is encountered

            print(f'Time {self.env.now:.2f}: EV {self.ev_id} traveling from "{current_node}" to "{next_node}" (estimated {travel_time:.2f} min).')
            # Simulate the time taken to travel this segment
            yield self.env.timeout(travel_time)

            current_node = next_node # Move to the next node
            current_node_index += 1 # Advance in the path
            print(f'Time {self.env.now:.2f}: EV {self.ev_id} arrived at "{current_node}".')

        # Step 3: Log the completion of the journey
        print(f'Time {self.env.now:.2f}: EV {self.ev_id} arrived at its final destination "{self.destination_node}".')
        print(f'EV {self.ev_id} journey details: Path taken: {self.path}, Calculated shortest time: {self.total_travel_time:.2f} minutes.')


# --- SimPy Source for Emergency Vehicles ---
def source_emergency_vehicles(env, graph, num_vehicles):
    """
    This SimPy process is responsible for periodically generating new
    EmergencyVehicle processes into the simulation.
    """
    for i in range(1, num_vehicles + 1):
        # Randomly choose a starting point from the entry/exit nodes
        start_node = random.choice(ENTRY_EXIT_NODES)
        # Randomly choose a central intersection as the destination
        destination_node = random.choice(CENTRAL_NODES)

        # Simulate a random inter-arrival time for vehicles (e.g., exponential distribution)
        # This makes vehicle arrivals more realistic and less simultaneous.
        inter_arrival_time = random.expovariate(1.0 / 1) # Average 1 minutes between vehicles; Parameter 1.0/10 means average rate of 1 vehicle per 1 minutes
        print(f'Time {env.now:.2f}: Next EV ({i}) will spawn in {inter_arrival_time:.2f} minutes.')
        yield env.timeout(inter_arrival_time) # Wait for the inter-arrival time

        # Start a new EmergencyVehicle process in the environment
        env.process(EmergencyVehicle(env, graph, start_node, destination_node, i).run())

# --- Main Simulation Execution ---
def run_simulation():
    """
    Main function to set up and run the entire emergency vehicle simulation.
    """
    # Set the random seed for reproducible results across multiple runs
    random.seed(RANDOM_SEED)

    # 1. Create the road network graph
    road_network_graph = create_road_network()

    # 2. Initialize the SimPy environment
    env = simpy.Environment()

    # 3. Start the process that sources (generates) emergency vehicles
    env.process(source_emergency_vehicles(env, road_network_graph, NUM_EMERGENCY_VEHICLES))

    print(f"\n--- Starting Emergency Vehicle Simulation for a maximum of {SIM_DURATION} minutes ---")
    # 4. Run the simulation until the specified duration
    env.run(until=SIM_DURATION)
    print("\n--- Simulation Ended ---")

# Execute the simulation when the script is run
if __name__ == '__main__':
    run_simulation()