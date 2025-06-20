import time
import enum

# --- Enums for Clarity ---

class LightState(enum.Enum):
    """
    Enumeration of possible traffic light states.
    
    Values:
        RED: Stop signal
        YELLOW: Caution/transition signal
        GREEN: Go signal
    """
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"

class Direction(enum.Enum):
    """
    Enumeration of cardinal directions for traffic flow.
    
    Values:
        NORTH: Northbound traffic
        SOUTH: Southbound traffic
        EAST: Eastbound traffic
        WEST: Westbound traffic
    """
    NORTH = "North"
    SOUTH = "South"
    EAST = "East"
    WEST = "West"

# --- Class Definitions ---

class TrafficLight:
    """
    Represents a single traffic light controlling a specific lane direction.
    
    Attributes:
        lane_direction (str): The direction of traffic this light controls
        state (LightState): Current state of the traffic light
    
    Methods:
        set_state: Changes the light's state if different from current state
        __str__: String representation of the traffic light
    """
    def __init__(self, lane_direction: str, initial_state: LightState):
        """
        Initialize a new traffic light.
        
        Args:
            lane_direction (str): Direction of traffic flow
            initial_state (LightState): Starting state of the light
        """
        self.lane_direction = lane_direction
        self.state = initial_state
        print(f"Traffic Light for {self.lane_direction} initialized to {self.state.value}")

    def set_state(self, new_state: LightState):
        """
        Changes the state of the traffic light.
        
        Args:
            new_state (LightState): The new state to set
        
        Note:
            Only changes state if different from current state
        """
        if self.state != new_state:
            print(f"  🚦 Changing {self.lane_direction} light from {self.state.value} to {new_state.value}")
            self.state = new_state
        else:
            print(f"  🚦 {self.lane_direction} light already {self.state.value}")

    def __str__(self):
        return f"Light({self.lane_direction}: {self.state.value})"

class Intersection:
    """
    Represents a four-way intersection with multiple traffic lights.
    
    Attributes:
        name (str): Name or identifier of the intersection
        traffic_lights (dict): Dictionary of TrafficLight objects
        conflict_map (dict): Maps each direction to its conflicting directions
    
    Methods:
        set_default_cycle: Sets the default traffic pattern
        prioritize_emergency_vehicle: Implements greedy algorithm for EV priority
        revert_to_normal: Returns intersection to normal operation
    """
    def __init__(self, name: str):
        """
        Initialize a new intersection.
        
        Args:
            name (str): Identifier for the intersection
        """
        self.name = name
        self.traffic_lights = {}
        # Define lanes and their initial states for a typical 4-way intersection
        # Simplified: one light per major direction for straight-through traffic
        self.traffic_lights[f"{Direction.NORTH.value}bound_Straight"] = TrafficLight(f"{Direction.NORTH.value}bound_Straight", LightState.RED)
        self.traffic_lights[f"{Direction.SOUTH.value}bound_Straight"] = TrafficLight(f"{Direction.SOUTH.value}bound_Straight", LightState.RED)
        self.traffic_lights[f"{Direction.EAST.value}bound_Straight"] = TrafficLight(f"{Direction.EAST.value}bound_Straight", LightState.RED)
        self.traffic_lights[f"{Direction.WEST.value}bound_Straight"] = TrafficLight(f"{Direction.WEST.value}bound_Straight", LightState.RED)

        # Define conflicting lanes for greedy algorithm
        # A crucial dictionary for the greedy algorithm. It defines which lanes conflict with each other. 
        # For example, if "Northbound_Straight" is green, then "Southbound_Straight," "Eastbound_Straight," and "Westbound_Straight" must be red to avoid collisions.
        self.conflict_map = {
            f"{Direction.NORTH.value}bound_Straight": [f"{Direction.SOUTH.value}bound_Straight", f"{Direction.EAST.value}bound_Straight", f"{Direction.WEST.value}bound_Straight"],
            f"{Direction.SOUTH.value}bound_Straight": [f"{Direction.NORTH.value}bound_Straight", f"{Direction.EAST.value}bound_Straight", f"{Direction.WEST.value}bound_Straight"],
            f"{Direction.EAST.value}bound_Straight": [f"{Direction.NORTH.value}bound_Straight", f"{Direction.SOUTH.value}bound_Straight", f"{Direction.WEST.value}bound_Straight"],
            f"{Direction.WEST.value}bound_Straight": [f"{Direction.NORTH.value}bound_Straight", f"{Direction.SOUTH.value}bound_Straight", f"{Direction.EAST.value}bound_Straight"],
        }
        self.set_default_cycle() # Start with a default light cycle

    def set_default_cycle(self):
        """
        Sets a default traffic light cycle.
        
        Implementation:
            - Sets North-South directions to GREEN
            - Sets East-West directions to RED
            - Creates a basic alternating pattern
        """
        print(f"\n--- {self.name}: Setting Default Traffic Cycle ---")
        self.traffic_lights[f"{Direction.NORTH.value}bound_Straight"].set_state(LightState.GREEN)
        self.traffic_lights[f"{Direction.SOUTH.value}bound_Straight"].set_state(LightState.GREEN)
        self.traffic_lights[f"{Direction.EAST.value}bound_Straight"].set_state(LightState.RED)
        self.traffic_lights[f"{Direction.WEST.value}bound_Straight"].set_state(LightState.RED)
        print("Default: North-South Green, East-West Red")

    # Prioritize Emergency Vehicle - This method implements a greedy algorithm to prioritize emergency vehicles at the intersection.
    # The greedy choice is to set the lane of the emergency vehicle to GREEN and all conflicting lanes to RED.
    def prioritize_emergency_vehicle(self, ev_lane: str):
        """
        Implements greedy algorithm for emergency vehicle priority. 
        The following implementation action represents the "greedy choice"—making the locally optimal decision at that precise moment to clear the path for the highest
        priority vehicle.
        
        Args:
            ev_lane (str): The lane/direction of the approaching emergency vehicle

        Implementation:
            1. Sets emergency vehicle's lane to GREEN
            2. Sets all conflicting lanes to RED
            3. Maintains this state until EV passes
        """
        print(f"\n--- {self.name}: EMERGENCY VEHICLE DETECTED! Prioritizing {ev_lane} ---")
        
        # 1. Set EV's path to GREEN
        self.traffic_lights[ev_lane].set_state(LightState.GREEN)

        # 2. Set all conflicting paths to RED
        if ev_lane in self.conflict_map:
            for conflicting_lane in self.conflict_map[ev_lane]:
                self.traffic_lights[conflicting_lane].set_state(LightState.RED)
        else:
            print(f"Warning: No conflict map defined for {ev_lane}. Prioritizing only EV lane.")

    def revert_to_normal(self):
        """Reverts the intersection to its default cycle or a simple phase after EV passes."""
        print(f"\n--- {self.name}: EV PASSED. Reverting to Normal Traffic Cycle ---")
        self.set_default_cycle() # For simplicity, revert to the initial default cycle

class EmergencyVehicle:
    """
    Represents an emergency vehicle traversing through intersections.
    
    Attributes:
        ev_id (str): Unique identifier for the emergency vehicle
        current_location (str): Current position description
        destination (str): Target destination
        approaching_lane (str): Direction of approach
        has_passed_intersection (bool): Track if vehicle has cleared intersection
    
    Methods:
        simulate_passage: Simulates the vehicle passing through an intersection
    """
    def __init__(self, ev_id: str, current_location: str, destination: str, approaching_lane: str):
        """
        Initialize a new emergency vehicle.
        
        Args:
            ev_id (str): Unique identifier
            current_location (str): Starting position
            destination (str): Target location
            approaching_lane (str): Direction of approach
        """
        self.ev_id = ev_id
        self.current_location = current_location # e.g., "Approaching Intersection A"
        self.destination = destination
        self.approaching_lane = approaching_lane # e.g., "Northbound_Straight"
        self.has_passed_intersection = False
        print(f"Emergency Vehicle {self.ev_id} created, approaching via {self.approaching_lane}.")

    def simulate_passage(self, intersection_name: str, duration: int):
        """
        Simulates emergency vehicle passing through intersection.
        
        Args:
            intersection_name (str): Name of intersection being traversed
            duration (int): Time in seconds to simulate passage
        
        Effects:
            - Pauses execution for specified duration
            - Updates has_passed_intersection flag
            - Prints progress messages
        """
        print(f"  Emergency Vehicle {self.ev_id} is passing through {intersection_name}...")
        time.sleep(duration) # Simulate time taken to clear the intersection
        self.has_passed_intersection = True
        print(f"  Emergency Vehicle {self.ev_id} has cleared {intersection_name}.")

# --- Main Simulation Logic ---

def simulate_traffic_signal_optimization():
    """
    Main simulation function for adaptive traffic signal optimization.
    
    Simulation Flow:
        1. Creates a test intersection
        2. Simulates normal traffic flow
        3. Introduces emergency vehicle
        4. Applies greedy optimization algorithm
        5. Simulates EV passage
        6. Reverts to normal operation
    
    Returns:
        None
    """
    print("Starting Adaptive Traffic Signal Optimization Simulation...")

    # Create an intersection
    intersection_A = Intersection("Main Street & Elm Avenue")

    # Simulate some time with normal traffic flow
    print("\nSimulating normal traffic flow for a few seconds...")
    time.sleep(3) # Normal operation

    # Create an emergency vehicle approaching Intersection A from the North
    ev1 = EmergencyVehicle("EV001", "Road North of Intersection A", "Hospital", f"{Direction.NORTH.value}bound_Straight")

    # --- Apply Greedy Algorithm ---
    # Intersection A detects EV1 and applies greedy prioritization
    intersection_A.prioritize_emergency_vehicle(ev1.approaching_lane)

    # Simulate the EV passing through the intersection
    ev1.simulate_passage(intersection_A.name, 4) # EV takes 4 seconds to pass

    # After EV passes, revert the signals
    if ev1.has_passed_intersection:
        intersection_A.revert_to_normal()

    print("\nSimulation complete.")

if __name__ == "__main__":
    simulate_traffic_signal_optimization()