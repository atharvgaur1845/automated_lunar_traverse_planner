# Automated Lunar Traverse Planner

An intelligent path planning system for lunar rover missions that generates safe, scientifically optimal traverse routes while considering crater hazards, solar illumination, and mission objectives.

## Overview

The Automated Lunar Traverse Planner is a sophisticated Python-based system designed to process terrain data and generate optimal paths for a lunar rover. Using crater detection data (e.g., from a YOLO model), the planner creates a traverse that navigates safely around hazards, visits high-value scientific waypoints, and ensures the rover maintains sufficient solar power by avoiding shadows. The system is built with a multi-layered safety protocol and redundant navigation strategies to handle complex and cluttered terrain.

## Key Features

* **Safety-First Navigation**: Implements a robust hazard avoidance system with configurable safety margins around craters.
* **Intelligent Waypoint Selection**: Automatically identifies and prioritizes two types of scientific sites: crater rim geology locations and undisturbed regolith sampling areas.
* **Advanced Solar Power Management**: Simulates shadows cast by craters based on the sun's position and ensures the rover's path stays within well-illuminated areas to maintain solar efficiency.
* **Three-Tier Navigation Algorithm**: A hierarchical navigation system that ensures robust pathfinding:

1. **Tier 1**: Primary pathfinding toward high-priority science targets.
2. **Tier 2**: A rotational search to find an escape route when the primary path is blocked.
3. **Tier 3**: A last-resort hazard breach protocol to navigate out of enclosed areas.
* **Automated Start Point Verification**: Ensures the mission's starting point is in a safe, viable location before beginning the traverse.
* **Comprehensive Output Visualization**: Generates detailed maps of the planned traverse, including crater hazards, safety zones, shadow regions, and the final path.


## Algorithm Explained

The planner's intelligence is rooted in a series of interconnected algorithms that handle everything from environmental simulation to decision-making.

### 1. Solar Shadow Simulation and Avoidance

To ensure the rover maintains power, the system simulates shadows cast by craters and actively avoids them. This is a critical function for missions relying on solar energy.[^1][^2]

* **Shadow Calculation**: For any crater with a radius greater than 1.5 meters, the algorithm calculates the length and direction of its shadow. This is based on the crater's height (assumed to be 30% of its radius) and the sun's elevation and azimuth angles.[^2][^1]

```python
# Shadow length calculation from model_with_sun_simulated.py
shadow_length = (crater.radius * 0.3) / math.tan(sun_elevation_rad)
```

* **Shadow Representation**: Each shadow is modeled as a polygon projected onto the map away from the sun. The vertices of this polygon are calculated based on the shadow's length and direction.[^2]
* **Illumination Check**: The `calculate_illumination` function checks if a given point on the map falls inside any of these shadow polygons. Points in direct sunlight are assigned an illumination value of 1.0, while points inside a shadow receive a value of 0.15.[^2]
* **Path Planning Constraint**: The core safety check function, `is_point_safe`, uses this illumination data. Before moving to any new point, it verifies that the point's illumination value is above a configurable threshold (e.g., `solar_efficiency_threshold: 0.9`). If a potential path segment is in shadow, the planner will discard it and search for an alternative, sunlit route.[^1][^2]


### 2. Scientific Stop Selection

The planner identifies and prioritizes scientifically valuable waypoints to maximize the mission's research output. Stops are categorized into two types, each with its own selection criteria and instrumentation plan.[^1][^2]

#### Crater Rim Geology Sites

These stops are chosen for in-situ analysis of crater geology, such as examining ejecta blankets or exposed strata.[^1][^2]

* **Location**: The algorithm generates 16 candidate points around the rim of each crater, at a fixed distance of 2.5 meters from the edge.[^2][^1]
* **Prioritization**: Waypoints are scored based on a weighted combination of the crater's detection confidence and its size. This focuses the mission on larger, more prominent features.[^1][^2]

```python
# Priority score from model_with_sun_simulated.py
priority = (crater.confidence * 7.0) + (crater.radius / 5.0 * 2.0)
```

* **Constraints**: Only points that are in well-lit areas are considered valid waypoints.[^2][^1]
* **Mission Profile**: Each stop is allocated 50 minutes for spectrometer analysis.[^1][^2]


#### Regolith Sampling Sites

These are located in undisturbed plains to collect pristine samples of lunar soil (regolith).[^2][^1]

* **Location**: The planner samples points on a 15-meter grid across the map and selects those that are at least 20 meters away from any crater to ensure the area is undisturbed.[^1][^2]
* **Prioritization**: These sites are assigned a fixed priority score of 6.5.[^2][^1]
* **Constraints**: Like rim sites, these must be in areas with sufficient sunlight.[^1][^2]
* **Mission Profile**: Each stop is allocated 30 minutes for sample collection with a scoop.[^2][^1]


### 3. The Three-Tier Navigation Algorithm

The rover's path is determined by a hierarchical algorithm that balances aggressive exploration with robust safety measures. If one tier of navigation fails, the next is automatically engaged.[^1][^2]

#### Tier 1: Direct Path Planning

This is the primary mode of operation, focused on efficiently reaching the highest-value scientific targets.[^2][^1]

1. **Target Selection**: The algorithm identifies the highest-scoring, unvisited scientific waypoint. The score considers both the waypoint's intrinsic scientific priority and its distance from the rover's current position.[^2]
2. **Path Verification**: It calculates a direct line to the target and checks for hazards (hazardous craters or shadow zones) at 2-meter intervals.[^2]
3. **Movement**: If the path is clear, the rover proceeds.
4. **Arc Navigation**: If the direct path is blocked by an obstacle, the system attempts to navigate around it by generating a smooth arc along the obstacle's edge before trying to continue toward the target.[^2]

#### Tier 2: Rotational Search

This tier is activated if Tier 1 fails to find a clear path, which can happen in cluttered terrain.[^1][^2]

1. **Escape Vector Search**: The planner probes 8 directions around the rover at a 15-meter radius.[^2]
2. **Safety Check**: It looks for the first available direction that leads to a safe, navigable point (i.e., outside a hazard and in sunlight).[^2]
3. **Repositioning**: If a safe escape vector is found, the rover moves to that new position.
4. **Return to Tier 1**: Once repositioned, the system reverts to Tier 1 to resume science-oriented navigation from its new, unblocked location.[^1][^2]

#### Tier 3: Hazard Breach Protocol

This is a last-resort maneuver for when the rover is completely trapped by hazardous craters or shadows and Tier 2 fails to find an exit.[^1][^2]

1. **Escape Target**: The system identifies the farthest corner of the map as an ultimate escape target.[^2]
2. **Modified Safety Rules**: To find a path, the planner temporarily relaxes its safety rules. It allows the rover to plot a course *through* smaller, non-hazardous craters, which are normally avoided.
3. **Hazard Avoidance**: Critically, the safety margin around large, *hazardous* craters is still strictly enforced. This allows the rover to escape while minimizing risk.[^2]
4. **Return to Tier 1**: If an escape is successful, the system returns to Tier 1. If all three tiers fail, the mission is considered ended for that location, as the rover is permanently trapped.[^2]

## Configuration

The planner's behavior can be customized via a central configuration dictionary in `model_with_sun_simulated.py`:

```python
CONFIG = {
    'min_traverse_distance': 300.0,    # Minimum path length in meters
    'crater_avoidance_radius': 2.0,    # Craters larger than this are hazardous
    'path_safety_margin': 0.5,         # Additional clearance around hazards
    'min_science_stops': 20,           # Target number of waypoints to visit
    'map_size_meters': 100.0,          # Dimension of the square map
    'sun_elevation': 25.0,             # Sun's angle above the horizon in degrees
    'sun_azimuth': 180.0,              # Sun's direction (180 is from the South)
    'solar_efficiency_threshold': 0.9  # Minimum illumination for safe travel
}
```


## Usage

1. **Prepare Data**: Place your terrain maps (`.jpg`) and YOLO crater annotations (`.txt`) in the `dataset/train/images/` and `dataset/train/labels/` directories, respectively.
2. **Run Planner**: Execute the main script from your terminal.

```bash
python model_with_sun_simulated.py
```

3. **Review Output**: The generated traverse maps and logs will be saved in the `output_with_sun_simulated/` directory, organized by map name.

## Output Visualization

The output is a detailed PNG map for each traverse, visualizing:

* **Hazardous Craters**: Red circles with a dashed red safety margin.
* **Navigable Craters**: Gray circles that can be traversed in Tier 3.
* **Shadow Regions**: A semi-transparent black overlay indicating areas of low illumination.
* **Science Waypoints**: Numbered markers (`S1`, `S2`, etc.) colored by type.
* **Traverse Path**: A bright cyan line showing the rover's complete journey.
* **Landing Zone**: A green pentagon marking the verified safe start point.


## Dependencies

* Python 3.x
* NumPy
* Matplotlib

