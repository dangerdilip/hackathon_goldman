# Goldman Sachs Hackathon: Drone Delivery Challenge - Comprehensive Blueprint

This document serves as a complete technical summary and blueprint of our drone delivery solution. It contains the exact logic, algorithms, and architectures used, detailed enough to rebuild the solution from scratch.

---

## 1. Problem Specification & Scoring

### 1.1 Objective
The goal is to schedule a fleet of drones to pick up packages from a central warehouse and deliver them to various coordinates, avoiding dynamic/static No-Fly Zones (NFZs) and managing drone battery levels by utilizing charging stations.

### 1.2 Constraints & Entities
- **Map/Environment:** 2D Euclidean space.
- **Warehouse:** The central hub where all drones start, pick up deliveries, and return.
- **Drones:** 
  - Max payload capacity (e.g., 5.0 kg).
  - Max battery capacity.
  - Constant flight speed.
  - Energy consumption formula: `Distance * (Base_Cost + Payload_Weight * Weight_Multiplier)`.
- **Deliveries:** Each has a location `(x, y)`, `weight`, and a `deadline`.
- **Charging Stations:** Fixed locations where drones can recharge. Charging takes time based on the amount of battery replenished.
- **No-Fly Zones (NFZs):** Circular zones with a radius, moving from a start point to an end point over a specific time interval. Drones cannot intersect these zones while they are active.

### 1.3 Scoring Metric
The total score is calculated based on three factors:
1. **Total Energy Consumed** by all drones.
2. **Makespan:** The time the last drone returns to the warehouse.
3. **Penalties:** A massive penalty (e.g., 100 points or huge energy equivalent) for every delivery not fulfilled.

**Goal:** Minimize the total score. Missing deliveries is the worst outcome.

---

## 2. System Architecture & Data Structures

Our Java solution (`Solution.java`) revolves around several core data classes:

- **`Point`**: Represents `(x, y)` coordinates. Includes Euclidean distance utility methods.
- **`Delivery`**: Stores `id`, `location` (Point), `weight`, and `deadline`.
- **`NFZ`**: Stores `startLocation`, `endLocation`, `radius`, `startTime`, and `endTime`. Includes an `intersectsGeom()` method to check if a line segment crosses the NFZ during its active time.
- **`ChargingStation`**: Stores `id` and `location`.
- **`Drone`**: Tracks the drone's `id`, `currentLocation`, `currentTime`, and `currentBattery`.
- **`Trip`**: Represents a proposed set of deliveries for a single drone. Stores the sequence of deliveries, total distance, energy cost, estimated completion time, and a computed `netValue`.
- **`Node`**: Used in A* pathfinding. Stores `location`, `time` reached, `gScore`, `fScore`, and `parent`.

---

## 3. Core Algorithms & Logic

### 3.1 Pathfinding and Collision Avoidance (A* / Dijkstra)
Because drones fly in straight lines, they might intersect an NFZ. We built an A* pathfinder.
- **Intersection Math:** We used a mathematical function to check if the line segment between the drone's start and end points intersects the moving circle of the NFZ during the specific time window. 
- **Wait Times:** If a direct path is blocked, the drone can choose to wait in place until the NFZ passes.
- **Graph Generation:** If waiting isn't optimal, the A* algorithm generates intermediate nodes around the bounding box of the NFZ to navigate around it.
- **Heuristic Function:** We used Euclidean distance. *(Bug Note: We failed to scale this heuristic correctly against the energy multiplier, causing the A* to be inadmissible and slow in complex scenarios).*

### 3.2 The Greedy Scheduling Engine
The core of the assignment logic is an iterative greedy algorithm.

1. **Filtering:** At the start of each loop, we filter out deliveries whose deadlines have already passed based on the current time of our drones.
2. **Urgency Sorting:** We sort the remaining deliveries based on urgency (`deadline - current_time`) and weight. *(Optimization: To prevent O(N^2) slowdowns on 10,000 deliveries, we only take the top ~60 deliveries into consideration per loop).*
3. **Trip Generation:** We attempt to build "Trips" for each available drone. A drone tries to pick up a batch of 1 to N deliveries (up to its payload limit).
4. **Route Optimization (TSP):** For a batch of deliveries in a Trip, we must decide the drop-off order. 
   - If the batch size is small (<= 5), we use **Full Permutation Search** to test every possible sequence.
   - For larger batches, we use a **2-opt** local search heuristic to untangle crossing paths and find a near-optimal route.
5. **Energy & Battery Check:** We simulate the entire trip (Warehouse -> D1 -> D2 -> ... -> Warehouse). 
   - We calculate the energy cost for every leg, factoring in the decreasing payload weight after each drop-off.
   - If the drone's battery drops below zero, the trip is invalid.
6. **Charging Station Detours:** If a trip fails the battery check, we attempt to route the drone to a Charging Station midway through the trip or on the way back to the warehouse. We find the station that minimizes the detour distance and adds the charging time to the drone's schedule.
7. **Scoring Trips (`netValue`):** Each valid trip is scored. 
   - `Value = (Number of Deliveries) * Penalty_Avoided`
   - `Cost = Total_Energy_Used * Energy_Multiplier + Time_Penalty`
   - `netValue = Value - Cost`
8. **Commitment:** We pick the trip with the absolute highest `netValue` across all drones and candidate batches, assign it, update the winning drone's time/battery/location, mark those deliveries as completed, and repeat the loop.

---

## 4. Time Management & Cutoffs
The hackathon platform enforced a strict 2.0-second time limit per test case.
- We implemented a `System.currentTimeMillis()` check inside the main `while` loop.
- If elapsed time exceeded `1800ms`, the solver would aggressively skip expensive operations (like A* pathfinding around complex NFZs or full permutation searches) and fall back to direct-line greedy paths.
- If it exceeded `1950ms`, the main loop would hard-break to ensure whatever schedule was generated got printed to standard output before the platform killed the process.

---

## 5. Known Bugs & Why We Got Stuck (Test Cases 9 & 10)

While the solution achieves high scores on smaller datasets (Test Cases 1-8), it catastrophically fails on Test Cases 9 (10,000 deliveries) and 10 (5,000 deliveries), resulting in 0 deliveries scheduled.

**Root Causes:**
1. **The Inadmissible A* Heuristic:** Our A* pathfinder's heuristic evaluated pure distance cost but did not multiply it by the specific drone energy scaling factor (e.g., `0.15`). As a result, the A* thought taking massive detours was "cheaper" than it actually was. When an NFZ blocked a path, the pathfinder explored thousands of useless nodes, timing out the algorithm instantly.
2. **Sorting Bottleneck in the Main Loop:** Initially, we sorted the *entire* list of unassigned deliveries (e.g., 5,000 items) on every single iteration of the greedy engine to find the "best" next package. `5000 * log(5000)` operations executed thousands of times per second caused the CPU to choke, hitting the 2.0s limit before a single trip could be committed.
3. **Recursive Fallback Failure:** When the A* algorithm failed to find a path, the fallback logic was supposed to try a simpler path. However, due to state corruption (not resetting the drone's time/battery correctly upon failure), the fallback would endlessly loop or return infinite costs, effectively paralyzing that drone.

---

## 6. The Blueprint to Fix It (The Unimplemented Final Plan)
If rebuilding this solution, the following changes are required to conquer the massive test cases:

1. **Scale the A* Heuristic:** `heuristicCost = distance(node, goal) * (base_cost + current_payload_weight) * 0.15`. This ensures A* expands nodes strictly toward the goal and doesn't get lost in deep, expensive search trees.
2. **Aggressive Sub-listing:** Maintain a priority queue of deliveries. Only extract and evaluate the top 40-60 deliveries in the greedy loop. Never sort the full 10,000-item array inside the main `while` loop.
3. **Post-Commit Route Optimization:** Instead of doing expensive 2-opt *during* the evaluation phase, assign the trips quickly using a simple greedy nearest-neighbor approach. Then, *after* all trips are assigned (if time remains before the 2.0s cutoff), run a background loop that swaps delivery orders within committed trips to reduce energy. 
4. **Pre-compute NFZ Bounding Boxes:** Do not do math intersection checks for every path. Use fast Axis-Aligned Bounding Box (AABB) checks. If a path is completely outside the AABB of an NFZ, skip the expensive circle-line time-intersection math entirely.
