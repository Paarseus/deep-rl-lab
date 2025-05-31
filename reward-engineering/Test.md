# Reward Engineering for CARLA Simulator + Gymnasium

**Author:** Alexander Assal  
**Date:** May 31, 2025  

---

# Expanded Reward Engineering Tables for CARLA Simulator

**Author:** Alexander Assal  
**Date:** May 31, 2025  

Below are exhaustive reward tables covering driving events detectable via CARLA sensors or API queries. Each row lists an event, the CARLA detection method, and a suggested reward or penalty. New entries have been added to reward correct yielding behavior at stop signs and similar nuanced situations.

---

## Positive Rewards

| Category           | Event                                         | CARLA Detection Method                                                                                      | Reward                                           |
|--------------------|-----------------------------------------------|-------------------------------------------------------------------------------------------------------------|--------------------------------------------------|
| **Goal**           | Reaching Goal                                 | `ego.get_transform().location.distance(goal) < 2`                                                           | **+100** (episode terminates)                    |
| **Goal**           | Progress Toward Goal                          | Δd = (prev_dist – curr_dist) via `ego.get_transform().location`                                              | `+ (Δd / initial_dist) × 100`                    |
| **Lane**           | Lane-Keeping (Centered)                       | `waypoint = world.get_map().get_waypoint(loc, project_to_road)`; `dist_center = loc.distance(waypoint)`    | `+ (3.0 – dist_center)` (∈ [0, +3])               |
| **Lane**           | Correct Lane Change                           | Map API: `prev_waypoint.lane_id != curr_waypoint.lane_id` & not triggered by `lane_invasion_sensor`         | **+5** per event                                 |
| **Lane**           | Staying in Right Lane on Highway              | Map lane type: detect multiple-lane road; ego remains in rightmost lane                                      | **+2** per second                                |
| **Speed**          | Target-Speed Proximity                        | `speed = √(v.x² + v.y² + v.z²)` from `ego.get_velocity()`                                                    | `+ max(0, 10 – |speed – 22|)`                    |
| **Speed**          | Maintaining Speed Limit Zone                  | If `speed` stays ≤ zone’s speed limit (map-provided) for ≥3 s                                                | **+3** per interval (e.g., per 3 seconds)        |
| **Speed**          | Smooth Acceleration (Low Jerk)                | IMU sensor: compute `jerk = (a_t – a_prev)/Δt`; if `|jerk| < threshold`                                      | **+2** per timestep                              |
| **Steering**       | Steady Steering (Low Steering Rate)           | `steering_rate = |(steer – steer_prev)/Δt|` < 15°/s                                                         | **+1** per timestep                              |
| **Following**      | Maintaining Safe Following Distance           | Radar sensor: `distance_to_front > safe_distance` (e.g., > 5 m)                                               | **+2** per second                                |
| **Stop Signs**     | Proper Stop at Stop Sign                      | Map API: detect stop sign waypoint; `speed < 0.5` inside stop zone for ≥2 s                                  | **+5**                                           |
| **Stop Signs**     | Yielding to Vehicle at Stop Sign              | Map API + radar/semantic-lidar: other vehicle stopped first; `ego_speed < 0.5` until it departs             | **+5**                                           |
| **Traffic Lights** | Proper Stop at Red Light                      | Semantic camera: detect red-light pixels; waypoint: `dist_to_stop_line > 0` while `speed < 0.5` for ≥1 s     | **+5**                                           |
| **Pedestrian**     | Yielding to Pedestrian                         | Semantic camera or SemanticLidar: detect pedestrian in crosswalk; `speed < 0.5` while present               | **+10**                                          |
| **Intersection**   | Efficient Intersection Crossing                | Map API: time spent in intersection area ≤ threshold                                                         | **+10**                                          |
| **Merging**        | Merging Safely onto Road                       | Radar: detect no vehicle within merge gap; ego changes lane successfully                                     | **+5**                                           |
| **Turn Maneuvers** | Correct U-Turn (in Allowed Zone)               | Map tag: detect U-turn allowed; ego performs ~180° turn detected via heading change                         | **+5**                                           |
| **Turn Maneuvers** | Navigating Roundabout Correctly                 | Map tag: detect roundabout entry waypoint; ego exits onto correct exit                                      | **+5**                                           |
| **Signals**        | Using Turn Signal Correctly *(if simulated)*    | Control signals: check `control.turn_signal` during lane change                                              | **+2** per correct usage                         |

---

## Negative Rewards

| Category           | Event                                         | CARLA Detection Method                                                                                          | Penalty                               |
|--------------------|-----------------------------------------------|-----------------------------------------------------------------------------------------------------------------|---------------------------------------|
| **Collision**      | Collision (Any Actor)                         | `sensor.other.collision` event                                                                                  | **–100** (episode terminates)         |
| **Collision**      | Collision with Pedestrian                     | `collision_sensor.parsed_data[0].type == 'walker.pedestrian.*'`                                                   | **–150**                              |
| **Collision**      | Collision with Vehicle                        | `collision_sensor.parsed_data[0].type.startswith('vehicle.')`                                                     | **–100**                              |
| **Collision**      | Collision with Static Object                  | `collision_sensor.parsed_data[0].type` not walker or vehicle                                                       | **–100**                              |
| **Collision**      | Traffic Cone / Debris Hit                     | `collision_sensor.parsed_data[0].type == 'static.prop.trafficcone'`                                               | **–20**                               |
| **Lane**           | Lane Crossing (Any Marking)                   | `sensor.other.lane_invasion` event                                                                                 | **–50**                               |
| **Lane**           | Off-Road (dist_center > 3 m)                  | `dist_center = loc.distance(waypoint_center)` > `Config.MAX_DISTANCE_FROM_CENTER`                                   | **–50**                               |
| **Speed**          | Below Minimum Speed (< 15 m/s)                | `speed = ego.get_velocity()` < `Config.MIN_SPEED`                                                                    | **–50**                               |
| **Speed**          | Above Maximum Speed (> 25 m/s)                | `speed = ego.get_velocity()` > `Config.MAX_SPEED`                                                                    | **–1** per timestep                   |
| **Speed**          | Speeding in School Zone                       | Map tag: “school_zone” waypoint; `speed > zone_speed_limit`                                                           | **–30**                               |
| **Speed**          | High-Curve Speeding                           | Map: detect sharp curve waypoint (curvature > threshold); `speed > curve_safe_speed`                                   | **–20**                               |
| **Speed**          | Hard Braking                                  | `brake > 0.7` immediately after `throttle > 0.5` within 0.5 s                                                          | **–10**                               |
| **Speed**          | Hard Acceleration                             | `throttle > 0.7` immediately after `brake > 0.5` or `speed < 1`                                                         | **–10**                               |
| **Speed**          | Stalling (speed < 0.5 for > 5 s)              | Count consecutive timesteps where `speed < 0.5` while not at red/stop sign                                             | **–10**                               |
| **Speed**          | Excessive Idle (speed < 0.5, No Red/Stop)     | Count consecutive timesteps; if > 3 s and not in red/stop zone                                                          | **–5** per second                     |
| **Collision**      | Hitting Pedestrian                            | `collision_sensor` & `other_actor` type is walker                                                                      | **–150**                              |
| **Collision**      | Hitting Vehicle                               | `collision_sensor` & `other_actor` type starts with “vehicle.”                                                          | **–100**                              |
| **Collision**      | Hitting Static Object (e.g., pole)            | `collision_sensor` & actor type not vehicle/pedestrian                                                                    | **–100**                              |
| **Road Rules**     | Driving Wrong Direction                       | `waypoint.road_id != next_waypoint.road_id` and heading differs by > 150°                                                | **–50**                               |
| **Road Rules**     | Illegal Lane Change (Cross Double Yellow)      | `lane_invasion_sensor` indicates crossing solid center line with `lane_type == 'Solid'`                                 | **–50**                               |
| **Road Rules**     | Reversing on Road                              | `ego.get_control().reverse == True` while on main road lane                                                               | **–20**                               |
| **Road Rules**     | Illegal U-Turn (in No-U-Turn Zone)             | Map tag: no-U-turn waypoint; ego heading change ~180° within restricted area                                              | **–50**                               |
| **Pedestrian**     | Passing Pedestrian Without Yield               | Semantic camera or SemanticLidar: pedestrian within 1.5 m ahead; `speed ≥ 0.5`                                             | **–30**                               |
| **Pedestrian**     | Running Red Light                              | Semantic camera: red-light pixels; `dist_to_stop_line < 0` and `speed > 1`                                                  | **–20**                               |
| **Pedestrian**     | Running Stop Sign                              | Map API: stop-sign waypoint; `dist_to_stop_sign < 0` and `speed > 1`                                                        | **–20**                               |
| **Pedestrian**     | Failing to Yield to Vehicle at Stop Sign       | Map API: stop-sign waypoint; radar/semantic-lidar: other vehicle arrived first; ego enters junction before yielding        | **–20**                               |
| **Intersection**   | Blocking Intersection                          | Ego remains inside intersection region (map waypoints) for > 5 s while cross traffic present                              | **–20**                               |
| **Merging**        | Tailgating (dist_front < 2 m)                   | Radar sensor: `distance_to_front < 2` while `speed > 5`                                                                       | **–10** per second                    |
| **Intersection**   | Improper Merge (cutting off traffic)           | Radar: detect vehicle in merge area; ego merges within unsafe distance                                                       | **–20**                               |
| **Environment**    | Environmental Hazard Drive-Through (e.g., water) | Semantic segmentation: detect water/obstacle ahead; ego drives through                                                        | **–20**                               |
| **Environment**    | Speeding Through Water/Slippery Zone            | Map tag: “water_zone” or “ice_zone”; `speed` > safe_speed_for_zone                                                            | **–30**                               |
| **Environment**    | Wrong Parking Maneuver (in Parking Lot)         | Map: parking lot region; ego moves outside designated slot waypoint                                                            | **–20**                               |
| **Environment**    | Traffic Cone / Debris Hit                       | `collision_sensor.parsed_data[0].type == 'static.prop.trafficcone'`                                                            | **–20**                               |

---
