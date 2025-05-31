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

| Event                                           | CARLA Detection Method                                                                                      | Reward                                                |
|-------------------------------------------------|-------------------------------------------------------------------------------------------------------------|-------------------------------------------------------|
| **Reaching Goal**                               | `ego.get_transform().location.distance(goal) < 2`                                                           | **+100** (episode terminates)                         |
| **Progress Toward Goal**                        | Δd = (prev_dist – curr_dist) via `ego.get_transform().location`                                              | `+ (Δd / initial_dist) × 100`                         |
| **Lane-Keeping (Centered)**                     | `waypoint = world.get_map().get_waypoint(loc, project_to_road)`; `dist_center = loc.distance(waypoint)`    | `+ (3.0 – dist_center)`                                |
| **Target-Speed Proximity**                      | `speed = √(v.x² + v.y² + v.z²)` from `ego.get_velocity()`                                                    | `+ max(0, 10 – |speed – 22|)`                          |
| **Smooth Acceleration (Low Jerk)**              | IMU sensor: compute `jerk = (a_t – a_prev)/dt`; if `|jerk| < threshold`                                      | **+2** per timestep                                    |
| **Steady Steering (Low Steering Rate)**         | Record `steering_rate = |(steer – steer_prev)/dt|`; if < 15°/s                                               | **+1** per timestep                                    |
| **Maintaining Safe Following Distance**         | Radar sensor: `distance_to_front > safe_distance` (e.g., > 5 m)                                               | **+2** per second                                      |
| **Proper Stop at Stop Sign**                    | Map API: detect stop sign waypoint; `speed < 0.5` inside stop zone for ≥2 s                                  | **+5**                                                  |
| **Yielding to Vehicle at Stop Sign**            | Map API: detect stop sign + radar/semantic-lidar: other vehicle stopped first; `ego_speed < 0.5` until it departs | **+5**                                                  |
| **Proper Stop at Red Light**                    | Semantic camera: detect red-light pixels; waypoint: compute `dist_to_stop_line > 0` while `speed < 0.5` for ≥1 s | **+5**                                                  |
| **Yielding to Pedestrian**                      | Semantic camera or SemanticLidar: detect pedestrian in crosswalk; `speed < 0.5` while pedestrian present     | **+10**                                                 |
| **Correct Lane Change**                         | Map API: `prev_waypoint.lane_id != curr_waypoint.lane_id` & not triggered by `lane_invasion_sensor`         | **+5** per event                                        |
| **Efficient Intersection Crossing**             | Map API: time spent in intersection area (defined by waypoints) ≤ threshold                                   | **+10**                                                 |
| **Merging Safely onto Road**                    | Radar: detect no vehicle within merge gap; ego changes lane successfully                                     | **+5**                                                  |
| **Maintaining Speed Limit Zone**                | If `speed` stays ≤ zone’s speed limit (map-provided) for ≥3 s                                                 | **+3** per interval (e.g., per 3 seconds)               |
| **Correct U-Turn (in Allowed Zone)**            | Map tag: detect U-turn allowed; ego performs ~180° turn maneuver detected via heading change                 | **+5**                                                  |
| **Navigating Roundabout Correctly**             | Map tag: detect roundabout entry waypoint; ego exits onto desired exit lane                                  | **+5**                                                  |
| **Staying in Right Lane on Highway**            | Map lane type: detect multiple-lane road; ego remains in rightmost lane until passing                        | **+2** per second                                       |
| **Safe Deceleration for Obstacle Ahead**        | Radar: detect static object < 10 m ahead; reduce speed smoothly (no brakes > 0.7)                             | **+3**                                                  |
| **Using Turn Signal Correctly** *(if simulated)*| Control signals: check `control.hand_brake == False` and turn indicator active during lane change            | **+2** per correct usage                                |

---

## Negative Rewards

| Event                                                | CARLA Detection Method                                                                                                        | Penalty                               |
|------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| **Collision (Any Actor)**                            | `sensor.other.collision` event                                                                                                 | **–100** (episode terminates)         |
| **Collision with Pedestrian**                        | `collision_sensor.parsed_data[0].type == 'walker.pedestrian.*'`                                                                 | **–150**                              |
| **Collision with Vehicle**                           | `collision_sensor.parsed_data[0].type.startswith('vehicle.')`                                                                   | **–100**                              |
| **Collision with Static Object**                     | `collision_sensor.parsed_data[0].type` not walker or vehicle                                                                     | **–100**                              |
| **Lane Crossing (Any Marking)**                      | `sensor.other.lane_invasion` event                                                                                              | **–50**                               |
| **Off-Road (dist_center > 3 m)**                     | `waypoint = world.get_map().get_waypoint(loc, project_to_road)`; `dist_center = loc.distance(waypoint)` > 3 m                    | **–50**                               |
| **Below Minimum Speed (< 15 m/s)**                   | `speed = ego.get_velocity()` < 15 m/s                                                                                              | **–50**                               |
| **Above Maximum Speed (> 25 m/s)**                   | `speed = ego.get_velocity()` > 25 m/s                                                                                              | **–1** per timestep                   |
| **Hard Braking**                                     | `brake > 0.7` immediately after `throttle > 0.5` within 0.5 s                                                                     | **–10**                               |
| **Hard Acceleration**                                | `throttle > 0.7` immediately after `brake > 0.5` or `speed < 1`                                                                    | **–10**                               |
| **Abrupt Steering**                                  | `steering_rate = |(steer – steer_prev)/dt|` > 30°/s                                                                                | **–5**                                |
| **High Jerk (|jerk| > threshold)**                   | IMU sensor: `jerk = (a_t – a_prev)/dt`; if `|jerk| > max_jerk`                                                                     | **–5**                                |
| **Stalling (speed < 0.5 for > 5 s)**                 | Count consecutive timesteps where `speed < 0.5` while not at red/stop sign                                                          | **–10**                               |
| **Reversing on Road**                                | `ego.get_control().reverse == True` while on main road lane                                                                        | **–20**                               |
| **Driving Wrong Direction**                          | `waypoint.road_id != next_waypoint.road_id` and heading differs by > 150°                                                          | **–50**                               |
| **Illegal Lane Change (Cross Double Yellow)**        | `lane_invasion_sensor` indicates crossing solid center line with `lane_type == 'Solid'`                                              | **–50**                               |
| **Tailgating (dist_front < 2 m)**                    | Radar sensor: `distance_to_front < 2` while `speed > 5`                                                                             | **–10** per second                    |
| **Blocking Intersection**                            | Ego remains inside intersection region (map waypoints) for > 5 s while cross traffic present (semantic camera)                          | **–20**                               |
| **Running Red Light**                                | Semantic camera: red-light pixels; `dist_to_stop_line < 0` and `speed > 1`                                                             | **–20**                               |
| **Running Stop Sign**                                | Map API: stop-sign waypoint; `dist_to_stop_sign < 0` and `speed > 1`                                                                   | **–20**                               |
| **Failing to Yield to Vehicle at Stop Sign**         | Map API: stop-sign waypoint; radar/semantic-lidar: other vehicle arrived first; ego enters junction (alt waypoint) before yielding     | **–20**                               |
| **Passing Pedestrian Without Yield**                 | Semantic camera or semantic lidar: pedestrian within 1.5 m ahead; `speed ≥ 0.5`                                                           | **–30**                               |
| **Driving on Sidewalk**                              | Semantic segmentation: detect sidewalk class under ego’s bounding box                                                                  | **–50**                               |
| **Hitting Pedestrian**                               | `collision_sensor` & `other_actor` type is walker                                                                                      | **–150**                              |
| **Hitting Vehicle**                                  | `collision_sensor` & `other_actor` type starts with “vehicle.”                                                                          | **–100**                              |
| **Hitting Static Object (e.g., pole)**               | `collision_sensor` & actor type not vehicle/pedestrian                                                                                  | **–100**                              |
| **Excessive U-Turn (in No-U-Turn Zone)**             | Map tag: no-U-turn waypoint; ego heading change ~180° within restricted area                                                            | **–50**                               |
| **Parking on Road (speed < 0.5 in Moving Lane)**     | `speed < 0.5` while map lane traffic lights are green or no stop condition                                                                | **–20**                               |
| **Speeding in School Zone**                          | Map tag: “school_zone” waypoint; `speed > zone_speed_limit`                                                                               | **–30**                               |
| **Illegal Overtake (Entering Oncoming Lane)**        | Map: detect opposite lane via waypoint; ego crosses center into oncoming lane                                                             | **–50**                               |
| **Excessive Loitering (dist_to_goal Increases >5 s)**| Track `distance_to_goal` over time; if increases continuously for >5 s                                                                      | **–10** per second                    |
| **Excessive Idle (speed < 0.5, No Red/Stop)**        | Count consecutive timesteps; if > 3 s and not in red/stop zone                                                                               | **–5** per second                     |
| **Traffic Cone / Debris Hit**                        | `collision_sensor.parsed_data[0].type == 'static.prop.trafficcone'`                                                                        | **–20**                               |
| **Wrong Parking Maneuver (in Parking Lot)**          | Map: parking lot region; ego moves outside designated slot waypoint                                                                          | **–20**                               |
| **Environmental Hazard Drive-Through (e.g., water)** | Semantic segmentation: detect water/obstacle ahead; ego drives through                                                                       | **–20**                               |
| **High-Curve Speeding**                              | Map: detect sharp curve waypoint (curvature > threshold); `speed > curve_safe_speed`                                                           | **–20**                               |

---
