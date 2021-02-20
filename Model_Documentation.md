# Model Documentation

My model is based on the code described in the lecture video "Project Q&A."
Then I would like only to describe the addition to the lecture code.

## Strategy for Changing Lanes

The only thing I added to the lecture code is a strategy for changing lanes.
The strategy is quite simple:

1. Check if my car is too close to the car ahead  
   Yes -> go to 2  
   No  -> keep the current lane
2. Check if my car is on the way to change lanes  
   Yes -> keep on changing lanes  
   No  -> go to 3  
3. Check if the left lane is safe to go  
   Yes -> change lanes to left  
   No  -> go to 4
4. Check if the right lane is safe to go  
   Yes -> change lanes to right  
   No  -> keep the current lane

The code for Step 1 and 2 is the following (`main.cpp: line 132`):

```c++
if (too_close) {                  // Step 1
    if (isInLane(lane, car_d)) {  // Step 2
      // check whether lane changing can be done
      // and do lane changing if it is safe
      lane = changeLane(lane, car_s, sensor_fusion, prev_size);
  }
}
```

The function `isInLane()` just decides if the car is in the `target_lane` (`main.cpp: line 276`):

```c++
inline bool isInLane(const int target_lane, const double current_d) {
  return (current_d < 2. + 4 * target_lane + 2) && current_d > (2. + 4 * target_lane - 2);
}
```

The function `changeLane()` looks as follows (`main.cpp: line 279`). It performs Step 3 and 4.

```c++
template<typename T>
int changeLane(int lane, double car_s, const T &sensor_fusion, const size_t &prev_size) {
  if (lane > 0) {  // Step 3: try changing lane to left
    const int change_lane = lane - 1;
    lane = tryChangeLane(lane, car_s, sensor_fusion, change_lane, prev_size);
    if (lane == change_lane)  // can be changed lane safely
      return lane;
  }
  if (lane < 2) {  // Step 4: try changing lane to right
    const int change_lane = lane + 1;
    lane = tryChangeLane(lane, car_s, sensor_fusion, change_lane, prev_size);
  }
  return lane;
}
```

The function `tryChangeLane()` is as follows (`main.cpp: line 293`). It checks if each car detected by sensors is in the target lane and is not too close to my car.

```c++
template<typename T>
int tryChangeLane(const int lane, const double car_s, const T &sensor_fusion,
                  const int change_lane, const size_t &prev_size) {
  for (const auto &sense: sensor_fusion) {
    const double sense_d = sense[6];
    if (!isInLane(change_lane, sense_d)) continue;
    if (!isLaneChangeSafe(car_s, sense, prev_size)) return lane;
  }
  return change_lane;  // safe to change lane
}
```

The function `isLaneChangeSafe()` plays a key role to measure the "safety" to change lanes (`main.cpp: line 303`). If the distance between my car and the other car in the target lane is far enough, the function returns `true`.

```c++
bool isLaneChangeSafe(const double car_s, const vector<double> &sense, const size_t &prev_size) {
  const double vx = sense[3];
  const double vy = sense[4];
  const double check_speed = std::sqrt(vx * vx + vy * vy);
  double check_car_s = sense[5];

  // if using previous points can project s value out
  check_car_s += prev_size * 0.02 * check_speed;
  const double safe_margin_ahead = 30.;   // meter
  const double safe_margin_behind = 15.;  // meter
  if (check_car_s >= car_s + safe_margin_ahead || check_car_s <= car_s - safe_margin_behind)
    return true;

  return false;
}
```


## Optimality

The strategy is greedy. It tries to change lanes to left first even if the path is not optimal. However, if the leftmost lane is the fastest one, the greedy strategy would perform very well.
