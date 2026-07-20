# Global Path Planner Architecture

This package is kept intentionally small for competition runs. Runtime behavior is
configured from launch parameters so the active code path stays stable during
experiments.

## Runtime Flow

```text
/gps
  -> ll2utm.py
  -> /gps_utm_odom
  -> odom_path_publisher
  -> /local_path1, /local_path2, /current_path
  -> local_path_selector
  -> /selected_path, /moving_obs

region_map + /gps_utm_odom
  -> region_state_publisher
  -> /state
```

## Active Files

- `launch/gpp_local_final.launch`: main runtime launch file.
- `launch/path_maker.launch`: path recording and smoothing launch file.
- `scripts/ll2utm.py`: MORAI GPS to UTM odometry conversion.
- `scripts/global_path_publisher.py`: text path loader and latched path publisher.
- `scripts/path_maker_sm.py`: waypoint recorder and smoother.
- `src/odom_path_publisher.cpp`: global-to-local path extraction.
- `src/local_path_selector.cpp`: path selection and obstacle-distance decisions.
- `src/region_state_publisher.cpp`: region-map based mission state publisher.

## Cleanup Policy

Legacy experiments and unused alternate implementations are not kept in the
active tree. Use git history for recovery instead of keeping `NOT_USED` folders,
copy files, or zipped backups in the repository.
