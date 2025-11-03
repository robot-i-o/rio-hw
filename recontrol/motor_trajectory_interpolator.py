import numbers

import numpy as np
import scipy.interpolate as si


class MotorTrajectoryInterpolator:
    def __init__(self, times: list | np.ndarray, values: list | np.ndarray):
        """Initialize the motor trajectory interpolator.

        Args:
            times: Array of timestamps for waypoints
            values: Array of n-dimensional motor values corresponding to timestamps
        """
        assert len(times) >= 1, "Must provide at least one waypoint"
        assert len(values) == len(times), "Number of values must match number of timestamps"

        if not isinstance(times, np.ndarray):
            times = np.array(times)
        if not isinstance(values, np.ndarray):
            values = np.array(values)

        if len(times) == 1:
            # Special handling for single waypoint
            self.single_step = True
            self._times = times
            self._values = values
        else:
            self.single_step = False
            assert np.all(times[1:] >= times[:-1]), "Times must be monotonically increasing"

            # Create linear interpolator for all dimensions
            self.interp = si.interp1d(times, values, axis=0, assume_sorted=True)

    @property
    def times(self) -> np.ndarray:
        """Get interpolation timestamps."""
        if self.single_step:
            return self._times
        else:
            return self.interp.x

    @property
    def values(self) -> np.ndarray:
        """Get interpolation values."""
        if self.single_step:
            return self._values
        else:
            return self.interp.y

    def trim(self, start_t: float, end_t: float) -> "MotorTrajectoryInterpolator":
        """Create new interpolator trimmed to specified time range.

        Args:
            start_t: Start time for trimmed trajectory
            end_t: End time for trimmed trajectory

        Returns:
            New MotorTrajectoryInterpolator instance
        """
        assert start_t <= end_t, "Start time must be before end time"
        times = self.times
        should_keep = (start_t < times) & (times < end_t)
        keep_times = times[should_keep]

        # Include start and end points
        all_times = np.concatenate([[start_t], keep_times, [end_t]])
        all_times = np.unique(all_times)  # Remove duplicates

        # Interpolate values at new timestamps
        all_values = self(all_times)
        return MotorTrajectoryInterpolator(times=all_times, values=all_values)

    def drive_to_waypoint(
        self,
        value,
        time,
        curr_time,
        max_speed=np.inf,
    ) -> "MotorTrajectoryInterpolator":
        """Create new interpolator that drives to specified waypoint.

        Args:
            value: Target motor values
            time: Desired arrival time
            curr_time: Current time
            max_speed: Maximum allowed speed for each dimension

        Returns:
            New MotorTrajectoryInterpolator instance
        """
        assert max_speed > 0, "Speed limit must be positive"
        time = max(time, curr_time)

        curr_value = self(curr_time)
        value_dist = np.linalg.norm(value - curr_value)
        min_duration = value_dist / max_speed
        duration = time - curr_time
        duration = max(duration, min_duration)
        assert duration >= 0

        last_waypoint_time = curr_time + duration

        # Create new interpolator with target waypoint
        trimmed_interp = self.trim(curr_time, curr_time)
        times = np.append(trimmed_interp.times, [last_waypoint_time], axis=0)
        values = np.append(trimmed_interp.values, [value], axis=0)

        return MotorTrajectoryInterpolator(times, values)

    def schedule_waypoint(
        self,
        value,
        time,
        max_speed=np.inf,
        curr_time=None,
        last_waypoint_time=None,
    ) -> "MotorTrajectoryInterpolator":
        """Schedule a new waypoint while respecting speed limits.

        Args:
            value: Target motor values
            time: Desired arrival time
            max_speed: Maximum allowed speed for each dimension
            curr_time: Current time (optional)
            last_waypoint_time: Time of last scheduled waypoint (optional)

        Returns:
            New MotorTrajectoryInterpolator instance
        """
        assert max_speed > 0, "Speed limit must be positive"
        if last_waypoint_time is not None:
            assert curr_time is not None

        # Handle timing constraints
        start_time = self.times[0]
        end_time = self.times[-1]
        assert start_time <= end_time

        if curr_time is not None:
            if time <= curr_time:
                return self
            start_time = max(curr_time, start_time)

            if last_waypoint_time is not None:
                if time <= last_waypoint_time:
                    end_time = curr_time
                else:
                    end_time = max(last_waypoint_time, curr_time)
            else:
                end_time = curr_time

        end_time = min(end_time, time)
        start_time = min(start_time, end_time)

        assert start_time <= end_time
        assert end_time <= time
        if last_waypoint_time is not None:
            if time <= last_waypoint_time:
                assert end_time == curr_time
            else:
                assert end_time == max(last_waypoint_time, curr_time)
        if curr_time is not None:
            assert curr_time <= start_time
            assert curr_time <= time

        # Trim trajectory
        trimmed_interp = self.trim(start_time, end_time)

        # Calculate required duration based on speed limit
        duration = time - end_time
        end_value = trimmed_interp(end_time)
        value_dist = np.linalg.norm(value - end_value)
        min_duration = value_dist / max_speed
        duration = max(duration, min_duration)
        assert duration >= 0
        last_waypoint_time = end_time + duration

        # Create new interpolator with scheduled waypoint
        times = np.append(trimmed_interp.times, [last_waypoint_time], axis=0)
        values = np.append(trimmed_interp.values, [value], axis=0)

        return MotorTrajectoryInterpolator(times, values)

    def __call__(self, t: numbers.Number | np.ndarray) -> np.ndarray:
        """Interpolate values at specified time(s).

        Args:
            t: Time or array of times to interpolate at

        Returns:
            Interpolated motor values
        """
        is_single = False
        if isinstance(t, numbers.Number):
            is_single = True
            t = np.array([t])

        if self.single_step:
            values = np.tile(self._values[0], (len(t), 1))
        else:
            start_time = self.times[0]
            end_time = self.times[-1]
            t = np.clip(t, start_time, end_time)
            values = self.interp(t)

        if is_single:
            values = values[0]
        return values
