"""Four pure geometry routines used by the initial ROS adapter; no ROS, IK, GPU or hardware."""
import numpy as np
from scipy.spatial.transform import Rotation as R
def orthonormalize_rotation(matrix):
    """防止数值漂移"""
    u, s, vh = np.linalg.svd(matrix)
    return u @ vh


class RosChunkGeometry:
    def compute_min_jerk_scalar(self, duration, dt):
        """最小jerk标量函数 s(t)"""
        t = np.arange(0, duration + dt/2, dt)
        tau = t / duration
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        return s, t

    def interpolate_rotation_slerp(self, start_rot, end_rot, s_values):
        """SLERP旋转插值"""
        from scipy.spatial.transform import Slerp
        key_rots = R.from_matrix([start_rot, end_rot])
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)
        return slerp(s_values).as_matrix()

    def delta_to_absolute_waypoints(self, delta_actions, start_pose):
        """将delta动作转换为绝对waypoints"""
        current_trans = np.array(start_pose.translation)
        current_rot = np.array(start_pose.rotation)

        waypoints = [{'translation': current_trans.copy(),
                      'rotation': current_rot.copy(),
                      'gripper': delta_actions[0, 6]}]

        for action in delta_actions:
            d_trans = action[:3]
            d_rot_euler = action[3:6]
            gripper = action[6]

            current_trans = current_trans + d_trans
            r_delta = R.from_euler('xyz', d_rot_euler, degrees=False).as_matrix()
            current_rot = orthonormalize_rotation(current_rot @ r_delta)

            waypoints.append({
                'translation': current_trans.copy(),
                'rotation': current_rot.copy(),
                'gripper': gripper
            })

        return waypoints

    def build_cartesian_trajectory(self, waypoints, segment_duration, dt):
        """在笛卡尔空间构建平滑轨迹"""
        n_segments = len(waypoints) - 1
        total_time = n_segments * segment_duration

        all_translations = []
        all_rotations = []
        all_gripper = []
        all_timestamps = []
        current_time = 0.0

        for i in range(n_segments):
            wp_start = waypoints[i]
            wp_end = waypoints[i + 1]

            s, t = self.compute_min_jerk_scalar(segment_duration, dt)

            # 位置插值
            start_pos = wp_start['translation']
            end_pos = wp_end['translation']
            for j in range(len(s)):
                pos = start_pos + s[j] * (end_pos - start_pos)
                all_translations.append(pos)

            # 旋转插值
            interpolated_rots = self.interpolate_rotation_slerp(
                wp_start['rotation'], wp_end['rotation'], s)
            all_rotations.extend(interpolated_rots)

            all_gripper.extend([wp_end['gripper']] * len(s))
            all_timestamps.extend(t + current_time)
            current_time += segment_duration

        return {
            'translations': np.array(all_translations),
            'rotations': all_rotations,
            'gripper': np.array(all_gripper),
            'timestamps': np.array(all_timestamps),
            'dt': dt,
            'total_duration': total_time
        }


