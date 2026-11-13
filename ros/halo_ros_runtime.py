"""ROS feedback guard; no V2 control client. Constructors are explicit."""
import threading
import time


def active_errors(errors):
    return [name for name in errors.__slots__ if bool(getattr(errors,name))]


class RosState:
    def __init__(self):
        import rospy
        from franka_interface_msgs.msg import RobotState, FrankaInterfaceStatus, RunLoopProcessInfoState
        from sensor_msgs.msg import JointState
        self.rospy=rospy
        self.lock=threading.Lock(); self.latest={}; self.progress=None; self.robot_time=None
        if not rospy.core.is_initialized():
            rospy.init_node('halo_ros_chunks',anonymous=True,disable_signals=True)
        self.subscribers=[]
        for key,topic,kind in [
            ('robot','/robot_state_publisher_node_1/robot_state',RobotState),
            ('status','/franka_interface_status_publisher_node_1/franka_interface_status',FrankaInterfaceStatus),
            ('process','/run_loop_process_info_state_publisher_node_1/run_loop_process_info_state',RunLoopProcessInfoState),
            ('gripper','/franka_gripper_1/joint_states',JointState)]:
            self.subscribers.append(rospy.Subscriber(topic,kind,lambda msg,k=key:self.receive(k,msg),queue_size=1))

    def receive(self,key,msg):
        now=time.monotonic()
        with self.lock:
            if key=='robot':
                if self.robot_time is not None and msg.robot_time<self.robot_time:
                    self.latest['clock_error']=('Robot clock reversed',now)
                if self.robot_time is None or msg.robot_time>self.robot_time:
                    self.progress=now
                self.robot_time=msg.robot_time
            self.latest[key]=(msg,now)

    def read(self,idle=False):
        import numpy as np
        from scipy.spatial.transform import Rotation
        now=time.monotonic()
        with self.lock:
            values=dict(self.latest); progress=self.progress
        if 'clock_error' in values: raise RuntimeError(values['clock_error'][0])
        if any(k not in values or now-values[k][1]>1 for k in ('robot','status','process','gripper')):
            raise RuntimeError('ROS robot/interface/process/gripper feedback missing or stale')
        if progress is None or now-progress>1:
            raise RuntimeError('Robot device time is not advancing')
        robot,status,process,gripper=[values[k][0] for k in ('robot','status','process','gripper')]
        if not status.is_ready or status.error_description:
            raise RuntimeError('Original driver not ready: '+status.error_description)
        errors=active_errors(robot.current_errors)
        if errors or robot.robot_mode not in (1,2):
            raise RuntimeError('Franka current error/mode: '+str((errors,robot.robot_mode)))
        if any(robot.joint_collision) or any(robot.cartesian_collision):
            raise RuntimeError('Franka native collision flags are active')
        if idle and (process.is_running_skill or process.new_skill_available or robot.robot_mode!=1):
            raise RuntimeError('ROS arm is not idle; no task takeover')
        matrix=np.asarray(robot.O_T_EE,dtype=float).reshape((4,4),order='F')
        if not np.isfinite(matrix).all() or not np.isfinite(robot.q).all():
            raise RuntimeError('Nonfinite robot feedback')
        if len(gripper.position)!=2 or not np.isfinite(gripper.position).all():
            raise RuntimeError('Expected fresh independent two-finger joint feedback')
        width=float(sum(gripper.position))
        xyzw=Rotation.from_matrix(matrix[:3,:3]).as_quat().tolist()
        state=matrix[:3,3].tolist()+xyzw+[width]
        return {'state':state,'robot_time':robot.robot_time,'robot_mode':robot.robot_mode,
                'q':list(robot.q),'dq':list(robot.dq),'q_d':list(robot.q_d),'dq_d':list(robot.dq_d),
                'ddq_d':list(robot.ddq_d),'O_T_EE':list(robot.O_T_EE),'O_T_EE_c':list(robot.O_T_EE_c),
                'O_F_ext_hat_K':list(robot.O_F_ext_hat_K),'current_errors':errors,
                'last_motion_errors':active_errors(robot.last_motion_errors),
                'is_running_skill':process.is_running_skill,'new_skill_available':process.new_skill_available,
                'current_skill_id':process.current_skill_id,'native_status_ready':status.is_ready,
                'host_received_monotonic_s':values['robot'][1],
                'frame':'ROS O_T_EE / FrankaPy world->franka_tool, no V2 TCP assumptions',
                'gripper_width_source':'independent /franka_gripper_1/joint_states',
                'gripper_is_grasped':None,
                'gripper_note':'native arm launched without gripper reader; no fabricated is_grasped value'}

    def ready(self,timeout=10):
        deadline=time.monotonic()+timeout
        last=None
        while time.monotonic()<deadline:
            try: return self.read(idle=True)
            except RuntimeError as exc: last=exc; time.sleep(.05)
        raise RuntimeError('ROS readiness failed: '+str(last))

    def close(self):
        for subscriber in self.subscribers: subscriber.unregister()
