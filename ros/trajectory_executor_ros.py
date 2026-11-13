"""Original ROS dynamic impedance, with auditable Cartesian chunk I/O.

Uses old minimum-jerk/SLERP geometry in measured O_T_EE; no V2 move_pose.
No URDF hand/TCP IK/FK remapping is applied. Gripper changes occur
after the corresponding row endpoint, never during a blocked reference stream.
No added 3mm consistency check, 8N delta check, retry, reset or recovery.
"""
import json
import time
from pathlib import Path
import numpy as np
import rospy
from ros_chunk_geometry import RosChunkGeometry
from frankapy import FrankaConstants as FC, SensorDataMessageType
from frankapy.proto import PosePositionSensorMessage, CartesianImpedanceSensorMessage, ShouldTerminateSensorMessage
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from franka_interface_msgs.msg import SensorDataGroup
from franka_gripper.msg import MoveGoal, GraspGoal
from autolab_core import RigidTransform
from halo_ros_protocol import validate


def groups(actions,previous):
    result=[]; start=0
    for i,row in enumerate(actions):
        if row[6]!=previous:
            result.append((start,i+1,row[6])); start=i+1; previous=row[6]
    if start<len(actions): result.append((start,len(actions),None))
    return result


class RosChunkExecutor:
    def __init__(self,arm,feedback,cfg,directory,camera_check=lambda:None):
        self.arm,self.feedback,self.cfg,self.directory=arm,feedback,cfg,Path(directory)
        self.camera_check=camera_check
        # Reuse pure legacy geometry only. Do not initialize the old GPU IK
        # model, whose fr3_hand frame is not the measured O_T_EE tool frame.
        self.geometry=RosChunkGeometry()
        self.previous_gripper=None; self.last_grasp_result=None
        self.active=False; self.gripper_client=None
        self.pub=rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,SensorDataGroup,queue_size=1)

    def log(self,event,**value):
        with (self.directory/'operations.jsonl').open('a') as output:
            output.write(json.dumps(dict(event=event,host_monotonic_s=time.monotonic(),**value),allow_nan=False)+'\n')

    def check(self):
        self.camera_check()
        return self.feedback.read()

    def cancel(self):
        # Only cancel this client's own goal. Never send cancel_all_goals/reset/recovery.
        failures=[]
        if self.active:
            try:
                self.arm._client.cancel_goal()
                acknowledged=self.arm._client.wait_for_result(rospy.Duration(5))
                self.log('arm_cancel_confirmation',acknowledged=bool(acknowledged),status=self.arm._client.get_state())
                if not acknowledged: raise RuntimeError('ROS arm cancellation not confirmed; inspect onsite')
                self.active=False; self.arm._in_skill=False
            except Exception as exc: failures.append(str(exc))
        if self.gripper_client is not None:
            try:
                self.gripper_client.cancel_goal()
                acknowledged=self.gripper_client.wait_for_result(rospy.Duration(3))
                self.log('gripper_cancel_confirmation',acknowledged=bool(acknowledged))
                if not acknowledged: raise RuntimeError('ROS gripper cancellation not confirmed; inspect onsite')
                self.gripper_client=None
            except Exception as exc: failures.append(str(exc))
        if failures: raise RuntimeError('; '.join(failures))

    def publish(self,pose,index,elapsed):
        stiffness=self.cfg['stiffnesses']
        msg=PosePositionSensorMessage(id=index,timestamp=elapsed,
            position=pose.translation.tolist(),quaternion=pose.quaternion.tolist())
        gains=CartesianImpedanceSensorMessage(id=index,timestamp=elapsed,
            translational_stiffnesses=stiffness[:3],rotational_stiffnesses=stiffness[3:])
        self.pub.publish(make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(msg,SensorDataMessageType.POSE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(gains,SensorDataMessageType.CARTESIAN_IMPEDANCE)))

    def stream(self,trajectory,label):
        poses=trajectory['poses']; dt=1/self.cfg['publish_hz']
        deadline=time.monotonic()+3
        while not self.pub.get_num_connections():
            if time.monotonic()>deadline: raise RuntimeError('No ROS sensor-data subscriber')
            self.check(); time.sleep(.02)
        self.feedback.ready(3)
        np.savez_compressed(self.directory/(label+'_trajectory.npz'),
            translations=np.asarray([p.translation for p in poses]),
            quaternions_wxyz=np.asarray([p.quaternion for p in poses]),
            joint_positions=trajectory.get('joint_positions',[]),dt=dt)
        started=time.monotonic()
        # This FrankaArm client is newly constructed for this executor and owns
        # no earlier goal. Mark ownership before sending: goto_pose can raise
        # during its post-send sleep (including Ctrl+C).
        self.active=True
        try:
            self.arm.goto_pose(self.arm.get_pose(),duration=len(poses)*dt+3,
                dynamic=True,buffer_time=2,cartesian_impedances=self.cfg['stiffnesses'],
                ignore_errors=False,skill_desc='Halo ROS action chunk '+label)
            self.log('stream_started',label=label,points=len(poses),publish_hz=self.cfg['publish_hz'])
            origin=time.monotonic(); deadline=origin
            for i,pose in enumerate(poses):
                self.check()
                if self.arm._client.get_state() in (2,3,4,5,8,9):
                    raise RuntimeError('ROS arm goal ended before stream completed: '+str(self.arm._client.get_state()))
                self.publish(pose,i,time.monotonic()-origin)
                if i%self.cfg['publish_hz']==0:
                    self.log('stream_feedback',label=label,index=i,state=self.feedback.read())
                deadline+=dt
                if time.monotonic()-deadline>.5:
                    raise RuntimeError('ROS reference publisher stalled >0.5s; stop, do not replay backlog')
                time.sleep(max(0,deadline-time.monotonic()))
            for k in range(round(.5/dt)):
                self.check(); self.publish(poses[-1],len(poses)+k,time.monotonic()-origin); time.sleep(dt)
            term=ShouldTerminateSensorMessage(timestamp=time.monotonic()-origin,should_terminate=True)
            self.pub.publish(make_sensor_group_msg(termination_handler_sensor_msg=sensor_proto2ros_msg(term,SensorDataMessageType.SHOULD_TERMINATE)))
            if not self.arm._client.wait_for_result(rospy.Duration(5)):
                raise RuntimeError('ROS stream termination not acknowledged')
            status=self.arm._client.get_state()
            self.active=False; self.arm._in_skill=False
            if status!=3: raise RuntimeError('ROS stream failed, action status '+str(status))
            after=self.feedback.ready(3)
            self.log('stream_finished',label=label,elapsed_s=time.monotonic()-started,state=after,
                     endpoint_error_m=float(np.linalg.norm(np.asarray(after['state'][:3])-poses[-1].translation)))
        except BaseException:
            self.cancel()
            raise

    def gripper(self,command):
        self.feedback.ready(3); self.camera_check()
        if command==1:
            client=self.arm._gripper_move_client
            goal=MoveGoal(width=self.cfg['gripper_width_m'],speed=self.cfg['gripper_speed_mps'])
        else:
            client=self.arm._gripper_grasp_client
            goal=GraspGoal(width=0,speed=self.cfg['gripper_speed_mps'],force=self.cfg['gripper_force_n'])
            goal.epsilon.inner=.005; goal.epsilon.outer=.08
        self.gripper_client=client
        client.send_goal(goal)
        self.log('gripper_sent',command=command)
        deadline=time.monotonic()+10
        try:
            while client.get_state() not in (2,3,4,5,8,9):
                self.check()
                if time.monotonic()>deadline: raise RuntimeError('ROS gripper timeout')
                time.sleep(.05)
            result=client.get_result()
            if client.get_state()!=3 or result is None or not result.success:
                raise RuntimeError('ROS gripper action unsuccessful')
            self.gripper_client=None
            self.previous_gripper=command
            self.last_grasp_result=(True if command==0 else None)
            self.log('gripper_finished',command=command,result_success=True,
                     note='Action result is not proof of object capture',state=self.feedback.read())
        except BaseException:
            self.cancel(); raise

    def execute(self,actions,chunk):
        before=self.feedback.ready(3)
        validate({'status':'continue','stage':'execute','message':'accepted model actions','act':actions},before['state'])
        waypoints=self.geometry.delta_to_absolute_waypoints(np.asarray(actions,dtype=float),self.arm.get_pose())
        self.log('chunk_started',chunk=chunk,actions=actions,state=before)
        for index,(start,end,command) in enumerate(groups(actions,self.previous_gripper)):
            self.check(); self.feedback.ready(3)
            dt=1/self.cfg['publish_hz']; duration=self.cfg['interp_per_row']*dt
            trajectory=self.geometry.build_cartesian_trajectory(waypoints[start:end+1],duration,dt)
            trajectory['poses']=[RigidTransform(translation=position,rotation=rotation,
                from_frame='franka_tool',to_frame='world')
                for position,rotation in zip(trajectory['translations'],trajectory['rotations'])]
            self.feedback.ready(3); self.camera_check()
            self.stream(trajectory,'chunk_%04d_group_%02d'%(chunk,index))
            if command is not None: self.gripper(command)
        after=self.feedback.ready(3)
        self.log('chunk_finished',chunk=chunk,rows=len(actions),state=after)
        return dict(after,last_gripper_command=self.previous_gripper,
                    last_grasp_result_success=self.last_grasp_result,
                    completion_note='Execution success is not visual object/task success')
