"""Operator-run ROS action-chunk Halo. No initial move, reset, retry or V2 control."""
import argparse
import base64
import datetime
import fcntl
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from halo_ros_protocol import CAMERAS, PROTOCOL, config, request, save, validate


class Recorder:
    def __init__(self,cfg,directory):
        self.cfg,self.directory=cfg,Path(directory)
        self.stop_event=threading.Event(); self.error=None; self.received=0.; self.count=0
        for camera in CAMERAS: (self.directory/camera).mkdir(parents=True)
        self.thread=None
    def capture(self):
        value=request(self.cfg['camera_url']+'/snapshot',timeout=5)
        if set(value['images'])!=set(CAMERAS): raise RuntimeError('Camera identity mismatch')
        for camera in CAMERAS:
            (self.directory/camera/('%08d.jpg'%self.count)).write_bytes(base64.b64decode(value['images'][camera],validate=True))
        with (self.directory/'frames.jsonl').open('a') as output:
            output.write(json.dumps({'frame':self.count,'metadata':value['metadata'],'received':time.monotonic()})+'\n')
        self.received=time.monotonic(); self.count+=1
    def start(self):
        self.capture()
        def loop():
            while not self.stop_event.wait(.5):
                try: self.capture()
                except Exception as exc: self.error=exc; return
        self.thread=threading.Thread(target=loop,daemon=True); self.thread.start()
    def check(self):
        if self.error is not None: raise RuntimeError('Camera recording failed: '+str(self.error))
        if time.monotonic()-self.received>3: raise RuntimeError('Camera recording is stale')
    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(6)
            if self.thread.is_alive(): raise RuntimeError('Camera recorder did not stop')
        if not self.count: return
        for camera in CAMERAS:
            subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y',
                '-framerate','2','-i',str(self.directory/camera/'%08d.jpg'),'-c:v','libx264',
                '-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',str(self.directory/(camera+'.mp4'))],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',choices=['ros'],default='ros'); parser.add_argument('--config')
    parser.add_argument('--check',action='store_true',help='Read-only readiness; no model prediction or motion')
    args=parser.parse_args(); cfg=config(args.config)
    lock=open(Path(cfg['run_dir'])/'executor.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from halo_ros_runtime import RosState
    feedback=RosState(); recorder=None; executor=None; prediction_attempted=False
    directory=None; outcome={'status':'starting','backend':cfg['backend'],'automatic_reset':False}
    def interrupt(signum,frame): raise KeyboardInterrupt('Operator stopped ROS Halo')
    signal.signal(signal.SIGINT,interrupt); signal.signal(signal.SIGTERM,interrupt)
    try:
        initial=feedback.ready()
        health=request(cfg['policy_url']+'/health')
        if (health.get('protocol')!=PROTOCOL or health.get('model')!=cfg['model']
                or health.get('status')!='waiting_for_halo' or health.get('predictions')!=0
                or health.get('robot_control') is not False):
            raise RuntimeError('Wrong or already-used ROS action service; no implicit replay')
        camera=request(cfg['camera_url']+'/health')
        if camera.get('ready') is not True: raise RuntimeError('Cameras not ready')
        if args.check:
            print(json.dumps({'ready':True,'robot_motion_sent':False,'model_inference_requested':False,
                'backend':cfg['backend'],'initial_state':initial,'policy':health},ensure_ascii=False),flush=True)
            return 0
        from frankapy import FrankaArm
        from trajectory_executor_ros import RosChunkExecutor
        directory=Path(tempfile.mkdtemp(prefix='run_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_',dir=cfg['run_dir']))
        save(directory/'config.json',cfg); save(directory/'initial_state.json',initial)
        recorder=Recorder(cfg,directory/'cameras'); recorder.start()
        # Constructor creates clients only; it does not home/reset/open the robot.
        arm=FrankaArm(init_node=False)
        signal.signal(signal.SIGINT,interrupt); signal.signal(signal.SIGTERM,interrupt)
        executor=RosChunkExecutor(arm,feedback,cfg,directory,recorder.check)
        print('ROS action-chunk Halo: '+str(directory),flush=True)
        print('Task: '+health['task'],flush=True)
        for index in range(cfg['max_chunks']+1):
            state=feedback.ready(3)
            observation=request(cfg['camera_url']+'/snapshot',timeout=5)
            payload={'exp_id':directory.name,'state':state['state'],'images':observation['images'],
                     'camera_metadata':observation['metadata'],'execution_feedback':state,
                     'last_gripper_command':executor.previous_gripper,
                     'last_grasp_result_success':executor.last_grasp_result}
            save(directory/('chunk_%04d_observation.json'%(index+1)),payload)
            prediction_attempted=True
            response=request(cfg['policy_url']+'/predict',payload,timeout=cfg['timeout_s']+45)
            if (response.get('protocol')!=PROTOCOL or response.get('exp_id')!=directory.name
                    or response.get('chunk_id')!=index+1 or response.get('model')!=cfg['model']):
                raise RuntimeError('Model response identity mismatch')
            validate({key:response[key] for key in ('status','stage','message','act')},state['state'])
            save(directory/('chunk_%04d.json'%(index+1)),response)
            print(json.dumps(response,ensure_ascii=False),flush=True)
            if response['done']:
                outcome.update(status='completed_by_model_visual_assessment',chunks_executed=index)
                break
            if index==cfg['max_chunks']: raise RuntimeError('Action-chunk budget exhausted')
            result=executor.execute(response['act'],index+1)
            save(directory/('chunk_%04d_result.json'%(index+1)),result)
        return 0
    except BaseException as exc:
        outcome.update(status='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',
                       error=type(exc).__name__+': '+str(exc))
        print(json.dumps(outcome,ensure_ascii=False),flush=True)
        return 1
    finally:
        if executor is not None:
            try: executor.cancel()
            except Exception as exc: outcome['stop_confirmation_error']=str(exc)
        if directory is not None:
            try: outcome['final_state']=feedback.read()
            except Exception as exc: outcome['final_state_error']=str(exc)
            save(directory/'outcome.json',outcome)
        if prediction_attempted and outcome['status']!='completed_by_model_visual_assessment':
            try: request(cfg['policy_url']+'/abort',{'exp_id':directory.name},timeout=3)
            except Exception as exc: outcome['abort_error']=str(exc)
        if recorder is not None:
            try: recorder.stop()
            except Exception as exc: outcome['recording_error']=str(exc)
        if directory is not None: save(directory/'outcome.json',outcome)
        feedback.close(); lock.close()


if __name__=='__main__': raise SystemExit(main())
