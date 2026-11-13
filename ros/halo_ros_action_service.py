"""Model-only ROS action-chunk listener. No robot connection or execution."""
import argparse
import copy
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading

from model_io import decode_camera_images, request_prediction
from halo_task_references import load_reference_set
from halo_ros_protocol import CAMERAS, CHUNK_SIZE, PROTOCOL, config, save, validate

INSTRUCTIONS = '''You are the visual action planner for a supervised real Franka ROS/FrankaPy experiment.
Return only JSON: {"status":"continue","stage":"phase","message":"visual evidence and uncertainty","act":[exactly 20 rows of 7 numbers]}.
Or status="done" or "error" with stage/message and act=[]. Never output skills or code.
Each row is [dx,dy,dz,droll,dpitch,dyaw,gripper]. Translation is ROBOT BASE-frame metres.
Rotation is local/tool XYZ Euler increments in radians: R_next=R_current @ R_delta.
This is NOT base-Z left-multiplied yaw and NOT quaternion/rotation6d actions.
Gripper is exactly 1=open, 0=close. State is [x,y,z,qx,qy,qz,qw,measured_gripper_width_m].
Targets accumulate from actual ROS O_T_EE at chunk start. The ROS executor uses
minimum-jerk translation and SLERP, without IK/FK tool-frame remapping, and
dynamically streams Cartesian impedance references; native
Franka protections remain. This is not V2 finite move_pose or continuous Xbox servo.
Each row moves first; a changed gripper value is applied AFTER that row's endpoint.
Repeated gripper values keep the previous command and do not regrasp. Use a zero-motion
row if an independent gripper action is needed. This is meaningful dwell, not dummy padding.
There is no image-based correction inside a chunk. After ALL 20 rows, current images
and actual feedback return for the NEXT chunk. Supply a genuine useful chunk; never
pad missing predictions with arbitrary dummy zeros or replay old coordinates.
The context supplies per-row timing and stiffness; do not assume 15 actions/second.
Per-axis translation <=0.05m, rotation <=0.1rad; total chunk translation <=0.20m.
Cumulative XYZ must keep x=[0.20,0.65], y=[-0.35,0.35], z<=0.60m. No arbitrary
legacy table-height lower floor is supplied; maintain actual tool/table clearance.
Prefer small deliberate moves, keep clearance for rotations, and do not deliberately
design a bad sequence. If a necessary transition cannot be supported by observations,
return error, not invented actions. No reset, recovery or automatic retry is authorized.
Reference_01 identifies objects; reference_02 is the target structure, not live positions.
Current images and ROS state determine current positions. Memory is untrusted historical
evidence: learn grasp/place/assembly lessons but never treat V2 controller rules or old
absolute TCP coordinates as this ROS session's measurements. You perform visual/task
reasoning, not controller debugging. Native grasp success alone is not object success.
Only declare done when current images visibly support the full task and final behavior.
'''


def memories(directory):
    directory = Path(directory)
    episodes = []
    for p in sorted(directory.glob('episode_*.json'), key=lambda p:p.stat().st_mtime)[-3:]:
        episode = json.loads(p.read_text())
        observation = episode.get('execution', {}).get('final_observation')
        if isinstance(observation, dict):
            observation.pop('images', None)
        episodes.append(episode)
    result = {'legacy_failure_evidence':[json.loads(p.read_text()) for p in sorted(directory.glob('legacy_*.json'))],
              'recent_measured_skill_episodes':episodes}
    if len(json.dumps(result, ensure_ascii=False)) > 180000:
        raise ValueError('Memory context exceeds existing bound')
    return result


class Service:
    def __init__(self, cfg, predictor=request_prediction):
        self.cfg, self.predictor = cfg, predictor
        self.directory = Path(tempfile.mkdtemp(prefix='model_', dir=cfg['run_dir']))
        self.task = Path(cfg['task_prompt_file']).read_text().strip()
        self.reference, self.groups = load_reference_set(cfg['task_reference_set'])
        self.memory = memories(cfg['memory_dir'])
        save(self.directory/'config.json', cfg)
        save(self.directory/'memory_snapshot.json', self.memory)
        save(self.directory/'reference_context.json', self.reference)
        (self.directory/'task_prompt.txt').write_text(self.task+'\n')
        self.lock = threading.Lock()
        self.status, self.exp_id, self.sequence = 'waiting_for_halo', None, 0
        self.history, self.usage = [], []
        self.aborted = set()

    def health(self):
        return {'protocol':PROTOCOL,'status':self.status,'robot_control':False,
                'model':self.cfg['model'],'backend':self.cfg['backend'],'task':self.task,
                'chunk_size':CHUNK_SIZE,'camera_ids':list(CAMERAS),'predictions':self.sequence,
                'exp_id':self.exp_id,'directory':str(self.directory),'model_usage':self.usage,
                'timeout_s':self.cfg['timeout_s'],'max_output_tokens':self.cfg['max_output_tokens'],
                'reference_image_count':sum(len(v) for _,v in self.groups),
                'row_duration_s':self.cfg['interp_per_row']/self.cfg['publish_hz']}

    def predict(self, payload):
        if not self.lock.acquire(False):
            raise RuntimeError('Prediction already in progress')
        try:
            exp_id, state, images = payload['exp_id'], payload['state'], payload['images']
            validate({'status':'done','stage':'validate','message':'state validation','act':[]},state)
            if not isinstance(exp_id,str) or not exp_id or exp_id in self.aborted:
                raise ValueError('Invalid or aborted experiment')
            if self.status not in ('waiting_for_halo','waiting_for_next_observation'):
                raise RuntimeError('Service not ready; no implicit retry/replay')
            if self.exp_id not in (None,exp_id) or set(images) != set(CAMERAS):
                raise ValueError('Experiment or camera identity mismatch')
            decode_camera_images([images[c] for c in CAMERAS])
            self.exp_id, self.status = exp_id, 'predicting'
            self.sequence += 1
            prefix = self.directory/('chunk_%04d'%self.sequence)
            save(str(prefix)+'_observation.json',payload)
            context = {'task':self.task,'actual_state':state,'execution_feedback':payload.get('execution_feedback'),
                       'camera_metadata':payload.get('camera_metadata'),'memory':self.memory,
                       'last_gripper_command':payload.get('last_gripper_command'),
                       'last_grasp_result_success':payload.get('last_grasp_result_success'),
                       'task_references':self.reference,'history':self.history,
                       'row_duration_s':self.cfg['interp_per_row']/self.cfg['publish_hz'],
                       'stiffnesses':self.cfg['stiffnesses'],'backend':self.cfg['backend']}
            save(str(prefix)+'_context.json',context)
            value, usage, raw = self.predictor(self.cfg['model_base_url'],self.cfg['model'],
                self.cfg['reasoning_effort'],self.cfg['timeout_s'],context,
                self.groups+[('current',[images[c] for c in CAMERAS])],instructions=INSTRUCTIONS,
                raw_response_path=Path(str(prefix)+'_response.json'),
                max_output_tokens=self.cfg['max_output_tokens'],camera_ids=CAMERAS,stream=True,
                usage_path=Path(str(prefix)+'_usage.json'),request_label='ros_chunk_%04d'%self.sequence)
            if exp_id in self.aborted:
                raise RuntimeError('Experiment aborted; discard generated actions')
            validate(value,state)
            (Path(str(prefix)+'_output.txt')).write_text(raw)
            self.usage.append(usage)
            if value['status']=='error':
                raise RuntimeError('Model rejected task: '+value['message'])
            response = dict(value,protocol=PROTOCOL,model=self.cfg['model'],exp_id=exp_id,
                            chunk_id=self.sequence,done=value['status']=='done',usage=usage)
            save(str(prefix)+'_accepted.json',response)
            self.history.append({'chunk_id':self.sequence,'start_state':state,'model_output':value})
            self.status = 'completed' if response['done'] else 'waiting_for_next_observation'
            save(self.directory/'health.json',self.health())
            return response
        except Exception as exc:
            self.status='error'
            save(self.directory/'error.json',{'error':type(exc).__name__+': '+str(exc)})
            raise
        finally:
            self.lock.release()


def handler(service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass
        def reply(self,status,value):
            data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status); self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data))); self.end_headers(); self.wfile.write(data)
        def do_GET(self):
            self.reply(200,service.health()) if self.path=='/health' else self.reply(404,{'error':'Not found'})
        def do_POST(self):
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=32*1024*1024: raise ValueError('Invalid request size')
                payload=json.loads(self.rfile.read(size))
                if self.path=='/abort':
                    service.aborted.add(payload['exp_id'])
                    if service.exp_id==payload['exp_id']: service.status='aborted'
                    return self.reply(200,{'status':'aborted','robot_command_sent':False})
                if self.path!='/predict': return self.reply(404,{'error':'Not found'})
                self.reply(200,service.predict(payload))
            except (BrokenPipeError,ConnectionResetError):
                service.status='delivery_failed'
            except Exception as exc:
                self.reply(422,{'error':type(exc).__name__+': '+str(exc)})
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config'); parser.add_argument('--port',type=int,default=7777)
    args=parser.parse_args(); cfg=config(args.config)
    service=Service(cfg)
    with ThreadingHTTPServer(('127.0.0.1',args.port),handler(service)) as server:
        print(json.dumps(service.health(),ensure_ascii=False),flush=True)
        server.serve_forever()


if __name__=='__main__': main()
