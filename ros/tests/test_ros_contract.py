"""Offline contract/service tests. No robot, camera, model call or listeners."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from halo_ros_protocol import config, validate
from halo_ros_action_service import Service


STATE=[.4,0,.4,1,0,0,0,.0795]
def prediction():
    return {'status':'continue','stage':'test','message':'fixture',
            'act':[[.001,0,0,0,0,0,1] for _ in range(20)]}


class ContractTests(unittest.TestCase):
    def test_exact_20x7(self):
        value=prediction(); self.assertIs(validate(value,STATE),value)
    def test_bad_shape_and_nonfinite(self):
        for rows in ([],[[0]*7]*19,[[0]*6]*20,[[float('nan')]*7]*20):
            value=prediction(); value['act']=rows
            with self.assertRaises(ValueError): validate(value,STATE)
    def test_bounds_and_binary_gripper(self):
        for index,value in ((0,.051),(3,.101),(6,.5),(6,True)):
            result=prediction(); result['act'][0][index]=value
            with self.assertRaises(ValueError): validate(result,STATE)
    def test_terminal_no_actions(self):
        value=prediction(); value['status']='done'
        with self.assertRaises(ValueError): validate(value,STATE)
        value['act']=[]; validate(value,STATE)
    def test_no_arbitrary_z_floor(self):
        value=prediction(); value['act']=[[0,0,-.001,0,0,0,1] for _ in range(20)]
        validate(value,[.4,0,.01,1,0,0,0,.0795])
    def test_path_length_limit(self):
        value=prediction(); value['act']=[[.011*(-1)**i,0,0,0,0,0,1] for i in range(20)]
        with self.assertRaises(ValueError): validate(value,STATE)


class ServiceTests(unittest.TestCase):
    def service(self,directory,predictor):
        cfg=config(); cfg['run_dir']=directory; cfg['memory_dir']=directory
        cfg['task_prompt_file']=str(Path(directory)/'task.txt'); Path(cfg['task_prompt_file']).write_text('fixture task')
        with patch('halo_ros_action_service.load_reference_set',return_value=({'id':'fixture'},[])):
            return Service(cfg,predictor)
    def payload(self):
        return {'exp_id':'offline','state':STATE,'images':{c:'fixture' for c in ('kinect_0','kinect_1','realsense_0','realsense_1')}}
    def test_waits_until_predict_and_preserves_rows(self):
        calls=[]
        def fake(*args,**kwargs):
            calls.append((args,kwargs)); value=prediction(); return value,{},json.dumps(value)
        with tempfile.TemporaryDirectory() as directory:
            service=self.service(directory,fake)
            self.assertEqual(service.health()['predictions'],0); self.assertFalse(calls)
            with patch('halo_ros_action_service.decode_camera_images'):
                result=service.predict(self.payload())
            self.assertEqual(result['act'],prediction()['act']); self.assertEqual(len(calls),1)
            self.assertEqual(service.status,'waiting_for_next_observation')
            self.assertTrue(calls[0][1]['stream']); self.assertEqual(calls[0][1]['max_output_tokens'],120000)
    def test_failure_no_retry(self):
        calls=[]
        def fake(*args,**kwargs): calls.append(1); raise RuntimeError('fixture failure')
        with tempfile.TemporaryDirectory() as directory:
            service=self.service(directory,fake)
            with patch('halo_ros_action_service.decode_camera_images'):
                with self.assertRaises(RuntimeError): service.predict(self.payload())
                with self.assertRaises(RuntimeError): service.predict(self.payload())
            self.assertEqual(len(calls),1)
    def test_reject_other_experiment_and_bad_cameras(self):
        with tempfile.TemporaryDirectory() as directory:
            service=self.service(directory,lambda *a,**kw:None)
            payload=self.payload(); payload['images'].pop('kinect_1')
            with self.assertRaises(ValueError): service.predict(payload)
            self.assertEqual(service.sequence,0)
    def test_done_has_no_robot_side_effect(self):
        value={'status':'done','stage':'complete','message':'fixture visual evidence','act':[]}
        with tempfile.TemporaryDirectory() as directory:
            service=self.service(directory,lambda *a,**kw:(copy.deepcopy(value),{},json.dumps(value)))
            with patch('halo_ros_action_service.decode_camera_images'):
                result=service.predict(self.payload())
            self.assertTrue(result['done']); self.assertFalse(service.health()['robot_control'])


if __name__=='__main__': unittest.main()
