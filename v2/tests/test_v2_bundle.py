"""Only fake state/model tests; no listeners, devices, model or original V2 SDK."""
import base64, io, json, math, tempfile, unittest
from pathlib import Path
from unittest.mock import Mock
from PIL import Image
from camera_ids import CAMERAS
from halo_skill_service import SkillService, configuration
from v2_state import require_state
from robot_calibration.block_skills_v2 import SkillExecutor, expected_pose, plan_sequence, arm_speeds
from fake_v2 import FakeRobot, robot_state

class GeometryTests(unittest.TestCase):
    def setUp(self):self.pose=robot_state()['actual_pose']
    def test_translate_keeps_actual_z(self):
        p=expected_pose(self.pose,{'op':'translate','position_m':[.42,.02,.403]})
        self.assertEqual(p['position_m'],[.42,.02,.4])
    def test_lift_descend(self):
        p=expected_pose(self.pose,{'op':'lift','distance_m':.15})
        self.assertAlmostEqual(p['position_m'][2],.55)
        p=expected_pose(p,{'op':'descend','distance_m':.1});self.assertAlmostEqual(p['position_m'][2],.45)
    def test_rotate_is_base_left_multiply(self):
        p=expected_pose(self.pose,{'op':'rotate','angle_deg':90})
        self.assertAlmostEqual(p['quaternion_wxyz'][1],math.sqrt(.5));self.assertAlmostEqual(p['quaternion_wxyz'][2],math.sqrt(.5))
    def test_reject_old_compound_and_bad_fields(self):
        for s in [{'op':'descend_grasp','distance_m':.02},{'op':'close','distance_m':.02},{'op':'rotate','angle_deg':181}]:
            with self.assertRaises(ValueError):expected_pose(self.pose,s)
    def test_workspace_and_count(self):
        with self.assertRaises(ValueError):expected_pose(self.pose,{'op':'translate','position_m':[.9,0,.4]})
        with self.assertRaises(ValueError):plan_sequence(self.pose,[])
    def test_half_speed(self):self.assertEqual(arm_speeds(.5),{'horizontal_mps':.07,'vertical_mps':.03,'angular_radps':.4})

class ExecutionTests(unittest.TestCase):
    def executor(self,folder,robot):return SkillExecutor(robot,folder,Mock(),recording=False,arm_speed_scale=.5)
    def test_separate_descent_close_lift(self):
        r=FakeRobot()
        with tempfile.TemporaryDirectory() as folder:
            e=self.executor(folder,r)
            plan=plan_sequence(r.get_state()['actual_pose'],[{'op':'descend','distance_m':.02},{'op':'close'},{'op':'lift','distance_m':.03}])
            for step in plan:e.execute_skill(step)
        self.assertEqual([c[0] for c in r.calls],['pose','grasp','pose'])
        self.assertAlmostEqual(r.calls[0][2][1],1)
    def test_failed_grasp_stops(self):
        r=FakeRobot();r.grasp_confirmed=False
        with tempfile.TemporaryDirectory() as folder:
            e=self.executor(folder,r)
            with self.assertRaisesRegex(RuntimeError,'confirm grasp'):e.execute_skill(plan_sequence(r.get_state()['actual_pose'],[{'op':'close'}])[0])
        self.assertEqual([c[0] for c in r.calls],['grasp'])
    def test_failed_operation_not_success(self):
        r=FakeRobot();r.operation_status='SETTLE_TIMEOUT'
        with tempfile.TemporaryDirectory() as folder:
            e=self.executor(folder,r)
            with self.assertRaisesRegex(RuntimeError,'SETTLE_TIMEOUT'):e.execute_skill(plan_sequence(r.get_state()['actual_pose'],[{'op':'lift','distance_m':.03}])[0])
    def test_contact_lock_blocks(self):
        s=robot_state();s['contact_guard'].update(latched=True,stage='CONTACT_LOCKED')
        with self.assertRaises(RuntimeError):require_state(s,'instance','session')
    def test_no_delta_force_threshold(self):
        s=robot_state();s['feedback']['O_F_ext_hat_K']=[9,0,0,0,0,0]
        require_state(s,'instance','session')
    def test_stale_feedback_blocks(self):
        s=robot_state();s['feedback']['stale']=True
        with self.assertRaises(RuntimeError):require_state(s,'instance','session')

class ModelTests(unittest.TestCase):
    def test_template_configuration_has_no_site_calibration(self):
        cfg=configuration(Path(__file__).parents[1]/'halo_skills.yaml')
        self.assertNotIn('calibration',cfg);self.assertEqual(cfg['model'],'gpt-6-astra');self.assertEqual(cfg['arm_speed_scale'],.5)
    def test_predict_only_once_and_load_empty_memory(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg=configuration(Path(__file__).parents[1]/'halo_skills.yaml')
            cfg.update(memory_dir=folder,run_dir=folder,recording_enabled=False,finish_behavior='hold')
            prediction={'status':'planned','message':'offline fixture','skills':[{'op':'lift','distance_m':.03}]}
            model=Mock(return_value=(prediction,{},json.dumps(prediction)))
            service=SkillService(cfg,'fixture task',model)
            self.assertEqual(service.memories(),{'legacy_failure_evidence':[],'recent_measured_skill_episodes':[]})
            image=Image.new('RGB',(640,360));out=io.BytesIO();image.save(out,format='JPEG');encoded=base64.b64encode(out.getvalue()).decode()
            payload={'exp_id':'offline','actual_state':robot_state(),'images':{c:encoded for c in CAMERAS},'camera_metadata':{}}
            result=service.predict(payload)
            self.assertEqual(result['skills'],prediction['skills']);self.assertEqual(model.call_count,1)
            # Any repeat must not trigger another model inference, irrespective of reuse policy.
            try:service.predict(payload)
            except Exception:pass
            self.assertEqual(model.call_count,1)

if __name__=='__main__':unittest.main()
