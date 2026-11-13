"""Pure initial ROS geometry; no ROS imports, nodes, drivers, devices or GPU."""
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from ros_chunk_geometry import RosChunkGeometry
class GeometryTests(unittest.TestCase):
    def setUp(self):self.g=RosChunkGeometry();self.pose=SimpleNamespace(translation=np.array([.4,.1,.4]),rotation=np.diag([1.,-1.,-1.]))
    def test_base_translation(self):
        a=np.array([[.01,0,-.02,0,0,0,1],[0,.01,0,0,0,0,1]])
        w=self.g.delta_to_absolute_waypoints(a,self.pose);t=self.g.build_cartesian_trajectory(w,40/15,1/15)
        np.testing.assert_allclose(t['translations'][0],self.pose.translation,atol=1e-12)
        np.testing.assert_allclose(t['translations'][-1],[.41,.11,.38],atol=1e-12)
    def test_local_rotation_right_multiply(self):
        w=self.g.delta_to_absolute_waypoints(np.array([[0,0,0,0,0,.1,1]]),self.pose)
        np.testing.assert_allclose(w[-1]['rotation'],self.pose.rotation@Rotation.from_euler('xyz',[0,0,.1]).as_matrix(),atol=1e-12)
    def test_minimum_jerk_endpoints(self):
        s,t=self.g.compute_min_jerk_scalar(40/15,1/15)
        self.assertEqual(len(s),41);self.assertAlmostEqual(s[0],0);self.assertAlmostEqual(s[-1],1)
    def test_slerp_endpoints(self):
        end=self.pose.rotation@Rotation.from_euler('xyz',[.05,-.02,.1]).as_matrix()
        r=self.g.interpolate_rotation_slerp(self.pose.rotation,end,np.array([0.,1.]))
        np.testing.assert_allclose(r[0],self.pose.rotation,atol=1e-12);np.testing.assert_allclose(r[-1],end,atol=1e-12)
if __name__=='__main__':unittest.main()
