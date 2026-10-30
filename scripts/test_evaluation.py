#!/usr/bin/env python3
import csv
import tempfile
from pathlib import Path
import unittest
import numpy as np
from evaluate import alignment, associate, quaternion_rotation, ground_truth, evaluate

class EvaluationTests(unittest.TestCase):
    def setUp(self):
        self.x=np.array([[0.,0,0],[1,0,0],[1,2,0],[0,1,1]])
    def test_rigid_rotation_translation(self):
        rotation=quaternion_rotation([np.cos(.3),0,0,np.sin(.3)])
        y=self.x@rotation+np.array([4.,-3,2])
        aligned,scale=alignment(self.x,y)
        self.assertLess(np.linalg.norm(aligned-y),1e-10);self.assertEqual(scale,1)
    def test_no_metric_scale_fit(self):
        aligned,scale=alignment(self.x,2*self.x)
        self.assertGreater(np.linalg.norm(aligned-2*self.x),.5);self.assertEqual(scale,1)
    def test_sim3_explicit(self):
        aligned,scale=alignment(self.x,2*self.x)
        self.assertEqual(scale,1)
        aligned,scale=alignment(self.x,2*self.x,True)
        self.assertAlmostEqual(scale,2);self.assertLess(np.linalg.norm(aligned-2*self.x),1e-10)
    def test_integer_association(self):
        base=1400000000000000000
        idx,gt=associate(np.array([base+1,base+5,base+20],dtype=np.int64),np.array([base,base+10],dtype=np.int64),np.array([[0.,0,0],[10,0,0]]))
        np.testing.assert_array_equal(idx,[0,1]);np.testing.assert_allclose(gt[:,0],[1,5])
    def test_gap_rejected(self):
        idx,_=associate(np.array([20]),np.array([1,100]),np.array([[0.,0,0],[1,0,0]]),10)
        self.assertEqual(len(idx),0)
    def test_camera_lever_ground_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'cal.yaml').write_text('T_BS: {data: [1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1]}')
            (root/'gt.csv').write_text('#timestamp,px,py,pz,qw,qx,qy,qz\n100,2,3,4,1,0,0,0\n200,2,3,4,0,0,0,1\n')
            times,centres=ground_truth(root/'gt.csv',root/'cal.yaml')
            np.testing.assert_array_equal(times,[100,200])
            np.testing.assert_allclose(centres,[[3,3,4],[1,3,4]],atol=1e-12)
    def test_missing_scale_is_not_replaced_by_ground_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=root/'run';run.mkdir()
            (root/'cam0').mkdir();(root/'state_groundtruth_estimate0').mkdir()
            (root/'cam0/sensor.yaml').write_text('T_BS: {data: [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}')
            (root/'state_groundtruth_estimate0/data.csv').write_text('100,0,0,0,1,0,0,0\n200,2,0,0,1,0,0,0\n300,2,2,0,1,0,0,0\n')
            with (run/'trajectory.csv').open('w',newline='') as f:
                writer=csv.writer(f)
                writer.writerow(['timestamp_ns','raw_x','raw_y','raw_z','metric_x','metric_y','metric_z','metric_valid','connected'])
                for stamp,x,y in [(100,0,0),(200,1,0),(300,1,1)]:writer.writerow([stamp,x,y,0,'','','',0,1])
            (run/'run_summary.csv').write_text('metric_status\ninsufficient_scale_windows\n')
            result=evaluate(run,root,True)
            self.assertEqual(result['metric_ate_rmse_m'],'')
            self.assertEqual(result['metric_evaluation_status'],'scale_not_established_or_visual_gap')
            self.assertAlmostEqual(result['sim3_fitted_scale'],2)
            self.assertEqual(result['sim3_status'],'diagnostic_shape_only_not_recovered_scale')
    def test_degenerate_alignment(self):
        with self.assertRaises(ValueError):alignment(np.zeros((3,3)),np.ones((3,3)))

if __name__=='__main__':unittest.main()
