import os
import shutil
import unittest
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg
from pathlib import Path

HERE = Path(__file__).resolve().parent


class TestSSCRunDemoFull(unittest.TestCase):
    def test_full_demo_generates_montage_and_meta(self):
        module_path = HERE / 'ssc_run_demo_full.py'
        spec = importlib.util.spec_from_file_location('ssc_run_demo_full', str(module_path))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        tmp_out = str(HERE / 'tmp_full_out')
        if os.path.exists(tmp_out):
            shutil.rmtree(tmp_out)

        # load helper config from ssc_run_demo
        ssc_spec = importlib.util.spec_from_file_location('ssc_run_demo', str(HERE / 'ssc_run_demo.py'))
        ssc_mod = importlib.util.module_from_spec(ssc_spec)
        ssc_spec.loader.exec_module(ssc_mod)

        cfg = ssc_mod.Config(map_size=(120, 40, 30), map_resolution=(0.5, 0.5, 0.2))
        cfg.init_vs = 3.0

        res = mod.build_rich_scene_and_run(cfg, tmp_out)

        self.assertIn('montage', res)
        self.assertIn('meta', res)
        self.assertTrue(os.path.exists(res['meta']))
        if res['montage']:
            repo_montage = HERE / 'ssc_demo_full_montage.png'
            self.assertTrue(repo_montage.exists())
            img = mpimg.imread(str(repo_montage))
            self.assertIsNotNone(img)
            self.assertGreater(np.asarray(img).sum(), 0)

        # cleanup
        if os.path.exists(tmp_out):
            shutil.rmtree(tmp_out)


if __name__ == '__main__':
    unittest.main()
