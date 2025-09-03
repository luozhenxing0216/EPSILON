import os
import shutil
import tempfile
import importlib.util
import unittest
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

class TestSSCRunDemo(unittest.TestCase):
    def test_run_demo_creates_outputs(self):
        # Ensure headless matplotlib backend
        os.environ['MPLBACKEND'] = 'Agg'
        module_path = os.path.join(os.path.dirname(__file__), 'ssc_run_demo.py')
        spec = importlib.util.spec_from_file_location('ssc_run_demo', module_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        # Run demo with small map to keep test fast
        out = mod.run_demo(map_sz=(60,20,10), res=(0.5,0.5,0.2), init_vs=2.0, obs_density=0.01, max_steps=100, out_dir=None)

        self.assertIn('saved', out)
        self.assertGreaterEqual(len(out['saved']), 1)
        for fp in out['saved']:
            self.assertTrue(os.path.exists(fp), f"expected file exists: {fp}")

        # Load images and perform a basic pixel sanity check
        imgs = []
        nonzero_found = False
        for fp in out['saved']:
            try:
                arr = mpimg.imread(fp)
            except Exception:
                # fallback: skip image if unreadable
                arr = None
            self.assertIsNotNone(arr, f"failed to read generated image: {fp}")
            imgs.append(arr)
            if np.asarray(arr).astype(np.float32).sum() > 0:
                nonzero_found = True

        self.assertTrue(nonzero_found, "expected at least one generated image to contain non-zero pixels")

        # Create a horizontal montage of the generated images and save it
        montage_fp = os.path.join(out['out_dir'], 'ssc_demo_montage.png')
        # normalize shapes: convert grayscale to RGB if needed
        norm_imgs = []
        for im in imgs:
            if im is None:
                continue
            a = np.asarray(im)
            if a.ndim == 2:
                a = np.stack([a, a, a], axis=-1)
            if a.dtype != np.uint8:
                # scale floats in [0,1] to 0-255
                if a.max() <= 1.0:
                    a = (a * 255).astype(np.uint8)
                else:
                    a = a.astype(np.uint8)
            norm_imgs.append(a)

        # resize to smallest height among images for simple concatenation
        heights = [im.shape[0] for im in norm_imgs]
        min_h = min(heights) if heights else 0
        resized = []
        for im in norm_imgs:
            if im.shape[0] != min_h:
                # resample using matplotlib's imresize via plt.imshow->savefig trick
                fig = plt.figure(figsize=(im.shape[1] / 100.0, min_h / 100.0), dpi=100)
                ax = fig.add_axes([0, 0, 1, 1])
                ax.axis('off')
                ax.imshow(im)
                tmp_fp = os.path.join(out['out_dir'], '._tmp_montage.png')
                fig.savefig(tmp_fp, dpi=100)
                plt.close(fig)
                im2 = mpimg.imread(tmp_fp)
                resized.append((im2 * 255).astype(np.uint8) if im2.dtype == np.float32 else im2)
                try:
                    os.remove(tmp_fp)
                except Exception:
                    pass
            else:
                resized.append(im)

        if resized:
            montage = np.hstack(resized)
            plt.imsave(montage_fp, montage)
            self.assertTrue(os.path.exists(montage_fp), "montage was not saved")
            # also copy montage into repository so it remains after cleanup for inspection
            repo_montage_fp = os.path.join(os.path.dirname(__file__), 'ssc_demo_montage.png')
            shutil.copyfile(montage_fp, repo_montage_fp)
            self.assertTrue(os.path.exists(repo_montage_fp), "repo montage was not saved")

        # cleanup generated output directory
        if 'out_dir' in out and os.path.exists(out['out_dir']):
            shutil.rmtree(out['out_dir'])

if __name__ == '__main__':
    unittest.main()
