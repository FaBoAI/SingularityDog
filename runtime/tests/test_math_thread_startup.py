"""Startup environment tests; no model, device, or motor access."""
import os
from unittest.mock import patch
import unittest

from singularitydog_hw import math_thread_startup as startup


class MathThreadStartupTests(unittest.TestCase):
    def test_opt_in_sets_exact_values_and_reports_them(self):
        with patch.dict(os.environ, {'OMP_NUM_THREADS':'8','OPENBLAS_NUM_THREADS':'4',
                                     'MKL_NUM_THREADS':'2'}), \
             patch.object(startup,'_loaded_math_modules',return_value=()):
            result=startup.configure_single_thread_math(True)
            self.assertEqual(result,{'selected':True,
                'effective_env':{name:'1' for name in startup.THREAD_ENV_NAMES},
                'verified_before_math_import':True})
            self.assertEqual(startup.verify_before_math_import(),result['effective_env'])

    def test_default_does_not_change_existing_environment(self):
        before={'OMP_NUM_THREADS':'6','OPENBLAS_NUM_THREADS':'3','MKL_NUM_THREADS':'5'}
        with patch.dict(os.environ,before):
            result=startup.configure_single_thread_math(False)
            self.assertEqual(result['effective_env'],before)
            self.assertFalse(result['selected'])
            self.assertEqual(startup.effective_math_thread_env(),before)

    def test_late_import_rejected_before_any_environment_change(self):
        before={'OMP_NUM_THREADS':'6','OPENBLAS_NUM_THREADS':'3','MKL_NUM_THREADS':'5'}
        with patch.dict(os.environ,before), \
             patch.object(startup,'_loaded_math_modules',return_value=('numpy','torch')):
            with self.assertRaisesRegex(startup.MathThreadStartupError,
                                        'before NumPy/Torch import'):
                startup.configure_single_thread_math(True)
            self.assertEqual(startup.effective_math_thread_env(),before)

    def test_recheck_rejects_intervening_import_or_environment_change(self):
        with patch.dict(os.environ,{name:'1' for name in startup.THREAD_ENV_NAMES}):
            with patch.object(startup,'_loaded_math_modules',return_value=('torch',)):
                with self.assertRaisesRegex(startup.MathThreadStartupError,'before NumPy/Torch import'):
                    startup.verify_before_math_import()
            os.environ['MKL_NUM_THREADS']='2'
            with patch.object(startup,'_loaded_math_modules',return_value=()):
                with self.assertRaisesRegex(startup.MathThreadStartupError,'environment changed'):
                    startup.verify_before_math_import()


if __name__=='__main__':unittest.main()
