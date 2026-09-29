"""Opt-in BLAS/OpenMP thread limits before importing NumPy or Torch.

These process-wide variables are startup settings. Changing them after a math
library has initialized can leave its thread pool unchanged, so late selection
is an error instead of a misleading performance label.
"""
import os
import sys


THREAD_ENV_NAMES = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')


class MathThreadStartupError(RuntimeError):
    pass


def effective_math_thread_env():
    return {name: os.environ.get(name) for name in THREAD_ENV_NAMES}


def _loaded_math_modules():
    return tuple(name for name in ('numpy', 'torch')
                 if any(module == name or module.startswith(name + '.')
                        for module in sys.modules))


def configure_single_thread_math(selected):
    """Apply the exact three-variable setting or leave the environment alone."""
    if type(selected) is not bool:
        raise MathThreadStartupError('Single-thread math selection must be a bool')
    if selected:
        loaded = _loaded_math_modules()
        if loaded:
            raise MathThreadStartupError(
                '--single-thread-math must run before NumPy/Torch import: ' + ', '.join(loaded))
        for name in THREAD_ENV_NAMES:
            os.environ[name] = '1'
        verify_before_math_import()
    return {'selected': selected, 'effective_env': effective_math_thread_env(),
            'verified_before_math_import': selected}


def verify_before_math_import():
    """Recheck immediately before Torch import, catching later startup imports."""
    loaded = _loaded_math_modules()
    if loaded:
        raise MathThreadStartupError(
            '--single-thread-math must run before NumPy/Torch import: ' + ', '.join(loaded))
    effective = effective_math_thread_env()
    if any(value != '1' for value in effective.values()):
        raise MathThreadStartupError('Single-thread math environment changed before Torch import')
    return effective
