"""Independent human-hold dispatch; PLAN unless explicitly executing new proof.

This dispatcher itself creates no serial, IMU, model, native or audio capability.
The separate foreground must own and authenticate those capabilities and frozen
input/native/source receipts. The proposed implementation is deliberately absent
from this tree until it is reviewed and put in a new frozen kit.
"""
from importlib import import_module
from pathlib import Path

from . import profile


def plan(prepared):
    value=profile.verify_prepared(prepared)
    result=value.plain_plan()
    result.update(status='PLAN_HUMAN_HOLD30_NOT_EXECUTED',opens_devices=False,
                  loads_library_model_or_torch=False,existing_R3_execution_changed=False)
    return result


def run(prepared, *, execute=False, **capabilities):
    if type(execute) is not bool:
        raise ValueError('Explicit execute boolean required')
    if execute is not True:
        return plan(prepared)
    # This checks new primary raw prerequisites, conditions, model, source and
    # integration receipt before importing the separately selected control loop.
    profile.runtime_spec(prepared)
    selected=import_module('experiments.four_bus_human_hold.type1_runner')
    profile.verify_runner_source(prepared,selected.__file__)
    return selected.run(prepared,execute=True,**capabilities)
