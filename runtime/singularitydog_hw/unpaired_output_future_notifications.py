"""Explicit original-Future output notifications for bounded unpaired owners.

Setup binds the actual authenticated active library and its original wait ABI.
This adds no I/O, motor request, owner or deadline allowance. The profile loader
separately requires current-source STOP-proxy timing and named software review.
"""
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import threading

from . import native_active_transport as active
from .unpaired_native_feedback_codec import _read_reference

SELECTION_SCHEMA='singularitydog.unpaired-output-notification-selection.v1'
PROOF_SCHEMA='singularitydog.unpaired-output-notification-source-binding.v1'
SOURCE_PATH='singularitydog_hw/unpaired_output_future_notifications.py'
REFERENCE_NAMES=frozenset(('library','build_record','library_source','runtime','binding','selection_source'))

def need(value,message):
    if not value:raise ValueError(message)

def verify_source_selection(selection):
    """Verify files/origins only; no native function/device/model is called."""
    need(type(selection) is dict and set(selection)=={'schema','references','output_allowed','approved_for_runtime'} and
         selection['schema']==SELECTION_SCHEMA and selection['output_allowed'] is False and
         selection['approved_for_runtime'] is False,'Explicit output notification source selection grants no output')
    refs=selection['references'];need(type(refs) is dict and set(refs)==REFERENCE_NAMES,'Complete notification source references required')
    pins={name:_read_reference(ref) for name,ref in refs.items()}
    from . import policy_output_runtime as runtime
    for name,module in (('runtime',runtime),('binding',active),('selection_source',__import__(__name__,fromlist=['']))):
        need(pins[name][0]==str(Path(module.__file__).absolute()),'Actual output notification module origin differs')
    build=json.loads(pins['build_record'][1]);library=Path(pins['library'][0])
    need(type(build) is dict and type(build.get('abi')) is int and build['abi']==1 and
         build.get('binary_sha256')==refs['library']['sha256'] and
         build.get('source_sha256')==refs['library_source']['sha256'] and
         pins['build_record'][0]==str(library.parent/'build-record.json') and
         pins['library_source'][0]==str(library.parent/'transport.cpp'),'Original active ABI/source/build notification binding differs')
    return {name:{'path':path,'sha256':refs[name]['sha256']} for name,(path,_) in pins.items()}

def selection_for_authenticated_library(library):
    """Diagnostic setup only; unknown/unpinned loaded libraries are rejected."""
    value=active.verified_active_source_binding(library)
    from . import policy_output_runtime as runtime
    refs={'library':{'path':value.path,'sha256':value.binary_sha256},
          'library_source':{'path':value.source_path,'sha256':value.source_sha256},
          'build_record':{'path':value.build_record_path,'sha256':value.build_record_sha256}}
    for name,module in (('runtime',runtime),('binding',active)):
        refs[name]={'path':str(Path(module.__file__).absolute()),'sha256':hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()}
    refs['selection_source']={'path':str(Path(__file__).absolute()),'sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    return {'schema':SELECTION_SCHEMA,'references':refs,'output_allowed':False,'approved_for_runtime':False}

def verify_loaded_library(library,selection):
    pins=verify_source_selection(selection);value=active.verified_active_source_binding(library)
    expected={'library':(value.path,value.binary_sha256),'library_source':(value.source_path,value.source_sha256),
              'build_record':(value.build_record_path,value.build_record_sha256)}
    need(all(pins[name]=={'path':path,'sha256':digest} for name,(path,digest) in expected.items()),
         'Notification selection differs from actually authenticated library')
    # Authenticated source binding seals the exact original readiness symbols,
    # including absence. The owned waiter separately validates their signatures.
    abi=getattr(library,'sda_future_readiness_abi',None)
    need(abi is not None and getattr(library,'sda_wait_future_ready',None) is not None and abi()==1,
         'Explicit notification selection needs complete original Future readiness ABI1')
    return pins

def prepare_notifications(sessions,waiter,selection=None):
    from . import policy_output_runtime as runtime
    need(type(sessions) is dict and set(sessions)==set(runtime.BUSES) and sessions['front'] is not sessions['rear'] and
         all(type(s) is active.ActiveSession for s in sessions.values()),'Genuine two unpaired active owners required')
    need(type(waiter) is active._OwnedActiveWaiter and waiter._owner is threading.current_thread() and
         waiter.future_readiness_available is True,'Original current-main owned notification waiter required')
    if selection is None:selection=selection_for_authenticated_library(waiter._library)
    pins=verify_loaded_library(waiter._library,selection)
    original_cancel=active.verified_owned_waiter_creation(waiter)
    with ExitStack() as stack:
        for scope in runtime.BUSES:
            session=sessions[scope]
            need(session.busy.acquire(blocking=False),'Notification setup requires idle original owner')
            stack.callback(session.busy.release)
            need(session.first_id==runtime.BUSES[scope][0] and session._phase_pair is None and session._handle and
                 not session.poisoned and session.lib is waiter._library and session._cancel_fd==waiter._cancel_fd,
                 'Notification session/source/cancellation binding differs')
            need(active.verified_active_session_creation(session)==original_cancel,
                 'Session and waiter original cancellation creation identities differ')
            verify_loaded_library(session.lib,selection)
    return {'schema':PROOF_SCHEMA,'references':pins,'active_abi':1,'future_readiness_abi':1,
            'scope':'ordinary_unpaired_output_original_futures.v1','adds_owner_or_future_or_request':False,
            'notifications_are_hints_only':True,'timestamps_and_deadlines_unchanged':True,
            'hardware_timing_improvement_proven':False}
