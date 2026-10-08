"""Private file-only output notification boundaries; no robot/network I/O."""
from concurrent.futures import CancelledError,Future
from pathlib import Path
import threading,unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace

from singularitydog_hw import native_active_transport as active
from singularitydog_hw import policy_output_runtime as runtime
KIT=Path(__file__).absolute().parents[2]
START=1_000_000_000;DEADLINE=START+1_000_000
class Clock:
 def __init__(self):self.now=START
 def __call__(self):return self.now
class OutputNotificationTests(unittest.TestCase):
 def fixture(self,selected=True):
  w=runtime.BusWorkers.__new__(runtime.BusWorkers);c=Clock();w.clock=c
  w.aborted=threading.Event();w.reason=None;w.native_pair=None
  w.unpaired_output_future_notifications=selected;w.emergency=Mock()
  fs={s:Future() for s in ('front','rear')};values={s:object() for s in fs}
  group=Mock();waiter=Mock();return w,c,fs,values,group,waiter
 def finish(self,f):
  for s,fut in f[2].items():
   if not fut.done():fut.set_result(f[3][s])
 def collect(self,f):
  with patch.object(runtime,'_owned_future_readiness_group',return_value=f[4]) as create:
   r=f[0].collect_output(f[2],deadline_ns=DEADLINE,deadline_wait=f[5]);return r,create
 def notification(self,f):
  def wake(end):
   self.assertEqual(end,DEADLINE);f[1].now+=1_000;self.finish(f)
   return {'kind':'NOTIFIED','actual_ns':f[1].now}
  return wake
 def test_two_true_futures_are_rechecked_after_every_hint(self):
  f=self.fixture();calls=[]
  def wake(end):
   calls.append(end);f[1].now+=1_000
   if len(calls)==1:f[2]['front'].set_result(f[3]['front'])
   if len(calls)==3:f[2]['rear'].set_result(f[3]['rear'])
   return {'kind':'NOTIFIED','actual_ns':f[1].now}
  f[4].wait.side_effect=wake
  with patch.object(f[0],'collect',wraps=f[0].collect) as take:
   r,create=self.collect(f)
  self.assertEqual(r,f[3]);take.assert_called_once_with(f[2]);create.assert_called_once_with(f[5],tuple(f[2].values()))
  self.assertEqual(calls,[DEADLINE]*3);self.assertEqual(f[0].output_notification_groups,1)
  self.assertEqual(f[0].output_notification_waits,3);f[4].close.assert_called_once();f[5].assert_not_called()
 def test_already_ready_has_no_notification_allocation(self):
  f=self.fixture();self.finish(f);r,create=self.collect(f)
  self.assertEqual(r,f[3]);create.assert_not_called();f[4].close.assert_not_called()
 def test_default_off_never_selects_capability(self):
  f=self.fixture(False)
  def wait(end):f[1].now=end;self.finish(f)
  f[5].side_effect=wait;r,create=self.collect(f)
  self.assertEqual(r,f[3]);create.assert_not_called();f[5].assert_called_once();f[4].wait.assert_not_called()
 def test_missing_selection_attribute_keeps_original_default(self):
  f=self.fixture();del f[0].unpaired_output_future_notifications
  f[5].side_effect=lambda end:(setattr(f[1],'now',end),self.finish(f))
  r,create=self.collect(f);create.assert_not_called();self.assertEqual(r,f[3])
 def test_constructor_default_and_selection_validation_before_pools(self):
  with patch.object(runtime,'ThreadPoolExecutor') as pool:
   for value,pair in ((1,False),(None,False),(True,True),(True,1)):
    with self.subTest(value=value,pair=pair),self.assertRaises(RuntimeError):
     runtime.BusWorkers({'front':object(),'rear':object()},lambda:None,
      native_phase_pair=pair,unpaired_output_future_notifications=value)
   pool.assert_not_called()
  w=runtime.BusWorkers({'front':object(),'rear':object()},lambda:None)
  self.assertFalse(w.unpaired_output_future_notifications);self.assertEqual(w.output_notification_groups,0);w.close()
 def test_old_library_fallback_preserves_tail_poll(self):
  f=self.fixture();wakes=[]
  def poll(end):wakes.append(end);f[1].now=end;self.finish(f)
  f[5].side_effect=poll
  with patch.object(runtime,'_owned_future_readiness_group',return_value=None) as create:
   r=f[0].collect_output(f[2],deadline_ns=DEADLINE,deadline_wait=f[5])
  self.assertEqual(r,f[3]);self.assertEqual(wakes,[START+50_000]);create.assert_called_once()
 def test_pair_path_never_constructs_unpaired_notification(self):
  f=self.fixture();pair=Mock();f[0].native_pair=pair;pair.owns_futures.return_value=True
  pair.completion_notification_available=True;pair.publication_complete.return_value=True
  def notify(fs,*,tick_ns,deadline_ns):
   self.assertIs(fs,f[2]);f[1].now=tick_ns;self.finish(f);return {'kind':'TICK','actual_ns':tick_ns}
  pair.wait_completion.side_effect=notify
  r,create=self.collect(f);self.assertEqual(r,f[3]);create.assert_not_called();pair.wait_published.assert_called_once_with(f[2])
 def test_registration_ready_rechecks_without_wait(self):
  f=self.fixture()
  def create(*_):self.finish(f);return f[4]
  with patch.object(runtime,'_owned_future_readiness_group',side_effect=create):
   r=f[0].collect_output(f[2],deadline_ns=DEADLINE,deadline_wait=f[5])
  self.assertEqual(r,f[3]);f[4].wait.assert_not_called();f[4].close.assert_called_once()
 def test_registration_late_or_cancelled_never_takes_results(self):
  for late in (False,True):
   f=self.fixture()
   def create(*_):
    if late:f[1].now=DEADLINE
    else:f[0].aborted.set()
    return f[4]
   with self.subTest(late=late),patch.object(runtime,'_owned_future_readiness_group',side_effect=create),self.assertRaises((RuntimeError,TimeoutError)):
    f[0].collect_output(f[2],deadline_ns=DEADLINE,deadline_wait=f[5])
   f[4].wait.assert_not_called();f[4].close.assert_called_once();f[0].emergency.assert_called_once()
 def test_registration_failure_keeps_original_owner_error(self):
  f=self.fixture();original=OSError('original rear')
  def create(*_):f[2]['rear'].set_exception(original);raise RuntimeError('pipe setup')
  with patch.object(runtime,'_owned_future_readiness_group',side_effect=create),self.assertRaises(OSError) as caught:
   f[0].collect_output(f[2],deadline_ns=DEADLINE,deadline_wait=f[5])
  self.assertIs(caught.exception,original)
 def test_owner_failure_precedes_wait_and_takeout(self):
  f=self.fixture();original=OSError('rear error');f[2]['rear'].set_exception(original)
  with self.assertRaises(OSError) as caught:self.collect(f)
  self.assertIs(caught.exception,original);f[4].wait.assert_not_called()
 def test_native_error_retains_original_failure_priority(self):
  f=self.fixture();original=ValueError('original front')
  def wake(_):f[2]['front'].set_exception(original);raise OSError('cancel wake')
  f[4].wait.side_effect=wake
  with self.assertRaises(ValueError) as caught:self.collect(f)
  self.assertIs(caught.exception,original);f[4].close.assert_called_once()
 def test_bad_and_late_hint_retain_owner_error_priority(self):
  for late in (False,True):
   f=self.fixture();original=OSError('owner')
   def wake(_):
    f[1].now=DEADLINE if late else START+1_000;f[2]['rear'].set_exception(original)
    return {'kind':'BAD','actual_ns':f[1].now}
   f[4].wait.side_effect=wake
   with self.subTest(late=late),self.assertRaises(OSError) as caught:self.collect(f)
   self.assertIs(caught.exception,original)
 def test_original_future_cancellation_is_not_readiness(self):
  f=self.fixture()
  def wake(_):f[1].now+=1_000;f[2]['rear'].cancel();return {'kind':'NOTIFIED','actual_ns':f[1].now}
  f[4].wait.side_effect=wake
  with self.assertRaises(CancelledError):self.collect(f)
  f[4].close.assert_called_once()
 def test_hint_does_not_override_abort(self):
  f=self.fixture()
  def wake(_):self.finish(f);f[0].aborted.set();return {'kind':'NOTIFIED','actual_ns':f[1].now}
  f[4].wait.side_effect=wake
  with self.assertRaisesRegex(RuntimeError,'aborted'):self.collect(f)
  f[4].close.assert_called_once()
 def test_invalid_event_cannot_take_pending_result(self):
  for event in ({'kind':'READY','actual_ns':START},{'kind':'NOTIFIED','actual_ns':True},
                {'kind':'NOTIFIED','actual_ns':START-1},{'kind':'NOTIFIED','actual_ns':START+1},
                {'kind':'DEADLINE','actual_ns':START},None):
   f=self.fixture();f[4].wait.return_value=event
   with self.subTest(event=event),patch.object(f[0],'collect',side_effect=AssertionError('takeout')),self.assertRaises(RuntimeError):self.collect(f)
   f[4].close.assert_called_once()
 def test_success_at_original_deadline_is_rejected(self):
  f=self.fixture()
  def wake(_):f[1].now=DEADLINE;self.finish(f);return {'kind':'NOTIFIED','actual_ns':DEADLINE}
  f[4].wait.side_effect=wake
  with self.assertRaisesRegex(TimeoutError,'coordinator deadline'):self.collect(f)
  f[4].close.assert_called_once()
 def test_success_one_ns_before_deadline_is_accepted_only_with_cleanup_before(self):
  f=self.fixture()
  def wake(_):f[1].now=DEADLINE-1;self.finish(f);return {'kind':'NOTIFIED','actual_ns':DEADLINE-1}
  f[4].wait.side_effect=wake;r,_=self.collect(f);self.assertEqual(r,f[3]);f[4].close.assert_called_once()
 def test_takeout_consumes_original_budget(self):
  f=self.fixture();f[4].wait.side_effect=self.notification(f);original=f[0].collect
  def take(fs):r=original(fs);f[1].now=DEADLINE;return r
  with patch.object(f[0],'collect',side_effect=take),self.assertRaisesRegex(TimeoutError,'takeout'):self.collect(f)
  f[4].close.assert_called_once()
 def test_cleanup_consumes_original_budget_and_checks_cancel(self):
  for late in (False,True):
   f=self.fixture();f[4].wait.side_effect=self.notification(f)
   def close():
    if late:f[1].now=DEADLINE
    else:f[0].aborted.set()
   f[4].close.side_effect=close
   with self.subTest(late=late),self.assertRaises((TimeoutError,RuntimeError)):self.collect(f)
   f[4].close.assert_called_once()
 def test_cleanup_failure_preserves_primary_error_with_note(self):
  f=self.fixture();primary=OSError('wait primary');f[4].wait.side_effect=primary;f[4].close.side_effect=RuntimeError('cleanup')
  with self.assertRaises(OSError) as caught:self.collect(f)
  self.assertIs(caught.exception,primary);self.assertIn('Output readiness cleanup',str(caught.exception.__notes__))
 def test_no_callback_route_keeps_finite_original_wait(self):
  f=self.fixture();self.finish(f)
  with patch.object(runtime,'_owned_future_readiness_group') as group:
   r=f[0].collect_output(f[2],deadline_ns=DEADLINE)
  self.assertEqual(r,f[3]);group.assert_not_called()
 def test_foreign_alias_and_nonfuture_inputs_rejected(self):
  f=self.fixture()
  for fs in ({'front':f[2]['front']},{'front':f[2]['front'],'rear':f[2]['front']},
             {'front':object(),'rear':f[2]['rear']}):
   with self.subTest(fs=fs),self.assertRaises(RuntimeError):f[0].collect_output(fs,deadline_ns=DEADLINE,deadline_wait=f[5])
 def test_default_run_and_cli_do_not_expose_selection(self):
  import inspect
  self.assertNotIn('unpaired_output_future_notifications',inspect.signature(runtime.run_supported_policy).parameters)
  self.assertIn('native_phase_pair',inspect.signature(runtime.run_supported_policy).parameters)
  text=(KIT/'runtime/singularitydog_hw/policy_output.py').read_text()
  self.assertNotIn('unpaired_output_future_notifications',text)

if __name__=='__main__':unittest.main(verbosity=2)
