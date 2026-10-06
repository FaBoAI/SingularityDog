"""Exact two pinned K37 functions for file-only parity; never loads runtime."""
from concurrent.futures import Future, wait, FIRST_EXCEPTION
from types import SimpleNamespace
import time
dual=SimpleNamespace(SCOPES={"front":None,"rear":None})
BASELINE_SHA256="0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93"

def _readiness_poll_target(now,deadline_ns):
    """Keep a final readiness opportunity without extending the deadline."""
    remaining=deadline_ns-now
    if remaining<=400_000:
        # With >=2 ns remaining, never deliberately sleep to the deadline.
        # Halving also bounds successive tail waits as the budget runs out.
        step=min(50_000,max(1,remaining//2))
    else:
        step=200_000
    return min(deadline_ns,now+step)


def _await_owned_ready(futures,validation_future,*,phase,deadline_ns,deadline_wait=None,
                         clock=time.monotonic_ns,check=lambda:None,
                         thread_clock=time.thread_time_ns):
    """Wait only for readiness; taking/validating results stays with the owner.

    The existing native release wait releases the GIL and spins for targets at
    most 200 us apart, reducing to <=50 us in the last 400 us and then
    halving the remaining budget. It never accepts a result at the deadline.
    It does not read an FD or publish/replace a source time.
    Without that callback, use one bounded all-future condition wait. Result
    and frame/proof validation stay with the existing owners and dispatch gate.
    """
    if set(futures)!=set(dual.SCOPES) or any(not isinstance(f,Future) for f in futures.values()):
        raise ValueError('Exact '+phase.lower()+' owner futures required')
    if validation_future is not None and not isinstance(validation_future,Future):
        raise ValueError(phase+' validation/IMU future required')
    if type(deadline_ns) is not int or deadline_ns<=0 or (deadline_wait is not None and not callable(deadline_wait)):
        raise ValueError('Absolute '+phase.lower()+' join deadline required')
    owners=tuple(futures.values())+(() if validation_future is None else (validation_future,))
    if len({id(future) for future in owners})!=len(owners):
        raise ValueError('Distinct '+phase.lower()+' owner/validation futures required')
    owner_count=len(owners)
    ready_flags=bytearray(owner_count)
    begin=clock();cpu_begin=thread_clock();calls=0
    if type(begin) is not int or begin<=0:
        raise ValueError('Causal '+phase.lower()+' join clock required')
    last_clock=begin;ready_count=0;decision_ns=None;stage='initial'
    last_poll_before_ns=None;last_poll_wake_ns=None;last_poll_returned_ns=None
    try:
        while True:
            stage='owner_readiness'
            decision_ns=None
            ready_count=0
            index=0
            while index<owner_count:
                ready_flags[index]=bool(owners[index].done())
                ready_count+=ready_flags[index]
                index+=1
            # A ready error wins over an unfinished second owner; never wait on it.
            index=0
            while index<owner_count:
                future=owners[index];ready=ready_flags[index];index+=1
                if not ready:continue
                if future.cancelled():raise RuntimeError(phase+' owner future cancelled')
                error=future.exception()
                if error is not None:raise error
            stage='guard_check'
            check()
            stage='decision_clock'
            now=clock()
            if type(now) is not int or now<last_clock:
                raise ValueError('Noncausal '+phase.lower()+' join clock')
            last_clock=now
            decision_ns=now
            if now>=deadline_ns:
                raise TimeoutError(phase+' pipeline exceeded 20 ms hard deadline at '+phase.lower()+' join')
            if ready_count==owner_count:
                stage='completion_clock'
                cpu_end=thread_clock();end=clock()
                if type(end) is not int or end<now or cpu_end<cpu_begin:
                    raise ValueError('Noncausal '+phase.lower()+' join completion clock')
                last_clock=end
                decision_ns=end
                if end>=deadline_ns:
                    raise TimeoutError(phase+' pipeline exceeded 20 ms hard deadline at '+phase.lower()+' join')
                return {'mode':'native_readiness_poll_v1' if deadline_wait is not None else 'bounded_future_wait_v1',
                        'native_tick_max_us':200 if deadline_wait is not None else None,
                        'native_tail_window_us':400 if deadline_wait is not None else None,
                        'native_tail_tick_max_us':50 if deadline_wait is not None else None,
                        'wait_calls':calls,'begin_ns':begin,'end_ns':end,
                        'thread_cpu_begin_ns':cpu_begin,'thread_cpu_end_ns':cpu_end,
                        'future_results_taken_only_after_ready':True}
            if deadline_wait is None:
                stage='condition_wait'
                wait(owners,timeout=(deadline_ns-now)/1e9,return_when=FIRST_EXCEPTION)
            else:
                wake=_readiness_poll_target(now,deadline_ns)
                last_poll_before_ns=now;last_poll_wake_ns=wake;last_poll_returned_ns=None
                stage='native_wait';calls+=1
                try:deadline_wait(wake)
                except BaseException:
                    # Cancellation can wake the native wait after an owner failed.
                    # Retain that original owner error rather than hiding it with
                    # the cancellation notification raised by the wait callback.
                    for future in owners:
                        if future.done() and not future.cancelled():
                            error=future.exception()
                            if error is not None:raise error
                    raise
                stage='native_return_clock'
                returned=clock()
                last_poll_returned_ns=returned if type(returned) is int and returned>0 else None
                if type(returned) is not int or returned<wake:
                    raise ValueError('Native '+phase.lower()+' readiness wait returned before requested wake')
                last_clock=returned
            if deadline_wait is None:calls+=1
    except BaseException as error:
        try:
            error.readiness_poll_failure={
                'schema':'singularitydog.readiness-poll-failure.v1',
                'phase':phase,'stage':stage,'deadline_ns':deadline_ns,
                'begin_ns':begin,'last_checked_clock_ns':last_clock,
                'last_poll_before_ns':last_poll_before_ns,
                'last_poll_wake_ns':last_poll_wake_ns,
                'last_poll_returned_ns':last_poll_returned_ns,
                'decision_ns':decision_ns if stage in ('decision_clock','completion_clock') else None,
                'wait_calls_attempted':calls,'last_ready_count':ready_count,
                'owner_count':owner_count,'native_wait_selected':deadline_wait is not None,
                'native_tick_max_us':200 if deadline_wait is not None else None,
                'native_tail_window_us':400 if deadline_wait is not None else None,
                'native_tail_tick_max_us':50 if deadline_wait is not None else None,
                'decision_is_owner_completion_time':False,
                'source_timestamps_changed':False}
        except BaseException:
            pass
        raise
