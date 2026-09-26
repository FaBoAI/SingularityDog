"""ID7-only one-degree raw diagnostic; the other eleven axes hold fresh centers.

This module contains no executable review or hardware launcher. A future
frozen launcher must bind a new same-boot twelve-UID review and a packet gate
before opening either UART. It does not validate model angles or standing.
"""
from .rs05_fullbody_step2 import (ID7_REVIEW_SCHEMA, ID7_REVIEW_SCOPE,
                                  _diagnostic, run_fullbody_step2)


def run_id7_step1(transports, expected_uids, check_interrupt, emit, *,
                  validated_review, preflight_only=True, **timing):
    if _diagnostic(validated_review) != 'id7-step1':
        raise ValueError('ID7-only runner requires its distinct review schema and scope')
    return run_fullbody_step2(transports, expected_uids, check_interrupt, emit,
                              validated_review=validated_review,
                              preflight_only=preflight_only, **timing)
