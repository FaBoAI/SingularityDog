"""Opt-in cached-view experiment; the existing build/loader stays unchanged."""

from .generator import generate_core, verify_aliases

__all__ = ["generate_core", "verify_aliases"]
