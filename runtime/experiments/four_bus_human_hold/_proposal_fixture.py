"""Test-only replay of a frozen proposal; never rebase the current runtime.

The materialized human-hold snapshot and its diff are historical, independently
pinned artifacts. Recover the old base in a private copy, then replay the exact
diff. Current boxed sources may evolve without changing that provenance.
"""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


@dataclass(frozen=True)
class FrozenProposal:
    base: bytes
    proposed: bytes


def reconstruct_proposal(manifest, patch, materialized, proposed_key):
    """Verify both endpoints and exact diff replay without touching either."""
    receipt = json.loads(Path(manifest).read_text())
    diff = Path(patch).read_bytes()
    candidate = Path(materialized).read_bytes()
    if hashlib.sha256(diff).hexdigest() != receipt['patch_sha256']:
        raise ValueError('Unreviewed proposed patch')
    if hashlib.sha256(candidate).hexdigest() != receipt[proposed_key]:
        raise ValueError('Unreviewed materialized proposal')
    with tempfile.TemporaryDirectory(prefix='human30-frozen-proposal-test-') as directory:
        file = Path(directory)/'source.py'
        diff_file = Path(directory)/'proposal.patch'
        file.write_bytes(candidate)
        diff_file.write_bytes(diff)
        command = ['patch', '--batch', '--fuzz=0', str(file), '-i', str(diff_file)]
        try:
            subprocess.run(command + ['--reverse'], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError as error:
            raise ValueError('Frozen proposal reverse replay failed') from error
        base = file.read_bytes()
        if hashlib.sha256(base).hexdigest() != receipt['base_sha256']:
            raise ValueError('Unreviewed historical proposal base')
        try:
            subprocess.run(command + ['--forward'], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError as error:
            raise ValueError('Frozen proposal forward replay failed') from error
        if file.read_bytes() != candidate:
            raise ValueError('Proposed replay differs')
    return FrozenProposal(base, candidate)
