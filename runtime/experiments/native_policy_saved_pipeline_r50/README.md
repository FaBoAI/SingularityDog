# R50 archived Observer/FK replay candidate — INCOMPLETE

This is a sealed code-only candidate. Actual Observer/FK model loading, independent
controller reset/state proof, sealed C-copy selection, and timed replay are **not
implemented**. `--mode EXEC` exits 2 before bootstrap, model/native loading,
hardware access or result-directory creation. No performance result or 20 ms
qualification is produced. R49's independently archived live measurements remain
separate evidence. Root stopped this extension after successful live 5/501 runs.

Default `PLAN` reads pinned data/source files and prints JSON without creating
outputs or importing Torch, Observer, device drivers or native extensions. It
binds exact R50 source9+manifest10, R48 source21, R47 source13, R49 source23,
K37 source650, the original FK proof d68…, its five-ref R49 rebind23b…, model
1085…, and snapshot artifact415…. Source dependencies execute only if a future
reviewed EXEC implementation is added. The current entrypoint executes only its
own `inputs.py` and `support.py` from pinned raw bytes, never pyc.

The distinct archived-data adapter accepts exactly the pinned ABORTED R37 report
with 33 completed ticks and the original 34 records. It excludes partial cycle34;
it does not alter the existing COMPLETE_DIAGNOSTIC/501 saved loader. Historical
calibration21eed… overrides the different default K37 calibration. Tick times stay
in original order and must use `measured_diagnostic_ticks=True` in any future
actual Observer construction. Saved dynamic snapshot digests remain per-tick.

`inputs.snapshots` is an uninvoked proposed pure reconstruction helper using the
sealed R48 harness/decoder; it does not construct hardware/model instances. Existing
independent read-only feasibility evidence proved 33 exact dynamic digests.
`support.compare_observed` tests input/output float32 bits and full result/provenance
binary64 bits. Only explicit run counter and profiling metadata (`run_number`,
`consume_profile`, `timing_scope`) are omitted for a future cross-run comparison.
The ABBA labels are a bounded planned order, not timing records. Named recurrent
state and disjoint-controller storage proof still require a future implementation.

`package.py` creates a separate exact source10 archive, an exact4 historical input
archive, PLAN/EXEC command specifications and a receipt. `requested_model_load`
in the EXEC specification describes intent; `model_loaded=False` states actual
status, and `target_run_permitted=False` forbids treating this unfinished candidate
as executable model timing. The four input copies preserve all original bytes and
nested historical references. They are outside the immutable R49 24-member source
inventory. Packaging's direct local closure is distinct from adapter PLAN's
recursive target closure; any pending recursive checks are declared explicitly.

Local tests exercise pinned reads, duplicate/nonfinite JSON rejection, exact33
selection/partial34 exclusion, signed-zero result bits, output/provenance mismatch,
bounded ABBA order, fail-closed EXEC, and exact archive inventory. Synthetic
comparison fixtures are tests only; no fake-policy performance evidence exists.
No ordinary runtime, strict20 guards, hardware, SSH, CPU frequency or boot guard
is changed by this package.
