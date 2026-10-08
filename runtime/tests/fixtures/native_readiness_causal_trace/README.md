# Historical K37 causal-trace test fixture

`runtime/singularitydog_hw/native_pipeline_benchmark.py` preserves the exact source bytes from commit `6b1c090659b906b0614e13797a88491882c04c19`. Its SHA is the existing causal candidate/generator baseline, `0bafa9b92bea7ea641f57239a5ff1cd4784078b7b5bf09b5c917d0019acd5c93`. Tests authenticate the bytes before importing them and pass this source explicitly.

This keeps the original transformation, inverse proof and imported-function checks meaningful after production source changes. It does not broaden candidate SHA acceptance, select an old runtime, or authorize hardware execution. No test calls the fixture main or opens devices.

The runtime-relative directory layout is retained so the historical support provenance can rehash the exact baseline file without a source-path override.
