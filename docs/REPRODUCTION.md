# Reproduction status

## Verified

- Application files exported unchanged from the documented freeze commit, with file hashes.
- Recorded FINAL scalar data collected for 120 unique configuration/seed combinations.
- Analysis runs with Python's standard library and reproduces the reported 28.8% and 25.6% headline reductions.

## Environment

Environment: OMNeT++ 6.1 and VEINS 5.3.1. SUMO 1.20 is the reported experiment version; fresh-build compatibility remains untested.

## Simulation integration

This is an overlay for a compatible VEINS source tree, not a standalone C++ application. In a separate VEINS checkout, place `overlay/src/veins/modules/application/` files at the corresponding source path, and the scenario directory at `examples/grid5x5/`. Configure and rebuild VEINS against OMNeT++; the message definition requires OMNeT++ message-code generation. SUMO and the VEINS launch daemon must be available when running the scenario.

Exact fresh-build and launch commands are intentionally not claimed as tested. The archived scenario assets were committed after the application freeze. Check NED parameters and scenario dependencies against that application version before running. Final runs also used launch-config overrides such as `grid_s0.launchd.xml`; simply selecting a base configuration does not reproduce every recorded run.

## Remaining validation

1. Reconstruct the per-run command-line overrides from the original FINAL scalar metadata.
2. Resolve every local scenario asset reference and verify frozen-source/configuration compatibility.
3. Build in an isolated checkout and rerun at least one baseline and protocol seed.
4. Compare the freshly generated scalars with the archived data before claiming end-to-end reproducibility.

## Interpretation

False alarms and subsequent recovery occurred in the experiment record; zero false positives is not a general property of the protocol. Detection performance depends on the attack model and experiment configuration. Local/shared-control and caching experiments use separate baselines from the main waiting-time comparisons.
