# Fault-Tolerant RSU Traffic Coordination

**C++ simulation research connecting distributed systems, traffic coordination and cybersecurity.**

Master's research by Kranti Dipak Bhujbal, Cyber Security & Cyber Defence, University of Luxembourg.

## Problem

Traffic coordination depends on roadside infrastructure remaining available and providing useful information. This project explores how roadside units (RSUs) can coordinate traffic and respond when units fail or send anomalous measurements.

## Implementation

- C++ RSU and vehicle application modules integrated with VEINS and OMNeT++.
- Vehicle destination beacons and traffic measurements obtained through TraCI/SUMO.
- Adaptive signal timing, shared proposals and median-based voting.
- Measurement validation, drift and cross-check logic.
- Heartbeat failure detection, recovery on beacon resumption and failure-aware rerouting.
- Optional caching and degraded-operation controls.
- Python analysis of recorded simulation results.

These are research simulation mechanisms. `BeaconAuth.h` explicitly implements a keyed-hash placeholder using `std::hash`; it is not cryptographic message authentication or an implementation of PKI/ECDSA. A shared voting mechanism is not by itself proof of general Byzantine fault tolerance.

## Verified recorded results

The following values were recalculated from the archived FINAL scalar files. For each seed, `REAL_wait_seconds` is summed across the RSU modules, compared against configuration A with zero failures, and then the percentage reductions are averaged across ten seeds.

| Full protocol configuration | Mean waiting-time reduction | Seeds |
| --- | ---: | ---: |
| C, no failures | 28.816% | 10 |
| C, one failure | 26.768% | 10 |
| C, two failures | 25.954% | 10 |
| C, three failures | 25.612% | 10 |

The baseline is the healthy, uncoordinated configuration A. These values describe this experiment and metric, not a guarantee for arbitrary road networks. Both headline comparisons are positive in all ten seeds. Recorded results have been reanalysed; simulations have not been rerun for this repository package.

## Reproduce the result summary

Requires Python 3, with no additional packages:

```sh
python3 analysis/summarize.py
```

The data contains RSU scalar records extracted from 120 runs: A/B/C × 0/1/2/3 failures × seeds 0–9. It omits local-machine metadata and large vector traces. Original scalar-file hashes are recorded in `provenance.json`.

## Repository layout

```text
overlay/src/veins/modules/application/   Frozen research application source
overlay/examples/grid5x5/               Archived simulation scenario assets
analysis/summarize.py                   Portable result analysis
data/final-scalars.csv                  Extracted FINAL experiment records
docs/REPRODUCTION.md                    Environment and verification status
provenance.json                         Source commits and SHA-256 hashes
THIRD_PARTY_COPYING                     License text from the VEINS repository
```

## Code provenance

Application source comes from commit `9a71c1ed829665f30373771a609a01ac73a3d7e2`. Scenario assets come from the later commit `e0dad975`. Compatibility between these revisions requires validation before rerunning experiments. Later takeover and road-speed extensions are outside this source snapshot.

See [environment and reproduction](docs/REPRODUCTION.md) for simulation requirements and validation status.

## What this work demonstrates

Event-driven C++ development, integration across simulation tools, failure-handling logic, experimental comparisons and reproducible data analysis. The project connects software implementation with security and resilience questions.

## Attribution

This research uses VEINS, OMNeT++ and SUMO. The VEINS license is preserved in `THIRD_PARTY_COPYING`, along with existing source notices. No additional license grant is made for the research contributions.
