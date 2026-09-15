"""Recalculate archived thesis comparisons using only Python's standard library."""
import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean

path = Path(__file__).resolve().parents[1] / 'data' / 'final-scalars.csv'
totals = defaultdict(float)
seen = set()
with path.open(newline='') as f:
    for row in csv.DictReader(f):
        if row['metric'] != 'REAL_wait_seconds':
            continue
        identity = (row['config'], int(row['seed']), row['module'])
        if identity in seen:
            raise ValueError(f'Duplicate measurement: {identity}')
        seen.add(identity)
        totals[identity[:2]] += float(row['value'])

print('Mean per-seed reduction versus healthy uncoordinated A_0fail')
print('Metric: sum of RSU REAL_wait_seconds; ten matched seeds')
for config in ['B_0fail', 'C_0fail', 'C_1fail', 'C_2fail', 'C_3fail']:
    reductions = []
    for seed in range(10):
        baseline = totals.get(('A_0fail', seed))
        value = totals.get((config, seed))
        if baseline is None or value is None or baseline <= 0:
            raise ValueError(f'Missing/invalid run: {config}, seed {seed}')
        reductions.append(100 * (baseline - value) / baseline)
    print(f'{config:10s} {mean(reductions):7.3f}%  '
          f'range {min(reductions):.3f}–{max(reductions):.3f}%  n=10')
