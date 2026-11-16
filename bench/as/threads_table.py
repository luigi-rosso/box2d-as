# SPDX-FileCopyrightText: 2026 Erin Catto
# SPDX-License-Identifier: MIT
# Turns a bench/as/threads.sh session log into markdown tables. ms/step is the mean over a repeat's timed steps, min
# over a run's repeats, then min over rounds, for native and AnimaScript alike; the stage table shows the round that
# gave it.
import re
import sys
from collections import defaultdict

# b2Profile fields: one parallel call each, plus the main lane's share around it
STAGES = [('pairs', 'pairs'), ('collide', 'collide'), ('solver stages', 'constraints'), ('finalize', 'transforms')]

load = None
config = None
parts = None
runs = defaultdict(list)
bad = []
for line in open(sys.argv[1]):
    m = re.search(r'load averages?: ([\d.]+)', line)
    if line.startswith('UPTIME') and m:
        load = float(m.group(1))
    elif line.startswith('AS '):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        config, parts = kv['config'], int(kv['parts'])
    elif line.startswith('RESULT'):
        kv = dict(p.split('=', 1) for p in line.split()[1:])
        kv['ms'] = float(kv['min'])
        kv['load'] = load
        runs[(kv['scene'], 'native simd', int(kv['workers']))].append(kv)
    elif 'BENCH RESULT' in line:
        kv = dict(p.split('=', 1) for p in line.split('BENCH RESULT ')[1].split())
        kv['ms'] = float(kv['mean'])
        kv['load'] = load
        runs[(kv['scene'], 'AS ' + config, parts)].append(kv)
    elif 'trapped' in line or 'failed' in line or 'gave up' in line:
        bad.append(line.strip())

scenes, counts = [], []
for scene, impl, n in runs:
    if scene not in scenes:
        scenes.append(scene)
    if impl != 'AS serial' and n not in counts:
        counts.append(n)
counts.sort()


def best(scene, impl, n):
    rs = runs.get((scene, impl, n))
    return min(rs, key=lambda r: r['ms']) if rs else None


print('ms per step, min over rounds (1 minute load of that run). x is the 1 count of the same row over the cell; the last column is AS parallel over native at each count.')
print()
print('| scene | impl | ' + ' | '.join(str(n) for n in counts) + ' | parallel / native |')
print('|---|---|' + '---|' * (len(counts) + 1))
for scene in scenes:
    serial = best(scene, 'AS serial', 1)
    if serial:
        print(f"| {scene} | AS serial | **{serial['ms']:.3f}** ({serial['load']:.0f}) |" + ' |' * len(counts))
    for impl in ('native simd', 'AS parallel'):
        one = best(scene, impl, counts[0]) if counts else None
        cells = []
        for n in counts:
            b = best(scene, impl, n)
            cells.append(f"{b['ms']:.3f} ({b['load']:.0f}, x{one['ms'] / b['ms']:.2f})" if b else '')
        ratios = ''
        if impl == 'AS parallel':
            pairs = [(best(scene, impl, n), best(scene, 'native simd', n)) for n in counts]
            ratios = ' / '.join(f"{a['ms'] / b['ms']:.2f}" if a and b else '' for a, b in pairs)
        print(f'| {scene} | {impl} | ' + ' | '.join(cells) + f' | {ratios} |')

print()
print('Step breakdown of the best run, ms per step: step = ' + ' + '.join(name for name, _ in STAGES) + ' + rest.')
print()
print('| scene | impl | n | step | ' + ' | '.join(name for name, _ in STAGES) + ' | rest |')
print('|---|---|---|---|' + '---|' * (len(STAGES) + 1))
for scene in scenes:
    for impl in ('native simd', 'AS serial', 'AS parallel'):
        for n in ([1] if impl == 'AS serial' else counts):
            b = best(scene, impl, n)
            if not b:
                continue
            step = float(b['step'])
            values = [float(b[field]) for _, field in STAGES]
            cells = [f'{v:.3f}' for v in values] + [f'{step - sum(values):.3f}']
            print(f'| {scene} | {impl} | {n} | {step:.3f} | ' + ' | '.join(cells) + ' |')

if bad:
    print()
    print('Problems:')
    for line in bad:
        print('- ' + line)
