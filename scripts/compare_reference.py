"""Compare a run with the published measurements without concealing differences."""
import argparse
import json
from pathlib import Path


def compare(results, reference, tolerance_pct):
    if not results or not reference['records']:
        raise ValueError('No measurements or reference cases to compare')
    indexed = {(r['model'], r['backend'], r['worlds']): r for r in reference['records']}
    reference_keys = [(r['model'], r['backend'], r['worlds']) for r in reference['records']]
    result_keys = [(r['model'], r['backend'], r['worlds']) for r in results]
    if len(set(reference_keys)) != len(reference_keys) or len(set(result_keys)) != len(result_keys):
        raise ValueError('Duplicate benchmark case keys are not a valid reproduction')
    rows = []
    for row in results:
        key = row['model'], row['backend'], row['worlds']
        if key not in indexed:
            continue
        old = indexed[key]
        percent = (row['aggregate_fps'] / old['aggregate_fps'] - 1) * 100
        protocol = (row['steps'] == old['steps'] and len(row['seconds']) == len(old['seconds'])
                    and row['action_seed'] == old['action_seed']
                    and row['action_decimation'] == old['action_decimation']
                    and (key[1] != 'raisim' or row['threads'] == old['threads']))
        checked = []
        for field in ('dt', 'action_scale', 'profile', 'iterations', 'ls_iterations', 'tolerance'):
            if field in old and field in row:
                checked.append(field)
                protocol = protocol and row[field] == old[field]
        actions = row['last_action_checksum'] == old['last_action_checksum']
        rows.append({'model':key[0], 'backend':key[1], 'worlds':key[2], 'published_fps':old['aggregate_fps'],
                     'measured_fps':row['aggregate_fps'], 'fps_per_usd':row['fps_per_usd'],
                     'change_percent':percent, 'timing_identical':row['aggregate_fps'] == old['aggregate_fps'],
                     'within_tolerance':abs(percent) <= tolerance_pct, 'protocol_matches':protocol, 'additional_fields_checked':checked,
                     'last_actions_match':actions, 'published_contacts':old['mean_contacts_per_world'],
                     'measured_contacts':row['mean_contacts_per_world']})
    return {'tolerance_percent':tolerance_pct, 'compared_cases':len(rows),
            'all_cases_present':set(result_keys) == set(reference_keys),
            'all_protocols_match':all(r['protocol_matches'] for r in rows),
            'all_last_actions_match':all(r['last_actions_match'] for r in rows),
            'all_within_tolerance':all(r['within_tolerance'] for r in rows),
            'all_timings_identical':all(r['timing_identical'] for r in rows), 'rows':rows}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,required=True)
    parser.add_argument('--reference',type=Path,default=Path(__file__).resolve().parents[1]/'reports/published_reference.json')
    parser.add_argument('--output',type=Path,default=Path('comparison.json'))
    parser.add_argument('--tolerance-pct',type=float,default=5.)
    parser.add_argument('--require-within-tolerance',action='store_true')
    args=parser.parse_args()
    if not 0 <= args.tolerance_pct < float('inf'):
        parser.error('Tolerance must be finite and nonnegative')
    result=compare(json.loads(args.results.read_text()),json.loads(args.reference.read_text()),args.tolerance_pct)
    args.output.write_text(json.dumps(result,indent=2))
    print('Model        Backend   Worlds   Published FPS     Measured FPS     Change')
    for row in result['rows']:
        print(f"{row['model']:12} {row['backend']:8} {row['worlds']:7,d} {row['published_fps']:15,.2f} {row['measured_fps']:16,.2f} {row['change_percent']:+9.2f}%")
    print(f"Protocol match: {result['all_protocols_match']}; exact last actions: {result['all_last_actions_match']}; all timings identical: {result['all_timings_identical']}")
    if args.require_within_tolerance and not (result['all_cases_present'] and result['all_protocols_match'] and result['all_last_actions_match'] and result['all_within_tolerance']):
        raise SystemExit(1)
