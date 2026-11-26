"""Command line for the sim parameter sweep (runs on any PC, no ROS)."""
import argparse
import sys

from trash_vision import config_logic as cl
from trash_vision import sweep_logic as sw


def parse_axis(text):
    """Parse 'name=v1,v2,...' into (name, [floats])."""
    name, sep, values = text.partition('=')
    try:
        numbers = [float(v) for v in values.split(',') if v.strip()]
    except ValueError:
        numbers = []
    if not sep or not name.strip() or not numbers:
        raise argparse.ArgumentTypeError(
            'expected name=v1,v2,... got %r' % text)
    return name.strip(), numbers


def positive_int(text):
    """Argparse type for a whole number of at least 1."""
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError('must be at least 1')
    return value


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(
        prog='sweep_sim',
        description='Try behavior parameter settings in the pure-Python sim '
        'and print a ranked table. ' + sw.SIM_ONLY,
        epilog='Parameters that can be swept: ' + ', '.join(sw.SPACE))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--param', action='append', type=parse_axis, metavar='NAME=V1,V2',
        help='grid search over these values (repeat for more parameters)')
    mode.add_argument(
        '--random', type=positive_int, metavar='N',
        help='random search over N settings (default: 20)')
    parser.add_argument('--seeds', type=positive_int, default=10,
                        help='noisy runs per setting (default: 10)')
    parser.add_argument('--seed', type=int, default=0,
                        help='first seed; same seed, same table (default: 0)')
    parser.add_argument('--duration', type=float, default=300.0,
                        help='simulated seconds per run (default: 300)')
    parser.add_argument('--top', type=positive_int, default=10,
                        help='rows to print (default: 10)')
    parser.add_argument(
        '--config', help='robot.yaml with the current values; only the '
        'sweepable and safety_* behavior values are read (default: code '
        'defaults, or the repo robot.yaml with tools/sweep_sim.py)')
    parser.add_argument('--csv', help='also write every row to this CSV file')
    return parser


def load_behavior_params(path):
    """Read the behavior section of a robot.yaml."""
    import yaml
    with open(path, encoding='utf-8') as fh:
        return cl.node_params(yaml.safe_load(fh), 'behavior')


def main(argv=None, default_config=None):
    """Run the sweep and print the ranked table; return an exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.duration > 0:
        parser.error('--duration must be above 0')
    path = args.config or default_config
    try:
        config = load_behavior_params(path) if path else {}
    except OSError as exc:
        parser.error('cannot read %s: %s' % (path, exc))
    base = sw.baseline_params(config)
    safety = sw.safety_params(config)
    try:
        if args.param:
            settings = sw.grid_settings(dict(args.param), base, safety)
        else:
            settings = sw.random_settings(
                args.random or 20, args.seed, base, safety)
    except ValueError as exc:
        parser.error(str(exc))
    seeds = list(range(args.seed, args.seed + args.seeds))

    def progress(done, total):
        print('setting %d/%d done' % (done, total), file=sys.stderr)

    rows = sw.sweep(settings, base, seeds, args.duration, safety=safety,
                    progress=progress)
    print('Current values from: %s' % (path or 'code defaults'))
    print('%d settings, %d seeds each (first %d), %g s per run' % (
        len(rows), len(seeds), args.seed, args.duration))
    print(sw.format_table(rows, args.top))
    if args.csv:
        with open(args.csv, 'w', encoding='utf-8') as fh:
            fh.write(sw.to_csv(rows, base))
        print('All rows written to %s' % args.csv)
    return 0


def run():
    """Console script entry point."""
    sys.exit(main())


if __name__ == '__main__':
    run()
