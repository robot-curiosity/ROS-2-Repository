"""Console script: print pickup rate by litter type and per run."""
import os
import sys

from trash_vision import trial_log as tl

DEFAULT_DIR = os.path.join('~', 'curiosity_trials')


def main(argv=None):
    """Print the summary for the given log files or directories."""
    args = sys.argv[1:] if argv is None else argv
    if not args:
        args = [os.path.expanduser(DEFAULT_DIR)]
    paths = tl.find_logs(args)
    if not paths:
        print('No trial logs found in: %s' % ' '.join(args))
        return 1
    records = tl.read_records(paths)
    if not records:
        print('No pickup attempts recorded in %d file(s).' % len(paths))
        return 1
    print(tl.format_summary(tl.summarize(records)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
