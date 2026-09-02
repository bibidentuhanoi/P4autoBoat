#!/usr/bin/env python3
"""Generate dashboard.html's embedded protobuf schema FROM boat.proto.

WHY THIS EXISTS. dashboard.html carries its own copy of the schema, because
protobuf.js has to be handed one at runtime. That copy was maintained by hand,
with a comment asking the next person to mirror boat.proto exactly -- and
protobuf.js SILENTLY DROPS fields its schema does not declare. A missed field
does not error anywhere; the value simply never arrives. That has already cost
this project a UI lockout, and every proto change since has needed a manual
mirroring step that nothing enforced.

So the copy is generated. Run this after any boat.proto change:

    .venv/bin/python tools/gen_dashboard_schema.py          # rewrite in place
    .venv/bin/python tools/gen_dashboard_schema.py --check  # verify only

--check is what the test suite runs, so a proto edit without a regenerate
fails the build rather than losing a field on the water.

Deliberately a text transform rather than a protobuf-library render: the goal
is a copy that is provably the same DECLARATIONS as the source file, and
comparing text is the strongest form of that. Comments and blank lines are
dropped because protobuf.js does not need them and they would make the
comparison noisy.
"""

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROTO = ROOT / 'main' / 'proto' / 'boat.proto'
DASH = ROOT / 'main' / 'dashboard.html'

BEGIN = 'const protoSchema = `'
END = '`;'

HEADER = """// GENERATED FROM main/proto/boat.proto BY tools/gen_dashboard_schema.py.
// Do not edit by hand: protobuf.js silently drops fields it does not know, so
// a hand-mirrored copy loses data with no error anywhere. Regenerate instead.
"""


def render_schema(proto_text):
    """boat.proto -> the declaration-only text protobuf.js is given."""
    out = []
    for raw in proto_text.splitlines():
        line = re.sub(r'//.*$', '', raw).rstrip()
        if not line.strip():
            continue
        if line.strip().startswith(('syntax', 'package', 'option', 'import')):
            continue
        out.append(line)
    # collapse a message onto one line, as the hand-written copy did -- it is
    # what protobuf.js parses either way, and one line per message keeps the
    # diff readable when a field is added
    text = '\n'.join(out)
    text = re.sub(r'\n\s+', ' ', text)
    text = re.sub(r'\s+\}', ' }', text)
    text = re.sub(r'[ \t]{2,}', ' ', text)
    return '\n'.join(l.strip() for l in text.splitlines() if l.strip())


def build_block():
    # The header rides INSIDE the generated block, so it cannot be separated
    # from the thing it describes by a later edit.
    return (HEADER + 'syntax = "proto3";\npackage boat;\n'
            + render_schema(PROTO.read_text()))


def current_block(dash_text):
    i = dash_text.index(BEGIN) + len(BEGIN)
    j = dash_text.index(END, i)
    return dash_text[i:j].strip('\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', action='store_true',
                    help='exit non-zero if the embedded copy is out of date')
    args = ap.parse_args()

    dash = DASH.read_text()
    want = build_block()
    have = current_block(dash)

    if have.strip() == want.strip():
        print('dashboard schema is up to date')
        return 0
    if args.check:
        print('dashboard schema is OUT OF DATE with main/proto/boat.proto.\n'
              'Regenerate:  .venv/bin/python tools/gen_dashboard_schema.py',
              file=sys.stderr)
        wl, hl = want.splitlines(), have.splitlines()
        for line in sorted(set(wl) - set(hl))[:10]:
            print('  missing from dashboard: %s' % line[:110], file=sys.stderr)
        for line in sorted(set(hl) - set(wl))[:10]:
            print('  stale in dashboard:     %s' % line[:110], file=sys.stderr)
        return 1

    i = dash.index(BEGIN) + len(BEGIN)
    j = dash.index(END, i)
    DASH.write_text(dash[:i] + '\n' + want + '\n' + dash[j:])
    print('dashboard schema regenerated from boat.proto')
    return 0


if __name__ == '__main__':
    sys.exit(main())
