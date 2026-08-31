#!/usr/bin/env python3
"""Read bench throttle-mismatch runs off the boat's SD card and work out the trim.

The boat records each run itself as T<pct>_<K>_<NN>.CSV (8.3 names -- the card
is FAT with long names disabled). K is B(ase), L(eft) or R(ight).

    BASE  both motors equal  -> any turn IS the mismatch
    LEFT  left stronger      -> gives the scale
    RIGHT right stronger     -> gives the scale

        gain = (LEFT - RIGHT) / (2 * delta)     deg/s per unit of split
        trim = -BASE / gain                     split that cancels the mismatch

Usage:
    python3 tools/bench_analyze.py /path/to/sdcard [--delta 0.04] [--tail 1.5]
"""

import argparse
import csv
import math
import re
import statistics
import sys
from pathlib import Path

NAME_RE = re.compile(r'^T(\d{2})_([BLR])_(\d{2})\.CSV$', re.IGNORECASE)
KIND_NAME = {'B': 'BASE', 'L': 'LEFT', 'R': 'RIGHT'}
DEFAULT_TAIL_S = 1.5
MIN_STEADY_SAMPLES = 10


def read_run(path):
    """One run file -> its rows, grouped by the phase column the boat wrote."""
    rows = []
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            try:
                left, right = float(row['left']), float(row['right'])
                # The boat writes c per sample. Older files predate the column;
                # for those it is recoverable from the commands, because the
                # throttle cancels: c = (right-left)/(right+left).
                if row.get('c') not in (None, ''):
                    c = float(row['c'])
                elif (left + right) > 1e-6:
                    c = (right - left) / (right + left)
                else:
                    c = float('nan')
                # Columns added 2026-08-30 for the P assist. Files from
                # before that simply lack them; default to "P was off", which
                # is exactly what those runs were.
                def _f(k):
                    v = row.get(k)
                    return float(v) if v not in (None, '') else 0.0
                rows.append((float(row['t_s']), row['phase'].strip(),
                             float(row['yaw_dps']), left, right, c,
                             _f('p_on') > 0.5, _f('p_yaw'), _f('p_corr'),
                             _f('at_cap') > 0.5))
            except (KeyError, TypeError, ValueError):
                # A line cut short by a power loss yields None for the missing
                # columns (TypeError), not just a bad float. Skip that row
                # rather than losing the whole run.
                continue
    return rows


def summarize(rows, tail_s=DEFAULT_TAIL_S):
    """baseline = motors-off drift; steady = TAIL of the drive window only.

    The tail matters: the boat needs a moment to reach a steady turn, and
    averaging the spin-up in would drag the number toward zero and understate
    the mismatch."""
    base = [y for _t, ph, y, _l, _r, _c, _po, _py, _pc, _ac in rows if ph == 'baseline']
    run = [(t, y) for t, ph, y, _l, _r, _c, _po, _py, _pc, _ac in rows if ph == 'run']
    coast = [y for _t, ph, y, _l, _r, _c, _po, _py, _pc, _ac in rows if ph == 'coast']
    drive = [(l, r) for _t, ph, _y, l, r, _c, _po, _py, _pc, _ac in rows if ph == 'run']
    live = [(t, y, c) for t, ph, y, _l, _r, c, _po, _py, _pc, _ac in rows
            if ph == 'run' and math.isfinite(c)]
    if run:
        end = max(t for t, _ in run)
        tail = [y for t, y in run if t >= end - tail_s]
    else:
        tail = []
    mean = lambda v: (sum(v) / len(v)) if v else float('nan')
    baseline, steady = mean(base), mean(tail)
    return {
        'baseline_dps': baseline,
        'steady_dps': steady,
        'corrected_dps': steady - baseline,
        'rows': len(rows), 'baseline_n': len(base),
        'steady_n': len(tail), 'coast_n': len(coast),
        'thin': len(tail) < MIN_STEADY_SAMPLES,
        'c_applied': applied_c(drive),
        'stalled': gyro_stalled([y for _t, _ph, y, _l, _r, _c, _po, _py, _pc, _ac in rows]),
        **adaptation(live),
    }


# A window at each end of the drive phase, long enough to average the noise
# down but short enough that the two do not overlap on a 10 s run.
ADAPT_WINDOW_S = 2.0


def adaptation(live):
    """Did the trim MOVE during this run, and did the boat straighten as it did?

    That is the whole question a live-learner run asks, and it can only be
    answered inside one file: c at the start against c at the end, and how hard
    the boat was turning over the same two windows. Comparing separate runs
    cannot do it -- the boat gets picked up and re-held between them."""
    blank = {'c_start': float('nan'), 'c_end': float('nan'),
             'yaw_early': float('nan'), 'yaw_late': float('nan'),
             'adapted': False}
    if len(live) < 2 * MIN_STEADY_SAMPLES:
        return blank
    t0 = min(t for t, _y, _c in live)
    t1 = max(t for t, _y, _c in live)
    if t1 - t0 < 2 * ADAPT_WINDOW_S:
        return blank                    # too short to have two ends
    early = [(y, c) for t, y, c in live if t <= t0 + ADAPT_WINDOW_S]
    late = [(y, c) for t, y, c in live if t >= t1 - ADAPT_WINDOW_S]
    if not early or not late:
        return blank
    return {
        'c_start': statistics.median([c for _y, c in early]),
        'c_end': statistics.median([c for _y, c in late]),
        'yaw_early': statistics.median([y for y, _c in early]),
        'yaw_late': statistics.median([y for y, _c in late]),
        'adapted': abs(statistics.median([c for _y, c in late]) -
                       statistics.median([c for _y, c in early])) >= 0.002,
    }


def applied_c(drive):
    """The trim the boat ACTUALLY ran, read back out of its own file.

    The bench records the post-trim commands, so

        c = (right - left) / (right + left)

    recovers the dimensionless trim directly -- the throttle cancels. That
    makes every run self-describing: no need to remember what was flashed, and
    a file whose c does not match the boot log means the two disagree about
    what is loaded, which is worth knowing before reading anything else."""
    pairs = [(l, r) for l, r in drive if (l + r) > 1e-6]
    if not pairs:
        return float('nan')
    return statistics.median([(r - l) / (r + l) for l, r in pairs])


def gyro_stalled(yaws):
    """A frozen gyro reads as a perfectly steady boat -- the most convincing
    wrong answer this whole rig can produce. Real gyro noise is never zero, so
    an unchanging column means the sensor stopped, not that the boat is
    straight."""
    return len(yaws) >= MIN_STEADY_SAMPLES and len(set(yaws)) <= 1


# deg/s of yaw per unit of c, measured from the LEFT/RIGHT pairs already on
# file: 21.6 at T10, 11.5 at T30, 17.9 at T35, 15.5 at T40. It is roughly flat
# in c (which is the whole point of working in c), so one number serves, with
# about +/-25% honest uncertainty. Only used for BASE-only sessions, where
# there are no lopsided runs to measure the gain from.
YAW_PER_UNIT_C = 16.0

# Biggest single change in c worth trusting from one reading. Beyond this the
# dead-band floors (LEFT 5%, RIGHT 19%) bend the response badly -- the T10 runs
# at c=0.76 sit almost entirely in the floor and read 4x off.
MAX_TRUSTED_DC = 0.10


def to_boat_trim(split):
    """Convert a measured split into the boat's own trim units.

    The bench applies    left += d ;      right -= d
    esc_trim_mix applies left -= t/2 ;    right += t/2
    so t = -2d. Writing the raw split into the trim table would push the wrong
    way at twice the strength -- which is the runaway the clamp is there to
    catch, but far better not to trigger at all."""
    return -2.0 * split


def trim_uncertainty(base_runs, gain):
    """Error bar on the trim, from how much the BASE runs disagree.

    Uses the standard error of the median (scatter / sqrt(n)), not the raw
    scatter: repeating the run is exactly how random wobble is beaten down, so
    the extra work has to show up as a tighter answer. Returns None for a
    single run -- one measurement cannot report its own scatter, and printing
    zero there would look far more confident than it is.

    Caveat: this assumes the wobble is RANDOM. If the mount changes the boat's
    response between runs, that is a systematic error and no amount of
    repeating removes it -- which is what the mirror check is there to catch."""
    if len(base_runs) < 2 or not gain:
        return None
    return (statistics.pstdev(base_runs) / len(base_runs) ** 0.5) / abs(gain)


def trim_from(base, left, right, delta):
    """None when the two lopsided runs read the same -- no measurable gain
    means no honest answer, so report nothing rather than divide by ~zero."""
    span = left - right
    if delta <= 0 or abs(span) < 1e-6 or not all(map(math.isfinite,
                                                     (base, left, right))):
        return None
    return -base / (span / (2.0 * delta))


def trim_sweep_report(pct, kind, by_c):
    """The same throttle driven with more than one trim -- the adaptive trim
    moving, or a hand sweep. Either way these are different measurements and
    must be read side by side, never averaged into one.

    This is the whole record of whether the trim is working: c on the left, how
    much the boat still turns on the right, and the turn shrinking as c walks
    toward the answer."""
    print('T%s %s: driven with %d different trims -- shown separately, NOT '
          'averaged' % (pct, KIND_NAME[kind], len(by_c)))
    best_c, best_abs = None, None
    for c in sorted(by_c, key=lambda x: (x is None, x)):
        runs = by_c[c]
        med = statistics.median([d for d, _n in runs])
        err = (statistics.pstdev([d for d, _n in runs]) / len(runs) ** 0.5
               if len(runs) > 1 else float('nan'))
        label = 'c=%.3f' % c if c is not None else 'c unknown'
        flag = ''
        if len(runs) > 1 and abs(med) <= err:
            flag = '   <-- STRAIGHT'
        print('    %-12s split %5.2f%%  n=%-3d yaw %+6.2f +/-%s deg/s%s'
              % (label, (c or 0) * int(pct), len(runs), med,
                 '%.2f' % err if len(runs) > 1 else ' n/a', flag))
        if c is not None and (best_abs is None or abs(med) < best_abs):
            best_abs, best_c = abs(med), c
    if best_c is not None:
        print('    flattest of these is c=%.3f (%.2f%% split at %s%% throttle)'
              % (best_c, best_c * int(pct), pct))


def base_only_verdict(pct, base_runs, c_runs):
    """Does the boat go straight at this throttle, and if not, by how much is c
    off?

    This is the question a BASE-only session asks. It needs no lopsided runs:
    equal commands mean any leftover turn IS the mismatch, and dividing by the
    known response gives the correction directly in c.

    "Straight" is decided against the scatter of the runs themselves, not
    against a number picked in advance -- if the boat wobbles more than it
    drifts, the honest verdict is that the drift is not measurable."""
    n = len(base_runs)
    med = statistics.median(base_runs)
    c_app = statistics.median(c_runs) if c_runs else float('nan')
    if n >= 2:
        err = statistics.pstdev(base_runs) / n ** 0.5
    else:
        err = float('nan')

    turn = 'RIGHT' if med > 0 else 'LEFT'
    if n >= 2 and abs(med) <= err:
        print('T%s: STRAIGHT -- %+.2f deg/s over %d runs, inside its own '
              '+/-%.2f scatter' % (pct, med, n, err))
    elif n >= 2:
        print('T%s: turns %s at %+.2f +/-%.2f deg/s over %d runs'
              % (pct, turn, med, err, n))
    else:
        print('T%s: %+.2f deg/s from a single run -- repeat it, one run cannot '
              'show its own repeatability' % (pct, med))

    if not math.isfinite(c_app):
        return
    dc = -med / YAW_PER_UNIT_C
    print('T%s: ran c=%.3f -> suggests c=%.3f (%+.3f)%s'
          % (pct, c_app, c_app + dc, dc,
             '' if n >= 2 else '  [one run only]'))
    if n >= 2 and abs(med) <= err:
        print('T%s: leave c alone -- the change this asks for (%+.3f) is '
              'smaller than the noise (+/-%.3f)'
              % (pct, dc, err / YAW_PER_UNIT_C))
        return
    # Far from the answer the response stops being a straight line -- measured:
    # the runs taken NEAR zero yaw all agree on c, the ones taken far from it
    # scatter by 4x. So a large jump is a direction, not a destination.
    if abs(dc) > MAX_TRUSTED_DC:
        step = MAX_TRUSTED_DC if dc > 0 else -MAX_TRUSTED_DC
        print('T%s: that is a big jump -- the straight-line response only '
              'holds near zero yaw. Move to c=%.3f (%+.3f) and re-run.'
              % (pct, c_app + step, step))
    if c_app + dc <= 0.0:
        print('T%s: a c of zero or below means the LEFT motor is the strong '
              'one -- every run on file says the opposite, so check the run '
              'before acting on it' % pct)


# The convergence experiment: reset the learner to a low c, run a sequence
# without resetting, then repeat from a high c. Success is judged on YAW, not
# on c landing anywhere in particular -- a straight boat is the goal and c is
# only the means.
SEQ_WINDOW = (0.5, 2.0)     # seconds INTO the drive phase: after the motors
                            # have spun up, before the hull reaches the wall
CONTACT_JERK = 400.0        # deg/s per s -- an impact is a step, not a big rate
CONTACT_SPAN_S = 0.04       # differentiate over this, NOT between adjacent rows
CONTACT_BEFORE_S = 2.0      # a hit this early poisons the whole window
RESET_JUMP = 0.03           # c discontinuity between runs = a deliberate reset


def sequence_runs(folder):
    """Every BASE run in the folder, in the order it was recorded, each reduced
    to the three numbers the experiment turns on: where c started, where it
    ended, and how hard the boat was actually turning in the clean window."""
    files = sorted((p for p in folder.iterdir() if NAME_RE.match(p.name)
                    and NAME_RE.match(p.name).group(2).upper() == 'B'),
                   key=lambda p: p.stat().st_mtime)
    out = []
    for path in files:
        rows = [r for r in read_run(path) if r[1] == 'run']
        if len(rows) < 60:
            continue
        t0 = rows[0][0]
        cs = [c for _t, _p, _y, _l, _r, c, _po, _py, _pc, _ac in rows if math.isfinite(c)]
        # Contact: the first big STEP in yaw rate once the boat is under way.
        #
        # Differentiated over a fixed SPAN, not between adjacent rows. The file
        # is written at the 100 Hz control tick but the gyro only updates at
        # 50 Hz, so half the rows are exact repeats and the rest carry a full
        # sample's change across half a sample's time -- differentiating
        # neighbours doubles the apparent jerk and flags ordinary noise as a
        # collision. Measured on the real runs: neighbour-differencing rejects
        # every single one; a 40 ms span rejects the 12 that actually hit.
        contact = None
        j = 0
        for i in range(len(rows)):
            while rows[i][0] - rows[j][0] > CONTACT_SPAN_S:
                j += 1
            span = rows[i][0] - rows[j][0]
            if span < CONTACT_SPAN_S * 0.75 or rows[i][0] - t0 < 0.5:
                continue
            if abs((rows[i][2] - rows[j][2]) / span) > CONTACT_JERK:
                contact = rows[i][0] - t0
                break
        win = [y for t, _p, y, _l, _r, _c, _po, _py, _pc, _ac in rows
               if SEQ_WINDOW[0] <= t - t0 < SEQ_WINDOW[1]]
        out.append({
            'name': path.name,
            'c_start': cs[0] if cs else float('nan'),
            'c_end': cs[-1] if cs else float('nan'),
            'yaw': statistics.median(win) if win else float('nan'),
            'contact': contact,
            'rejected': contact is not None and contact < CONTACT_BEFORE_S,
        })
    return out


def split_sequences(runs):
    """A deliberate reset shows up as c jumping between the end of one run and
    the start of the next. Nothing else moves c that fast, so it is a reliable
    marker for where one sequence stops and the next begins."""
    seqs, cur = [], []
    for r in runs:
        if cur and math.isfinite(r['c_start']) and math.isfinite(cur[-1]['c_end']) \
                and abs(r['c_start'] - cur[-1]['c_end']) > RESET_JUMP:
            seqs.append(cur); cur = []
        cur.append(r)
    if cur:
        seqs.append(cur)
    return seqs


def _half_stats(vals):
    if not vals:
        return float('nan'), float('nan')
    med = statistics.median(vals)
    err = statistics.pstdev(vals) / len(vals) ** 0.5 if len(vals) > 1 else float('nan')
    return med, err


def report_sequence(seq, index):
    good = [r for r in seq if not r['rejected'] and math.isfinite(r['yaw'])]
    print('--- sequence %d: %d runs, %d usable ---'
          % (index, len(seq), len(good)))
    print('   %-14s %8s %8s %9s %s' % ('file','c start','c end','yaw','' ))
    for r in seq:
        flag = ''
        if r['rejected']:
            flag = '  REJECTED (wall at %.1fs)' % r['contact']
        elif r['contact'] is not None:
            flag = '  (wall at %.1fs, after the window)' % r['contact']
        print('   %-14s %8.3f %8.3f %+9.2f%s'
              % (r['name'], r['c_start'], r['c_end'], r['yaw'], flag))

    if len(good) < 4:
        print('   too few usable runs to judge a trend')
        return None

    h = max(2, len(good) // 3)
    early, late = good[:h], good[-h:]
    c0, c1 = statistics.median([r['c_start'] for r in early]), \
             statistics.median([r['c_end'] for r in late])
    ye, ee = _half_stats([r['yaw'] for r in early])
    yl, el = _half_stats([r['yaw'] for r in late])

    print()
    print('   c        %.3f -> %.3f   (%+.3f over the sequence)' % (c0, c1, c1-c0))
    print('   yaw      %+.2f +/-%.2f  ->  %+.2f +/-%.2f deg/s   (first %d vs last %d runs)'
          % (ye, ee, yl, el, len(early), len(late)))
    # An improvement smaller than the noise is not an improvement. With yaw as
    # the PRIMARY criterion this guard is what stops a flat sequence reading as
    # a pass: on synthetic data where c never moved at all, raw medians still
    # showed |yaw| 1.62 -> 0.74 purely from scatter.
    # Noise scale taken from the SETTLED end only. A converging sequence's
    # early runs contain the convergence itself, so their spread is trend, not
    # noise -- using it would make a working learner read as "inside the
    # noise". sqrt(2) because two means are being compared.
    combined = (el * 1.414) if math.isfinite(el) and el > 0 else 0.0
    delta = abs(ye) - abs(yl)
    straighter = delta > combined
    print('   |yaw|    %.2f -> %.2f   (change %+.2f, noise +/-%.2f)   %s'
          % (abs(ye), abs(yl), -delta, combined,
             'CLOSER TO ZERO' if straighter
             else ('smaller, but inside the noise' if delta > 0
                   else 'no better')))

    moves = [abs(r['c_end']-r['c_start']) for r in good]
    me, ml = statistics.median(moves[:h]), statistics.median(moves[-h:])
    faulted = any(r['c_end'] <= 0.1005 or r['c_end'] >= 0.3495 for r in good)
    print('   movement %.4f -> %.4f per run%s'
          % (me, ml, '   (secondary)' if True else ''))
    if faulted:
        print('   c REACHED A CLAMP -- the learner latches a fault there and stops')
    return {'c0': c0, 'c1': c1, 'yaw_early': ye, 'yaw_late': yl,
            'straighter': straighter, 'faulted': faulted,
            'move_early': me, 'move_late': ml}


def verdict(results):
    """Success is a straighter boat. c meeting in the middle and settling down
    are corroboration -- welcome, but a low bounded yaw is the thing that
    matters, because that is what the trim is for."""
    print('=== VERDICT ===')
    if len(results) < 2:
        print('   need two sequences (one from a low c, one from a high c) --')
        print('   a single sequence cannot separate "found the trim" from')
        print('   "drifted for its own reasons"')
        return 1
    lo = min(results, key=lambda r: r['c0'])
    hi = max(results, key=lambda r: r['c0'])
    up = lo['c1'] > lo['c0']
    down = hi['c1'] < hi['c0']
    print('   low  start %.3f -> %.3f   %s' % (lo['c0'], lo['c1'],
          'RISES' if up else 'does not rise'))
    print('   high start %.3f -> %.3f   %s' % (hi['c0'], hi['c1'],
          'FALLS' if down else 'does not fall'))
    print('   yaw toward zero: low %s, high %s'
          % ('yes' if lo['straighter'] else 'NO',
             'yes' if hi['straighter'] else 'NO'))
    primary = up and down and lo['straighter'] and hi['straighter']
    print()
    print('   PRIMARY: %s' % ('PASS -- both ends move toward each other and the '
                              'boat gets straighter' if primary else
                              'FAIL -- see which of the four lines above missed'))
    gap = abs(lo['c1'] - hi['c1'])
    print('   secondary: final c %.3f vs %.3f, gap %.3f%s'
          % (lo['c1'], hi['c1'], gap, '  (within 0.02)' if gap <= 0.02 else ''))
    print('   secondary: movement settling  low %.4f->%.4f, high %.4f->%.4f'
          % (lo['move_early'], lo['move_late'], hi['move_early'], hi['move_late']))
    if any(r['faulted'] for r in results):
        print('   WARNING: c hit a clamp and latched a fault -- that run is void')
    elif not primary:
        worst = max(abs(lo['yaw_late']), abs(hi['yaw_late']))
        if worst < 0.5:
            print('   NOTE: yaw ended below 0.5 deg/s in both and c stayed bounded')
            print('         with no fault -- the boat is straight even though the')
            print('         trend test did not pass. Not a failure of the trim.')
    return 0 if primary else 2


# Seconds into the drive phase. Starts at 1.0, not 0.2, because P cannot have
# acted before then: its correction reaches the yaw only after the filter
# (0.25 s) plus the measured 1.10 s hull lag. Simulated on 90 real runs the
# same setting reads -10% over 0.2-2.0 s and -21% over 1.0-3.0 s -- the early
# window measures the loop's dead time, not the loop.
#
# 3.0 s is the end of the run. The hull reaches the pool wall at a median of
# 4 s and contact runs are rejected separately, so this stays in clean water.
# Three windows, all reported every time. Fixed in advance so a result cannot
# be rescued by picking the flattering one after the fact.
#
#   WHOLE is the PRIMARY: the complete trajectory the boat actually flew, and
#         the only one that carries to a lake where runs are continuous.
#   EARLY is mostly the loop's dead time -- P's correction reaches the yaw only
#         after its filter (0.25 s) plus the measured 1.10 s hull lag.
#   LATE  is where P has had time to act, so it should improve the most.
AB_WINDOWS = (('whole 0.2-3.0s', 0.2, 3.0),
              ('early 0.2-2.0s', 0.2, 2.0),
              ('late  1.0-3.0s', 1.0, 3.0))


def ab_run(path):
    """One run reduced to the A/B measures, in all three windows.

    PRIMARY is |integrated yaw| -- the heading the boat actually threw away.
    Mean yaw can read zero for a boat that swings hard both ways, and that is
    not straight; integrating catches it and a mean does not.

    Wall contact is detected the same way as elsewhere (a step in yaw rate over
    a fixed 40 ms span) and the run is REJECTED, not quietly averaged in: a
    bounce is not a trajectory the trim had any say in."""
    rows = [r for r in read_run(path) if r[1] == 'run']
    if len(rows) < 60:
        return None
    t0 = rows[0][0]

    contact = None
    j = 0
    for i in range(len(rows)):
        while rows[i][0] - rows[j][0] > CONTACT_SPAN_S:
            j += 1
        span = rows[i][0] - rows[j][0]
        if span < CONTACT_SPAN_S * 0.75 or rows[i][0] - t0 < 0.5:
            continue
        if abs((rows[i][2] - rows[j][2]) / span) > CONTACT_JERK:
            contact = rows[i][0] - t0
            break

    # Classify by the WHOLE file, not by any sample. `any()` would file a run
    # that was P-off for 99% of its length as a B, which quietly poisons the
    # arm it lands in. A file whose p_on CHANGES mid-run is neither arm -- the
    # toggle was pressed during the run -- and is rejected outright.
    flags = {bool(r[6]) for r in rows}
    mixed = len(flags) > 1
    out = {'name': path.name, 'contact': contact,
           'mixed_p': mixed,
           'rejected': contact is not None or mixed,
           'p_on': (flags == {True}),
           'windows': {}}
    for label, w0, w1 in AB_WINDOWS:
        w = [(t - t0, y, ac) for t, _p, y, _l, _r, _c, _po, _py, _pc, ac in rows
             if w0 <= t - t0 < w1]
        if len(w) < 20:
            continue
        heading = sum(0.5 * (w[k][1] + w[k-1][1]) * (w[k][0] - w[k-1][0])
                      for k in range(1, len(w)))
        ys = [y for _t, y, _ac in w]
        hf = statistics.pstdev([ys[k] - ys[k-1] for k in range(1, len(ys))])
        out['windows'][label] = {
            'abs_heading': abs(heading),
            'mean': statistics.mean(ys),
            'mean_abs': statistics.mean([abs(y) for y in ys]),
            'rms': (sum(y * y for y in ys) / len(ys)) ** 0.5,
            'hf': hf,
            'cap_frac': sum(1 for _t, _y, ac in w if ac) / len(w),
        }
    return out if out['windows'] else None


def report_ab(runs):
    """A = P off (control), B = P on. Same firmware, same day, interleaved."""
    bad = [r for r in runs if r['rejected']]
    for r in bad:
        if r.get('mixed_p'):
            print("   REJECTED %-14s p_on CHANGED during the run -- neither arm"
                  % r['name'])
        else:
            print("   REJECTED %-14s wall contact at %.1f s"
                  % (r['name'], r['contact']))
    good = [r for r in runs if not r['rejected']]
    A = [r for r in good if not r['p_on']]
    B = [r for r in good if r['p_on']]
    print("\n=== A/B: fast P assist OFF vs ON ===")
    print("   %d runs, %d rejected for wall contact, A=%d B=%d\n"
          % (len(runs), len(bad), len(A), len(B)))

    if not A or not B:
        print("   need BOTH arms, interleaved on the same day, to conclude anything")
        return 1

    # Every B file must actually say p_on=1, or the arm is not what it claims.
    print("   every B file confirms p_on=1: %s" % ("yes" if all(r['p_on'] for r in B) else "NO"))
    print()

    res = {}
    print("   %-16s %-5s %11s %9s %9s %8s %8s %7s"
          % ('window', 'arm', '|heading|', 'mean yaw', 'mean|yaw|', 'RMS', 'HF', 'cap'))
    for label, _w0, _w1 in AB_WINDOWS:
        row = {}
        for arm, g in (('A', A), ('B', B)):
            v = [r['windows'][label] for r in g if label in r['windows']]
            if not v:
                continue
            row[arm] = {
                'n': len(v),
                'abs_heading': statistics.median([x['abs_heading'] for x in v]),
                'hf': statistics.median([x['hf'] for x in v]),
                'cap': statistics.mean([x['cap_frac'] for x in v]),
            }
            print("   %-16s %-5s %10.2f d %+9.2f %9.2f %8.2f %8.2f %6.0f%%"
                  % (label if arm == 'A' else '', arm,
                     row[arm]['abs_heading'],
                     statistics.median([x['mean'] for x in v]),
                     statistics.median([x['mean_abs'] for x in v]),
                     statistics.median([x['rms'] for x in v]),
                     row[arm]['hf'], 100 * row[arm]['cap']))
        if 'A' in row and 'B' in row:
            res[label] = row
        print()

    print("=== VERDICT ===")
    ok = True
    for label, _w0, _w1 in AB_WINDOWS:
        if label not in res:
            continue
        a, b = res[label]['A'], res[label]['B']
        d = 100.0 * (b['abs_heading'] - a['abs_heading']) / a['abs_heading'] \
            if a['abs_heading'] else 0.0
        print("   %-16s |heading| %+.0f%%" % (label, d))
        if label.startswith('whole') and d > -15.0:
            ok = False
            print("      PRIMARY short of ~20%%")
        if label.startswith('late') and d >= 0.0:
            ok = False
            print("      the late window did not improve -- P is not acting")
    w = res.get(AB_WINDOWS[0][0])
    if w:
        hf = 100.0 * (w['B']['hf'] - w['A']['hf']) / w['A']['hf'] if w['A']['hf'] else 0.0
        print("   high-frequency yaw %+.0f%%   cap use %.0f%%"
              % (hf, 100 * w['B']['cap']))
        if hf > 25.0:
            ok = False
            print("      HF yaw rose substantially -- P is adding motion")
        if w['B']['cap'] >= 0.25:
            ok = False
            print("      cap use at or above 25%% -- reduce kp before judging")
    if not all(r['p_on'] for r in B):
        ok = False
        print("   a B file does not confirm p_on=1 -- that arm is not what it claims")
    print()
    print("   %s" % ("B PASSES" if ok else "B does not pass"))
    return 0 if ok else 2


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', nargs='?', default='.',
                    help='folder holding the run files (the SD card)')
    ap.add_argument('--delta', type=float, default=0.04,
                    help='split used during the runs, as a fraction (default 0.04 = 4%%)')
    ap.add_argument('--tail', type=float, default=DEFAULT_TAIL_S,
                    help='seconds at the end of the drive window counted as steady')
    ap.add_argument('--ab', action='store_true',
                    help='A/B the fast P assist: runs split by the p_on column, '
                         'primary measure is absolute integrated heading change')
    ap.add_argument('--sequence', action='store_true',
                    help='judge a low-start/high-start convergence experiment: '
                         'per-run c and yaw, sequences split at each reset')
    args = ap.parse_args(argv)

    folder = Path(args.folder)
    files = sorted(p for p in folder.iterdir() if NAME_RE.match(p.name)) \
        if folder.is_dir() else []
    if not files:
        print('no run files (T<pct>_<B|L|R>_<NN>.CSV) in %s' % folder)
        return 1

    if args.ab:
        runs = [ab_run(p) for p in files
                if NAME_RE.match(p.name).group(2).upper() == 'B']
        runs = [r for r in runs if r]
        if not runs:
            print('no usable BASE runs in %s' % folder)
            return 1
        return report_ab(runs)

    if args.sequence:
        runs = sequence_runs(folder)
        if not runs:
            print('no BASE runs in %s' % folder)
            return 1
        seqs = split_sequences(runs)
        print('%d BASE runs -> %d sequence(s), split where c jumped by more '
              'than %.2f (a reset)\n' % (len(runs), len(seqs), RESET_JUMP))
        results = []
        for i, seq in enumerate(seqs, start=1):
            r = report_sequence(seq, i)
            if r:
                results.append(r)
            print()
        return verdict(results)

    levels = {}
    applied = {}
    print('%-14s %-6s %9s %9s %11s %7s %7s' %
          ('file', 'run', 'still', 'turning', 'corrected', 'c', 'rows'))
    for path in files:
        pct, kc, _idx = NAME_RE.match(path.name).groups()
        s = summarize(read_run(path), args.tail)
        flags = ''
        if s['stalled']:
            flags += '  GYRO STALLED -- ignore this run'
        elif s['thin']:
            flags += '  THIN'
        print('%-14s %-6s %8.2f%s %8.2f%s %10.2f%s %7.3f %7d%s' % (
            path.name, KIND_NAME[kc.upper()],
            s['baseline_dps'], '', s['steady_dps'], '',
            s['corrected_dps'], '', s['c_applied'], s['rows'], flags))
        if s['adapted']:
            moved = s['c_end'] - s['c_start']
            straighter = abs(s['yaw_late']) < abs(s['yaw_early'])
            print('               trim moved %.3f -> %.3f (%+.3f)  |  turn '
                  '%+.2f -> %+.2f deg/s  %s'
                  % (s['c_start'], s['c_end'], moved,
                     s['yaw_early'], s['yaw_late'],
                     'STRAIGHTENING' if straighter else 'still working'))
        if s['stalled']:
            continue                    # a frozen gyro is not a measurement
        # Keep EVERY run, and keep the trim it ran with ATTACHED to it. Once
        # the trim adapts, two runs at the same throttle are no longer two
        # measurements of one thing -- averaging them together would blend the
        # before and the after into a number that describes neither, and hide
        # the adaptation completely.
        levels.setdefault(pct, {}).setdefault(kc.upper(), []).append(
            (s['c_applied'], s['corrected_dps'], path.name))
        if math.isfinite(s['c_applied']):
            applied.setdefault(pct, {}).setdefault(kc.upper(), []).append(
                s['c_applied'])

    print()
    for pct in sorted(levels):
        got_runs = levels[pct]
        # Runs are grouped by the trim they ran with. Same trim -> repeats of
        # one measurement, and the median is the answer. Different trims -> a
        # SWEEP, and each is its own answer.
        mixed = set()
        for k in ('B', 'L', 'R'):
            if k not in got_runs:
                continue
            by_c = {}
            for c, d, name in got_runs[k]:
                by_c.setdefault(round(c, 3) if math.isfinite(c) else None,
                                []).append((d, name))
            if len(by_c) > 1:
                trim_sweep_report(pct, k, by_c)
                mixed.add(k)
        got = {k: [d for _c, d, _n in v] for k, v in got_runs.items()}
        for k in ('B', 'L', 'R'):
            if k in got and k not in mixed:
                v = got[k]
                med = statistics.median(v)
                spread = (max(v) - min(v)) if len(v) > 1 else 0.0
                print('T%s %-5s median %+7.2f deg/s   n=%d   spread %.2f'
                      % (pct, KIND_NAME[k], med, len(v), spread))
                # A single jerked run among good ones: name it, so it is obvious
                # the median is carrying the result rather than the data being clean.
                mad = statistics.median([abs(x - med) for x in v])
                if len(v) >= 3 and mad > 0:
                    for x in v:
                        if abs(x - med) > 3 * mad:
                            print('        OUTLIER %+.2f (%.0fx the usual scatter)'
                                  % (x, abs(x - med) / mad))
        if mixed:
            # Everything below -- the verdict, the gain, the suggested trim --
            # assumes one trim per level. With several it is the sweep above
            # that answers the question, and a blended number would only
            # invite reading it as one.
            print('T%s: more than one trim was used here, so the per-run table '
                  'above IS the result -- no single number can stand for it'
                  % pct)
            continue
        if 'B' in got:
            base_only_verdict(pct, got['B'], applied.get(pct, {}).get('B', []))
        if not all(k in got for k in 'BLR'):
            have = ', '.join(KIND_NAME[k] for k in ('B', 'L', 'R') if k in got)
            print('T%s: have %s -- LEFT and RIGHT too would measure the gain '
                  'here instead of assuming it' % (pct, have))
            continue

        b = statistics.median(got['B'])
        mid = (statistics.median(got['L']) + statistics.median(got['R'])) / 2.0
        t = trim_from(b, statistics.median(got['L']),
                      statistics.median(got['R']), args.delta)
        if t is None:
            print('T%s: LEFT and RIGHT read the same -- no measurable gain, '
                  'so no trim can be worked out' % pct)
            continue
        gain = (statistics.median(got['L']) - statistics.median(got['R'])) / (2 * args.delta)

        # LEFT and RIGHT are equal and opposite splits, so their response about
        # BASE must mirror. If it does not, the straight-line assumption the
        # trim rests on is broken -- usually because whatever is holding the
        # boat is absorbing the turning force unevenly.
        dl = statistics.median(got['L']) - b
        dr = statistics.median(got['R']) - b
        if dl * dr >= 0 or max(abs(dl), abs(dr)) > 2.5 * max(min(abs(dl), abs(dr)), 1e-9):
            print('T%s: LEFT and RIGHT do not mirror about BASE '
                  '(%+.2f vs %+.2f) -- the response is not a straight line, so '
                  'the trim below cannot be believed. Usually the mount is '
                  'absorbing the turn.' % (pct, dl, dr))

        # How much is the BASE reading actually moving between repeats? That
        # noise maps straight onto the trim, and if it swamps the answer the
        # honest report is "not measurable", not a confident number.
        trim_err = trim_uncertainty(got['B'], gain)
        side = 'RIGHT' if t < 0 else 'LEFT'
        print('T%s: gain %+.0f deg/s per unit of split' % (pct, gain))

        # Two independent readings of the same thing. Anchoring on the BASE runs
        # uses the zero-split measurement directly; anchoring on the LEFT/RIGHT
        # midpoint ignores BASE entirely. If they agree the answer is solid; if
        # they disagree, that IS the mirror failure and neither can be trusted.
        t_mid = trim_from(mid, statistics.median(got['L']),
                          statistics.median(got['R']), args.delta)
        if t_mid is not None:
            agree = (t * t_mid > 0) and (max(abs(t), abs(t_mid)) <=
                                         2.5 * max(min(abs(t), abs(t_mid)), 1e-9))
            print('T%s: from BASE runs %+.2f%% (give %s more) | from LEFT/RIGHT '
                  'midpoint %+.2f%% (give %s more) -> %s'
                  % (pct, t * 100.0, side, t_mid * 100.0,
                     'RIGHT' if t_mid < 0 else 'LEFT',
                     'AGREE' if agree else 'DISAGREE, so neither is settled'))
            print('T%s: boat trim value would be %+.4f (BASE) / %+.4f (midpoint)'
                  '  [esc_trim_mix units: left -= t/2, right += t/2]'
                  % (pct, to_boat_trim(t), to_boat_trim(t_mid)))
        if trim_err is None:
            print('T%s: trim %+.3f (%+.2f%%) -- give %s more, but with only one '
                  'BASE run there is no way to know how repeatable it is'
                  % (pct, t, t * 100.0, side))
        elif abs(t) < trim_err:
            print('T%s: trim %+.2f%% but the BASE runs scatter +/-%.2f%% -- '
                  'NOT MEASURABLE. The boat is moving more than the motors are '
                  'unbalanced; hold it in a rigid mount and repeat.'
                  % (pct, t * 100.0, trim_err * 100.0))
        else:
            print('T%s: trim %+.3f (%+.2f%%) +/-%.2f%% -- give %s more'
                  % (pct, t, t * 100.0, trim_err * 100.0, side))
    return 0


if __name__ == '__main__':
    sys.exit(main())
