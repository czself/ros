#!/usr/bin/env python3
"""Pair recorded actuator commands with timestamped watchdog decisions.

The public gate String and Twist have no Header. Check values and same-cycle
timing against the structured record, rather than assuming topic arrival order.
"""
import bisect
import math


def pair_command_decisions(commands, decisions, gate_events, maximum_skew_s=.025):
    valid = []
    errors = []
    last_sequence = None
    for receive_time, record in decisions:
        try:
            stamp = float(record['source_stamp']['seconds'])
            sequence = record['sequence']
            linear = float(record['linear_x'])
            angular = float(record['angular_z'])
            status = record['gate_status']
            if (record['schema_version'] != 1 or
                    not isinstance(sequence, int) or isinstance(sequence, bool) or
                    not isinstance(status, str) or not status or
                    not all(math.isfinite(v) for v in (stamp, linear, angular)) or stamp <= 0 or
                    sequence <= 0 or last_sequence is not None and sequence <= last_sequence):
                raise ValueError('invalid schema, time or sequence')
            last_sequence = sequence
            valid.append((stamp, sequence, linear, angular, status, receive_time))
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            errors.append(str(error))
    valid.sort(key=lambda row: (row[0], row[1]))
    times = [row[0] for row in valid]
    gate_times = [row[0] for row in gate_events]
    used = set()
    statuses = [None]*len(commands)
    unmatched_moving = []
    skews = []
    matched_moving = 0
    for index, (stamp, linear, angular) in enumerate(commands):
        moving = abs(linear) > .01 or abs(angular) > .05
        candidates = []
        low = bisect.bisect_left(times, stamp-maximum_skew_s)
        high = bisect.bisect_right(times, stamp+maximum_skew_s)
        for candidate in range(low, high):
            t, sequence, v, w, status, _ = valid[candidate]
            if (candidate in used or abs(v-linear) > 1e-9 or abs(w-angular) > 1e-9):
                continue
            g0 = bisect.bisect_left(gate_times, t-maximum_skew_s)
            g1 = bisect.bisect_right(gate_times, t+maximum_skew_s)
            if not any(value == status for _, value in gate_events[g0:g1]):
                continue
            candidates.append((abs(t-stamp), sequence, candidate))
        if not candidates:
            if moving:
                unmatched_moving.append({'command_index': index, 'stamp': stamp,
                                         'linear_x': linear, 'angular_z': angular})
            continue
        skew, _, candidate = min(candidates)
        used.add(candidate)
        statuses[index] = valid[candidate][4]
        skews.append(skew)
        matched_moving += int(moving)
    unmatched_moving_decisions = [
        {'sequence': sequence, 'stamp': stamp, 'linear_x': linear, 'angular_z': angular}
        for index, (stamp, sequence, linear, angular, _, _) in enumerate(valid)
        if index not in used and (abs(linear) > .01 or abs(angular) > .05)]
    return {'present': bool(decisions), 'decision_records': len(decisions),
            'command_records': len(commands), 'matched_commands': len(used),
            'matched_moving_commands': matched_moving,
            'unmatched_moving_commands': unmatched_moving, 'schema_errors': errors,
            'unmatched_moving_decisions': unmatched_moving_decisions,
            'maximum_allowed_pair_skew_s': maximum_skew_s,
            'maximum_observed_pair_skew_s': max(skews, default=0.0),
            'paired_statuses': statuses,
            'pass': (bool(decisions) and not errors and not unmatched_moving and
                     not unmatched_moving_decisions),
            'scope': 'Each moving Twist must match one unused decision by time and values, '
                     'and its public gate status must be present in the same time window. '
                     'Speed limits and independent green/paint checks are applied separately.'}
