
import datetime
import re

def run(path):

    marks = []

    for line in open(path).readlines():
        if 'Current running the state' in line:
            parts = line.strip().split(' ')
            state = parts[-1]
            time = parts[1].partition('.')[0]

            parts = [int(x) for x in re.split(r':|,', time)]
            seconds = 3600*parts[0] + 60*parts[1] + parts[2] + parts[3]/1000.0

            marks.append([time, state, seconds, 0])

    for (i, (_, _, seconds, _)) in enumerate(marks):
        if i != len(marks) - 1:
            delta = marks[i + 1][2] - seconds
        else:
            delta = 0

        marks[i][3] = delta

    #marks.sort(key=lambda m: -m[3])

    # for (time, state, seconds, delta) in marks:
    #     highlight = '*' * int(delta)
    #     print '{} {:6s} {:6.2f} {}'.format(time, state, delta, highlight)

    planning = 0
    perception = 0
    system = 0
    motion_planning = 0

    planning_total = 0
    perception_total = 0
    system_total = 0
    motion_planning_total = 0
    exec_total = 0

    for (time, state, seconds, delta) in marks:
        highlight = '*' * int(delta)
        print '{} {:6s} {:6.2f} {}'.format(time, state, delta, highlight)

        if re.match(r'ER\d+', state):
            exec_total += delta
            times = [planning, perception, system, motion_planning, delta]
            total = sum(times)

            print '=' * 80
            print 'Pl {0[0]:5.2f} {0[1]:.0f}%  Pe {1[0]:5.2f} {1[1]:.0f}%  Sys {2[0]:5.2f} {2[1]:.0f}%  MP {3[0]:5.2f} {3[1]:.0f}%  Ex {4[0]:5.2f} {4[1]:.0f}%  => {5:5.2f}'.format(*zip(times, [100 * x / total for x in times]) + [total])
            print

            planning = 0
            perception = 0
            system = 0
            motion_planning = 0

        elif re.match(r'CPB|CPS|CPI|SP\d+|RP\d+', state):
            perception += delta
            perception_total += delta

        elif re.match(r'EGVS|EP', state):
            planning += delta
            planning_total += delta

        elif re.match(r'SI|II|CI|RS\d*|DG', state):
            system += delta
            system_total += delta

        elif re.match(r'PSG|PPS|PVL|PPO|PIS', state):
            motion_planning += delta
            motion_planning_total += delta

        else:
            raise RuntimeError('unrecognized state category: {}'.format(state))


    times = [planning_total, perception_total, system_total, motion_planning_total, exec_total]
    total = sum(times)

    print
    print '=' * 36, 'TOTALS', '=' * 36
    print 'Pl {0[0]:5.2f} {0[1]:.0f}%  Pe {1[0]:5.2f} {1[1]:.0f}%  Sys {2[0]:5.2f} {2[1]:.0f}%  MP {3[0]:5.2f} {3[1]:.0f}%  Ex {4[0]:5.2f} {4[1]:.0f}%  => {5:5.2f}'.format(*zip(times, [100 * x / total for x in times]) + [total])


if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='state timeline analysis', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('logfile', nargs='+', metavar='LOGFILE', help='path of log file to analyze')

    args = parser.parse_args()

    for path in args.logfile:
        run(path)
