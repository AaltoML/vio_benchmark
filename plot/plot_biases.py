import json
import numpy as np

BIASES = ['baa', 'bga', 'bat']

def read_jsonl(fn):
    with open(fn) as f:
        for l in f: yield(json.loads(l))

def read_biases(fn):
    ts = []
    biases = { b: [] for b in BIASES }
    errors = { b: [] for b in BIASES }

    for o in read_jsonl(fn):
        if 'mean' not in o: continue
        ts.append(o['time'])
        for k in BIASES:
            biases[k].append([o['mean'][k][c] for c in 'xyz'])
            errors[k].append([np.sqrt(o['covDiag'][k][c]) for c in 'xyz'])

    to_np = lambda d: { k: np.array(v) for k, v in d.items() }

    return np.array(ts), to_np(biases), to_np(errors)

def read_temperatures(fn):
    ts = []
    temps = []
    C_TO_K = 273.15

    for o in read_jsonl(fn):
        if 'temperature' not in o: continue
        ts.append(o['time'])
        temps.append(o['temperature'] - C_TO_K)

    return np.array(ts), np.array(temps)

if __name__ == '__main__':
    import argparse
    import matplotlib.pyplot as plt

    p = argparse.ArgumentParser(__doc__)
    p.add_argument('vio_jsonl_out_file', help='VIO output file with biases')
    p.add_argument('which_bias', nargs='?', choices=BIASES, default='bga')
    p.add_argument('-T', '--temperature_file')
    args = p.parse_args()

    ts, biases, errors = read_biases(args.vio_jsonl_out_file)

    y = biases[args.which_bias]
    err = errors[args.which_bias]

    FIT_RANGE_T0 = 30

    if args.temperature_file:
        ts_temp, temps = read_temperatures(args.temperature_file)
        temps = np.interp(ts, ts_temp, temps)

        ts_range = ts > (ts[0] + FIT_RANGE_T0)

        for c in range(3):
            yt = y[ts_range, c]
            plt.scatter(temps[ts_range], yt, c=ts[ts_range] - ts[ts_range][0], s=5)
            plt.ylim([np.min(yt), np.max(yt)])
            plt.xlabel('temperature (C)')
            plt.ylabel('bias (rad/s)')
            plt.colorbar()
            plt.show()

    ts -= ts[0]

    err_color = [0.9 for _ in 'rgb']
    plt.plot(ts, y + err, c=err_color)
    plt.plot(ts, y - err, c=err_color)
    plt.plot(ts, y)
    plt.xlabel('t (s)')
    plt.ylabel('bias (rad/s)')
    plt.title(args.which_bias)

    fit_range = ts > FIT_RANGE_T0
    ylim = [np.min(y[fit_range, :]), np.max(y[fit_range, :])]
    margin = (ylim[1] - ylim[0])*0.25
    plt.ylim([ylim[0] - margin, ylim[1] + margin])
    plt.show()
