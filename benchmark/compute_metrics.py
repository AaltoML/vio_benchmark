# To compute and plot a metric for an already run benchmark, use eg:
#   python3 vio_benchmark/benchmark/compute_metrics.py \
#     output/2021-01-04_13-08-26 \
#     -p --metric_set=full_3d_align

import argparse, os, sys
from collections import OrderedDict
import numpy as np
import json
import math
from pathlib import Path


SHADES_OF_BLUE = ['blue', 'darkturquoise', 'darkorchid', 'dodgerblue', 'darkslateblue']

EXTERNAL_COLORS = {
    'groundtruth': 'salmon',
    'realsense': 'black',
    'arkit': 'deeppink',
    'arcore': 'lawngreen',
    'arengine': 'violet',
    'ondevice': 'steelblue',
    'slam': '#202020',
    'gps': 'darkred',
    'rtkgps': 'salmon'
}

DEFAULT_COMPARISONS = 'groundTruth,ARKit,ARCore,AREngine,OnDevice,RealSense,GPS,RTKGPS' # Exclude slam map by default

def parse_args():
    p = argparse.ArgumentParser(
        description="compute benchmarking metrics for a single test case")
    p.add_argument('folder')
    p.add_argument('-case', '--casename', type=str)
    p.add_argument('-p', '--show_plot', action='store_true')
    p.add_argument('-z', '--z_axis', action='store_true')
    p.add_argument('--columns', type=int, default=0)
    p.add_argument('-fo', '--figure_output', type=str)
    p.add_argument('-t', '--title', type=str)
    p.add_argument('--metricFile', type=str)
    p.add_argument('--metric_set', type=str, default='default', choices={'default', 'xy_only', 'full_2d_align', 'full_3d_align', 'full_sim3'})
    return p.parse_args()

def metric_set_to_alignment_params(metric_set):
    if metric_set == 'full_2d_align':
        return dict(fix_origin=False)
    elif metric_set == 'full_3d_align':
        return dict(fix_origin=False, align3d=True)
    elif metric_set == 'full_sim3':
        return dict(fix_origin=False, align3d=True, fix_scale=False)
    else:
        return {}

def getColor(datasetName="ours", index=0):
    # return "black"
    if datasetName == "ours":
        return SHADES_OF_BLUE[index % len(SHADES_OF_BLUE)]
    else:
        return EXTERNAL_COLORS[datasetName.lower()]

def wordWrap(s):
    LINE_MAX_LENGTH = 150
    out = ""
    l = 0
    for i, token in enumerate(s.split(" ")):
        l += len(token) + 1
        if l > LINE_MAX_LENGTH:
            out += "\n"
            l = 0
        elif i > 0:
            out += " "
        out += token
    return out

# Use covariance to rotate data to minimize height
def optimalRotation(data, ax1=1, ax2=2):
    dataxy = data[:,[ax1, ax2]]
    ca = np.cov(dataxy, y = None, rowvar = 0, bias = 1)
    v, vect = np.linalg.eig(ca)
    tvect = np.transpose(vect)
    ar = np.dot(dataxy, np.linalg.inv(tvect))
    dimensions = np.max(ar,axis=0) - np.min(ar,axis=0)
    if dimensions[0] > dimensions[1]:
        data[:,[ax1, ax2]] = ar
    else:
        ar_90_degrees = np.dot(ar, np.linalg.inv([[0, -1], [1, 0]]))
        data[:,[ax1, ax2]] = ar_90_degrees

def get_overlapping_xyz_parts_on_same_time_grid(out, gt):
    gt_t = gt[:,0]
    out_t = out[:,0]
    min_t = max(np.min(out_t), np.min(gt_t))
    max_t = min(np.max(out_t), np.max(gt_t))
    # Use the ground-truth grid because ground-truth may have gaps (GPS and prisms)
    # and VIO developers can always increase their methods' output rate.
    gt_part = gt[(gt_t >= min_t) & (gt_t <= max_t), :]
    out_part = np.hstack([np.interp(gt_part[:, 0], out_t, out[:,i])[:, np.newaxis] for i in range(out.shape[1])])
    return out_part[:, 1:], gt_part[:, 1:]

def align_to_gt(out, gt, rel_align_time=1/3.0, fix_origin=True, align3d=False, fix_scale=True):
    """
    Align by rotating so that angle(gt[t]) = angle(out[t]), relative to the
    origin at some timestamp t, which is, determined as e.g, 1/3 of the
    session length
    """
    if len(out) <= 0 or len(gt) <= 0: return out

    out_part, gt_part = get_overlapping_xyz_parts_on_same_time_grid(out, gt)
    if out_part.shape[0] <= 0: return out

    if fix_origin:
        gt_ref = gt_part[0, :]
        out_ref = out_part[0, :]
    else:
        gt_ref = np.mean(gt_part, axis=0)
        out_ref = np.mean(out_part, axis=0)

    if align3d:
        if rel_align_time > 0:
            # partial 3D align, not very well tested, use with caution
            t = int(len(out[:,0]) * rel_align_time)
            if out_part.shape[0] > t and t > 0:
                out_part = out_part[:t, :]
                gt_part = gt_part[:t, :]

        out_xyz = (out_part - out_ref).transpose()
        gt_xyz = (gt_part - gt_ref).transpose()

        if out_xyz.shape[1] <= 0: return out

        if fix_scale:
            scale = 1
        else:
            get_scale = lambda xyz: np.mean(np.sqrt(np.sum(xyz**2, axis=0)))
            scale = min(get_scale(gt_xyz) / max(get_scale(out_xyz), 1e-5), 100)

        # Procrustes / Wahba SVD solution
        B = np.dot(gt_xyz, scale * out_xyz.transpose())
        U, S, Vt = np.linalg.svd(B)
        R = np.dot(U, Vt)
        # Check for mirroring (not sure if this ever happens in practice)
        if np.linalg.det(R) < 0.0:
            flip = np.diag([1, 1, -1])
            R = np.dot(U, np.dot(flip, Vt))
        R *= scale

        aligned = out * 1
        aligned[:, 1:4] = np.dot(R, (out[:, 1:] - out_ref).transpose()).transpose() + gt_ref
        return aligned

    # else align in 2d
    # represent track XY as complex numbers
    xy_to_complex = lambda arr: arr[:,0] + 1j * arr[:,1]
    gt_xy = xy_to_complex(gt_part - gt_ref)
    out_xy = xy_to_complex(out_part - out_ref)

    rot = 1
    if rel_align_time > 0.0:
        # rotate to match direction vectors at a certain time
        t = int(len(out[:,0]) * rel_align_time)
        max_t = min(len(out_xy), len(gt_xy))

        if t < max_t and np.minimum(np.abs(gt_xy[t]), np.abs(out_xy[t])) > 1e-5:
            rot = gt_xy[t] / out_xy[t]
        else:
            # align using full track if fails
            rel_align_time = -1

    if rel_align_time <= 0:
        # align using the full track
        valid = np.minimum(np.abs(gt_xy), np.abs(out_xy)) > 1e-5
        if np.sum(valid) > 0:
            rot = gt_xy[valid] / out_xy[valid]
            rot = rot / np.abs(rot)
            rot = np.mean(rot)

    if fix_scale:
        rot = rot / np.abs(rot)

    # rotate track, keeping also the parts that do not have GT
    align_xy = xy_to_complex(out[:, 1:] - out_ref) * rot

    # convert back to real
    aligned = out * 1
    aligned[:,1:] -= out_ref
    aligned[:,1] = np.real(align_xy)
    aligned[:,2] = np.imag(align_xy)
    aligned[:,1:] += gt_ref
    return aligned

def piecewise_align_to_gt(out, gt, piece_len_sec=10.0, na_breaks=False):
    gt_t = gt[:,0]
    out_t = out[:,0]
    max_t = np.max(gt_t)
    #n_pieces = int(np.ceil(max_t / piece_len_sec))
    t = np.min(gt_t)
    aligned = []
    while t < max_t:
        t1 = t + piece_len_sec
        #print t
        #print t1
        gt_slice = gt[(gt_t >= t) & (gt_t < t1), :]
        out_slice = out[(out_t >= t) & (out_t < t1), :]
        aligned_slice = align_to_gt(out_slice, gt_slice)
        aligned.append(aligned_slice)
        if na_breaks:
            na_spacer = aligned_slice[-1:,:]
            na_spacer[:,1:] = np.nan
            aligned.append(na_spacer)
        t = t1

    return np.vstack(aligned)

def rmse(a, b):
    """Root Mean Square Error"""
    return np.sqrt(np.mean(np.sum((a - b)**2, axis=1)))

def mean_absolute_error(a, b):
    """Mean Absolute Error (MAE)"""
    return np.mean(np.sqrt(np.sum((a - b)**2, axis=1)))

def compute_piecewise_metric(out, gt, piece_len_sec=10.0, measureZError=True):
    """RMSE of the aligned XY track (in ground truth time grid)"""
    aligned = piecewise_align_to_gt(out, gt, piece_len_sec)
    if not measureZError:
        aligned = aligned[:,:-1]
        gt = gt[:,:-1]
    interpolated, gt = get_overlapping_xyz_parts_on_same_time_grid(aligned, gt)
    return rmse(gt, interpolated)

def compute_metric_set(out, gt, metric_set):
    if metric_set in ['default', 'xy_only']:
        measureZError = metric_set != 'xy_only'
        return OrderedDict([
            ("1s", compute_piecewise_metric(out, gt, 1.0, measureZError)),
            ("10s", compute_piecewise_metric(out, gt, 10.0, measureZError)),
            ("30s", compute_piecewise_metric(out, gt, 30.0, measureZError)),
            ("100s", compute_piecewise_metric(out, gt, 100.0, measureZError))
        ])
    elif metric_set.startswith('full_'):
        aligned = align_to_gt(out, gt, -1, **metric_set_to_alignment_params(metric_set))
        aligned, gt = get_overlapping_xyz_parts_on_same_time_grid(aligned, gt)
        return OrderedDict([
            ('RMSE', rmse(gt, aligned)),
            ('MAE', mean_absolute_error(gt, aligned))
        ])
    else:
        raise RuntimeError("Unknown metric set " + metric_set)

def average_metric(metrics):
    total = 0.0
    for x in metrics.values():
        total += x
    return total / len(metrics)

def metrics_to_string(metrics, short=True):
    mean = "{:.3g}".format(np.mean([metrics[x] for x in metrics]))
    submetrics = " | ".join(["{:.3g}".format(metrics[x]) for x in metrics])
    met = "{} -- ({})".format(mean, submetrics)
    if short:
        return met
    legend = " | ".join([x for x in metrics])
    return "{} -- ({})".format(met, legend)


def agg_metrics(metrics):
    result = OrderedDict()
    for x in metrics[0]:
        result[x] = np.mean(list(map(lambda i: i[x], metrics)))
    return result


def read_gt(fn):
    d = np.genfromtxt(fn, delimiter=',')
    txyz = np.hstack([d[:,i][:,np.newaxis] for i in [0,1,3,2]])
    txyz[:,1] = -txyz[:,1]
    return {"datasets": [{"name": "groundTruth", "data": txyz}]}


def read_gt_json(fn, comparisonList):
    with open(fn) as f:
        data = json.load(f)
        # Filter out datasets we don't want to compare with if comparisonList is not falsey
        if comparisonList:
            lowerCaseList = [x.lower() for x in comparisonList.split(",")]
            sortedDatasets = []
            for s in lowerCaseList:
                for d in data['datasets']:
                    if d["name"].lower() == s:
                        sortedDatasets.append(d)
            data['datasets'] = sortedDatasets
        # data['datasets'] = [x for x in data['datasets'] if not (comparisonList and not x["name"].lower() in [x.lower() for x in comparisonList.split(",")])]
        for dataset in data['datasets']:
            # Perform same tricks as def read_gt(fn):
            d = np.array(dataset['data'])
            txyz = np.hstack([d[:,i][:,np.newaxis] for i in [0,1,3,2]])
            txyz[:,1] = -txyz[:,1]
            dataset['data'] = txyz
        return data


def read_out(fn):
    bias_norm = lambda x: np.sqrt(np.sum(x**2, axis=1))
    to_arr = lambda obj: [obj["x"], obj["y"], obj["z"]]
    txyz = []
    bga = []
    baa = []
    bat = []
    stat = []
    with open(fn) as f:
        for line in f.readlines():
            row = json.loads(line)
            txyz.append([row["time"], row["position"]["x"], row["position"]["y"], row["position"]["z"]])
            stat.append(row.get("stationary", False))
            if (row.get("biasMean")):
                bga.append(to_arr(row["biasMean"]["gyroscopeAdditive"]))
                baa.append(to_arr(row["biasMean"]["accelerometerAdditive"]))
                if "accelerometerTransform" in row["biasMean"]:
                    bat.append(to_arr(row["biasMean"]["accelerometerTransform"]))
    return {
        'txyz': np.array(txyz),
        'BGA': bias_norm(np.array(bga)) if bga else 0.0,
        'BAA': bias_norm(np.array(baa)) if baa else 0.0,
        'BAT': bias_norm(np.array(bat) - 1.0) if bat else 0.0,
        'stationary': np.array(stat)
    }


def getHeight(dataset, ax):
    return np.amax(dataset[:,ax]) - np.amin(dataset[:,ax])


def doOffset(trackOffset, maxHeight, dataset, ax):
    height = getHeight(dataset, ax)
    dataset[:,ax] = dataset[:,ax] + (trackOffset - np.max(dataset[:,ax])) - (maxHeight - height) / 2
    return trackOffset - maxHeight


def exportTsvPaths(data, name):
    with open(name + ".tsv", "w") as f:
        f.write("time\tx\ty\tz\n")
        for row in data:
            f.write("{}\t{}\t{}\t{}\n".format(row[0], row[1], row[2], row[3]))


def plot(benchmarkSet, gt, plot_axis, ax1=1, ax2=2, plot_title=None, gt_color='orange', piecewise=True, offsetTracks=False, exportPath=None, **alignment_kwargs):
    import matplotlib.pyplot as plt
    title_str = plot_title if plot_title else ""
    if plot_axis is None:
        axis = plt
        is_subplot = False
    else:
        axis = plot_axis
        is_subplot = True

    # Align data with ground truth
    groundtruth = None
    maxHeight = 0
    if gt is not None:
        groundtruth = gt['datasets'][0]['data']
        if exportPath: exportTsvPaths(groundtruth, "{}/{}-{}".format(exportPath, title_str,  gt['datasets'][0]['name']))
        # TODO: Use this all the time, because most of the time there is more horzontal space?
        if offsetTracks: optimalRotation(groundtruth, ax1, ax2)
        maxHeight = getHeight(groundtruth, ax2)
        for dataset in gt['datasets'][1:]:
            dataset['data'] = align_to_gt(dataset['data'], groundtruth, -1, **alignment_kwargs)
            if exportPath: exportTsvPaths(dataset['data'], "{}/{}-{}".format(exportPath, title_str,  dataset['name']))
            maxHeight = max(maxHeight, getHeight(dataset['data'], ax2))
        for index, bench in enumerate(benchmarkSet["benchmarks"]):
            bench["aligned"] = align_to_gt(bench["out"]['txyz'], groundtruth, -1, **alignment_kwargs)
            if exportPath: exportTsvPaths(bench["aligned"], "{}/{}-{}".format(exportPath, title_str,  bench["paramSet"]))
            maxHeight = max(maxHeight, getHeight(bench["aligned"], ax2))
        maxHeight = maxHeight * 1.05 # 5% padding

    trackOffset = 0

    # Draw ground truth and other external plots
    if gt is not None:
        axis.plot(groundtruth[:,ax1], groundtruth[:,ax2], label=gt['datasets'][0]['name'], color=getColor(gt['datasets'][0]['name']), linewidth=1)
        trackOffset = np.amin(groundtruth[:,ax2])
        for dataset in gt['datasets'][1:]:
            if offsetTracks: trackOffset = doOffset(trackOffset, maxHeight, dataset['data'], ax2)
            axis.plot(dataset['data'][:,ax1], dataset['data'][:,ax2], label=dataset['name'], color=getColor(dataset['name']), linewidth=1)

    # Draw results
    for index, bench in enumerate(benchmarkSet["benchmarks"]):
        if "aligned" in bench:
            out = bench["aligned"]
        else:
            out = bench["out"]['txyz']

        if offsetTracks: trackOffset = doOffset(trackOffset, maxHeight, out, ax2)
        axis.plot(out[:,ax1], out[:,ax2], label=bench["paramSet"], color=getColor(index=index), linewidth=1.5)

        if groundtruth is not None and piecewise:
            pieces = piecewise_align_to_gt(out, groundtruth, 5.0, na_breaks=True)
            axis.plot(pieces[:,ax1], pieces[:,ax2], label='Ours (parts)', color=getColor(index=4), alpha=0.2, linewidth=1)

        # Draw point information.
        # out_full = bench["out"]
        # if 'stationary' in out_full:
        #     stat = out_full['stationary'] != 0
        #     axis.scatter(out[stat,ax1], out[stat,ax2], s=5, color='magenta', alpha=0.3) #, label='stationary')
        #     bad_bias = out_full['BAA'] > 1.0
        #     warn_bias = out_full['BAA'] > 0.7
        #     axis.scatter(out[warn_bias,ax1], out[warn_bias,ax2], s=5, color='red', alpha=0.1)
        #     axis.scatter(out[bad_bias,ax1], out[bad_bias,ax2], s=5, color='red', alpha=0.5, label=('bad bias' if index == 0 else None))

        # Draw ruler
        if offsetTracks and index == len(benchmarkSet["benchmarks"]) - 1:
            scale = max(1, math.pow(10, math.floor(math.log(np.amax(out[:,ax1]) - np.amin(out[:,ax1]), 10))))
            ruler_offset = np.array([np.amin(out[:,ax1]), np.amin(out[:,ax2])])
            ruler = [[-scale * 2,0], [-scale,0]] + ruler_offset
            axis.text(ruler_offset[0] - scale * 2, ruler_offset[1] + scale * 0.2, "{:.0f}m".format(scale))
            axis.plot(ruler[:,0], ruler[:,1], color='black', alpha=1, linewidth=2)

    # Draw legend
    if is_subplot:
        set_title = axis.title.set_text
        for item in axis.get_xticklabels() + axis.get_yticklabels():
            item.set_size(6)
    else: set_title = axis.title
    for case in benchmarkSet["benchmarks"]:
        if case.get("metric"):
            if title_str:
                title_str += "\n"
            hostname = case.get("hostname")
            if hostname:
                title_str += "[" + hostname + "] "
            if case["paramSet"] != "DEFAULT":
                title_str += case["paramSet"] + " "
            title_str += metrics_to_string(case["metric"])
    set_title(title_str)
    axis.axis('equal')


def figureSize(num_plots):
    if num_plots < 10:
        return (15,15)
    if num_plots < 20:
        return (20,20)
    return (30,30)


def compute_metrics(folder, casename=None, show_plot=None, z_axis=None, columns=0, figure_output=None, title=None,
    metricFile=None, comparisonList=None, piecewise=False, offsetTracks=False, metric_set='default', exportAlignedData=False):
    do_plot = show_plot or figure_output is not None
    plot_kwargs = metric_set_to_alignment_params(metric_set)
    if z_axis: plot_kwargs['ax2'] = 3

    if exportAlignedData:
        exportPath = folder + "/aligned"
        Path(exportPath).mkdir(parents=True, exist_ok=True)
    else:
        exportPath = None

    if do_plot:
        if not show_plot:
            import matplotlib
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt

    # Group all benchmarks by their original name or directory, they will share plot
    benchmarkSets = {}
    if not casename:
        for x in os.walk(folder + "/info"):
            for file in x[2]:
                benchmarkInfo = json.loads(open(os.path.join(x[0], file)).read())
                benchmarkSetName = benchmarkInfo.get("origName")
                benchmarkSet = benchmarkSets.get(benchmarkSetName)
                if not benchmarkSet:
                    benchmarkSet = {"name": benchmarkSetName, "benchmarks": []}
                    benchmarkSets[benchmarkSetName] = benchmarkSet
                benchmarkSet["benchmarks"].append(benchmarkInfo)
    else: # For single benchmarks
        benchmarkSets[casename] = {
            "name": casename,
            "benchmarks": [
                {
                    "paramSet": "DEFAULT",
                    "caseName": casename
                }
            ]
        }

    totalBenchmarks = len(benchmarkSets)
    for x in benchmarkSets:
        benchmarkSets[x]["benchmarks"] = sorted(benchmarkSets[x]["benchmarks"], key = lambda i: i['paramSet'])

    metrics = {}

    if do_plot:
        n_per_row = columns
        if n_per_row <= 0:
            n_per_row = int(np.ceil(np.sqrt(totalBenchmarks)))
        n_rows = int(np.ceil(float(totalBenchmarks) / n_per_row))
        figure, subplots = plt.subplots(n_rows, n_per_row, figsize=figureSize(totalBenchmarks))
        subplots = np.ravel(subplots)

    for i, benchmarkSetName in enumerate(sorted(benchmarkSets.keys())):
        benchmarkSet = benchmarkSets[benchmarkSetName]
        setName = benchmarkSet["name"]
        plot_kwargs['plot_title'] = setName
        if do_plot:
            plot_axis = subplots[i]
        else:
            plot_axis = None

        # All parameter sets share groundtruth, so pick first one
        gt_json = os.path.join(folder + "/ground-truth", benchmarkSet["benchmarks"][0]["caseName"] + ".json")
        gt_csv = os.path.join(folder + "/ground-truth", benchmarkSet["benchmarks"][0]["caseName"] + ".csv")

        gt = None
        if os.path.isfile(gt_json):
            if not comparisonList:
                comparisonList = DEFAULT_COMPARISONS
            gt = read_gt_json(gt_json, comparisonList)
        elif os.path.isfile(gt_csv):
            gt = read_gt(gt_csv)

        # Add output and gt comparison metric to each benchmark case
        for case in benchmarkSet["benchmarks"]:
            name = case["caseName"]
            case["out"] = read_out("{}/output/{}.jsonl".format(folder, name))
            if gt:
                case["metric"] = compute_metric_set(case["out"]['txyz'], gt['datasets'][0]['data'], metric_set)

                # Save numeric results as JSON.
                os.makedirs("{}/results".format(folder), exist_ok=True)
                with open("{}/results/{}.json".format(folder, name), "w") as f:
                    jsonResults = { "methods": {} }
                    jsonResults["methods"]["our"] = case["metric"]
                    if gt["datasets"]:
                        jsonResults["groundTruth"] = gt["datasets"][0]["name"]
                    # Compare other methods against the same ground truth.
                    for i in range(1, len(gt["datasets"])):
                        jsonResults["methods"][gt["datasets"][i]["name"]] = compute_metric_set(gt["datasets"][i]["data"], gt["datasets"][0]["data"], metric_set)
                    json.dump(jsonResults, f, indent=4, sort_keys=True)

                if metricFile:
                    with open(metricFile, "a") as f: f.write("%s,%.9f\n" % (name, average_metric(case["metric"])))
                if not metrics.get(case["paramSet"]):
                    metrics[case["paramSet"]] = []
                metrics[case["paramSet"]].append(case["metric"])
        plot(benchmarkSet, gt, plot_axis, piecewise=piecewise, offsetTracks=offsetTracks, exportPath=exportPath, **plot_kwargs)
        plot_axis.legend()

    if do_plot:
        for subplot in subplots[totalBenchmarks:]: subplot.axis('off')
        suptitle = "\n".join(["{} {}".format(
            setName,
            metrics_to_string(agg_metrics(metrics[setName]), short=False))
            for setName in sorted(metrics.keys())])
        if title is not None:
            suptitle = wordWrap(title) + "\n" +  suptitle
        figure.suptitle(suptitle, fontsize=12)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        if figure_output: figure.savefig(figure_output)
        if show_plot: plt.show()

    # Return mean over everything
    if metrics:
        return np.mean([average_metric(agg_metrics(metrics[m])) for m in metrics])
    else:
        return None


if __name__ == '__main__':
    compute_metrics(**vars(parse_args()))
