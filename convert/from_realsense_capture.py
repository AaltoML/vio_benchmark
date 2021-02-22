"""
Convert a single or a pair of recordings, Android & RealSense to a benchmark session
"""
import json
import os
import shutil
from os.path import join

def get_rs_video_file_names(jsonl_path):
    base = jsonl_path.rpartition('.jsonl')[0]
    return (base + '-video.avi', base + '-video2.avi')


def parse_args():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('realsense_jsonl_path')
    p.add_argument('phone_video_path', nargs='?')
    p.add_argument('output_folder', nargs='?')
    p.add_argument('--time_shift', type=float, default=0,
        help="timeshift: phone - realsense in seconds")
    p.add_argument('--parameters',
        default='focalLength 980;videoRotation CW90;rot 0;',
        help='contents of parameters.txt')
    return p.parse_args()


def coord_transform(x, y, z):
    return {
        'x': x,
        'y': -z,
        'z': y
    }


def rs_to_ground_truth_gen(in_file, time_shift=0, fix_gt_time=True, include_everything=False):
    lastT = 0
    for line in in_file:
        d = json.loads(line)
        if 'output' in d:
            if d['output']['position']['x'] == None:
                print("Warning! Skipping null position!")
                continue
            if fix_gt_time:
                t = lastT
            else:
                t = d['time']
            yield(json.dumps({
                'time': t + time_shift,
                'groundTruth': {
                    'position': coord_transform(**d['output']['position'])
                }
            })+'\n')
        elif 'time' in d:
            lastT = d['time']
        if include_everything:
            yield(json.dumps(d) + '\n')


def combine_jsonls(rs_path, phone_path, time_shift, target_path):
    with open(target_path, 'wt') as out, open(rs_path) as rs, open(phone_path) as phone:
        for line in rs_to_ground_truth_gen(rs, time_shift):
            out.write(line)
        for line in phone: out.write(line)


def processSingle(realsense_jsonl_path):
    outputFolder = os.path.dirname(realsense_jsonl_path)
    data = outputFolder + "/data.jsonl"
    backup = outputFolder + "/data_backup.jsonl"
    if os.path.exists(backup):
        raise Exception("Backup file already exists! {}".format(backup))
    os.rename(data, backup)
    with open(data, 'wt') as out, open(backup) as rs:
        for line in rs_to_ground_truth_gen(rs, include_everything=True):
            out.write(line)


def process(output_folder, realsense_jsonl_path, phone_video_path, parameters, time_shift):
    if not phone_video_path:
        return processSingle(realsense_jsonl_path)

    rsdir = join(output_folder, 'realsense')
    try:
        os.mkdir(output_folder)
        os.mkdir(rsdir)
    except: pass

    left_cam_src, right_cam_src = get_rs_video_file_names(realsense_jsonl_path)
    shutil.copyfile(realsense_jsonl_path, join(rsdir, 'data.jsonl'))
    shutil.copyfile(left_cam_src, join(rsdir, 'data.avi'))
    shutil.copyfile(right_cam_src, join(rsdir, 'data2.avi'))

    phone_vid_ext = phone_video_path.split('.')[-1]
    shutil.copyfile(phone_video_path, join(output_folder, 'data.' + phone_vid_ext))

    target_jsonl = join(output_folder, 'data.jsonl')
    phone_jsonl_path = phone_video_path.rpartition(phone_vid_ext)[0] + 'jsonl'
    if len(parameters) > 0:
        with open(join(output_folder, 'parameters.txt'), 'wt') as params:
            params.write(parameters + '\n')

    combine_jsonls(realsense_jsonl_path, phone_jsonl_path, time_shift, target_jsonl)

process(**vars(parse_args()))
