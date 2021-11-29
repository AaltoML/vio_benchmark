"""
Postprocessing: Combine VIO and SLAM tracks
"""

import csv, json, math

def quat2rmat(q):
    import numpy as np
    return np.array([
        [q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3], 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
        [2*q[1]*q[2] + 2*q[0]*q[3], q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3], 2*q[2]*q[3] - 2*q[0]*q[1]],
        [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]]
    ])

def rmat2quat(mat):
    # from https://eigen.tuxfamily.org/dox/Quaternion_8h_source.html
    t = mat.trace()
    if t > 0.0:
        t = math.sqrt(t + 1.0)
        qw = 0.5*t
        t = 0.5/t
        qx = (mat[2,1] - mat[1,2]) * t
        qy = (mat[0,2] - mat[2,0]) * t
        qz = (mat[1,0] - mat[0,1]) * t
    else:
        i = 0;
        if (mat[1,1] > mat[0,0]):
            i = 1
        if (mat[2,2] > mat[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = math.sqrt(mat[i,i]-mat[j,j]-mat[k,k] + 1.0)
        qxyz = [0, 0, 0]
        qxyz[i] = 0.5 * t
        t = 0.5/t
        qw = (mat[k,j]-mat[j,k])*t
        qxyz[j] = (mat[j,i]+mat[i,j])*t
        qxyz[k] = (mat[k,i]+mat[i,k])*t
        qx,qy,qz = qxyz

    return [qw, qx, qy, qz]

def slerp(q0, q1, t):
    import numpy as np
    q0 = np.array(q0)
    q1 = np.array(q1)

    # also from Eigen
    d = q0.dot(q1)
    absD = abs(d)

    EPS = 1e-10 # ?
    if absD >= 1.0 - EPS:
        scale0 = 1.0 - t
        scale1 = t
    else:
        # theta is the angle between the 2 quaternions
        theta = math.acos(absD)
        sinTheta = math.sin(theta)

        scale0 = math.sin( ( 1.0 - t ) * theta) / sinTheta
        scale1 = math.sin( ( t * theta) ) / sinTheta
        if d < 0.0: scale1 = -scale1

    return scale0 * q0 + scale1 * q1

def interpolate(slam0, slam1, vioObjs):
    import numpy as np

    slam = [slam0, slam1]
    vioEnds = [vioObjs[0], vioObjs[-1]]
    # find vio frames that best match SLAM timestamps (usually first & last)
    for o in vioObjs:
        t = o['time']
        for i in range(2):
            if abs(slam[i][0] - t) < abs(slam[i][0] - vioEnds[i]['time']):
                vioEnds[i] = o

    def posQuatTo4x4(pos, quat):
        M = np.eye(4)
        M[:3,:3] = quat2rmat(quat)
        M[:3, 3] = pos
        return M

    def slamTo4x4(s):
        assert(len(s) == 1+3+4)
        return posQuatTo4x4(s[1:4], s[4:8])

    def vioTo4x4(d):
        M = posQuatTo4x4(
            [d['position'][c] for c in 'xyz'],
            [d['orientation'][c] for c in 'wxyz'])
        M[:3,:3] = M[:3,:3].transpose() # <--- TODO: should not be here
        return M

    def solveTransformAtoB(localToWA, localToWB):
        return np.dot(localToWB, np.linalg.inv(localToWA))

    transforms = [solveTransformAtoB(vioTo4x4(vioEnds[i]), slamTo4x4(slam[i])) for i in (0, 1)]

    for d in vioObjs:
        theta = (d['time'] - vioEnds[0]['time']) / float(vioEnds[1]['time'] - vioEnds[0]['time'])
        T0 = transforms[0]
        T1 = transforms[1]
        T = np.eye(4)
        T[:3, 3] = theta * T1[:3, 3] + (1-theta) * T0[:3, 3]
        T[:3,:3] = quat2rmat(slerp(rmat2quat(T0[:3,:3]), rmat2quat(T1[:3,:3]), theta))

        M = vioTo4x4(d)
        M = np.dot(T, M)
        q = rmat2quat(M[:3,:3])
        t = M[:3, 3]

        d['position'] = { 'xyz'[i]: t[i] for i in range(3) }
        d['orientation'] = { 'wxyz'[i]: q[i] for i in range(4) }

    return vioObjs

def readSlamCsvFile(slamCsvFile):
    def gen():
        with open(slamCsvFile) as f:
            for row in csv.reader(f):
                yield([float(c) for c in row])
    return list(gen())

def convertSlamOutput(slamCsvFile, outputFile):
    slamTrack = readSlamCsvFile(slamCsvFile)
    with open(outputFile, 'wt') as fOut:
        for d in slamTrack:
            j = {
                "time": d[0],
                "position": { "x": d[1], "y": d[2], "z": d[3] },
                "orientation": { "w": d[4], "x": d[5], "y": d[6], "z": d[7] },
            }
            fOut.write(json.dumps(j) + '\n')

def postprocessMergeSlamAndVioOutput(vioOutFile, slamCsvFile, mergedOutputFile):
    slamTrack = readSlamCsvFile(slamCsvFile)
    assert(len(slamTrack) >= 2)
    inputPiece = []
    with open(vioOutFile) as fIn, open(mergedOutputFile, 'wt') as fOut:
        for line in fIn:
            d = json.loads(line)
            while len(slamTrack) > 2 and d['time'] > slamTrack[1][0]:
                for o in interpolate(slamTrack[0], slamTrack[1], inputPiece):
                    fOut.write(json.dumps(o) + '\n')
                slamTrack.pop(0)
                inputPiece = []
            inputPiece.append(d)
        for d in interpolate(slamTrack[0], slamTrack[1], inputPiece):
            fOut.write(json.dumps(d) + '\n')

if __name__ == '__main__':
    import argparse, os, tempfile
    parser = argparse.ArgumentParser(__doc__.strip())
    parser.add_argument('vio_jsonl_file')
    parser.add_argument('slam_csv_file')
    parser.add_argument('output_jsonl_file', nargs='?')
    parser.add_argument('-p', '--plot', action='store_true')
    args = parser.parse_args()

    out = args.output_jsonl_file
    if out is None:
        out = tempfile.NamedTemporaryFile(delete=False).name

    postprocessMergeSlamAndVioOutput(args.vio_jsonl_file, args.slam_csv_file, out)

    if args.plot:
        import matplotlib.pyplot as plt
        import numpy as np
        xyz = []
        with open(out) as f:
            for line in f:
                p = json.loads(line)['position']
                xyz.append([p[c] for c in 'xyz'])
        xyz = np.array(xyz)
        plt.plot(xyz[:,0], xyz[:,1])
        plt.show()

    if args.output_jsonl_file is None:
        os.remove(out)
