import numpy as np
import pprint
import open3d as o3d
import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--file', help='file name', required=True)
parser.add_argument('--number', help='number of save frames')


def main(ReadFileName, file_number=1,  verbose=False):
    # Input: Read File Name
    # Output: [y,z,x,distance(r),Lidar ID]

    Idx = 28
    rf = open(ReadFileName, 'rb')
    d = rf.read()
    FSIZE = len(d)
    DeviceCount = d[Idx]
    Idx += 1
    print("LidarSN", d[Idx:Idx+16].decode())

    Idx = Idx + 59 * DeviceCount
    list1 = []
    pcd_frame_list = []

    end = 0
    b = 0
    while (Idx < FSIZE):
        if verbose:
            print("Idx", Idx)
            print("current offset:", int.from_bytes(d[Idx:(Idx+8)], 'little'))
            print("next offset:", int.from_bytes(d[Idx+8:(Idx+16)], 'little'))
            print("frame index:", int.from_bytes(d[Idx+16:(Idx+24)], 'little'))
        nxt = int.from_bytes(d[Idx+8:(Idx+16)], 'little')

        Idx = Idx + 24
        b = b+1
        while Idx < nxt:
            dtype = d[Idx+10]
            Idx = Idx + 19

            if dtype == 6:
                # The data is gyro outputs. Please modify here if you want to use that data.
                Idx = Idx + 24
            elif dtype == 2:
                # Data is point clouds.
                for i in range(96):

                    B2D_X = int.from_bytes(d[Idx:Idx+4], 'little', signed=True)
                    B2D_Y = 0 - \
                        int.from_bytes(d[Idx+4:Idx+8], 'little', signed=True)
                    B2D_Z = int.from_bytes(
                        d[Idx+8:Idx+12], 'little', signed=True)
                    list_tmp = ([B2D_Y,
                                 B2D_Z,
                                 B2D_X,
                                 d[Idx+12]
                                 ])
                    if not B2D_X+B2D_Y+B2D_Z == 0 and B2D_X+B2D_Y+B2D_Z < 1e6:
                        list1.append(list_tmp)
                    Idx = Idx + 14
            else:
                # Something is wrong..
                print("dtype", dtype)

        OutData = np.array(list1)
        OutData = np.delete(OutData, 3, 1)
        OutData = OutData/1000
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(OutData)
        pcd_frame_list.append(pcd)
        if (b % file_number == 0 ):
            o3d.io.write_point_cloud(str(b) + '.pcd', pcd)

    rf.close

    return pcd_frame_list


args = parser.parse_args()

if __name__ == "__main__":
    main(args.file, int(args.number))
