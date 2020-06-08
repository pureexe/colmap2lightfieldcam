import numpy as np
from scipy.spatial.transform import Rotation

def write_point3d(f, point3ds):
    for point3d in point3ds:
        x, y, z = point3d['position']
        #x,y,z = np.random.normal(0, 0.1, 3)
        r, g, b = point3d['color']
        f.write('{:f} {:f} {:f} {:d} {:d} {:d}\n'.format(
            float(x), float(y), float(z), int(r), int(g), int(b)
        ))
            
def instrinsic_builder(intrisics):
    model = intrisics['model']
    params = intrisics['params']
    params.astype(np.float32)
    if model == 'SIMPLE_PINHOLE':
        return '{:f} {:f} 1 {:f} 0\n'.format(
            params[1],
            params[2],
            params[0],
        )
    elif model == 'PINHOLE':
        return "{:f} {:f} 2 {:f} {:f} 0\n".format(
            params[2],
            params[3],
            params[0],
            params[1],
        )
    elif model == 'SIMPLE_RADIAL':
        return '{:f} {:f} 1 {:f} 1 {:f}\n'.format(
            params[1],
            params[2],
            params[0],
            params[3] 
        )
    elif model == 'RADIAL':
        return '{:f} {:f} 1 {:f} 2 {:f} {:f}\n' [
            params[1],
            params[2],
            params[0],
            params[3],
            params[4]
        ]
    else:
        raise RuntimeError(
            'Camera number {} using {} which is currently not support'
                .format(intrisics['id'], model)
        )

def write_instrinsic(f, intrinsics):
    for intrinsic in intrinsics:
        f.write(instrinsic_builder(intrinsic))

def write_file(output_path, data):
    api_version = 0.01
    point2ds, intrinsics, extrinsics, point3ds = data
    extrinsics_for_point2d = extrinsics

    with open(output_path,'w') as f:
        f.write('{:f}\n'.format(api_version)) #version at head
        f.write('{:d} {:d} {:d} {:d} {:d}\n'.format(
            len(point2ds),
            len(intrinsics),
            len(extrinsics),
            0,
            len(point3ds)
        ))

        point3d_ids = {}
        point3d_count = 0

        #build lookup for point2d
        for point3d in point3ds:
            point3d_ids[point3d['id']] = point3d_count
            point3d_count = point3d_count + 1

        for point2d in point2ds:
                x,y = point2d['position']
                f.write('{:d} {:d} {:d} {:f} {:f}\n'.format(
                    int(point2d['camera_id'] - 1), #index start from 0
                    int(point2d['image_id'] - 1),
                    int(point3d_ids[point2d['point3d_id']]),
                    float(x),
                    float(y)
                ))
        write_instrinsic(f,intrinsics)
        for extrinsic in extrinsics:
            tvec = extrinsic['translation']
            qvec = extrinsic['rotation']
            rotation = Rotation.from_quat([qvec[1],qvec[2],qvec[3],qvec[0]])
            rotvec = rotation.as_rotvec()
            f.write('{:f} {:f} {:f} 3 {:f} {:f} {:f}\n'.format(
                tvec[0],
                tvec[1],
                tvec[2],
                rotvec[0],
                rotvec[1],
                rotvec[2]
            ))
        write_point3d(f, point3ds)