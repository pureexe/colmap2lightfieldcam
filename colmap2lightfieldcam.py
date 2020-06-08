# colmap to  format
import argparse
from deeparc.reader import database_reader_bfs, detect_model, detect_database
from deeparc.writer import write_file


def main(args):
    colmap_data = {}
    if detect_database(args.input):
        # if input is database file do read database
        print("reading database")
        colmap_data = database_reader_bfs(
            args.input,
            shift_point3d=[
                args.shift_point3d_x,
                args.shift_point3d_y,
                args.shift_point3d_z
            ]
        )
    else:
        raise RuntimeError(
            'input isn\'t valid database(.db)'
        )
    write_file(args.output,colmap_data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='colmap2deeparc.py - convert colmap into deeparc format')
    parser.add_argument(
        '-i',
        '--input',
        type=str,
        default="car.db",
        #required=True,
        help='colmap model directory / colmap database file (.db)',
    )
    parser.add_argument(
        '-o',
        '--output',
        type=str,
        #required=True,
        default='car.deeparc',
        help='deeparc file output')
    parser.add_argument(
        '-sx',
        '--shift-point3d-x',
        type=float,
        default=0.0,
        help='shift point3d in x axis (only using .db as input)'
    )
    parser.add_argument(
        '-sy',
        '--shift-point3d-y',
        type=float,
        default=0.0,
        help='shift point3d in y axis (only using .db as input)'
    )
    parser.add_argument(
        '-sz',
        '--shift-point3d-z',
        type=float,
        default=0.0,
        help='shift point3d in z axis (only using .db as input)'
    )    
    main(parser.parse_args())