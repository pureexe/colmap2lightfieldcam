import os
from colmap.database import COLMAPDatabase, blob_to_array, pair_id_to_image_ids
from colmap.read_write_model import read_model, CAMERA_MODEL_IDS
import multiprocessing
import numpy as np
import sqlite3
import sys
from collections import deque

def database_reader_bfs(database_path, shift_point3d = [0,0,0]):
  db = COLMAPDatabase.connect(database_path)
  c = db.cursor()
  c.execute('SELECT camera_id,model,params FROM cameras')
  cameras_data = c.fetchall()
  c.execute('SELECT image_id,camera_id,name FROM images')
  images_data = c.fetchall()
  c.execute("SELECT pair_id,rows,cols,data FROM matches")
  matches_data = c.fetchall()
  c.execute("SELECT image_id,rows,cols,data FROM keypoints")
  keypoints_data = c.fetchall()
  db.close()

  keypoints = {}
  intrinsic = []
  extrinsic = []
  point2d = []
  point3d = []

  print("Building keypoint search")
  for keypoint_image in keypoints_data:
    img_id, r, c, data = keypoint_image
    kpt = blob_to_array(data, np.float32,(r,c))
    keypoints[img_id] = kpt[:, :2]

  camera_lookup = {}
  for image in images_data:
    image_id, camera_id, image_name = image
    extrinsic.append({
      'id': image_id,
      'name': image_name,
      'rotation': np.array([1.0, 0.0, 0.0, 0.0]),
      'translation': np.random.normal(size=3)
    })
    camera_lookup[image_id] = camera_id

  for camera in cameras_data:
    camera_id, model, params = camera
    params = blob_to_array(params, np.float64)
    intrinsic.append({
      'id': camera_id,
      'model': CAMERA_MODEL_IDS[model][1],
      'params': params
    })

  print("Creating graph")
  edges = {}
  cc = 0

  # debug = {}
  for match_record in matches_data:
    cc += 1
    if cc % 1000 == 0:
      print('%.2f%%' % (cc / len(matches_data) * 100))
    image_from, image_to = pair_id_to_image_ids(match_record[0])
    image_from = int(image_from)
    image_to = int(image_to)
    if match_record[1] == 0:
      continue

    lookup = blob_to_array(match_record[3], np.int32, (match_record[1], match_record[2]))
    for pair in lookup:
      k0 = (image_from, pair[0])
      k1 = (image_to, pair[1])

      if k0 not in edges: edges[k0] = []
      if k1 not in edges: edges[k1] = []

      edges[k0].append(k1)
      edges[k1].append(k0)

  keypoint_counter = 0
  keypoint_ids = {}

  print("Assigning Ids")
  def bfs(node):
    q = deque()
    q.append(node)
    keypoint_ids[node] = keypoint_counter
    while q:
      node = q.pop()
      for nnode in edges[node]:
        if nnode not in keypoint_ids:
          keypoint_ids[nnode] = keypoint_counter
          q.append(nnode)
  for k in edges:
    if k in keypoint_ids: continue
    bfs(k)
    keypoint_counter += 1

  print("Verifying track")
  lst = {}
  for k in edges:
    keypoint_id = keypoint_ids[k]
    if keypoint_id not in lst:
      lst[keypoint_id] = []
    lst[keypoint_id].append(k)

  remapper = {}
  num_bad = 0
  keypoint_counter = 0
  for l, v in lst.items():
    check = {}
    bad = False
    for kp in v:
      if kp[0] in check:
        bad = True
        break
      check[kp[0]] = 1
    if bad:
      num_bad += 1
      remapper[l] = -1
    else:
      remapper[l] = keypoint_counter
      keypoint_counter += 1

  print("Removing", num_bad)
  print("Good track", keypoint_counter)

  for k in edges:
    img_id = k[0]
    keypoint_id = remapper[keypoint_ids[k]]
    if keypoint_id == -1: continue
    point2d.append({
      'image_id': img_id,
      'camera_id': camera_lookup[img_id],
      'point3d_id': keypoint_id,
      'position': keypoints[img_id][k[1]]
    })

  for i in range(keypoint_counter):
    position = np.random.normal(0,0.1,3)
    position += shift_point3d
    point3d.append({
      'id': i,
      'position': list(position),
      'color':  [255,255,255]
    })

  print("Done")
  print(keypoint_counter)
  return (point2d, intrinsic, extrinsic, point3d)

def detect_database(database_path):
    """
    make sure file end with (.db) and db file also exist
    """
    if database_path[-3:] != '.db':
        return False
    if not os.path.exists(database_path):
        return False
    return True

def detect_model(model_path,filetype = '.bin'):
    """ 
        detect is this colmap model directory by detect 3 files
        which is cameras.bin images.bin and points3D.bin
    """
    paths = [
        os.path.join(model_path,'cameras{}'.format(filetype)),
        os.path.join(model_path,'images{}'.format(filetype)),
        os.path.join(model_path,'points3D{}'.format(filetype)),
    ]
    for path in paths:
        if not os.path.exists(path):
            return False
    return True