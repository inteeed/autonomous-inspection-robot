#!/usr/bin/env python3
"""Generate ArUco marker textures, Gazebo Classic model dirs, and the factory/tank world.

Run once from the package root:
    python3 scripts/generate_world_assets.py
"""
import os
import sys
import textwrap

import cv2
import cv2.aruco as aruco

# Marker layout: id -> (x, y, yaw_deg, label). Yaw is the marker's facing direction.
# These positions sit on or against world geometry placed below.
MARKERS = [
    (0, 2.5, 0.0, 180, 'tank_A_inlet'),
    (1, 0.0, 2.5, 270, 'pipe_junction_1'),
    (2, -2.5, 0.0,   0, 'tank_B_outlet'),
    (3, 0.0, -2.5, 90, 'control_panel'),
    (4, 1.8, 1.8, 225, 'valve_cluster'),
]

MARKER_DICT = aruco.DICT_4X4_50
MARKER_SIZE_M = 0.4   # physical edge length of the marker face in meters
MARKER_THICKNESS = 0.01
MARKER_PIXELS = 600   # texture resolution


def write(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content)


def generate_marker_model(pkg_root: str, marker_id: int, label: str) -> None:
    model_name = f'aruco_marker_{marker_id}'
    model_dir = os.path.join(pkg_root, 'models', model_name)
    tex_dir = os.path.join(model_dir, 'materials', 'textures')
    scr_dir = os.path.join(model_dir, 'materials', 'scripts')
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(scr_dir, exist_ok=True)

    # Texture: white border around the aruco bits so the inner code reads cleanly.
    dictionary = aruco.Dictionary_get(MARKER_DICT)
    img = aruco.drawMarker(dictionary, marker_id, MARKER_PIXELS)
    border = MARKER_PIXELS // 10
    bordered = cv2.copyMakeBorder(img, border, border, border, border,
                                  cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(os.path.join(tex_dir, f'{model_name}.png'), bordered)

    # Ogre material script — Gazebo Classic resolves this via the model:// URIs in SDF.
    write(os.path.join(scr_dir, f'{model_name}.material'), textwrap.dedent(f'''\
        material {model_name}
        {{
          receive_shadows off
          technique
          {{
            pass
            {{
              lighting off
              texture_unit
              {{
                texture {model_name}.png
                filtering none
              }}
            }}
          }}
        }}
        '''))

    # model.config
    write(os.path.join(model_dir, 'model.config'), textwrap.dedent(f'''\
        <?xml version="1.0"?>
        <model>
          <name>{model_name}</name>
          <version>1.0</version>
          <sdf version="1.6">model.sdf</sdf>
          <description>ArUco marker {marker_id} ({label}) for inspection points.</description>
        </model>
        '''))

    # model.sdf — a thin static box with the marker textured on its +X face.
    half_t = MARKER_THICKNESS / 2.0
    write(os.path.join(model_dir, 'model.sdf'), textwrap.dedent(f'''\
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{model_name}">
            <static>true</static>
            <link name="link">
              <visual name="back">
                <geometry>
                  <box><size>{MARKER_THICKNESS} {MARKER_SIZE_M} {MARKER_SIZE_M}</size></box>
                </geometry>
                <material>
                  <ambient>0.9 0.9 0.9 1</ambient>
                  <diffuse>0.9 0.9 0.9 1</diffuse>
                </material>
              </visual>
              <visual name="face">
                <pose>{half_t + 0.0005} 0 0 0 0 0</pose>
                <geometry>
                  <box><size>0.0005 {MARKER_SIZE_M} {MARKER_SIZE_M}</size></box>
                </geometry>
                <material>
                  <script>
                    <uri>model://{model_name}/materials/scripts</uri>
                    <uri>model://{model_name}/materials/textures</uri>
                    <name>{model_name}</name>
                  </script>
                </material>
              </visual>
              <collision name="col">
                <geometry>
                  <box><size>{MARKER_THICKNESS} {MARKER_SIZE_M} {MARKER_SIZE_M}</size></box>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>
        '''))


def generate_world(pkg_root: str) -> None:
    # Static factory props: outer wall ring + two cylindrical "tanks" + a pipe-ish box.
    walls = []
    # 10x10 floor; place 4 walls just outside the play area.
    for i, (x, y, sx, sy) in enumerate([
        ( 5.5,  0.0, 0.2, 11.0),
        (-5.5,  0.0, 0.2, 11.0),
        ( 0.0,  5.5, 11.0, 0.2),
        ( 0.0, -5.5, 11.0, 0.2),
    ]):
        walls.append(textwrap.dedent(f'''\
            <model name="wall_{i}">
              <static>true</static>
              <link name="link">
                <pose>{x} {y} 1.0 0 0 0</pose>
                <visual name="v"><geometry><box><size>{sx} {sy} 2.0</size></box></geometry>
                  <material><ambient>0.55 0.55 0.6 1</ambient><diffuse>0.55 0.55 0.6 1</diffuse></material>
                </visual>
                <collision name="c"><geometry><box><size>{sx} {sy} 2.0</size></box></geometry></collision>
              </link>
            </model>'''))

    tanks = []
    for i, (x, y, r, h) in enumerate([(3.0, 3.0, 0.9, 1.5), (-3.0, 3.0, 0.9, 1.5)]):
        tanks.append(textwrap.dedent(f'''\
            <model name="tank_{i}">
              <static>true</static>
              <link name="link">
                <pose>{x} {y} {h/2} 0 0 0</pose>
                <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
                  <material><ambient>0.3 0.4 0.5 1</ambient><diffuse>0.3 0.4 0.5 1</diffuse></material>
                </visual>
                <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
              </link>
            </model>'''))

    # Place each marker as an <include> of its model.
    marker_includes = []
    for marker_id, x, y, yaw_deg, _label in MARKERS:
        # Marker face needs to be ~robot-camera height (camera on waffle_pi ~0.13 m, but
        # raise to 0.5 so it's clearly visible from ~1 m away).
        z = 0.5
        yaw = yaw_deg * 3.14159265 / 180.0
        marker_includes.append(textwrap.dedent(f'''\
            <include>
              <uri>model://aruco_marker_{marker_id}</uri>
              <name>aruco_marker_{marker_id}</name>
              <pose>{x} {y} {z} 0 0 {yaw:.4f}</pose>
            </include>'''))

    world = textwrap.dedent('''\
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <world name="inspection_world">
            <physics type="ode">
              <max_step_size>0.001</max_step_size>
              <real_time_factor>1</real_time_factor>
              <real_time_update_rate>1000</real_time_update_rate>
            </physics>

            <include><uri>model://sun</uri></include>
            <include><uri>model://ground_plane</uri></include>

        ''') + '\n'.join(walls) + '\n' + '\n'.join(tanks) + '\n' + '\n'.join(marker_includes) + textwrap.dedent('''

          </world>
        </sdf>
        ''')

    write(os.path.join(pkg_root, 'worlds', 'inspection.world'), world)


def main():
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    for marker_id, _x, _y, _yaw, label in MARKERS:
        generate_marker_model(pkg_root, marker_id, label)
    generate_world(pkg_root)
    print(f'Generated {len(MARKERS)} marker models and inspection.world under {pkg_root}')


if __name__ == '__main__':
    sys.exit(main())
