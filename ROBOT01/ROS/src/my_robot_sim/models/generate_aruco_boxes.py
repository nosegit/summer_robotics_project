#!/usr/bin/env python3
"""Generate Gazebo box models carrying OpenCV DICT_4X4_250 markers 0-19."""

from pathlib import Path

from PIL import Image, ImageDraw


# First rotation bytes from OpenCV's DICT_4X4_1000 table. DICT_4X4_250 is
# the first 250 entries of that table, so these are DICT_4X4_250 IDs 0-19.
MARKER_BYTES = (
    (181, 50), (15, 154), (51, 45), (153, 70), (84, 158),
    (121, 205), (158, 46), (196, 242), (254, 218), (207, 86),
    (249, 145), (17, 167), (14, 183), (42, 15), (36, 177),
    (38, 62), (70, 101), (102, 0), (108, 94), (118, 175),
)

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="aruco_box_{marker_id:02d}">
    <static>false</static>
    <link name="box_link">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.000333333</ixx>
          <iyy>0.000333333</iyy>
          <izz>0.000333333</izz>
        </inertia>
      </inertial>
      <collision name="box_collision">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
      </collision>
      <visual name="cardboard_box">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <material>
          <ambient>0.55 0.32 0.14 1</ambient>
          <diffuse>0.65 0.38 0.16 1</diffuse>
        </material>
      </visual>
      <!-- Six labels sit just outside the collision geometry to avoid z-fighting. -->
      <visual name="aruco_label_top">
        <pose>0 0 0.0501 0 0 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.09 0.09</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/aruco_4x4_250_id_{marker_id}.png</albedo_map>
              <roughness>1.0</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="aruco_label_bottom">
        <pose>0 0 -0.0501 3.141593 0 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.09 0.09</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>materials/textures/aruco_4x4_250_id_{marker_id}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
      <visual name="aruco_label_positive_x">
        <pose>0.0501 0 0 0 1.570796 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.09 0.09</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>materials/textures/aruco_4x4_250_id_{marker_id}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
      <visual name="aruco_label_negative_x">
        <pose>-0.0501 0 0 0 -1.570796 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.09 0.09</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>materials/textures/aruco_4x4_250_id_{marker_id}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
      <visual name="aruco_label_positive_y">
        <pose>0 0.0501 0 -1.570796 0 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.09 0.09</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>materials/textures/aruco_4x4_250_id_{marker_id}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
      <visual name="aruco_label_negative_y">
        <pose>0 -0.0501 0 1.570796 0 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.09 0.09</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>materials/textures/aruco_4x4_250_id_{marker_id}.png</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>ArUco Box {marker_id}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>10 cm cube with DICT_4X4_250 marker ID {marker_id} on top.</description>
</model>
"""


def marker_bits(marker_bytes):
    """Return the 16 dictionary bits in row-major order."""
    value = (marker_bytes[0] << 8) | marker_bytes[1]
    return tuple((value >> bit) & 1 for bit in range(15, -1, -1))


def write_marker(path, marker_bytes):
    cell_size = 100
    image = Image.new("L", (6 * cell_size, 6 * cell_size), 0)
    draw = ImageDraw.Draw(image)
    for index, bit in enumerate(marker_bits(marker_bytes)):
        if not bit:
            continue
        row, column = divmod(index, 4)
        x0 = (column + 1) * cell_size
        y0 = (row + 1) * cell_size
        draw.rectangle(
            (x0, y0, x0 + cell_size - 1, y0 + cell_size - 1),
            fill=255,
        )
    image.save(path, optimize=True)


def main():
    models_dir = Path(__file__).resolve().parent
    for marker_id, marker_bytes in enumerate(MARKER_BYTES):
        model_dir = models_dir / f"aruco_box_{marker_id:02d}"
        texture_dir = model_dir / "materials" / "textures"
        texture_dir.mkdir(parents=True, exist_ok=True)
        (model_dir / "model.sdf").write_text(
            MODEL_SDF.format(marker_id=marker_id), encoding="utf-8"
        )
        (model_dir / "model.config").write_text(
            MODEL_CONFIG.format(marker_id=marker_id), encoding="utf-8"
        )
        write_marker(
            texture_dir / f"aruco_4x4_250_id_{marker_id}.png", marker_bytes
        )


if __name__ == "__main__":
    main()
