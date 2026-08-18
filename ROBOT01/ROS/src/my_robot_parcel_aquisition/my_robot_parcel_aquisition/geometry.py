import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(vector, quaternion):
    conjugate = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (*vector, 0.0)), conjugate
    )
    return rotated[:3]


def transform_detection(detection, transform):
    rotation = transform.transform.rotation
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    position = detection.pose.position
    xyz = rotate_vector((position.x, position.y, position.z), quaternion)
    translation = transform.transform.translation
    center = (
        xyz[0] + translation.x,
        xyz[1] + translation.y,
        xyz[2] + translation.z,
    )
    normal = detection.face_normal
    face_normal = rotate_vector((normal.x, normal.y, normal.z), quaternion)
    return center, face_normal


def desaturate(vx, vy, wz, max_linear, max_angular):
    scale = max(
        abs(vx) / max_linear if max_linear > 0.0 else 0.0,
        abs(vy) / max_linear if max_linear > 0.0 else 0.0,
        abs(wz) / max_angular if max_angular > 0.0 else 0.0,
        1.0,
    )
    return vx / scale, vy / scale, wz / scale
