#!/usr/bin/env python3
"""Stream the Pi CSI camera to the local Docker ROS receiver with low latency.

Picamera2 uses libcamera and the Pi hardware H.264 encoder. RTP packetisation
is performed here instead of through a shell pipe, so closing a receiver cannot
turn into a SIGPIPE in the camera process.
"""

import argparse
import random
import signal
import socket
import struct
import sys
import time

try:
    from libcamera import Transform
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import Output
except ImportError as error:
    raise SystemExit(
        'Picamera2 is required on the Raspberry Pi host. Install it with: '
        'sudo apt install python3-picamera2'
    ) from error


RTP_VERSION = 0x80
RTP_PAYLOAD_TYPE_H264 = 96


def split_annex_b_nals(frame):
    """Yield H.264 NAL units from an Annex-B encoded frame (C-optimized)."""
    # bytes.split runs entirely in C, vastly outperforming a Python while loop
    for part in frame.split(b'\x00\x00\x01'):
        if part == b'\x00':
            continue
        # If the chunk ends with a zero byte, it was likely part of a 4-byte 
        # start code (\x00\x00\x00\x01) for the *next* NAL unit. Strip it.
        if part.endswith(b'\x00'):
            part = part[:-1]
        if part:
            yield part


class RtpH264Output(Output):
    """Picamera2 encoder output that sends each encoded access unit as RTP."""

    def __init__(self, destination, port, fps, mtu):
        super().__init__()
        if mtu <= 14:
            raise ValueError('mtu must be greater than 14 bytes')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((destination, port))
        self.sequence_number = random.randrange(0, 1 << 16)
        self.timestamp = random.randrange(0, 1 << 32)
        self.timestamp_step = round(90_000 / fps)
        self.max_payload = mtu - 12
        self.recording = False
        self.needs_pacing = False
        self.needs_add_stream = False
        self.closed = False

    def start(self):
        self.recording = True

    def _add_stream(self, *_args, **_kwargs):
        """Accept encoder stream metadata; RTP/H.264 needs no extra setup."""

    def _send_packet(self, payload, marker):
        header = struct.pack(
            '!BBHII',
            RTP_VERSION,
            RTP_PAYLOAD_TYPE_H264 | (0x80 if marker else 0),
            self.sequence_number,
            self.timestamp,
            0,
        )
        try:
            self.socket.send(header + payload)
        except ConnectionRefusedError:
            # The receiver is not currently listening. 
            # Drop the packet silently so the camera keeps running.
            pass
            
        self.sequence_number = (self.sequence_number + 1) & 0xffff

    def _send_nal(self, nal, marker):
        if len(nal) <= self.max_payload:
            self._send_packet(nal, marker)
            return

        nal_header = nal[0]
        # RFC 6184 FU-A: preserve forbidden/NRI bits, replace the type with 28.
        fu_indicator = (nal_header & 0xe0) | 28
        nal_type = nal_header & 0x1f
        chunk_size = self.max_payload - 2
        payload = nal[1:]
        
        for offset in range(0, len(payload), chunk_size):
            chunk = payload[offset:offset + chunk_size]
            start = offset == 0
            end = offset + len(chunk) == len(payload)
            fu_header = nal_type | (0x80 if start else 0) | (0x40 if end else 0)
            self._send_packet(bytes((fu_indicator, fu_header)) + chunk, marker and end)

    def outputframe(self, frame, keyframe=True, timestamp=None, packet=None, audio=False):
        """Picamera2 Output protocol callback for one encoded video frame."""
        del keyframe, timestamp, packet
        if self.closed or not self.recording or audio:
            return

        nals = list(split_annex_b_nals(frame))
        if not nals:
            return
        for index, nal in enumerate(nals):
            self._send_nal(nal, marker=index == len(nals) - 1)
        self.timestamp = (self.timestamp + self.timestamp_step) & 0xffffffff

    def stop(self):
        """Match Picamera2's Output lifecycle hook."""
        self.recording = False
        self.close()

    def close(self):
        if not self.closed:
            self.socket.close()
            self.closed = True


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', nargs='?', default='127.0.0.1')
    parser.add_argument('port', nargs='?', type=int, default=5600)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--bitrate', type=int, default=2_000_000)
    parser.add_argument('--mtu', type=int, default=1200)
    arguments = parser.parse_args()
    if not 1 <= arguments.port <= 65535:
        parser.error('port must be between 1 and 65535')
    if arguments.width <= 0 or arguments.height <= 0:
        parser.error('width and height must be positive')
    if arguments.fps <= 0:
        parser.error('fps must be positive')
    if arguments.bitrate <= 0:
        parser.error('bitrate must be positive')
    return arguments


def main():
    arguments = parse_arguments()
    frame_duration_us = round(1_000_000 / arguments.fps)
    
    camera = Picamera2()
    configuration = camera.create_video_configuration(
        main={'size': (arguments.width, arguments.height), 'format': 'YUV420'},
        controls={'FrameDurationLimits': (frame_duration_us, frame_duration_us)},
        # The CSI camera is physically upside down. Let libcamera's image
        # pipeline rotate it before hardware H.264 encoding.
        transform=Transform(hflip=True, vflip=True),
        buffer_count=2,
        queue=False,
    )
    camera.configure(configuration)
    
    # Ultra low-latency encoder settings
    encoder = H264Encoder(
        bitrate=arguments.bitrate,
        profile='baseline',         # No B-frames, zero delay
        repeat=True,                # Inject SPS/PPS headers with every I-frame
        iperiod=int(arguments.fps)  # Force I-frame every 1s for quick error recovery
    )
    
    output = RtpH264Output(arguments.destination, arguments.port, arguments.fps, arguments.mtu)

    def stop_camera(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_camera)
    signal.signal(signal.SIGTERM, stop_camera)
    
    try:
        camera.start_recording(encoder, output)
        print(
            f'Streaming {arguments.width}x{arguments.height} at {arguments.fps:g} fps '
            f'to H.264/RTP UDP {arguments.destination}:{arguments.port}',
            flush=True,
        )
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop_recording()
        output.close()


if __name__ == '__main__':
    main()
