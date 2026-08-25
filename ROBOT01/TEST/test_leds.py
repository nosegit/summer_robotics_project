#!/usr/bin/env python3
"""Test two Raspberry Pi LEDs without ROS, using BCM GPIO numbering."""

import argparse
import time


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--red-pin', type=int, default=17)
    parser.add_argument('--green-pin', type=int, default=27)
    parser.add_argument(
        '--mode',
        choices=('blink', 'red', 'green', 'off'),
        default='blink',
    )
    parser.add_argument(
        '--period', type=float, default=0.5,
        help='Seconds between LED changes in blink mode.',
    )
    return parser.parse_args()


def set_leds(red, green, red_on, green_on):
    red.value = red_on
    green.value = green_on


def main():
    args = parse_args()
    try:
        from gpiozero import LED
    except ImportError as error:
        raise SystemExit(
            'gpiozero is not installed. Run this inside the rebuilt server '
            'container or install python3-gpiozero.'
        ) from error

    if args.red_pin == args.green_pin:
        raise ValueError('red-pin and green-pin must be different')
    if args.period <= 0.0:
        raise ValueError('period must be greater than zero')

    red = LED(args.red_pin, initial_value=False)
    green = LED(args.green_pin, initial_value=False)
    try:
        if args.mode == 'red':
            set_leds(red, green, True, False)
        elif args.mode == 'green':
            set_leds(red, green, False, True)
        elif args.mode == 'off':
            set_leds(red, green, False, False)
        else:
            red_phase = True
            while True:
                set_leds(red, green, red_phase, not red_phase)
                print('RED' if red_phase else 'GREEN', flush=True)
                red_phase = not red_phase
                time.sleep(args.period)

        print(f'{args.mode.upper()} mode active; press Ctrl+C to stop')
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\nStopping LED test')
    finally:
        set_leds(red, green, False, False)
        red.close()
        green.close()


if __name__ == '__main__':
    main()
