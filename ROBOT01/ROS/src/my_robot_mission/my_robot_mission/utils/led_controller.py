#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from gpiozero import LED
except ImportError:
    LED = None


class LedController(Node):
    """Alternate red/green mission LEDs and provide a solid-red state."""

    def __init__(self):
        super().__init__('mission_led_controller')
        self.declare_parameter('red_gpio_pin', 17)
        self.declare_parameter('green_gpio_pin', 27)
        self.declare_parameter('active_high', True)
        self.declare_parameter('blink_period', 0.5)

        red_pin = int(self.get_parameter('red_gpio_pin').value)
        green_pin = int(self.get_parameter('green_gpio_pin').value)
        active_high = bool(self.get_parameter('active_high').value)
        blink_period = float(self.get_parameter('blink_period').value)
        if red_pin < 0 or green_pin < 0 or red_pin == green_pin:
            raise ValueError(
                'red_gpio_pin and green_gpio_pin must be different valid BCM pins'
            )
        if blink_period <= 0.0:
            raise ValueError('blink_period must be greater than zero')
        if LED is None:
            raise RuntimeError(
                'gpiozero is unavailable; install python3-gpiozero'
            )

        self._red = LED(red_pin, active_high=active_high, initial_value=True)
        self._green = LED(
            green_pin, active_high=active_high, initial_value=False
        )
        self._blinking = False
        self._red_phase = True
        self.create_timer(blink_period, self._blink)
        self.create_service(
            Trigger, '~/start_blinking', self._start_blinking
        )
        self.create_service(Trigger, '~/solid_red', self._solid_red)
        self.create_service(Trigger, '~/solid_green', self._solid_green)
        self.create_service(Trigger, '~/off', self._off)
        self.get_logger().info(
            f'Mission LEDs ready: red=BCM {red_pin}, green=BCM {green_pin}'
        )

    def _set_outputs(self, red_on, green_on):
        self._red.value = red_on
        self._green.value = green_on

    def _start_blinking(self, request, response):
        del request
        self._blinking = True
        self._red_phase = True
        self._set_outputs(True, False)
        response.success = True
        response.message = 'alternating red and green'
        return response

    def _solid_red(self, request, response):
        del request
        self._blinking = False
        self._set_outputs(True, False)
        response.success = True
        response.message = 'solid red'
        return response

    def _solid_green(self, request, response):
        del request
        self._blinking = False
        self._set_outputs(False, True)
        response.success = True
        response.message = 'solid green'
        return response

    def _off(self, request, response):
        del request
        self._blinking = False
        self._set_outputs(False, False)
        response.success = True
        response.message = 'off'
        return response

    def _blink(self):
        if not self._blinking:
            return
        self._red_phase = not self._red_phase
        self._set_outputs(self._red_phase, not self._red_phase)

    def destroy_node(self):
        if hasattr(self, '_red'):
            self._set_outputs(False, False)
            self._red.close()
            self._green.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LedController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
