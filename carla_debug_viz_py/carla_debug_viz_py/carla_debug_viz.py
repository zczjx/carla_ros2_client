#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from threading import Thread
import pygame, numpy


class DebugViz(Node):

    def __init__(self):
        super().__init__('DebugViz')
        self._surface = None
        self.sub = self.create_subscription(Image, '/carla/video_enc/image_h264', self.on_view_image, qos_profile=10)

    def h264_msg_info(self, image_msg):
        self.get_logger().info('image_msg.header.frame_id: [%s]' % image_msg.header.frame_id)
        self.get_logger().info('image_msg.height: [%d]' % image_msg.height)
        self.get_logger().info('image_msg.width: [%d]' % image_msg.width)
        self.get_logger().info('image_msg.encoding: [%s]' % image_msg.encoding)

    def on_view_image(self, image):
        """
        Callback when receiving a camera image
        """
        # self.h264_msg_info(image)
        array = numpy.frombuffer(image.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))

def main(args=None):
    # resolution should be similar to spawned camera with role-name 'view'
    resolution = {"width": 800, "height": 600}

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("CARLA Debug Viz")

    try:
        display = pygame.display.set_mode((resolution['width'], resolution['height']),
                                          pygame.HWSURFACE | pygame.DOUBLEBUF)
        rclpy.init(args=args)
        debug_node = DebugViz()
        print('init debug viz')
        spin_thread = Thread(target=rclpy.spin, args=(debug_node, ))
        spin_thread.start()

        while rclpy.ok():
            if debug_node.render(display):
                return
            # print('pygame.display.flip')
            pygame.display.flip()

    except KeyboardInterrupt:
        debug_node.get_logger().info("User requested shut down.")
    finally:
        debug_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()
        pygame.quit()

if __name__ == '__main__':
    main()
