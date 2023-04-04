# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

import rclpy
import asyncio
from rclpy.node import Node 
from modules.utils.func_utils import load_configuration
from myrobotics_protocol.srv import WebRTCNegociation
from modules.camera.WebRTCCamera import WebRTCCamera
import concurrent.futures
from rclpy.executors import MultiThreadedExecutor
import threading

class AsyncioThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.loop = asyncio.new_event_loop()

    def run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

def asyncio_loop(func):
    def wrapper(*args, **kwargs):
        return asyncio.new_event_loop().run_until_complete(
            func(*args, **kwargs))
    return wrapper

class WebRTCCameraNode(Node):
    def __init__(self):
        super().__init__('webrtccamera')
        config = load_configuration("camera_raspberry.json", True)
        self.camera = WebRTCCamera(config)
        self.asyncio_thread = AsyncioThread()
        self.asyncio_thread.start()
        self.negociate_service = self.create_service(WebRTCNegociation, 'negociate', self.negociate)
        print("init")

    # @asyncio_loop
    def negociate(self, request, response):
        print("nego", flush=True)
        # r = await self.camera.start_session(request.sdp, request.type)
        future = asyncio.run_coroutine_threadsafe(self.camera.start_session(request.sdp, request.type), self.asyncio_thread.loop)
        r = future.result()
        response.sdp = r["sdp"]
        response.type = r["type"]
        print("negok")
        return response


def main(args=None):
    rclpy.init(args=args)
    camera = WebRTCCameraNode()    

    executor = MultiThreadedExecutor()
    executor.add_node(camera)
    print("camera ready")
    executor.spin()    

    camera.asyncio_thread.loop.stop()
    camera.asyncio_thread.join()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
