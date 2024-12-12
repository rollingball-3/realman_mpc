import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
import random
import threading
import time

class TurtleServiceTester(Node):
    def __init__(self):
        super().__init__('turtle_service_tester')
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.lock = threading.Lock()  # 添加互斥锁
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务可用...')

    def call_service(self, thread_id):
        request = TeleportAbsolute.Request()
        request.x = random.uniform(0, 11)
        request.y = random.uniform(0, 11)
        request.theta = random.uniform(0, 6.28)

        print(f"线程 {thread_id} 开始请求，位置: x={request.x:.2f}, y={request.y:.2f}, theta={request.theta:.2f}")
        
        start_time = time.perf_counter()
        future = self.client.call_async(request)
        
        # 使用互斥锁确保同一时间只有一个线程在执行 spin
        with self.lock:
            rclpy.spin_until_future_complete(self, future)
            
        end_time = time.perf_counter()
        
        elapsed_time = end_time - start_time
        print(f"线程 {thread_id} 完成请求，耗时: {elapsed_time:.3f} 秒")
        
        if future.result() is not None:
            print(f"线程 {thread_id} 请求成功")
        else:
            print(f"线程 {thread_id} 请求失败")

def main():
    rclpy.init()
    tester = TurtleServiceTester()
    
    num_threads = 5
    try:
        while True:
            print("\n开始新一轮并发测试")
            threads = []
            
            for i in range(num_threads):
                thread = threading.Thread(target=tester.call_service, args=(i,))
                threads.append(thread)
                thread.start()

            for thread in threads:
                thread.join()

            print("本轮测试完成")
            time.sleep(1)

    except KeyboardInterrupt:
        print("测试被中断")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
