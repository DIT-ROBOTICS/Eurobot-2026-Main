#!/usr/bin/env python3
"""
Mock Testing Script for Startup Pipeline

This script allows interactive testing of the startup pipeline by:
1. Monitoring which groups have reported ready to startup
2. Allowing you to manually send ready signals for specific groups

Usage:
    python3 src/startup/scripts/mock_groups_test.py

Groups:
    1 = main (bt_engine)
    2 = vision
    3 = navigation
    4 = localization

Commands:
    1-4     = Send ready signal for that group
    s       = Show current status of all groups
    p       = Send plug signal to start game
    q       = Quit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from btcpp_ros2_interfaces.srv import StartUpSrv
import threading
import sys
import select


class MockGroupsTester(Node):
    """Interactive mock groups tester for startup pipeline."""

    # Group definitions
    GROUPS = {
        1: 'main',
        2: 'vision',
        3: 'navigation',
        4: 'localization',
    }

    def __init__(self):
        super().__init__('mock_groups_tester')
        
        # Subscribe to are_you_ready signal
        self.are_you_ready_sub = self.create_subscription(
            Bool,
            '/robot/startup/are_you_ready',
            self.are_you_ready_callback,
            10
        )
        
        # Client to send ready signal
        self.ready_client = self.create_client(
            StartUpSrv,
            '/robot/startup/ready_signal'
        )
        
        # Publisher to simulate plug signal (start trigger)
        self.plug_pub = self.create_publisher(
            Bool,
            '/robot/startup/plug',
            10
        )
        
        # Track which groups have reported ready
        self.groups_reported = {g: False for g in self.GROUPS.keys()}
        self.plug_sent = False
        self.receiving_are_you_ready = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Mock Groups Tester - Interactive Mode')
        self.get_logger().info('=' * 60)
        self.print_help()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
    def print_help(self):
        """Print help message."""
        print('\n--- Commands ---')
        print('1-4  = Send ready signal for group (1=main, 2=vision, 3=nav, 4=loc)')
        print('a    = Send ready for ALL groups')
        print('s    = Show current status')
        print('p    = Send plug signal (start game)')
        print('r    = Reset to initial state')
        print('q    = Quit')
        print('----------------\n')

    def reset_state(self):
        """Reset internal state to allow re-testing."""
        self.groups_reported = {g: False for g in self.GROUPS.keys()}
        self.plug_sent = False
        self.receiving_are_you_ready = False
        self.get_logger().info('State has been reset. Waiting for new signals...')
        self.print_status()

    def are_you_ready_callback(self, msg: Bool):
        """Handle are_you_ready signal from startup."""
        if msg.data and not self.receiving_are_you_ready:
            self.receiving_are_you_ready = True
            self.get_logger().info('>>> Startup is asking: ARE YOU READY? <<<')
            self.print_status()
            
    def print_status(self):
        """Print current status of all groups."""
        print('\n--- Group Status ---')
        for group_id, group_name in self.GROUPS.items():
            status = '✓ READY' if self.groups_reported[group_id] else '✗ waiting'
            print(f'  Group {group_id} ({group_name:12}): {status}')
        
        ready_count = sum(self.groups_reported.values())
        total = len(self.GROUPS)
        print(f'\nReady: {ready_count}/{total}')
        
        if ready_count == total:
            print('\n>>> All groups ready! Press "p" to send plug signal <<<')
        print('--------------------\n')
        
    def send_ready_signal(self, group_id: int):
        """Send ready signal for a specific group."""
        if group_id not in self.GROUPS:
            print(f'Invalid group ID: {group_id}')
            return
            
        group_name = self.GROUPS[group_id]
        
        if not self.ready_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Ready signal service not available')
            return
            
        request = StartUpSrv.Request()
        request.group = group_id
        request.state = 1  # Success
        
        future = self.ready_client.call_async(request)
        future.add_done_callback(
            lambda f: self.ready_callback(f, group_id, group_name)
        )
        
    def ready_callback(self, future, group_id: int, group_name: str):
        """Handle response from ready signal service."""
        try:
            response = future.result()
            if response.success:
                self.groups_reported[group_id] = True
                self.get_logger().info(f'✓ Group {group_id} ({group_name}) reported ready')
                
                # Check if all groups ready
                if all(self.groups_reported.values()):
                    self.get_logger().info('>>> ALL GROUPS READY! Press "p" to start <<<')
            else:
                self.get_logger().error(f'✗ Group {group_id} ({group_name}) failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            
    def send_plug_signal(self):
        """Send plug signal to trigger game start."""
        if self.plug_sent:
            print('Plug signal already sent!')
            return
            
        self.plug_sent = True
        msg = Bool()
        msg.data = True
        
        for _ in range(5):
            self.plug_pub.publish(msg)
            
        self.get_logger().info('>>> PLUG SIGNAL SENT! Game starting... <<<')

    def input_loop(self):
        """Handle user input in a separate thread."""
        while rclpy.ok():
            try:
                cmd = input('> ').strip().lower()
                
                if cmd == 'q':
                    print('Exiting...')
                    rclpy.shutdown()
                    break
                elif cmd == 's':
                    self.print_status()
                elif cmd == 'r':
                    self.reset_state()
                elif cmd == 'p':
                    self.send_plug_signal()
                elif cmd == 'a':
                    print('Sending ready for ALL groups...')
                    for group_id in self.GROUPS.keys():
                        self.send_ready_signal(group_id)
                elif cmd in ['1', '2', '3', '4']:
                    group_id = int(cmd)
                    print(f'Sending ready for group {group_id} ({self.GROUPS[group_id]})...')
                    self.send_ready_signal(group_id)
                elif cmd == 'h' or cmd == '?':
                    self.print_help()
                elif cmd == '':
                    pass  # Ignore empty input
                else:
                    print(f'Unknown command: {cmd}. Type "h" for help.')
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)
    node = MockGroupsTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
