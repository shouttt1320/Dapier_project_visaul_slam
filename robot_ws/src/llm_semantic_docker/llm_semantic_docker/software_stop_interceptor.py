#!/usr/bin/env python3
"""
[Software STOP Interceptor] Deterministic Low-Latency Emergency Software Stop
- 사용자의 긴급 정지 키워드("정지", "멈춰", "그만", "stop", "취소")를 정규식으로 <1ms 이내 가로챔
- LLM 지연 시간(1~2초)을 100% 우회하여 즉각 /cmd_vel로 0 속도 발행 및 모든 액션 취소
- 입력 시점부터 cmd_vel 발행 완료 시점까지의 소프트웨어 지연 시간 실측 로깅
"""

import re
import time
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


STOP_KEYWORD_PATTERN = re.compile(
    r"(정지|멈춰|그만|스톱|stop|halt|취소|비상정지|비상|서라|중단|abort)",
    re.IGNORECASE
)


class SoftwareStopInterceptor:
    """정규식 기반 초고속 키워드 검출기 (LLM 호출 전 인터셉트)"""

    @staticmethod
    def is_stop_command(user_text: str) -> bool:
        if not user_text:
            return False
        clean = user_text.strip()
        return bool(STOP_KEYWORD_PATTERN.search(clean))

    @staticmethod
    def inspect(user_text: str) -> Tuple[bool, float]:
        """(is_stop, check_duration_sec) 반환"""
        t0 = time.perf_counter()
        matched = SoftwareStopInterceptor.is_stop_command(user_text)
        duration = time.perf_counter() - t0
        return matched, duration


class SoftwareStopNode(Node):
    """ROS 2 환경에서 직접 cmd_vel=0을 쏘는 비상 정지 퍼블리셔/노드"""

    def __init__(self, node_name="software_stop_node"):
        super().__init__(node_name)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def trigger_software_stop(self) -> float:
        """cmd_vel=0 즉각 발행 및 소프트웨어 반응 지연 실측 (초)"""
        t0 = time.perf_counter()

        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.linear.y = 0.0
        zero_twist.linear.z = 0.0
        zero_twist.angular.x = 0.0
        zero_twist.angular.y = 0.0
        zero_twist.angular.z = 0.0

        # 3회 연속 발행하여 Wi-Fi UDP 패킷 유실 방지
        for _ in range(3):
            self.cmd_pub.publish(zero_twist)

        duration = time.perf_counter() - t0
        self.get_logger().warn(
            f"[DETERMINISTIC SOFTWARE STOP TRIGGERED] cmd_vel=0 published in {duration*1000:.2f} ms"
        )
        return duration
