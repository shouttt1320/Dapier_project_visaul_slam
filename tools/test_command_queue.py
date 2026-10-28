#!/usr/bin/env python3
"""
Unit and Integration Test Script for Multi-Step Sequential Task Queue System.
Tests:
1. Compound Natural Language Sentence Splitting & Regex Quick Parsing.
2. All-or-Nothing Sequence Validation & Displacement Safety Clamping.
3. Task Queue FSM Lifecycle & Preemption Logic.
"""

import sys
import os
import unittest
import json
import rclpy

# Add workspace path to sys.path
WS_ROOT = "/home/jjj/Documents/Dapier/Project/robot_ws/src/llm_semantic_docker"
sys.path.insert(0, WS_ROOT)

from llm_semantic_docker.command_validator import CommandValidator
from llm_semantic_docker.llm_agent_node import LLMAgentNode


class TestSequentialCommandQueue(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.node = LLMAgentNode()
        cls.validator = cls.node.validator

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_quick_parse_compound_relative_motion(self):
        """Test splitting compound clause with '가서', '회전하고', '전진해'"""
        clause = "10cm 앞으로 가서 15도 좌회전하고 20cm 전진해"
        seq_dict = self.node._quick_parse_sequence(clause)
        self.assertIsNotNone(seq_dict, "Failed to parse compound relative motion")
        self.assertTrue(seq_dict["is_sequence"])
        self.assertEqual(len(seq_dict["tasks"]), 3)
        
        # 1st task: 10cm forward
        t1 = seq_dict["tasks"][0]
        self.assertEqual(t1["action"], "NAVIGATE")
        self.assertAlmostEqual(t1["target"]["relative_x"], 0.10, places=2)
        
        # 2nd task: 15 deg left turn
        t2 = seq_dict["tasks"][1]
        self.assertEqual(t2["action"], "NAVIGATE")
        self.assertAlmostEqual(t2["target"]["relative_yaw"], 15.0, places=1)
        
        # 3rd task: 20cm forward
        t3 = seq_dict["tasks"][2]
        self.assertEqual(t3["action"], "NAVIGATE")
        self.assertAlmostEqual(t3["target"]["relative_x"], 0.20, places=2)

        # Validate whole sequence through CommandValidator
        val_res = self.validator.validate_sequence(seq_dict)
        self.assertTrue(val_res.is_valid, f"Validation failed: {val_res.error_msg}")
        self.assertEqual(len(val_res.resolved_task["tasks"]), 3)
        resolved_tasks = val_res.resolved_task["tasks"]
        self.assertEqual(resolved_tasks[0]["action"], "NAVIGATE_RELATIVE")
        self.assertEqual(resolved_tasks[1]["action"], "NAVIGATE_RELATIVE")
        self.assertEqual(resolved_tasks[2]["action"], "NAVIGATE_RELATIVE")

    def test_quick_parse_waypoint_and_docking(self):
        """Test splitting 'waypoint1 갔다가 workplace1에 도킹해'"""
        clause = "waypoint1 갔다가 workplace1에 도킹해"
        seq_dict = self.node._quick_parse_sequence(clause)
        self.assertIsNotNone(seq_dict, "Failed to parse waypoint + docking compound sequence")
        self.assertEqual(len(seq_dict["tasks"]), 2)
        
        t1 = seq_dict["tasks"][0]
        self.assertEqual(t1["action"], "NAVIGATE")
        t2 = seq_dict["tasks"][1]
        self.assertEqual(t2["action"], "DOCK")

    def test_quick_parse_complex_separators(self):
        """Test parsing with '그다음', '한 뒤', commas"""
        clause = "5cm 뒤로 가고, 그다음 30도 우회전한 뒤 15cm 앞으로 가"
        seq_dict = self.node._quick_parse_sequence(clause)
        self.assertIsNotNone(seq_dict)
        self.assertEqual(len(seq_dict["tasks"]), 3)
        self.assertAlmostEqual(seq_dict["tasks"][0]["target"]["relative_x"], -0.05, places=2)
        self.assertAlmostEqual(seq_dict["tasks"][1]["target"]["relative_yaw"], -30.0, places=1)
        self.assertAlmostEqual(seq_dict["tasks"][2]["target"]["relative_x"], 0.15, places=2)

    def test_quick_parse_forward_and_workplace(self):
        """Test user command '앞으로 20cm 이동 후 workplace1에 이동해'"""
        clause = "앞으로 20cm 이동 후 workplace1에 이동해"
        seq_dict = self.node._quick_parse_sequence(clause)
        self.assertIsNotNone(seq_dict, "Failed to parse '앞으로 20cm 이동 후 workplace1에 이동해'")
        self.assertEqual(len(seq_dict["tasks"]), 2)
        
        # 1st task: 20cm forward
        t1 = seq_dict["tasks"][0]
        self.assertEqual(t1["action"], "NAVIGATE")
        self.assertEqual(t1["target"]["mode"], "RELATIVE")
        self.assertAlmostEqual(t1["target"]["relative_x"], 0.20, places=2)

        # 2nd task: navigate to waypoint workplace1
        t2 = seq_dict["tasks"][1]
        self.assertEqual(t2["action"], "NAVIGATE")
        self.assertEqual(t2["target"]["mode"], "WAYPOINT")
        self.assertEqual(t2["target"]["semantic_name"], "workplace1")

        # Validation check
        val_res = self.validator.validate_sequence(seq_dict)
        self.assertTrue(val_res.is_valid, f"Validation failed: {val_res.error_msg}")
        self.assertEqual(len(val_res.resolved_task["tasks"]), 2)
        r_tasks = val_res.resolved_task["tasks"]
        self.assertEqual(r_tasks[0]["action"], "NAVIGATE_RELATIVE")
        self.assertEqual(r_tasks[1]["action"], "NAVIGATE")
        self.assertEqual(r_tasks[1]["waypoint_name"], "workplace1")

    def test_validator_all_or_nothing_valid(self):
        """Valid sequence passes and returns normalized sequence dictionary"""
        raw_seq = {
            "mode": "PREEMPT_AND_REPLACE",
            "tasks": [
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.20, "relative_y": 0.0, "relative_yaw": 0.0}},
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.0, "relative_y": 0.0, "relative_yaw": 45.0}},
                {"action": "DOCK", "target": {"target_structure": "marker", "perception_type": "aruco", "direct_visual": True}}
            ]
        }
        res = self.validator.validate_sequence(raw_seq)
        self.assertTrue(res.is_valid, f"Expected valid sequence, got error: {res.error_msg}")
        self.assertEqual(res.resolved_task["mode"], "PREEMPT_AND_REPLACE")
        self.assertEqual(len(res.resolved_task["tasks"]), 3)

    def test_validator_all_or_nothing_invalid_rejects_whole_sequence(self):
        """If one step is invalid (e.g. unknown action), all tasks must be rejected!"""
        raw_seq = {
            "mode": "PREEMPT_AND_REPLACE",
            "tasks": [
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.20, "relative_y": 0.0, "relative_yaw": 0.0}},
                {"action": "INVALID_EXPLODE_ROBOT_ACTION", "target": {}},
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.10, "relative_y": 0.0, "relative_yaw": 0.0}}
            ]
        }
        res = self.validator.validate_sequence(raw_seq)
        self.assertFalse(res.is_valid, "Invalid step should cause entire sequence to be rejected (All-or-Nothing)")
        self.assertEqual(res.error_code, "SUBTASK_INVALID")

    def test_validator_cumulative_displacement_safety_limit(self):
        """Exceeding max cumulative displacement (5.0m) should be rejected"""
        raw_seq = {
            "mode": "PREEMPT_AND_REPLACE",
            "tasks": [
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 3.0, "relative_y": 0.0, "relative_yaw": 0.0}},
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 2.5, "relative_y": 0.0, "relative_yaw": 0.0}}
            ]
        }
        res = self.validator.validate_sequence(raw_seq)
        self.assertFalse(res.is_valid, "Cumulative displacement > 5.0m must be rejected")
        self.assertEqual(res.error_code, "EXCEEDED_SAFETY_LIMIT")


class TestTaskManagerQueueLifecycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def test_task_manager_sequence_reception_and_dispatch(self):
        from llm_semantic_docker.task_manager import TaskManagerNode
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        tm = TaskManagerNode()
        seq_msg = String()
        seq_msg.data = json.dumps({
            "is_sequence": True,
            "sequence_id": "test_seq_001",
            "title": "테스트 시퀀스",
            "mode": "PREEMPT_AND_REPLACE",
            "failure_policy": "STOP_ON_FAILURE",
            "tasks": [
                {"action": "NAVIGATE_RELATIVE", "distance_m": 0.10, "yaw_deg": 0.0, "task_id": "task_1"},
                {"action": "NAVIGATE_RELATIVE", "distance_m": 0.20, "yaw_deg": 0.0, "task_id": "task_2"}
            ]
        })
        tm._on_command_sequence_callback(seq_msg)

        # Task 1 must be dispatched immediately
        self.assertEqual(tm.active_sequence_id, "test_seq_001")
        self.assertEqual(tm.sequence_total_tasks, 2)
        self.assertEqual(tm.sequence_completed_tasks, 0)
        self.assertEqual(len(tm.task_queue), 1)  # 1 remains pending in queue
        self.assertEqual(tm.current_task_id, "task_1")

        # Simulate completion of Task 1 -> auto-dispatch Task 2
        tm._dispatch_next_task()
        self.assertEqual(len(tm.task_queue), 0)
        self.assertEqual(tm.current_task_id, "task_2")
        self.assertEqual(tm.sequence_completed_tasks, 1)

        # Test preemption: new sequence arrives while previous is finishing
        new_seq_msg = String()
        new_seq_msg.data = json.dumps({
            "is_sequence": True,
            "sequence_id": "test_seq_002",
            "title": "긴급 새 시퀀스",
            "mode": "PREEMPT_AND_REPLACE",
            "failure_policy": "STOP_ON_FAILURE",
            "tasks": [
                {"action": "NAVIGATE_RELATIVE", "distance_m": 0.05, "yaw_deg": 0.0, "task_id": "task_new_1"}
            ]
        })
        tm._on_command_sequence_callback(new_seq_msg)
        self.assertEqual(tm.active_sequence_id, "test_seq_002")
        self.assertEqual(tm.sequence_total_tasks, 1)
        self.assertEqual(tm.current_task_id, "task_new_1")

        # Test Cancel All Service
        req = Trigger.Request()
        resp = Trigger.Response()
        tm._handle_cancel_all_srv(req, resp)
        self.assertTrue(resp.success)
        self.assertIsNone(tm.active_sequence_id)
        self.assertEqual(len(tm.task_queue), 0)

        tm.destroy_node()

    def test_consecutive_sequences_clean_reset(self):
        """Verify that after sequence 1 completes, sequence 2 starts with 0/N completed and accurate task descriptions."""
        from llm_semantic_docker.task_manager import TaskManagerNode
        from std_msgs.msg import String

        tm = TaskManagerNode()

        # 1. First Sequence with nested target dict (as produced by LLM agent)
        seq1_msg = String()
        seq1_msg.data = json.dumps({
            "is_sequence": True,
            "sequence_id": "seq_run_1",
            "title": "첫 번째 시퀀스",
            "mode": "PREEMPT_AND_REPLACE",
            "tasks": [
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.10, "relative_yaw": 0.0}, "task_id": "s1_t1"},
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.0, "relative_yaw": -30.0}, "task_id": "s1_t2"}
            ]
        })
        tm._on_command_sequence_callback(seq1_msg)

        # Check description formatting
        self.assertEqual(tm.sequence_completed_tasks, 0)
        desc1 = tm._describe_task(tm.current_task)
        self.assertIn("전진 10.0cm", desc1)

        # Complete Task 1 -> Task 2
        tm._dispatch_next_task()
        self.assertEqual(tm.sequence_completed_tasks, 1)
        desc2 = tm._describe_task(tm.current_task)
        self.assertIn("우회전 30.0°", desc2)

        # Complete Task 2 -> Sequence 1 Finished
        tm._dispatch_next_task()
        self.assertEqual(tm.sequence_completed_tasks, 0)
        self.assertIsNone(tm.current_task)
        self.assertIsNone(tm.current_task_id)

        # 2. Second Sequence arrives after first sequence finished!
        seq2_msg = String()
        seq2_msg.data = json.dumps({
            "is_sequence": True,
            "sequence_id": "seq_run_2",
            "title": "두 번째 시퀀스",
            "mode": "PREEMPT_AND_REPLACE",
            "tasks": [
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.20, "relative_yaw": 0.0}, "task_id": "s2_t1"},
                {"action": "NAVIGATE", "target": {"mode": "RELATIVE", "relative_x": 0.0, "relative_yaw": 15.0}, "task_id": "s2_t2"}
            ]
        })
        tm._on_command_sequence_callback(seq2_msg)

        # MUST start with 0 completed tasks, NOT 1!
        self.assertEqual(tm.sequence_completed_tasks, 0, "Second sequence must start with 0 completed tasks!")
        self.assertEqual(tm.sequence_total_tasks, 2)
        self.assertEqual(tm.current_task_id, "s2_t1")

        # Description of s2_t1 must be 20.0cm forward
        desc_s2_t1 = tm._describe_task(tm.current_task)
        self.assertIn("전진 20.0cm", desc_s2_t1)

        tm.destroy_node()


if __name__ == '__main__':
    unittest.main()

