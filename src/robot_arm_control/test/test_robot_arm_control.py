# 테스트 파일
import unittest


class TestRobotArmControl(unittest.TestCase):
    def test_import(self):
        """패키지 임포트 테스트"""
        try:
            import robot_arm_control
            self.assertTrue(True)
        except ImportError:
            self.fail("Failed to import robot_arm_control")


if __name__ == '__main__':
    unittest.main()
