import zerorpc
from polymetis import GripperInterface


class FrankaGripperInterface:
    def __init__(self):
        self.gripper = GripperInterface()
        self._max_width = 0.08

    def get_gripper_width(self):
        state = self.gripper.get_state()
        return state.width

    def goto_gripper(self, width, speed, force):
        self.gripper.goto(width=width, speed=speed, force=force)

    def open_gripper(self):
        self.gripper.goto(width=self._max_width, speed=0.1, force=20.0)

    def close_gripper(self):
        self.gripper.goto(width=0.0, speed=0.1, force=20.0)

    def grasp(self, width, speed, force):
        return self.gripper.grasp(width=width, speed=speed, force=force)


s = zerorpc.Server(FrankaGripperInterface())
s.bind("tcp://0.0.0.0:4243")
s.run()
