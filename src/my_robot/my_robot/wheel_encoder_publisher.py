"""Compatibility import and entry point for the reusable encoder implementation."""
from my_robot.components import load_implementation
_module = load_implementation('encoder/joint_states/adapter.py')
WheelEncoderPublisher = _module.WheelEncoderPublisher
main = _module.main
if __name__ == '__main__': main()
