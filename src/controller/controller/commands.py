# import rclpy
import time
from abc import ABC, abstractmethod
from typing import List, Optional, Protocol

class Position:
    def __init__(self, x, y, z, theta):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

    def as_array(self) -> List[float]:
        return [self.x, self.y, self.z, self.theta]

class ControllerInterface(Protocol):
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def set_setpoint(self, position: Position) -> None: ...
    def get_current_position(self) -> Position: ...
    def get_logger(self): ...


class Command(ABC):
    @abstractmethod
    def start(self, controller: ControllerInterface) -> None:
        ...

    @abstractmethod
    def update(self, controller: ControllerInterface) -> bool:
        """Returns True when the command is complete."""
        ...


class Arm(Command):
    def start(self, controller: ControllerInterface) -> None:
        controller.get_logger().info('Executing arm command.')
        controller.arm()

    def update(self, _controller: ControllerInterface) -> bool:
        return True   # Instant completion

class Disarm(Command):
    def start(self, controller: ControllerInterface) -> None:
        controller.get_logger().info('Executing disarm command.')
        controller.disarm()

    def update(self, _controller: ControllerInterface) -> bool:
        return True  # Instant completion

class Delay(Command):
    def __init__(self, delay_secs: float = 1) -> None:
        self.delay = delay_secs

    def start(self, controller: ControllerInterface) -> None:
        controller.get_logger().info(f'Delaying {self.delay}')
        self.start_time = time.time()

    def update(self, controller: ControllerInterface) -> bool:
        if time.time() - self.start_time > self.delay:
            controller.get_logger().info(f'Delay {self.delay} seconds has elapsed')
            return True # Completed only after delay has elapsed
        return False

class Takeoff(Command):
    def __init__(self, delay_secs: float = 0.5) -> None:
        self.delay = delay_secs

    def start(self, controller: ControllerInterface) -> None:
        controller.get_logger().info(f'Taking off for {self.delay}')
        controller.start_takeoff()
        self.start_time = time.time()

    def update(self, controller: ControllerInterface) -> bool:
        if time.time() - self.start_time > self.delay:
            controller.get_logger().info(f'Takeoff took {self.delay} seconds')
            controller.end_takeoff()
            return True # Completed only after delay has elapsed
        return False


class PTP(Command):
    def __init__(self, target_position: Position, tolerance: float = 0.05) -> None:
        self.target_position = target_position
        self.tolerance = tolerance

    def start(self, controller: ControllerInterface) -> None:
        controller.get_logger().info(f'Moving to {self.target_position.as_array()}')
        controller.set_setpoint(self.target_position)

    def update(self, controller: ControllerInterface) -> bool:
        current_position = controller.get_current_position()
        error: List[float] = [(abs(c - t) if c is not None else float('inf')) for c, t in zip(current_position.as_array(), self.target_position.as_array())]
        if all(e <= self.tolerance for e in error):
            controller.get_logger().info(f'Reached {self.target_position.as_array()} with error {error}')
            return True
        return False


class Sequence:
    def __init__(self, commands: List[Command]) -> None:
        self.commands: List[Command] = commands
        self.current_index: int = 0
        self.active_command: Optional[Command] = None

    def update(self, controller: ControllerInterface) -> bool:
        if self.current_index >= len(self.commands):
            return True  # Sequence complete

        if self.active_command is None:
            self.active_command = self.commands[self.current_index]
            self.active_command.start(controller)

        if self.active_command.update(controller):
            self.current_index += 1
            self.active_command = None

        return False  # Sequence still in progress
