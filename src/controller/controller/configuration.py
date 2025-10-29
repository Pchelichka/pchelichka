import requests
from enum import Enum, auto
from typing import List
from .commands import Position, Command, Arm, Disarm, Delay, Takeoff, PTP, Sequence

class ConfigType(Enum):
   CODE = auto()
   SERVER = auto()

TAKEOFF_DELAY_SECS = 3

class Config:
    def __init__(self, type = ConfigType.CODE):
        if type == ConfigType.CODE:
            self.manual = {
                "roll": True,
                "pitch": True,
                "yaw": True,
                "throttle": True,
                "mode": True
            }
            self.roll_ff = 1500
            self.roll_pid_constants = [[1, 0, 0],   # pos
                                   [1.5, 1.2, 0], # vel 
                                   [25, 0, 0]]
            self.pitch_ff = 1500
            self.pitch_pid_constants = [[1, 0, 0],   # pos 
                                    [1.5, 1.2, 0], # vel
                                    [25, 0, 0]]
            self.throttle_manual = False
            self.throttle_ff = 1580
            self.throttle_pid_constants = [[1.5, 0.2, 0],  # pos
                                       [4, 0, 0], # vel 
                                       [14, 28, 0]]
            self.yaw_ff = 1500
            self.yaw_pid_constants = [[40, 0, 2]]
            self.sequence = Sequence([Arm(), Delay(TAKEOFF_DELAY_SECS), Takeoff(), PTP(Position(0, 0, 1, 0), 0.15)])
        elif type == ConfigType.SERVER:
            raw_data = requests.get(r'http://localhost:8000/config').json()
            for k, v in raw_data.items():
                setattr(self, k, v)
            self.sequence = Config.parse_code(self.code)
        else:
            raise ValueError("Invalid config type!")

    @staticmethod
    def parse_code(code: str) -> Sequence:
        lines = code.strip().splitlines()
        commands: List[Command] = []
        variables: dict[str, float | Position] = {}

        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue  # Skip blank lines and comments

            # Variable assignment
            if '=' in line:
                var_name, value_str = map(str.strip, line.split('=', maxsplit=1))
                parts = [v.strip() for v in value_str.split(',')]

                try:
                    if len(parts) == 1:
                        # Single float value
                        variables[var_name] = float(parts[0])
                    elif len(parts) == 4:
                        # Full position
                        coords = [float(p) for p in parts]
                        variables[var_name] = Position(*coords)
                    else:
                        raise ValueError(f"Unsupported variable format: {line}")
                except Exception as e:
                    raise ValueError(f"Invalid variable definition: {line}") from e
                continue  # Done with variable assignment

            # Command parsing
            parts = line.split(maxsplit=1)
            cmd = parts[0].lower()

            if cmd == 'arm':
                commands.append(Arm())
            elif cmd == 'disarm':
                commands.append(Disarm())
            elif cmd == 'takeoff':
                commands.append(Takeoff())
            elif cmd == 'dly':
                if len(parts) != 2:
                    raise ValueError(f"Invalid Delay syntax: {line}")
                arg = parts[1].strip()
                if arg in variables:
                    value = variables[arg]
                    if not isinstance(value, float):
                        raise ValueError(f"Dly expects a float variable, got: {arg}")
                    commands.append(Delay(value))
                else:
                    try:
                        commands.append(Delay(float(arg)))
                    except ValueError:
                        raise ValueError(f"Invalid Dly argument: {arg}")
            elif cmd == 'mov':
                if len(parts) != 2:
                    raise ValueError(f"Invalid Mov syntax: {line}")
                arg = parts[1].strip()

                if arg in variables:
                    val = variables[arg]
                    if not isinstance(val, Position):
                        raise ValueError(f"Mov expects a Position variable, got: {arg}")
                    commands.append(PTP(val))
                else:
                    # Try to resolve partial values and variable references
                    coord_parts = [v.strip() for v in arg.split(',')]
                    if len(coord_parts) != 4:
                        raise ValueError(f"Mov expects 4 values or a variable with 4: {line}")

                    resolved = []
                    for p in coord_parts:
                        if p in variables:
                            val = variables[p]
                            if not isinstance(val, float):
                                raise ValueError(f"Expected float for coordinate, got: {p}")
                            resolved.append(val)
                        else:
                            try:
                                resolved.append(float(p))
                            except ValueError:
                                raise ValueError(f"Invalid coordinate or unknown variable: {p}")

                    commands.append(PTP(Position(*resolved)))
            else:
                raise ValueError(f"Unknown command: {cmd}")

        return Sequence(commands)

                
