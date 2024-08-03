import pygame
import serial
import time
def main():
    dev = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()
    FPS = 30
    running = True
    joystick = None
    while running:
        clock.tick(FPS)
        if joystick:
            for i in range(6):
                dev.write(int((joystick.get_axis(i) + 1) * 125).to_bytes(1, 'little', signed=False))
            dev.write(b'\n')
            dev.flush()

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED and not joystick:
                joystick = pygame.joystick.Joystick(event.device_index)
            if event.type == pygame.JOYDEVICEREMOVED and joystick.get_id == event.device_index:
                joystick = None
            if event.type == pygame.QUIT:
                running = False
    dev.close()
if __name__ == '__main__':
    main()