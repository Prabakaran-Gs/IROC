import serial
import pygame
import sys

# Initialize serial connection
ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
ser1 = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

# Initialize pygame
pygame.init()

# Set up the display
screen = pygame.display.set_mode((100, 100))
pygame.display.set_caption('Keyboard Control')

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                ser.write('f'.encode())
            elif event.key == pygame.K_s:
                ser.write('b'.encode())
            elif event.key == pygame.K_a:
                ser.write('l'.encode())
            elif event.key == pygame.K_d:
                ser.write('r'.encode())
            elif event.key == pygame.K_u:
                ser1.write('u'.encode())
            elif event.key == pygame.K_j:
                ser1.write('d'.encode())
            elif event.key == pygame.K_h:
                ser1.write('a'.encode())
            elif event.key == pygame.K_k:
                ser1.write('o'.encode())
                
            
        elif event.type == pygame.KEYUP:
            if event.key in (pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d):
                ser.write('s'.encode())  # Stop command when key is released

    pygame.display.flip()

# Clean up
ser.close()
pygame.quit()
sys.exit()
