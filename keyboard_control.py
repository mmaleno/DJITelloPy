from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time

# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 60


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        #self.screen = pygame.display.set_mode([960, 720])
        self.screen = pygame.display.set_mode([100, 100])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False
        self.send_custom_command = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 50)

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break

            # self.screen.fill([0, 0, 0])
            # frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
            # frame = np.rot90(frame)
            # frame = np.flipud(frame)
            # frame = pygame.surfarray.make_surface(frame)
            # self.screen.blit(frame, (0, 0))
            # pygame.display.update()

            #time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False
        elif key == pygame.K_c:  # do custom flight path
            self.send_rc_control = False
            self.send_custom_command = True

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)
        if self.send_custom_command:
            self.send_rc_control = False
            self.tello.takeoff()
            
            # Small-scale linear trial
            #self.tello.move_forward(200)
            #self.tello.move_back(200)

            # Small-scale rectangle trial
            #self.tello.go_xyz_speed(100, 0, 0, 20)
            #time.sleep(7)
            #self.tello.go_xyz_speed(0, -150, 0, 20)
            #time.sleep(10)
            #self.tello.go_xyz_speed(-100, 0, 0, 20)
            #time.sleep(7)
            #self.tello.go_xyz_speed(0, 150, 0, 20)
            #time.sleep(10)

            # Small-scale complex trial
            #self.tello.go_xyz_speed(50, 0, 0, 20)
            #time.sleep(4)
            #self.tello.go_xyz_speed(0, -50, 0, 20)
            #time.sleep(4)
            #self.tello.go_xyz_speed(75, 0, 0, 20)
            #time.sleep(5)
            #self.tello.go_xyz_speed(0, 25, 0, 20)
            #time.sleep(3)
            #self.tello.go_xyz_speed(-125, 25, 0, 20)
            #time.sleep(10)

            # Large-scale linear trial
            #self.tello.go_xyz_speed(500, 0, 0, 80)
            #self.tello.go_xyz_speed(500, 0, 0, 80)
            #self.tello.go_xyz_speed(200, 0, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(-200, 0, 0, 80)
            
            # Large-scale rectangle trial
            #self.tello.go_xyz_speed(500, 0, 0, 80)
            #self.tello.go_xyz_speed(500, 0, 0, 80)
            #self.tello.go_xyz_speed(0, -500, 0, 80)
            #self.tello.go_xyz_speed(0, -500, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(0, 500, 0, 80)
            #self.tello.go_xyz_speed(0, 500, 0, 80)

            # Large-scale complex trial 1
            #self.tello.go_xyz_speed(0, -250, 0, 80)
            #self.tello.go_xyz_speed(250, 0, 0, 80)
            #self.tello.go_xyz_speed(500, -500, 0, 80)
            #self.tello.go_xyz_speed(500, -500, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(500, 500, 0, 80)
            #self.tello.go_xyz_speed(500, 500, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(-500, 0, 0, 80)
            #self.tello.go_xyz_speed(0, 250, 0, 80)
            #self.tello.go_xyz_speed(-250, 0, 0, 80)

            # Large-scale complex trial 2
            self.tello.go_xyz_speed(0, -500, 0, 80)
            self.tello.rotate_clockwise(15)
            self.tello.go_xyz_speed(500, 0, 0, 80)
            self.tello.rotate_counter_clockwise(30)
            self.tello.go_xyz_speed(500, 0, 0, 80)
            self.tello.rotate_clockwise(30)
            self.tello.go_xyz_speed(-500, 0, 0, 80)
            self.tello.rotate_counter_clockwise(30)
            self.tello.go_xyz_speed(-500, 0, 0, 80)
            self.tello.rotate_clockwise(15)
            self.tello.go_xyz_speed(0, 500, 0, 80)
            
            self.tello.land()            
            self.send_custom_command = False


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
