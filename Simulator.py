import pygame
from numpy import *
# hand = end affector
# wrist = last actuator
# elbow = middle actuator

#theta 1 = shoulder
#theta 2 = elbow
#theta 3 = wrist
from pygame.locals import *
pygame.init()
screen = pygame.display.set_mode((640, 480))
clock = pygame.time.Clock()
def main():
    while True:
        #detect when close event
        for event in pygame.event.get():
            if event.type == QUIT:
               pygame.quit()
               return
        screenCntr = (320, 240)
        mX, mY = pygame.mouse.get_pos()

        #physical sizes
        a1= 300/2 #30cm
        a2 = 180/2 #18cm
        a3 = 150/2 #15cm

        # Desired Position of End effector
        px = -(320 - mX)
        py = 240 - mY

        #angle of end affector
        phi = deg2rad(180)

        # Equations for Inverse kinematics

        #get wrist position from hand 
        wx = px - a3*cos(phi)
        wy = py - a3*sin(phi)


        delta = wx**2 + wy**2
        c2 = (delta -a1**2 -a2**2)/(2*a1*a2)
        s2 = sqrt(1-c2**2)  # elbow down
        theta_2 = arctan2(s2, c2)

        s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
        c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
        theta_1 = arctan2(s1,c1)
        theta_3 = phi-theta_1-theta_2

        Theta1 = theta_1
        Theta2 = theta_2
        Theta3 = theta_3
        print("theta1:{:.4f} theta2:{:.4f} theta3:{:.4f} X:{} Y:{}".format(rad2deg(Theta1), (180 - rad2deg(Theta2)), rad2deg(Theta3), px, py))
        screen.fill((0,0,0))
        elbow = rcord(screenCntr, Theta1, a1)
        wrist = rcord(elbow, Theta1 + Theta2, a2)
        hand = rcord(wrist, Theta1 + Theta2 + Theta3, a3)
        pygame.draw.line(screen, (255,0,0), screenCntr, elbow)
        pygame.draw.line(screen, (0,255,0), elbow, wrist)
        pygame.draw.line(screen, (0,0,255), wrist, hand)
        pygame.display.flip()
        clock.tick(60)
def rcord(start, ang, length):
    x, y = start
    return (x + (length*cos(ang)), y - (length*sin(ang)))

# Execute game:
main()