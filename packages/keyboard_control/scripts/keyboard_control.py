#!/usr/bin/env python

import roslib; roslib.load_manifest('keyboard_control')
import rospy
from keyboard_control.msg import KeyboardStates

import threading

import pygame

screen = pygame.display.set_mode((100, 100))
clock = pygame.time.Clock()

running = True

keyboardStates = KeyboardStates()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('keyboard_control/KeyboardStates', KeyboardStates)
        rospy.init_node('keyboard_control')

        screen.set_at((0, 0), (255, 0, 0))

        while running and not rospy.is_shutdown():
            # poll events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                    if event.key == pygame.K_LEFT: keyboardStates.bLeftArrow = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_RIGHT: keyboardStates.bRightArrow = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_UP: keyboardStates.bUpArrow = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_DOWN: keyboardStates.bDownArrow = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_0: keyboardStates.b0 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_1: keyboardStates.b1 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_2: keyboardStates.b2 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_3: keyboardStates.b3 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_4: keyboardStates.b4 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_5: keyboardStates.b5 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_6: keyboardStates.b6 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_7: keyboardStates.b7 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_8: keyboardStates.b8 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_9: keyboardStates.b9 = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_a: keyboardStates.bA = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_b: keyboardStates.bB = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_c: keyboardStates.bC = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_d: keyboardStates.bD = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_e: keyboardStates.bE = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_f: keyboardStates.bF = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_g: keyboardStates.bG = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_h: keyboardStates.bH = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_i: keyboardStates.bI = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_j: keyboardStates.bJ = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_k: keyboardStates.bK = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_l: keyboardStates.bL = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_m: keyboardStates.bM = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_n: keyboardStates.bN = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_o: keyboardStates.bO = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_p: keyboardStates.bP = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_q: keyboardStates.bQ = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_r: keyboardStates.bR = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_s: keyboardStates.bS = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_t: keyboardStates.bT = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_u: keyboardStates.bU = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_v: keyboardStates.bV = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_w: keyboardStates.bW = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_x: keyboardStates.bX = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_y: keyboardStates.bY = event.type == pygame.KEYDOWN
                    if event.key == pygame.K_z: keyboardStates.bZ = event.type == pygame.KEYDOWN

            pygame.display.flip()
            clock.tick(100)

            # publish keyboard states
            pub.publish(keyboardStates)
            
            rospy.sleep(0.01)
    except rospy.ROSInterruptException:
        rospy.loginfo('keyboard_control shut down after exception!')
