#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Project

Author: Liviu Stan and Guillaume Oudet

Master ISI - Sorbonne Universite


Mybot_teleop.py

This code has been made to pass to control de robot manually

"""
# importe les fichier 
import rospy
import click
#IMporte le type de message que l'on veut
from geometry_msgs.msg import Twist #import du type de data Twist

# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}


if __name__ == '__main__':
        try: 
            angular_param = 1
            linear_param = 0.2
            # defini quel type de donné et à quel topic (créer le noeud) envoie le message
            # queu_size est juste un parametre permettant de dire combine de message peuvent s'accumule
            # dans la size
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

            #donne un nom de node au publisher
            # anonymous permet de donner un nom unique au noeud avec un numero
            rospy.init_node('cmd_teleop', anonymous=True) 
            rate = rospy.Rate(10) # 10hz on peut definir un rate si on en a besoin en 
            while not rospy.is_shutdown():
            
                msg = Twist()
                mykey = click.getchar()
                if mykey in keys.keys():
                    char=keys[mykey]
                    
                if char == 'up':    # UP key
                    msg.linear.x = linear_param
                elif char == 'down':  # DOWN key
                    msg.linear.x = -linear_param
                elif char == 'left':  # RIGHT key
                    msg.angular.z = angular_param 
                elif char == 'right': # LEFT
                    msg.angular.z = -angular_param 
                if char == "s":  # s arrete le robot
                    msg.linear.x = 0
                    msg.angular.z = 0
                if char == "quit":  # QUIT
                    break
                
                pub.publish(msg)
                rate.sleep() #va attendre le temps du rate
        
        except rospy.ROSInterruptException:
            pass
        
        
        
     
