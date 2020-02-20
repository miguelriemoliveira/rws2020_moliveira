#!/usr/bin/env python
import rospy
from rws2020_msgs.msg import MakeAPlay
from std_msgs.msg import String


class Player:

    def __init__(self, player_name):
        self.player_name = player_name

        red_team = rospy.get_param('/red_team')
        green_team = rospy.get_param('/green_team')
        blue_team = rospy.get_param('/blue_team')

        if self.player_name in red_team:
            self.my_team, self.prey_team, self.hunter_team = 'red', 'green', 'blue'
            self.my_players, self.preys, self.hunters = red_team, green_team, blue_team

        elif self.player_name in green_team:
            self.my_team, self.prey_team, self.hunter_team = 'green', 'blue', 'red'
            self.my_players, self.preys, self.hunters = green_team, blue_team, red_team

        elif self.player_name in blue_team:
            self.my_team, self.prey_team, self.hunter_team = 'blue', 'red', 'green'
            self.my_players, self.preys, self.hunters = blue_team, red_team, green_team

        else:
            rospy.logerr('My name is not in any team. I want to play!')
            exit(0)

        rospy.logwarn(
            'I am ' + self.player_name + ' and I am from this team ' + self.my_team + '. ' + self.prey_team + ' players are all going die!')
        rospy.loginfo('I am afraid of ' + str(self.hunters))

        # Subscribe make a play msg
        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallBack)

    def makeAPlayCallBack(self, msg):
        self.max_vel = msg.turtle
        print('Received message make a play ... my max velocity is ' + str(self.max_vel))

def callback(msg):
    print("Recevied a message containing string " + msg.data)


def main():
    print("Hello player node!")

    rospy.init_node('moliveira', anonymous=False)
    player = Player('moliveira')

    # rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
