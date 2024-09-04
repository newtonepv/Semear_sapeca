#!/usr/bin/env python

import rospy
from my_package.srv import VerifyPair, VerifyPairRequest

def request_verify_pair(number):
    rospy.wait_for_service("verify_pair_service")
    try:
        verify_pair_func = rospy.ServiceProxy("verify_pair_service", VerifyPair)
        response = verify_pair_func(number)
        if response.success:
            if response.result != -1:
                rospy.loginfo("par")
            else:
                rospy.loginfo("impar")
        else:
            rospy.loginfo("Erro ao verificar o número")

    except rospy.ServiceException as e:
        rospy.logerr(f"Falha de serviço: {e}")

if __name__ == "__main__":
    rospy.init_node("client_verify_pair")
    numbers = [1, 2, 3, 4]

    for n in numbers:
        request_verify_pair(n)
        rospy.sleep(1)
