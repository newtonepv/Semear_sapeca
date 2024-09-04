#!/usr/bin/env python

import rospy
from my_package.srv import VerifyPair, VerifyPairResponse

def verify_pair(request):
    if request.number % 2 == 0:
        rospy.loginfo(f"Número par: {request.number}")
        return VerifyPairResponse(success=True, result=request.number)
    else:
        rospy.loginfo(f"Número ímpar: {request.number}")
        return VerifyPairResponse(success=True, result=-1)

def create_node():
    rospy.init_node("verify_pair_node")
    service = rospy.Service("verify_pair_service", VerifyPair, verify_pair)
    rospy.loginfo("Serviço verify_pair iniciado")
    rospy.spin()

if __name__ == "__main__":
    create_node()
