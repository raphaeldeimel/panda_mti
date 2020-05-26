#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Copyright 2020 Raphael Deimel
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
@author: Raphael Deimel
@copyright: 2020
@licence: 2-clause BSD licence
"""

import rospy
import rosparam
from tf2_msgs.msg import TFMessage

rospy.init_node('tf_prefixer')

publisher = rospy.Publisher("/tf", TFMessage, queue_size=10)
prefix = rospy.get_param('~tf_prefix')

def callback_tftopic(data):
    global publisher, prefix
    n_transforms = len(data.transforms)
    for i in range(n_transforms):
        data.transforms[i].header.frame_id = prefix + data.transforms[i].header.frame_id
        data.transforms[i].child_frame_id = prefix + data.transforms[i].child_frame_id
    publisher.publish(data)


listener = rospy.Subscriber("tf", TFMessage, callback_tftopic)

rospy.spin()

