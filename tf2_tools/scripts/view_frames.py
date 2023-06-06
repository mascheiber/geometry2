#!/usr/bin/env python3
# tf2 view frame & generate_dot code Copyright (c) 2008, Willow Garage, Inc.
# argparser & check_file code Copyright (c) 2023, Martin Scheiber, 
#    Control of Networked Systems Group, University of Klagenfurt, Austria
# positive_float code Copyright (c) 2018, Lucas Walter
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# authors: Wim Meeussen, Martin Scheiber

import argparse
import sys
import os
import rospy
import tf2_py as tf2
import yaml
import subprocess
from tf2_msgs.srv import FrameGraph
import tf2_ros

def main():
    rospy.init_node('view_frames')

    # parse arguments
    other_args = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--duration",
                        help="duration for listening to tf data, must be > 0.0",
                        default=5.0,
                        type=positive_float)
    parser.add_argument("-f", "--file",
                        help="file to store the tf graph",
                        default="frames",
                        type=str)

    args = parser.parse_args(other_args[1:]) # Remove first arg
    file = check_file(args.file)
    
    # listen to tf for 5 seconds
    rospy.loginfo('Listening to tf data during %.1f seconds...' % args.duration)
    rospy.sleep(0.00001)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(args.duration)

    # check file
    rospy.loginfo('Generating graph in %s.pdf file...' % file)
    rospy.wait_for_service('~tf2_frames')
    srv = rospy.ServiceProxy('~tf2_frames', FrameGraph)
    data = yaml.safe_load(srv().frame_yaml)
    with open(file + '.gv', 'w') as f:
        f.write(generate_dot(data))
    cmd = 'dot -Tpdf ' + file + '.gv -o ' + file + '.pdf'
    subprocess.Popen(cmd.split(' ')).communicate()

def generate_dot(data):
    if len(data) == 0:
        return 'digraph G { "No tf data received" }'

    dot = 'digraph G {\n'
    for el in data: 
        map = data[el]
        dot += '"'+map['parent']+'" -> "'+str(el)+'"'
        dot += '[label=" '
        dot += 'Broadcaster: '+map['broadcaster']+'\\n'
        dot += 'Average rate: '+str(map['rate'])+'\\n'
        dot += 'Buffer length: '+str(map['buffer_length'])+'\\n' 
        dot += 'Most recent transform: '+str(map['most_recent_transform'])+'\\n'
        dot += 'Oldest transform: '+str(map['oldest_transform'])+'\\n'
        dot += '"];\n'
        if not map['parent'] in data:
            root = map['parent']
    dot += 'edge [style=invis];\n'
    dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
    dot += '"Recorded at time: '+str(rospy.Time.now().to_sec())+'"[ shape=plaintext ] ;\n'
    dot += '}->"'+root+'";\n}'
    return dot

def check_file(file):
    # check if file ends with '.pdf'
    if file.endswith('.pdf'):
        file = file[:-4] # remove format, will be added seperatly
    
    # check if path to file exists
    fparts = file.split('/')
    path = '/'.join(fparts[:-1])
    if not os.path.exists(path):
        print('%s does not exist, creating it' % path)
        os.makedirs(path)
    
    return file

def positive_float(x):
    """temporaray import from https://github.com/ros/geometry2/blob/bd06488f/tf2_tools/scripts/echo.py#L201-205 for internal use only"""
    x = float(x)
    if x <= 0.0:
        raise argparse.ArgumentTypeError("{} must be > 0.0".format(x))
    return x

if __name__ == '__main__':
    main()
