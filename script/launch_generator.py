import subprocess
import signal
import os
import roslaunch
import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import subprocess
import shlex
import sys
import signal
import psutil
import time

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        print("try to kill child: " + str(process))
        process.send_signal(sig)

class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    __initialized = False
    def __init__(self):
        if Roscore.__initialized:
            raise Exception("You can't create more than 1 instance of Roscore.")
        Roscore.__initialized = True
    def run(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
            self.roscore_pid = self.roscore_process.pid  # pid of the roscore process (which has child processes)
        except OSError as e:
            sys.stderr.write('roscore could not be run')
            raise e
    def terminate(self):
        print("try to kill child pids of roscore pid: " + str(self.roscore_pid))
        kill_child_processes(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()  # important to prevent from zombie process
        Roscore.__initialized = False



process_generate_running = True
exploration_status = False

class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, process_name, exit_code):
        global process_generate_running
        process_generate_running = False
        #rospy.logwarn("%s died with code %s", name, exit_code)
        print('{} died with code {}'.format(process_name, exit_code) )

def init_launch(launchfile):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile]#,
        #process_listeners=[process_listener],
    )
    return launch

def isDoneCallback(isdoneflag):
    global exploration_status
    exploration_status = isdoneflag
    print("[python] exploration done: {}".format(exploration_status))

def main(argv):
    
    global exploration_status
    num_rounds = int(argv[1])
    roscore = Roscore()

    for runidx in range(1,num_rounds+1):
        
        start_time = time.time()
        elapsed_time = time.time() - start_time

        print("roscore begins \n")
        print("exploration status {}".format(exploration_status))
        roscore.run()
    
        #for idx in range(1,3):
        rospy.init_node("gazebo_launcher")
        rospy.Subscriber("/exploration_is_done", Bool, isDoneCallback)

        gazebo_launch_file = "/home/hankm/catkin_ws/src/aws_robotics/aws-robomaker-small-house-world/launch/small_house.launch"
        autoexplorer_launch_file = "/home/hankm/catkin_ws/src/autoexplorer/launch/autoexplorer.launch"
        
        rospy.sleep(10)
    
        gz_launch = init_launch(gazebo_launch_file )
        gz_launch.start()
        rospy.sleep(10.0)
        print('{}th gz has been started \n'.format(runidx))
                
        ae_launch = init_launch(autoexplorer_launch_file)
        ae_launch.start()
        print('{}th ae has been started \n'.format(runidx))
        
        global exploration_status
        while( exploration_status == False and elapsed_time < 600 ):
            #rospy.wait_for_message("exploration_is_done", Bool, timeout=None)
            rospy.sleep(.5)
            elapsed_time = time.time() - start_time

    #    while process_generate_running:
    #            rospy.sleep(1)

    #    while True:
    #        print('process_generate_running: {}'.format(process_generate_running))

        aesd = ae_launch.shutdown()
        #exploration_status = False
        print('ae is dead... waiting for 10 secs \n')
        rospy.sleep(10)

#        rospy.wait_for_service('/gazebo/reset_world')
#        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
#        reset_world()
        
        # kill gazebo
        os.system("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient")
        print('kill gazebo... waiting for 10 sec')
        rospy.sleep(10)
        roscore.terminate()
        rospy.sleep(2)
        
        exploration_status = False

#    gzsd = gz_launch.shutdown()
#    print('gz is dead \n')

    

if __name__ == '__main__':
    main(sys.argv)

