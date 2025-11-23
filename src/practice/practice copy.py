import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

# import pygame


TIMEOUT_DURATION = 20

TARGET_POSE = {
    "x" : 0.2,
    "y" : 0.2,
    "z" : 0.205,
    "theta_x" : 180,
    "theta_y" : 0, 
    "theta_z" : 90
}

def connect_to_kinova(router):
    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)
    
    return base, base_cyclic

def check_for_end_or_abort(e):
    def check(notification, e=e):
        print(" EVNET :", Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
            e.set()
        
    return check

def example_move_to_home_base(base):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    print("Moving the arm to a safe position")
    
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle
            
    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False
    
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    
    base.Unsubscribe(notification_handle)
    
    if finished:
        print("Safe Position Reached!")
        
    else:
        print("Timeout on action notification wait")
        
    return finished

def move_to_pose(base, base_cyclic, target_pose):
    action = Base_pb2.Action()
    action.name = "MoveToPose"
    
    pose = action.reach_pose.target_pose
    pose.x = target_pose["x"]
    pose.y = target_pose["y"]
    pose.z = target_pose["z"]
    pose.theta_x = target_pose["theta_x"]
    pose.theta_y = target_pose["theta_y"]
    pose.theta_z = target_pose["theta_z"]
    
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteAction(action)
    finished = e.wait(TIMEOUT_DURATION)
    
    base.Unsubscribe(notification_handle)
    
    if not finished:
        print("Action time out!")
        return False
        
    return finished


def main():
    import utilities
    
    args = utilities.parseConnectionArguments()
    
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base, base_cyclic = connect_to_kinova(router)
        
        success = example_move_to_home_base(base)
        time.sleep(1.0)
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.5)
        
        for i in range(10):
            TARGET_POSE["z"] += 0.01
            move_to_pose(base, base_cyclic, TARGET_POSE)
            
        for i in range(10):
            TARGET_POSE["z"] -= 0.01
            move_to_pose(base, base_cyclic, TARGET_POSE)
            
        for i in range(10):
            TARGET_POSE["x"] += 0.01
            move_to_pose(base, base_cyclic, TARGET_POSE)
            
        for i in range(10):
            TARGET_POSE["x"] -= 0.01
            move_to_pose(base, base_cyclic, TARGET_POSE)
            
if __name__ == "__main__":
    main()