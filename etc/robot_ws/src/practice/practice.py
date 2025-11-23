import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

import pygame, os


TIMEOUT_DURATION = 20

TARGET_POSE = {
    "x" : 0.3,
    "y" : -0.125,
    "z" : 0.205,
    "theta_x" : 90,
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

def control_gripper(base, base_cyclic, gripper_pose = 0):
    grip_cmd = Base_pb2.GripperCommand()
    finger = grip_cmd.gripper.finger.add()
    
    grip_cmd.mode = Base_pb2.GRIPPER_POSITION
    finger.finger_identifier = 1
    finger.value = gripper_pose
    
    base.SendGripperCommand(grip_cmd)
    
    str_time = time.time()
    
    while True:
        feedback = base_cyclic.RefreshFeedback()
        cur_pos = (feedback.interconnect.gripper_feedback.motor[0].position) / 100
        print(cur_pos)
        
        if abs(cur_pos - gripper_pose) <= 0.01:
            break
        
        if time.time() - str_time > 5:
            print("Gripper did not reach target position within timeout.")
            break
        
        time.sleep(0.025)
        
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
    print("Current position is : {0}".format(gripper_measure.finger[0].value))

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
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.init() == 0:
        print("No JoyStick connected")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    import utilities
    
    args = utilities.parseConnectionArguments()
    
    mode = 'c'
    
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base, base_cyclic = connect_to_kinova(router)
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.5)
            
        try:
            while True:
                os.system('clear')
                pygame.event.pump()
                sticks = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
                buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
                print(f"mode : {mode}")
                print(f"RAW DATA : {sticks}")
                
                if buttons[7] == 1:
                    time.sleep(0.5)
                    if mode == 'c':
                        mode = 'r'
                    else:
                        mode = 'c'
                
                if mode == 'c':
                    ratio = -0.05
                    # x 방향 움직임의 변위
                    dx = round(sticks[1], 1)
                    x = dx * ratio
                    
                    # y 방향 움직임 변위 
                    dy = round(sticks[0], 1)
                    
                    y = dy * ratio
                    
                    # z 방향 움직임 변위
                    dz = round(sticks[4], 1)
                    
                    z = dz * ratio
                    
                    TARGET_POSE["x"] += x
                    TARGET_POSE["y"] += y
                    TARGET_POSE["z"] += z
                    
                    print(f"Processed DATA : {x}, {y}, {z}")
                    
                    if abs(x) > 0 or abs(y) > 0 or abs(z) > 0:
                        move_to_pose(base, base_cyclic, TARGET_POSE)
                        
                elif mode == 'r':
                    ratio = 5.0
                    # x 방향 움직임의 변위
                    dx = round(sticks[1], 1)
                    x = dx * ratio
                    
                    # y 방향 움직임 변위 
                    dy = round(sticks[0], 1)
                    
                    y = dy * ratio
                    
                    # z 방향 움직임 변위
                    dz = round(sticks[4], 1)
                    
                    z = dz * ratio
                    
                    TARGET_POSE["theta_x"] += x
                    TARGET_POSE["theta_y"] += y
                    TARGET_POSE["theta_z"] += z
                    
                    print(f"Processed DATA : {x}, {y}, {z}")
                    
                    if abs(x) > 0 or abs(y) > 0 or abs(z) > 0:
                        move_to_pose(base, base_cyclic, TARGET_POSE)
                
                gripper_open = round(sticks[2])
                gripper_close = round(sticks[5])
                
                if gripper_open > 0.5:
                    control_gripper(base, base_cyclic, 0)
                
                elif gripper_close > 0.5:
                    control_gripper(base, base_cyclic, 0.5)
                
                pygame.time.Clock().tick(60)
                
        except KeyboardInterrupt:
            print("Exiting...")
            
        finally:
            pygame.quit()
            
if __name__ == "__main__":
    main()