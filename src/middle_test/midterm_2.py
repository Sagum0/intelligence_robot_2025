import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

TIMEOUT_DURATION = 20

TARGET_POSE = {
    "x" : 0.15,
    "y" : 0.3,
    "z" : 0.205,
    "theta_x" : 180,
    "theta_y" : 0, 
    "theta_z" : 0
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

grip_open = 0.0
grip_close = 0.7

def main():
    import utilities
    
    args = utilities.parseConnectionArguments()
    
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base, base_cyclic = connect_to_kinova(router)
        
        # success = example_move_to_home_base(base)
        # time.sleep(1.0)
        
        # 초기 위치로 이동
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # 초기 위치에서 하강
        TARGET_POSE['z'] -= 0.17
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # gripper 잡기
        control_gripper(base, base_cyclic, grip_close)
        time.sleep(0.3)
        
        # 잡고 상승
        TARGET_POSE['z'] += 0.17
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        TARGET_POSE['theta_z'] += 90
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        
        # X 방향으로 이동
        TARGET_POSE['x'] += 0.12
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # X로 10 이동한 위치에서 하강
        TARGET_POSE['z'] -= 0.165
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # gripper 놓기
        control_gripper(base, base_cyclic, grip_open)
        time.sleep(0.3)
        
        # 약간의 잡고 상승
        TARGET_POSE['z'] += 0.05
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # 다시 잡으러 하강
        TARGET_POSE['z'] -= 0.055
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # gripper 잡기
        control_gripper(base, base_cyclic, grip_close)
        time.sleep(0.3)
        
        # 잡고 상승
        TARGET_POSE['z'] += 0.17
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        TARGET_POSE['theta_z'] -= 180
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # Y 방향으로 이동
        TARGET_POSE['y'] += 0.08
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # Y로 10 이동한 위치에서 하강
        TARGET_POSE['z'] -= 0.165
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # gripper 놓기
        control_gripper(base, base_cyclic, grip_open)
        time.sleep(0.3)
        
        # 약간의 잡고 상승
        TARGET_POSE['z'] += 0.05
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # 다시 잡으러 하강
        TARGET_POSE['z'] -= 0.055
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # gripper 잡기
        control_gripper(base, base_cyclic, grip_close)
        time.sleep(0.3)
        
        # 잡고 상승
        TARGET_POSE['z'] += 0.17
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # 초기 위치 재정립
        TARGET_POSE['x'] = 0.15
        TARGET_POSE['y'] = 0.3
        TARGET_POSE['z'] = 0.205
        TARGET_POSE['theta_x'] = 180
        TARGET_POSE['theta_y'] = 0
        TARGET_POSE['theta_z'] = 0
        
        # 최기 위치로 이동
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # 초기로 이동한 위치에서 하강
        TARGET_POSE['z'] -= 0.165
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.3)
        
        # gripper 놓기
        control_gripper(base, base_cyclic, grip_open)
        time.sleep(0.3)
        
        # 약간의 잡고 상승
        TARGET_POSE['z'] += 0.17
        
            
if __name__ == "__main__":
    main()