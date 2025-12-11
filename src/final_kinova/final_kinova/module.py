class Final:
    def step(self, step):
        # 상부 Detect 위치로 이동 
        if step == 'step_1':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.15, 0.0, 0.32, 179.0, 0.0, 90.0],
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'camera_trigger' : 'obj',
                'camera_target' : 'squ',
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'obj_move_top'    : True,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'camera_trigger' : 'obj',
                'camera_target' : 'squ',
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        elif step == 'step_5':
            ans = {
                'obj_move_pick'    : True,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'gripper'    : 0.65,
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        # 집었으니 Maker 디텍하러 상부 이동
        elif step == 'step_7':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.15, 0.0, 0.32, 179.0, 0.0, 90.0],
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        # 마커 좌표 요청
        elif step == 'step_8':
            ans = {
                'camera_trigger' : 'maker',
                'camera_target' : 1,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans
        
        # 마커 상부로 이동
        elif step == 'step_9':
            ans = {
                'maker_move_top'    : True,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        # 마커 좌표 다시 요청
        elif step == 'step_10':
            ans = {
                'camera_trigger' : 'maker',
                'camera_target' : 1,
                'requires_ack' : True,
                'next_step': 'step_11'
            }
            return ans
        
        # 정확한 마커 수직 위치로 이동
        elif step == 'step_11':
            ans = {
                'maker_move_drop_top'    : True,
                'hex_offset' : False,
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans
        
        # 정확한 마커 위치로 하강
        elif step == 'step_12':
            ans = {
                'maker_move_drop'    : True,
                'hex_offset' : False,
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans
        
        # 놓기
        elif step == 'step_13':
            ans = {
                'gripper'    : 0.25,
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        # 다시 상부로 이동
        elif step == 'step_14':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.15, 0.0, 0.32, 179.0, 0.0, 90.0],
                'requires_ack' : True,
                'next_step': 'step_15'
            }
            return ans
        
        
        ### 2회전
        ### 2회전
        elif step == 'step_15':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.15, 0.0, 0.32, 179.0, 0.0, 90.0],
                'requires_ack' : True,
                'next_step': 'step_16'
            }
            return ans
        
        elif step == 'step_16':
            ans = {
                'camera_trigger' : 'obj',
                'camera_target' : 'hex',
                'requires_ack' : True,
                'next_step': 'step_17'
            }
            return ans
        
        elif step == 'step_17':
            ans = {
                'obj_move_top'    : True,
                'requires_ack' : True,
                'next_step': 'step_18'
            }
            return ans
        
        elif step == 'step_18':
            ans = {
                'camera_trigger' : 'obj',
                'camera_target' : 'hex',
                'requires_ack' : True,
                'next_step': 'step_19'
            }
            return ans
        
        elif step == 'step_19':
            ans = {
                'obj_move_pick'    : True,
                'requires_ack' : True,
                'next_step': 'step_20'
            }
            return ans
        
        elif step == 'step_20':
            ans = {
                'gripper'    : 0.65,
                'requires_ack' : True,
                'next_step': 'step_21'
            }
            return ans
        
        # 집었으니 Maker 디텍하러 상부 이동
        elif step == 'step_21':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.15, 0.0, 0.32, 179.0, 0.0, 90.0],
                'requires_ack' : True,
                'next_step': 'step_22'
            }
            return ans
        
        # 마커 좌표 요청
        elif step == 'step_22':
            ans = {
                'camera_trigger' : 'maker',
                'camera_target' : 0,
                'requires_ack' : True,
                'next_step': 'step_23'
            }
            return ans
        
        # 마커 상부로 이동
        elif step == 'step_23':
            ans = {
                'maker_move_top'    : True,
                'requires_ack' : True,
                'next_step': 'step_24'
            }
            return ans
        
        # 마커 좌표 다시 요청
        elif step == 'step_24':
            ans = {
                'camera_trigger' : 'maker',
                'camera_target' : 0,
                'requires_ack' : True,
                'next_step': 'step_25'
            }
            return ans
        
        # 정확한 마커 수직 위치로 이동
        elif step == 'step_25':
            ans = {
                'maker_move_drop_top'    : True,
                'hex_offset' : True,
                'requires_ack' : True,
                'next_step': 'step_26'
            }
            return ans
        
        elif step == 'step_26':
            ans = {
                'maker_move_turn' : True,
                'requires_ack' : True,
                'next_step': 'step_27'
            }
            return ans
        
        # 정확한 마커 위치로 하강
        elif step == 'step_27':
            ans = {
                'maker_move_drop'    : True,
                'hex_offset' : True,
                'requires_ack' : True,
                'next_step': 'step_28'
            }
            return ans
        
        # 놓기
        elif step == 'step_28':
            ans = {
                'gripper'    : 0.25,
                'requires_ack' : True,
                'next_step': 'step_29'
            }
            return ans
        
        # 다시 상부로 이동
        elif step == 'step_29':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.17, 0.0, 0.32, 179.0, 0.0, 90.0],
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        
        else:
            return None

