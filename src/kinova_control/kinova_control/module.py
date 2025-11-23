
class Midterm_1:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'request_id' : '1',
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'id_move_top' : True,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'id_move_pick' : True,
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans

        elif step == 'step_5':
            ans = {
                'gripper' : 0.7,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'request_id' : '2',
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'id_move_place' : True,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans

        elif step == 'step_9':
            ans = {
                'gripper' : 0.0,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        elif step == 'step_10':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        else:
            return None

class Midterm_2:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'request_id' : '1',
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'id_move_top' : True,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'id_move_pick' : True,
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans

        elif step == 'step_5':
            ans = {
                'gripper' : 0.7,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'request_id' : '2',
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'id_move_place' : True,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans

        elif step == 'step_9':
            ans = {
                'gripper' : 0.0,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        elif step == 'step_10':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans
        
        elif step == 'step_12':
            ans = {
                'request_id' : '3',
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans
        
        elif step == 'step_13':
            ans = {
                'id_move_top' : True,
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        elif step == 'step_14':
            ans = {
                'id_move_pick' : True,
                'requires_ack' : True,
                'next_step': 'step_15'
            }
            return ans

        elif step == 'step_15':
            ans = {
                'gripper' : 0.7,
                'requires_ack' : True,
                'next_step': 'step_16'
            }
            return ans
        
        elif step == 'step_16':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_17'
            }
            return ans
        
        elif step == 'step_17':
            ans = {
                'request_id' : '4',
                'requires_ack' : True,
                'next_step': 'step_18'
            }
            return ans
        
        elif step == 'step_18':
            ans = {
                'id_move_place' : True,
                'requires_ack' : True,
                'next_step': 'step_19'
            }
            return ans

        elif step == 'step_19':
            ans = {
                'gripper' : 0.0,
                'requires_ack' : True,
                'next_step': 'step_20'
            }
            return ans
        
        elif step == 'step_20':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        else:
            return None
