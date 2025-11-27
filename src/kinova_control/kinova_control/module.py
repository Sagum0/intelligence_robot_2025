
class Midterm_1:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.2, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'camera_trigger' : [1, 2],
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        
        elif step == 'step_3':
            ans = {
                'move_pick_top' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans

        elif step == 'step_4':
            ans = {
                'move_pick' : True,
                'idx_offset' : 0,
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
                'move_pick_after' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans

        elif step == 'step_7':
            ans = {
                'move_drop_top' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans

        elif step == 'step_8':
            ans = {
                'move_drop' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans

        elif step == 'step_9':
            ans = {
                'gripper' : 0.05,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        elif step == 'step_10':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.2, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
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
                'position' : [0.2, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'camera_trigger' : [1, 2, 3, 4],
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        
        elif step == 'step_3':
            ans = {
                'move_pick_top' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans

        elif step == 'step_4':
            ans = {
                'move_pick' : True,
                'idx_offset' : 0,
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
                'move_pick_after' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans

        elif step == 'step_7':
            ans = {
                'move_drop_top' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans

        elif step == 'step_8':
            ans = {
                'move_drop' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans

        elif step == 'step_9':
            ans = {
                'gripper' : 0.05,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        elif step == 'step_10':
            ans = {
                'move_drop_top' : True,
                'idx_offset' : 0,
                'requires_ack' : True,
                'next_step': 'step_11'
            }
            return ans

        elif step == 'step_11':
            ans = {
                'move_pick_top' : True,
                'idx_offset' : 2,
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans

        elif step == 'step_12':
            ans = {
                'move_pick' : True,
                'idx_offset' : 2,
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans

        elif step == 'step_13':
            ans = {
                'gripper' : 0.7,
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans

        elif step == 'step_14':
            ans = {
                'move_pick_after' : True,
                'idx_offset' : 2,
                'requires_ack' : True,
                'next_step': 'step_15'
            }
            return ans

        elif step == 'step_15':
            ans = {
                'move_drop_top' : True,
                'idx_offset' : 2,
                'requires_ack' : True,
                'next_step': 'step_16'
            }
            return ans

        elif step == 'step_16':
            ans = {
                'move_drop' : True,
                'idx_offset' : 2,
                'requires_ack' : True,
                'next_step': 'step_17'
            }
            return ans

        elif step == 'step_17':
            ans = {
                'gripper' : 0.0,
                'requires_ack' : True,
                'next_step': 'step_18'
            }
            return ans
        
        elif step == 'step_18':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.2, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans

        else:
            return None
