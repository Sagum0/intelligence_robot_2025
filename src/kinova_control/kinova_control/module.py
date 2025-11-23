
class Midterm_1:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.4, 0.0, 0.3, 179.0, 0.0, 90.0], # 홈 위치
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 1
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'gripper' : 0.6,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans

        elif step == 'step_4':
            ans = {
                'gripper' : 0.0,
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
                'position' : [0.3, 0.0, 0.25, 179.0, 0.0, 90.0], # 홈 위치
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'frame'    : 'cartesian',
                'position' : [0.3, 0.0, 0.3, 179.0, 0.0, 90.0], # 이동 2 (다른 방향)
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        else:
            return None
