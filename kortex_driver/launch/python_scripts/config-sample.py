class EnvConfig:
    def __init__(self):
        self.USE_SIM = False
        self.JOINT_STATE_TOPIC = '/my_gen3/joint_states'

        if self.USE_SIM:
            self.COLOR_CAMERA_TOPIC = '/my_gen3/camera/color/image_raw'    # For Sim Arm - color
            self.DEPTH_CAMERA_TOPIC = '/my_gen3/camera/depth/image_raw'    # For Sim Arm - depth
        else:
            self.COLOR_CAMERA_TOPIC = '/camera/color/image_raw'              # For Real Arm
            self.DEPTH_CAMERA_TOPIC = '/camera/depth/image'                  # For Real Arm