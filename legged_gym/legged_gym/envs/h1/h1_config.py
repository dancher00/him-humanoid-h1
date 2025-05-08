# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class H1RoughCfg( LeggedRobotCfg ):

    class terrain( LeggedRobotCfg.terrain):
        mesh_type = "trimesh"  # plane, heightfield,trimesh
        #measure_heights = False

        
    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_one_step_observations = 42
        num_observations = 42 #num_one_step_observations * 6
        num_one_step_privileged_obs = 45 # 45 + 3 + 3 + 187 # additional: base_lin_vel, external_forces, scan_dots
        num_privileged_obs = num_one_step_privileged_obs * 1 # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 10
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 1.0 + 0.17756] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0,
            "left_hip_pitch_joint": -0.4,
            "left_knee_joint": 0.8,
            "left_ankle_joint": -0.4,

            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0,
            "right_hip_pitch_joint": -0.4,
            "right_knee_joint": 0.8,
            "right_ankle_joint": -0.4,

            "torso_joint": 0.0,

            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,

            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {
            "hip_yaw": 200,
            "hip_roll": 200,
            "hip_pitch": 200,
            "knee": 300,
            "ankle": 40,
            "torso": 300,
            "shoulder": 100,
            "elbow": 100,
        }  # [N*m/rad]
        damping = {
            "hip_yaw": 5,
            "hip_roll": 5,
            "hip_pitch": 5,
            "knee": 6,
            "ankle": 2,
            "torso": 6,
            "shoulder": 2,
            "elbow": 2,
        }    # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_reduction = 1.0

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/h1/urdf/h1.urdf'
        name = "h1"
        foot_name = "ankle"
        penalize_contacts_on = ['ankle', 'knee']
        terminate_after_contacts_on = ['pelvis',"imu"]
        #privileged_contacts_on = ["base", "thigh", "calf"] #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = True # Some .obj meshes must be flipped from y-up to z-up
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 300.
        only_positive_rewards = False
        
        class scales( LeggedRobotCfg.rewards.scales ):
            # tracking_lin_vel = 1.5
            # orientation = -2.  
            # collision = -1.
            # termination = -300.
            # tracking_ang_vel = 0.001
            # torques = -3.e-6
            # dof_acc = -3.e-7
            # lin_vel_z = -1.
            # feet_air_time = 10.
            # dof_pos_limits = -1.
 
            # dof_vel = -5.e-3
            # ang_vel_xy = 0.0 
            # feet_contact_forces = -0.
            # hip_symmetry = 2.
            # action_rate = -0.0001


            tracking_lin_vel = 1.5
            orientation = -2.  
            collision = -1.
            termination = -300.
            tracking_ang_vel = 0.001
            torques = -3.e-6
            dof_acc = -3.e-7
            lin_vel_z = -1.
            feet_air_time = 10.
            dof_pos_limits = -1.
            #no_fly = 0.5
            dof_vel = -5.e-3
            ang_vel_xy = 0.0 
            feet_contact_forces = -0.
            # hip_symmetry = 2.
            action_rate = -0.0001
            # lin_vel_y = -1.
            #lin_vel_x = 1.




class H1RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_h1'

  