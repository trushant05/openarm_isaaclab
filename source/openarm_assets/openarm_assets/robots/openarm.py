"""Configuration for the OpenArm Robot.

The following configurations are available:

* :obj:`OPENARM_ROBOT_CFG`: Bimanual OpenArm Robot

Reference: https://github.com/
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from openarm_assets import OPENARM_ASSETS_DATA_DIR

##
# Configuration
##

OPENARM_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{OPENARM_ASSETS_DATA_DIR}/openarm/openarm.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "openarm_left_joint1": 0.0,
            "openarm_left_joint2": 0.0,
            "openarm_left_joint3": 0.0,
            "openarm_left_joint4": 0.0,
            "openarm_left_joint5": 0.0,
            "openarm_left_joint6": 0.0,
            "openarm_left_joint7": 0.0,
            "openarm_left_finger_joint1": 0.0,
            "openarm_left_finger_joint2": 0.0,
            "openarm_right_joint1": 0.0,
            "openarm_right_joint2": 0.0,
            "openarm_right_joint3": 0.0,
            "openarm_right_joint4": 0.0,
            "openarm_right_joint5": 0.0,
            "openarm_right_joint6": 0.0,
            "openarm_right_joint7": 0.0,
            "openarm_right_finger_joint1": 0.0,
            "openarm_right_finger_joint2": 0.0,
        },
    ),
    actuators={
        "body": ImplicitActuatorCfg(
            joint_names_expr=["openarm_left_joint[1-7]",
                              "openarm_left_finger_joint[1-2]",
                              "openarm_right_joint[1-7]",
                              "openarm_right_finger_joint[1-2]",
                              ],
            stiffness=80.0,
            damping=4.0, 
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

"""Configuration of OpenArm."""

OPENARM_ROBOT_HIGH_PD_CFG = OPENARM_ROBOT_CFG.copy()
OPENARM_ROBOT_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
OPENARM_ROBOT_HIGH_PD_CFG.actuators["body"].stiffness = 400.0
OPENARM_ROBOT_HIGH_PD_CFG.actuators["body"].damping = 80.0
"""Configuration of OpenArm robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
