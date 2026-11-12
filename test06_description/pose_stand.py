# -*- coding: utf-8 -*-
"""
test06 / pose_stand

由 Fusion 的 JointSetup 產生於 2026-11-12 15:44。
角度基準是 URDF 的零位，不是 Fusion 的 joint 零位。
"""

# joint 名稱 -> 角度（度）。USD 的角度單位是度，不是弧度。
JOINT_POS_DEG = {
    "base_leg_lf": 0.0000,
    "leg_lf1_lf2": -45.0000,
    "leg_lf2_lf3": -45.0000,
    "leg_lf3_lf4": 0.0000,
    "leg_lf_wheel": 0.0000,
    "base_leg_lr": 0.0000,
    "leg_lr1_lr2": -45.0000,
    "leg_lr2_lr3": -45.0000,
    "leg_lr3_lr4": 0.0000,
    "leg_lr_wheel": 0.0000,
    "base_leg_rf": 0.0000,
    "leg_rf1_rf2": -45.0000,
    "leg_rf2_rf3": -45.0000,
    "leg_rf3_rf4": 0.0000,
    "leg_rf_wheel": 0.0000,
    "base_leg_rr": 0.0000,
    "leg_rr1_rr2": -45.0000,
    "leg_rr2_rr3": -45.0000,
    "leg_rr3_rr4": 0.0000,
    "leg_rr_wheel": 0.0000,
    "base_arm_l": 0.0000,
    "arm_l1_l2": -45.0000,
    "arm_l2_l3": -45.0000,
    "arm_l3_wrist_l": -179.4010,
    "wrist_l_gripper_l": 0.0000,
    "base_arm_r": 0.0000,
    "arm_r1_r2": -45.0000,
    "arm_r2_r3": -45.0000,
    "arm_r3_wrist_r": -179.4010,
    "wrist_r_gripper_r": 0.0000,
}

# 給 Isaac Lab 的 ArticulationCfg.InitialStateCfg 用（弧度）：
# joint_pos={
#     "base_leg_lf": 0.000000,
#     "leg_lf1_lf2": -0.785398,
#     "leg_lf2_lf3": -0.785398,
#     "leg_lf3_lf4": 0.000000,
#     "leg_lf_wheel": 0.000000,
#     "base_leg_lr": 0.000000,
#     "leg_lr1_lr2": -0.785398,
#     "leg_lr2_lr3": -0.785398,
#     "leg_lr3_lr4": 0.000000,
#     "leg_lr_wheel": 0.000000,
#     "base_leg_rf": 0.000000,
#     "leg_rf1_rf2": -0.785398,
#     "leg_rf2_rf3": -0.785398,
#     "leg_rf3_rf4": 0.000000,
#     "leg_rf_wheel": 0.000000,
#     "base_leg_rr": 0.000000,
#     "leg_rr1_rr2": -0.785398,
#     "leg_rr2_rr3": -0.785398,
#     "leg_rr3_rr4": 0.000000,
#     "leg_rr_wheel": 0.000000,
#     "base_arm_l": 0.000000,
#     "arm_l1_l2": -0.785398,
#     "arm_l2_l3": -0.785398,
#     "arm_l3_wrist_l": -3.131138,
#     "wrist_l_gripper_l": 0.000000,
#     "base_arm_r": 0.000000,
#     "arm_r1_r2": -0.785398,
#     "arm_r2_r3": -0.785398,
#     "arm_r3_wrist_r": -3.131138,
#     "wrist_r_gripper_r": 0.000000,
# }

import omni.usd
from pxr import UsdPhysics, PhysxSchema

stage = omni.usd.get_context().get_stage()

found = {}
for prim in stage.Traverse():
    name = prim.GetName()
    if name in JOINT_POS_DEG and (prim.IsA(UsdPhysics.RevoluteJoint)
                                  or prim.IsA(UsdPhysics.PrismaticJoint)):
        found[name] = prim

ok_state, ok_drive, missing, errors = 0, 0, [], []

for name, deg in JOINT_POS_DEG.items():
    prim = found.get(name)
    if prim is None:
        missing.append(name)
        continue

    axis_token = 'angular' if prim.IsA(UsdPhysics.RevoluteJoint) else 'linear'

    # 1) 關節狀態 —— 決定 reset 之後回到哪裡
    try:
        state = PhysxSchema.JointStateAPI.Apply(prim, axis_token)
        state.CreatePositionAttr().Set(float(deg))
        ok_state += 1
    except Exception as e:
        errors.append('%s state: %s' % (name, e))

    # 2) drive 目標 —— 位置控制時追蹤的目標
    try:
        drive = UsdPhysics.DriveAPI.Get(prim, axis_token)
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, axis_token)
        drive.CreateTargetPositionAttr().Set(float(deg))
        ok_drive += 1
    except Exception as e:
        errors.append('%s drive: %s' % (name, e))

print('[pose_stand] %d joints in table, %d found in stage'
      % (len(JOINT_POS_DEG), len(found)))
print('  joint state set: %d, drive target set: %d' % (ok_state, ok_drive))
if missing:
    print('  not found in stage (%d): %s' % (len(missing), ', '.join(missing)))
    print('  -> make sure the URDF is imported and joint names are unchanged')
for e in errors[:10]:
    print('  error: %s' % e)

# 這個姿勢下輪子剛好碰地時，base_link 原點離地 0.536 m。
# 給 ArticulationCfg.InitialStateCfg 用：pos=(0.0, 0.0, 0.536)
