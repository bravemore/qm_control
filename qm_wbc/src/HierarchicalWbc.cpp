//
// Created by skywoodsz on 2023/4/9.
//

#include "qm_wbc/HierarchicalWbc.h"

#include "qm_wbc/HoQp.h"

namespace qm{
HierarchicalWbc::HierarchicalWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                 const ocs2::PinocchioEndEffectorKinematics& armEeKinematics,
                                 ros::NodeHandle &controller_nh)
        : WbcBase(pinocchioInterface, info, eeKinematics, armEeKinematics, controller_nh){
}


vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t time) {

    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    //Task返回的是a，b，d，f矩阵的组合
    Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
            + formulateNoContactMotionTask() + formulateFrictionConeTask();
    Task taskInit = formulateArmJointNomalTrackingTask();

    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask()
                 + formulateEeLinearMotionTrackingTask() + formulateEeAngularMotionTrackingTask()
                 + formulateSwingLegTask() * 100;
    Task task2 = formulateContactForceTask(inputDesired) + formulateBaseLinearMotionTask();

    if(time < 10)
    {
        HoQp hoQp(task2, std::make_shared<HoQp>(taskInit, std::make_shared<HoQp>(task0)));
        vector_t x_optimal = hoQp.getSolutions();
        //决策变量x_optimal包括两个量：机器人关节的加速度和接触力。但返回的cmd中还包括关节力矩
        return WbcBase::updateCmd(x_optimal);
    }
    else
    {
        HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0))); //
        vector_t x_optimal = hoQp.getSolutions();
        return WbcBase::updateCmd(x_optimal);
    }
}
}