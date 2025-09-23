#include "Y2RobMotion/kuka_motion.hpp"

/*** Select Force Control Mode ***/
/* 0: Classical Force Controller - static parameter, k=0 at contact*/
/* 1: FAAC Force Controller - variable parameter, MDK variation (Jay's Controller)*/
#define Force_Con_Mode 0

/*** Select Force Control Coordinate ***/
/* 0: Cartesian Coordinate */
/* 1: TCP Coordinate */
#define Force_Con_Coordinate 0

void kuka_motion::control_force()
{
    control_mode = "Force";
    force_con_mode = (Force_Con_Mode == 0)? "Classic":"FAAC";
    /* Target position init - accential!!! */
    if(pre_control_mode != control_mode)
    {
        /* Motion initial */
        target_pose = current_pose;
        target_angles = current_angles;
        
        /* admittance contorl parameters */
        FC_AC_desX = current_pose; // Current_pose: mm, rad
        FC_AC_desX[0] = FC_AC_desX[0]/1000; // mm->m, AC input: m, rad
        FC_AC_desX[1] = FC_AC_desX[1]/1000; // mm->m, AC input: m, rad
        FC_AC_desX[2] = FC_AC_desX[2]/1000; // mm->m, AC input: m, rad

        FC_MASS = {1, 1, 1, 0.05, 0.05, 0.05}; // Position(3), Orientation(3)
        FC_DAMPER = {3000, 3000, 3000, 10, 10, 10}; // Position(3), Orientation(3)
        FC_STIFFNESS = {2000,2000,2000,20,20,20}; // Position(3), Orientation(3)

        for(int i=0;i<6;i++){AControl[i].adm_1D_MDK(FC_MASS[i],FC_DAMPER[i],FC_STIFFNESS[i]);}

        /* FAAC initialization */
        for(int i=0;i<3;i++){FAAC3step[i]->FAAC_Init(FC_MASS[i],FC_DAMPER[i],FC_STIFFNESS[i]);}

        RCLCPP_INFO(node_->get_logger(),"Guiding was initialized");
    }

    /**** START OF FORCE-CONTROL ****/

    for(int i=0;i<6;i++) // Xd(HG: previous target pose), Fd(HG: 0), Fext
    {
        #if(Force_Con_Mode == 0) // Classical Force Controller
            /* Position - Force control */
            if(i<3)
            {
                /* if it contact with surf. -> k=0 */
                if(fabs(FC_AC_desX[i+6]) > 0.01) {AControl[i].adm_1D_MDK(FC_MASS[i],FC_DAMPER[i],0.0);}
                /* k recovery */
                else{
                    if(AControl[i].adm_MDK_monitor(2) != FC_STIFFNESS[i])
                    {
                        double tau_k = 3.0; // Recovery time (s)
                        double alpha = 1.0 - std::exp(-Control_period_/ tau_k);
                        double target_k = AControl[i].adm_MDK_monitor(2) + alpha * (FC_STIFFNESS[i] - AControl[i].adm_MDK_monitor(2));

                        AControl[i].adm_1D_MDK(FC_MASS[i],FC_DAMPER[i],target_k);
                    }
                }

                /* Force control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], FC_AC_desX[i+6], ft1data[i]);
            }
            /* Orientation - Position control */
            else
            {
                /* Position control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, ft1data[i]);
            }
        #elif(Force_Con_Mode == 1) // FAAC Force Controller
            /* Position - Force control */
            if(i<3)
            {
                /* FAAC flag (if desired force not zero -> flag on -> FAAC control) */
                FAAC_flag[i] = ((fabs(FC_AC_desX[i+6]) > 0.01) || FAAC_flag[i])? true:false;
                FAAC_flag[abs(i-1)] = (FAAC_flag[i])? false:FAAC_flag[abs(i-1)];
                FAAC_flag[abs(i-2)] = (FAAC_flag[i])? false:FAAC_flag[abs(i-2)];
                
                /* In the case of flag true */
                if (FAAC_flag[i]){
                    /* Three Step FAAC MDK update */
                    double Tank_energy = 5; // in here, we fix the tank energy (must be revised)
                    /* FAAC_MDKob_RUN(Tank_energy(J),Ext_Force(N),Dis_Force(N),AC_Pose(m),Act_Pose(m)) */
                    auto TSFAAC_MDK = FAAC3step[i]->FAAC_MDKob_RUN(Tank_energy, ft1data[i], FC_AC_desX[i+6], AC_pose[i], current_pose[i]/1000);
                    AControl[i].adm_1D_MDK(TSFAAC_MDK.Mass,TSFAAC_MDK.Damping,TSFAAC_MDK.Stiffness);
                }
                
                /* Force control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], FC_AC_desX[i+6], ft1data[i]);
            }
            /* Orientation - Position control */
            else
            {
                /* Position control using AC */
                AC_pose[i] = AControl[i].adm_1D_control(FC_AC_desX[i], 0.0, ft1data[i]);
            }


        #endif
    } 
    target_pose = AC_pose;
    target_pose[0] = target_pose[0]*1000; // m -> mm
    target_pose[1] = target_pose[1]*1000; // m -> mm
    target_pose[2] = target_pose[2]*1000; // m -> mm

    /**** END OF FORCE-CONTROL ****/

    /* Generate target HTM */
    std::vector<double> target_ori = {target_pose[3], target_pose[4], target_pose[5]};
    auto target_rot = YMatrix::fromSpatialAngle(target_ori);
    target_HTM = YMatrix::identity(4);
    target_HTM.insert(0, 0, target_rot);
    target_HTM[0][3] = target_pose[0]; // mm unit
    target_HTM[1][3] = target_pose[1]; // mm unit
    target_HTM[2][3] = target_pose[2]; // mm unit

    /* Inverse kinematics using QP-solver + execution */
    target_angles = solve_IK(target_angles, target_HTM);

    /* Data upload to past */
    pre_control_mode = control_mode; // Store previous control mode for comparison
}