#include <custom_joint_publisher.h>
#include <math.h>
#include <ur5_kinematics.h>

void send_des_jstate(const JointStateVector & joint_pos)
{
    std::cout << "q_des " << joint_pos.transpose() << std::endl;
   if (real_robot)
   {
        for (int i = 0; i < joint_pos.size(); i++)
        {
          jointState_msg_robot.data[i] = joint_pos[i];
        }

        pub_des_jstate.publish(jointState_msg_robot);

   }else {
        for (int i = 0; i < joint_pos.size(); i++)
        {
          jointState_msg_sim.position[i] = joint_pos[i];
          jointState_msg_sim.velocity[i] = 0.0;
          jointState_msg_sim.effort[i] = 0.0;
        }

        pub_des_jstate.publish(jointState_msg_sim);
    }
/*   if (pub_des_jstate_sim_rt->trylock())
  {
    pub_des_jstate_sim_rt->msg_ = jointState_msg;
    pub_des_jstate_sim_rt->unlockAndPublish();
  } */
}


void initFilter(const JointStateVector & joint_pos)
{
        filter_1 = joint_pos;
        filter_2 = joint_pos;
}

JointStateVector secondOrderFilter(const JointStateVector & input, const double rate, const double settling_time)
{

        double dt = 1 / rate;
        double gain =  dt / (0.1*settling_time + dt);
        filter_1 = (1 - gain) * filter_1 + gain * input;
        filter_2 = (1 - gain) * filter_2 + gain *filter_1;
        return filter_2;
}

int main () {
    th <<   1.48357, -1.0231, 0.549017, -1.09672, 1.5708, 0.0872224;

    for(int i=0 ; i<6 ; i++){
        dist[i] *= SCALE_FACTOR;
        a[i] *= SCALE_FACTOR;
    }
    
    Vector3d pe;
    pe << 0.574422, 1.02901, 1.06601; // desired point

    MatrixXd th_m = InverseKinematicsUr5(pe);
    th << th_m(0,0), th_m(0,1), th_m(0,2), th_m(0,3), th_m(0,4), th_m(0,5); // choose one line of possible th

    cout << th << endl << endl;

    Vector3d pe_2 = DirectKinematicsUr5(th); // test inverse kin

    cout << pe_2-pe << endl;
    

    return 0;
}
