#include "pid.h"
#include "fuzzy_pid.h"

#define POSITION_PID 1 //位置式
#define DELTA_PID  2   //增量式
#define PID_MODE POSITION_PID // <--位置式 

static void abs_limit(float *x,int32_t limit)
{
	if(*x > limit)
		*x = limit;
	if(*x < -limit)
		*x = -limit;
}
/*PID电机堵转检测*/
static void f_PID_ErrorHandle(pid_t *pid)
{
    //排除PID输出本身很小的情况
    if (pid->out < pid->maxout * 0.01)
        return;

    //考虑到该判断策略的灵活性，0.9这个常数的选取是很灵活的
    if ((fabs(pid->set - pid->get) / pid->set) > 0.9f)
    {
        //电机堵转计数
        pid->ERROR_Handler.ERRORCount++;
    }
    else
    {
        pid->ERROR_Handler.ERRORCount = 0;
    }

    if (pid->ERROR_Handler.ERRORCount > 1000)
    {
        //上述现象持续一段时间则被认定为电机堵转
        pid->ERROR_Handler.ERROR_TYPE = MOTTOR_BLOCKED;
    }
}
static void f_Integral_Limit(pid_t *pid)
{
      if(fabs(pid->out) > pid->maxout) //判断PID增益是否大于最大限幅
      {
        if (pid->error[NOW_ERR] * pid->iout > 0) //若超调且误差与Ki输出同号
        {
            //在取消积分作用前，同样需要先判断当前周期内积分是否积累
            //如果积分为减小趋势则不需要限制其减小
            //如果iterm积分项在增大则令积分项为零
            pid->iterm = 0;
        }
      }
      //判断并抉择当前Ki 总增益是否大于最大Ki 增益限幅，若大于则限制为最大项
      if (pid->iout > pid->integral_limit) //积分限幅
      {
          pid->iterm = 0;
          pid->iout = pid->integral_limit;
      }
      if (pid->iout  < - pid->integral_limit)
      {
          pid->iterm = 0;
          pid->iout = -pid->integral_limit;
      }

}
static void pid_init(pid_t *pid,float p,float i,float d,int32_t max_out,int32_t integral_limit)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxout = max_out;
  pid->iterm = 0;
	pid->integral_limit = integral_limit;
  pid->output_deadband = 5;
  pid->ERROR_Handler.ERRORCount = 0;
  pid->ERROR_Handler.ERROR_TYPE = MOTTOR_NORMAL;
}

static void pid_reset(pid_t *pid,float p,float i,float d)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	
	pid->pout = 0;
  pid->iterm = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out  = 0;

  pid->ERROR_Handler.ERRORCount = 0;
  pid->ERROR_Handler.ERROR_TYPE = MOTTOR_NORMAL;
}

float pid_calc(pid_t *pid,float get,float set)
{
	pid->get = get;
	pid->set = set;
	pid->error[NOW_ERR] = set - get;

  #if (PID_MODE == POSITION_PID)
      pid->pout = pid->kp * pid->error[NOW_ERR]; // Kp总增益
     // pid->iout += pid->ki * pid->error[NOW_ERR];
      pid->iterm = pid->ki * pid->error[NOW_ERR];//Ki ITerm项
      pid->iout = pid->iout + pid->iterm;        //Ki 总增益
      pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]); //Kd 总增益

      pid->out = pid->pout + pid->iout + pid->dout; //PID增益
      /*积分限幅*/
      f_Integral_Limit(pid);
      /*PID电机堵转检测*/
      f_PID_ErrorHandle(pid);
      /*PID输出限幅*/
      abs_limit(&(pid->out),pid->maxout);//判断并抉择当前 PID 总增益是否大于最大 PID 增益限幅，若大于则限制为最大项


  #elif (PID_MODE == DELTA_PID)
      pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
      pid->iout = pid->ki * pid->error[NOW_ERR];
      pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]);

      pid->out += pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->maxout);
	#endif
  
  pid->error[LLAST_ERR] = pid->error[LAST_ERR];
  pid->error[LAST_ERR]  = pid->error[NOW_ERR];
  
  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
    return 0;
  else
    return pid->out;
  
}	

float fuzzy_pid_calc(pid_t *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->error[NOW_ERR] = set - get;

//  if ((pid->input_max_err != 0) && (fabs(pid->error[NOW_ERR]) > pid->input_max_err))
//      return 0;
	//PID模糊器
	fuzzy_calc(&(*pid));
	
  #if (PID_MODE == POSITION_PID)
      pid->pout = pid->kp * pid->error[NOW_ERR]; // Kp总增益
     // pid->iout += pid->ki * pid->error[NOW_ERR];
      pid->iterm = pid->ki * pid->error[NOW_ERR];//Ki ITerm项
      pid->iout = pid->iout + pid->iterm;        //Ki 总增益
      pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]); //Kd 总增益

      pid->out = pid->pout + pid->iout + pid->dout; //PID增益
      /*积分限幅*/
      f_Integral_Limit(pid);
      /*PID电机堵转检测*/
      f_PID_ErrorHandle(pid);
      /*PID输出限幅*/
      abs_limit(&(pid->out),pid->maxout);//判断并抉择当前 PID 总增益是否大于最大 PID 增益限幅，若大于则限制为最大项

  #elif (PID_MODE == DELTA_PID)
      pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
      pid->iout = pid->ki * pid->error[NOW_ERR];
      pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]);

      pid->out += pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->maxout);

  #endif
  
  pid->error[LLAST_ERR] = pid->error[LAST_ERR];
  pid->error[LAST_ERR]  = pid->error[NOW_ERR];
  
  
//  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
//    return 0;
//  else
    return pid->out;

}

/* 误差积分清除函数 */
void PID_Clear(pid_t *pid)
{
	pid->pout = 0;
  pid->iterm = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->error[NOW_ERR] = 0;
	pid->error[LAST_ERR] = 0;

  pid->ERROR_Handler.ERRORCount = 0;
  pid->ERROR_Handler.ERROR_TYPE = MOTTOR_NORMAL;
}
/*PID初始化*/
void PID_Struct_Init(pid_t *pid,float p,float i,float d,int32_t max_out,int32_t integral_limit,INIT_STATUS init_status)
{
	if(init_status == INIT)//用于初始化
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_reset = pid_reset;
		
		pid->f_pid_init(pid,p,i,d,max_out,integral_limit);
		pid->f_pid_reset(pid,p,i,d);
	}
	else									 //用于debug
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_init(pid,p,i,d,max_out,integral_limit);
	}
}

pid_t pid_yaw           = {0};
pid_t pid_pit           = {0};
pid_t pid_yaw_spd       = {0};
pid_t pid_pit_spd       = {0};
pid_t pid_pit_cur       = {0};
pid_t pid_chassis_vx_vy_spd[4]        = {0};
pid_t pid_chassis_cali_angle = {0};
pid_t pid_loader_angle[2]    = {0};
pid_t pid_loader_spd[2] = {0};
pid_t pid_fric[2]       = {0};
pid_t pid_heat_limit    = {0};
pid_t pid_imu_tmp       = {0};
pid_t pid_power					= {0};

pid_t pid_chassis_vw   	= {0};
pid_t pid_chassis_power_buffer	= {0};
