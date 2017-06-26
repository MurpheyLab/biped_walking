#ifndef SYS_DYNAM_HPP
#define SYS_DYNAM_HPP

namespace sac {

  //[ The rhs of x' = f(x) defined as a class
  // USER SPECIFIED:
  class sys_dynam {
    b_control & u_;
    state_type u_curr_;
  
  public:
    sys_dynam( b_control & u ) : u_(u) , u_curr_(ulen) { }
  
    void operator() (const state_type &x, state_type &dxdt, const double t)
    {
      u_(t, x, u_curr_);//u_curr_[0] = 0;u_curr_[1] = 0;

	  
	  dxdt[0] = x[5];
	  dxdt[1] = x[6];
	  dxdt[2] = x[7];
	  dxdt[3] = cos(x[0])*x[5];
	  dxdt[4] = -sin(x[0])*x[5];
	  dxdt[5] = (sin(x[0])*(-2.2563E2)-sin(x[0]+x[1]*2.0)*(9.81E2/1.0E2)-sin(x[0]-x[2]*2.0)*(9.81E2/5.0E1)-cos(x[0]-x[2])*u_curr_[0]*(4.0/5.0)-cos(x[0]-x[2])*u_curr_[1]*(4.0/5.0)-sin(x[0]+x[1])*(x[6]*x[6])*2.0+sin(x[0]-x[2])*(x[7]*x[7])*4.0-cos(x[0]+x[1])*u_curr_[1]*(4.0/5.0)-u_curr_[0]*(4.0/5.0)+sin(x[0]*2.0+x[1]*2.0)*(x[5]*x[5])+sin(x[0]*2.0-x[2]*2.0)*(x[5]*x[5])*2.0)/(cos(x[0]*2.0+x[1]*2.0)+cos(x[0]*2.0-x[2]*2.0)*2.0-2.3E1);
	  dxdt[6] = (cos(x[0]+x[1])*(sin(x[0])*3.18825E2+sin(x[0]+x[1])*(x[6]*x[6])*(5.0/2.0)-sin(x[0]-x[2])*(x[7]*x[7])*5.0+u_curr_[0])*(2.0/5.0))/(pow(cos(x[0]-x[2]),2.0)*2.0+pow(cos(x[0]+x[1]),2.0)-1.3E1)-((pow(cos(x[0]-x[2]),2.0)*2.0-1.3E1)*(sin(x[1])*(9.81E2/4.0E1)-sin(x[0]+x[1])*(x[5]*x[5])*(5.0/2.0)+u_curr_[1])*(2.0/5.0))/(pow(cos(x[0]-x[2]),2.0)*2.0+pow(cos(x[0]+x[1]),2.0)-1.3E1)-(cos(x[0]+x[1])*cos(x[0]-x[2])*(sin(x[2])*(9.81E2/2.0E1)+sin(x[0]-x[2])*(x[5]*x[5])*5.0-u_curr_[0]-u_curr_[1])*(2.0/5.0))/(pow(cos(x[0]-x[2]),2.0)*2.0+pow(cos(x[0]+x[1]),2.0)-1.3E1);
	  dxdt[7] = ((pow(cos(x[0]+x[1]),2.0)-1.3E1)*(sin(x[2])*(9.81E2/2.0E1)+sin(x[0]-x[2])*(x[5]*x[5])*5.0-u_curr_[0]-u_curr_[1])*(1.0/5.0))/(pow(cos(x[0]-x[2]),2.0)*2.0+pow(cos(x[0]+x[1]),2.0)-1.3E1)+(cos(x[0]-x[2])*(sin(x[0])*3.18825E2+sin(x[0]+x[1])*(x[6]*x[6])*(5.0/2.0)-sin(x[0]-x[2])*(x[7]*x[7])*5.0+u_curr_[0])*(2.0/5.0))/(pow(cos(x[0]-x[2]),2.0)*2.0+pow(cos(x[0]+x[1]),2.0)-1.3E1)+(cos(x[0]+x[1])*cos(x[0]-x[2])*(sin(x[1])*(9.81E2/4.0E1)-sin(x[0]+x[1])*(x[5]*x[5])*(5.0/2.0)+u_curr_[1])*(2.0/5.0))/(pow(cos(x[0]-x[2]),2.0)*2.0+pow(cos(x[0]+x[1]),2.0)-1.3E1);
	  dxdt[8] = 0;
	  dxdt[9] = 0;
    }
  };
  //]

}

#endif  // SYS_DYNAM_HPP
