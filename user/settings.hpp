#ifndef SETTINGS_HPP
#define SETTINGS_HPP

/* Define Constants */
#define PI (3.14159)

namespace sac {

	/*********************************************/
	/* The type of container used to hold the state vector */
	typedef std::vector< double > state_type;


	/*********************************************/
	/* State and control vector length*/
	const size_t xlen = 10, ulen = 2;

	/*********************************************/
	/* Function prototypes */
	inline void initialize_cost_weights();
	inline void u_nominal( const double , state_type & );
	inline void get_DesTraj( const double , Eigen::Matrix< double, xlen, 1 > &  );


	/*********************************************/
	/* Parameters of dynamics*/



	/*********************************************
	/* Initial condition */
	//double th1 = PI/8;
	//const double x_init[] = {th1, PI/8, PI/6, sin(th1), cos(th1), 1.6, 1.6, 0, 0, 0};
	double th1 = 0;
	const double x_init[] = {th1, 0, 0, sin(th1), cos(th1), 0, 0, 0, 0, 0};

	/*********************************************
	/* Indexes of states to be angle wrapped - leave brace blank if no states are to be wrapped */
	const int x_wrap[] = {};

	/*********************************************/
	/* Time Parameters */
	const double t_init = 0.0; // initial time
	const double t_final = 20; // final time

	double T = 1.0; // prediction horizon
	const double ts = 1.0/60.0; //sampling time
	const double impact_duration = 1E-4;// used to store state before and after impact


	/*********************************************/
	/* Control saturation */
	const double usat[ulen][2] = { {30, -30}, {30, -30} };

	
	/*********************************************/
	/* Aggressiveness */
	const double lam = -50000; 


	/*********************************************/
	/*Search for application time? */
	const bool u2Search = false;
	
	/*********************************************/
	/*Write actions to file? */
	const bool save_actions_to_file = true;


	/*********************************************/
	/* Backtracking parameters */
	const double maxdt = .2;//initial control duration
	const double factor = 0.5;//how much the duration decreases at each iteration
	const int max_iters = 10;//number of backtracking iterations (# of times the duration is decreased)


	/*********************************************/
	/* Backtracking parameters for physical hardware
	Basically we are only allowing that are multiples of 2*ctr_com_freq*/
	/*const double ctr_com_freq = 50.0; //how fast the control can be updated in the physical hardware (Hz)
	const double max_iters = 4;//number of backtracking iterations (# of times the duration is decreased)

	//Do not change
	const double min_duration = 2.0/ctr_com_freq;//minimum possible duration of action
	const double maxdt = min_duration + max_iters*min_duration;//initial control duration
	//! No "factor" here; must change backtracking such that*/


	/*********************************************/
	/* Numerical tolerance */
	const double epsilon = 1e-5;


	/*********************************************/
	/*Initialize cost weight matrices (has to be within a function due to <<)*/
	Eigen::Matrix< double, xlen, xlen > Q = Eigen::Matrix< double, xlen, xlen >::Zero();
	Eigen::Matrix< double, ulen, ulen > R = Eigen::Matrix< double, ulen, ulen >::Identity(ulen, ulen);
	Eigen::Matrix< double, xlen, xlen > P = Eigen::Matrix< double, xlen, xlen >::Zero();

	inline void initialize_cost_weights() {
		//Q matrix
		Q(0,0) = 10;
		Q(1,1) = 350;
		Q(2,2) = 350;
		Q(5,5) = 0;  
		Q(6,6) = 0;	
		Q(7,7) = 0;			


		//P matrix
		/*P(0,0) = 50;
		P(2,2) = 100;*/

		//R matrix         
		R = 0.1*R;    		
	}


	/*********************************************/
	/* Define unominal */
	inline void u_nominal( const double t, const state_type & x, state_type & u_nom_t ) {
		for ( size_t i=0; i<ulen; ++i ) { u_nom_t[i] = 0; }//Calculate unominal at t
	}


	/*********************************************/
	/* Desired trajectory */
	inline void get_DesTraj( const double t, Eigen::Matrix< double, xlen, 1 > &m_mxdes ) {
		/*if (t>=3){
		m_mxdes << 0, 0, 0, 0.3*t-0.3, 1, 0, 0, 0, 0, 0;
		Q(3,3) = 200; 
		}
		else
			m_mxdes << 0, 0, 0, 0.2*t, 1, 0, 0, 0, 0, 0;*/
		
		m_mxdes << PI/8, PI/8, PI/6, 0, 0, 1.6, 1.6, 0, 0, 0;
	}
	
	/*********************************************/
	/*Ground level. If changed, then the impact and adjoint map have to be recalculated in MATLAB*/
	inline double Grnd(const state_type &x) {
		return 0;//sin(sin(x[1]) + x[4])//this is the sine of the x coord of toe2
	}
	/*Derivative of above wrt time*/
	inline double DGrnd(const state_type &x) {
		return 0;//sin(sin(x[1]) + x[4])//this is the sine of the x coord of toe2
	}
	/* Impact condition: if swinging foot is in front of supporting foot, and */
/*	inline bool imp_condition(const state_type &x) {
		double xtoe1 = x[3] - sin(x[0]), xtoe2 = x[3] - sin(x[1]);
		double ztoe2 = x[4]-cos(x[1]) - Grnd(x);
		double Dztoe2 = x[9]+sin(x[1])*x[6] - DGrnd(x);//derivative of above
		*/
		/*if((ztoe2<=1E-4)&&(Dztoe2<-1E-4)){
			std::cout << xtoe1 << " " << xtoe2 << " " << ztoe2 << " " << Dztoe2 << "\n";
			system("PAUSE");
		}*/
	/*	
        return (ztoe2<=1E-4)&&(Dztoe2<-1E-4);
    }*/
	
	/* Impact condition */
	inline bool imp_condition(const state_type &x, const state_type &xprev) {
		double xtoe1 = x[3] - sin(x[0]), xtoe2 = x[3] + sin(x[1]);
		double ztoe2 = x[4]-cos(x[1]) - Grnd(x);
		double ztoe2_prev = xprev[4]-cos(xprev[1]) - Grnd(xprev);
		
		/*if((ztoe2_prev>0)&&(ztoe2<0)){
		std::cout << xtoe1 << " " << xtoe2 << " " << ztoe2 << " " <<ztoe2_prev<< "\n";
			system("PAUSE");
		}*/
	
        return ((xtoe2>xtoe1+0.01)&&(ztoe2_prev>=0)&&(ztoe2<=0));
    }


}

#endif  // SETTINGS_HPP
