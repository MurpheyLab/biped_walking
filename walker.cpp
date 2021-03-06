 /* Copyright (C) 2017 Emmanouil Tzorakoleftherakis

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <master.hpp>               // Master include file

#include <ctime> //for timing

using namespace sac;

/* iSACstep() function operator
  input:  initial state and time 
  return: Does not explicitly return anything but the following fields of class "sac_step" can be accessed
  
  iSACstep.xnext - integrated state at time t0+ts

  iSACstep.u_switch - vector of SAC action values applied from [t_i, t_f] which is a subset of [t0, t0+ts].
          If [t_i, t_f] is not equal to [t0, t0+ts] then the default control is applied over the remaining interval. 
  t_i - initial time for application of the control.  t0 <= t_i <= t0+ts
  t_f - final time for control application.  t0 <= t_f <= t0+ts

  WARNING: iSACstep.u_switch is only applied when t_f-t_i > 0, otherwise u_default is applied.
  WARNING: If [t_i, t_f] is not equal to [t0, t0+ts] then u_default is applied 
           over the remaining interval.
  NOTE: for speed return and input types should be changed and passed as
        references / pointers
*/


int main(int /* argc */ , char** /* argv */ )
{
	using namespace std;

	/*********************************************/
	/* Vars etc*/
	isac_step iSACstep;//instance 
	state_type x0(xlen);
	Eigen::Matrix< double, xlen, 1 > xnext;//for prints
	int i;
	ofstream myfile;
  	myfile.open ("./data/states.csv");//open file to save stuff

	/*********************************************/
	/* Initializations*/
	//state	
	for (i=0; i < xlen; ++i) { x0[i] = x_init[i]; }

	//cost weights
	initialize_cost_weights();
	
	State2Mat( x0, xnext );
	myfile << t_init << " " << xnext.transpose() << "\n";//write to file

	clock_t begin = clock();//for timing

	/*********************************************/
	/* Receding horizon loop*/
	for (double t0 = t_init; t0 < t_final; t0 = t0 + ts)
	{
		if(x0[4]<0.5) std::cout << "Robot fell\n";
		
		/* Perform SAC iteration - updates: J0, Jn, u, x_intp */
		iSACstep( t0, x0 );

		//update state
		for (i=0; i < xlen; ++i) { x0[i] = iSACstep.xnext[i]; }	

		//Prints
		if(((t0+ts) > iSACstep.t_imp) && (iSACstep.t_imp < t_final)) { //store impact point
			State2Mat( iSACstep.x_imp, xnext ); // convert state to matrix form to be able to print state directly
			myfile << iSACstep.t_imp << " " << xnext.transpose() << "\n";//write to file
			cout << iSACstep.t_imp << "\n";
		}
		impacts::tm = t_final; //reset for next iterations
		State2Mat( iSACstep.xnext, xnext ); // convert state to matrix form to be able to print state directly
		myfile << t0+ts << " " << xnext.transpose() << " " << iSACstep.Jn <<"\n";//write to file
		//cout << t0 << ", " << iSACstep.xnext[0] << ", " << iSACstep.xnext[2] << "\n";
		cout << t0+ts << "\n";
		//system("pause");
	}

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Elapsed time (s): " << elapsed_secs << "\n";//print elapsed time


	myfile.close();//close file



	return 0;
}






