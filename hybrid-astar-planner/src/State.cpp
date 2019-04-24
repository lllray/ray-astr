#include "../include/State.hpp"

State::State(float x, float y, float theta){
	this->x=x;
	this->y=y;
	this->theta=theta;

	this->gx=x/Grid_Res;
	this->gy=y/Grid_Res;
	this->gtheta=theta+0.01;
}

State::State(){
	this->x=-1;
	this->y=-1;
	this->theta=-1;
}

vector<State> State::getNextStates(){

	vector<State> next;
	State n;
	float alpha,beta, r, d=BOT_MOVE_DISTANCE;

	for(alpha=-BOT_M_ALPHA; alpha<=BOT_M_ALPHA+0.001; alpha+=BOT_M_ALPHA){ //three direction -30 0 30
		beta=d*tan(alpha*PI/180)/BOT_L;

		if(abs(beta)<0.001){//if direction is not change
			n.x=x+d*cos(theta*2.0*PI/Theta);
			n.y=y+d*sin(theta*2.0*PI/Theta);
			n.theta=theta;
		}
		else{//if direction is change
			r=BOT_L/tan(alpha*PI/180);
			n.x=x+r*sin(theta*2.0*PI/Theta+beta)-r*sin(theta*2.0*PI/Theta);
			n.y=y-r*cos(theta*2.0*PI/Theta+beta)+r*cos(theta*2.0*PI/Theta);
			if(theta + beta*180/PI/Theta_Res>0)
				n.theta=fmod(theta + beta*180/PI/Theta_Res,Theta);
			else
				n.theta=theta + beta*180/PI/Theta_Res+Theta;
		}
		n.gx=n.x/Grid_Res;
		n.gy=n.y/Grid_Res;
		n.gtheta=n.theta+0.01;
		next.push_back(n);
	}
//LX ADD 0424 #add the three next when the car just turn but not move
    for(alpha=-BOT_M_ALPHA; alpha<=BOT_M_ALPHA+0.001; alpha+=2*BOT_M_ALPHA){ //three direction -30 0 30
        beta=d*tan(alpha*PI/180)/BOT_L;
            n.x=x;
            n.y=y;
            if(theta + beta*180/PI/Theta_Res>0)
                n.theta=fmod(theta + beta*180/PI/Theta_Res,Theta);
            else
                n.theta=theta + beta*180/PI/Theta_Res+Theta;
        n.gx=n.x/Grid_Res;
        n.gy=n.y/Grid_Res;
        n.gtheta=n.theta+0.01;
        next.push_back(n);
    }
//END 0424
	// cout<<"getNextStates() called from "<<x<<","<<y<<","<<theta<<endl;
	// for(int i=0;i<3;i++) cout<<next[i].x<<","<<next[i].y<<","<<next[i].theta<<"  "; cout<<endl;

	return next;
}
