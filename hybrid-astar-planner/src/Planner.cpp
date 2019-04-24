#include "../include/Planner.hpp"


State previous[GX][GY][Theta];

void Planner::plan(State start, State target, Map map){

	//initialize variables for the Compare class
	Compare::target=target;
	Compare::obs_map=map.obs_map;
	Compare::grid_obs_map=new int*[DX];
	for(int i=0;i<DX;i++)
	{
		Compare::grid_obs_map[i]=new int[DY];
		for(int j=0;j<DY;j++)
			Compare::grid_obs_map[i][j]=0;
	}
	for(int i=0;i<MAPX;i++)
		for(int j=0;j<MAPY;j++)
		{
			if(Compare::obs_map[i][j])
				Compare::grid_obs_map[i*DX/MAPX][j*DY/MAPY]=1;
		}
	Compare cmp;
	cmp.runDijkstra();


	map.initCollisionChecker();
	map.find_near_obs();
	priority_queue<State, vector<State>, Compare> pq;
	start.cost3d=0;
	pq.push(start);
	GUI display(800, 800);
	display.drawObs(map);
	display.drawCar(start);
	display.drawCar(target);


    int current_node_id=0;

	int vis[GX][GY][Theta];
	memset(vis, 0, sizeof(int)*GX*GY*Theta);

	int iter=0;

	while(pq.size()>0)
	{
        current_node_id++;
        //cout<<current_node_id<<endl;
		State current=pq.top();
		pq.pop();

		if(abs(current.gx-target.gx)<=1 && abs(current.gy-target.gy)<=1 && abs(current.gtheta-target.gtheta)<=5){
			cout<<"Reached target."<<endl;

			State Dummy;
			current.change=PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(current))/(float)(map.obs_dist_max-1)+
						   fabs(current.theta)/BOT_M_ALPHA+1; 
				
			while(current.x!=start.x || current.y!=start.y || current.theta!=start.theta){
				current.velocity=VELOCITY_MAX/current.change;
				display.drawCar(current);
			        display.show(2000/current.velocity);//This can be removed while executing the algo
				Dummy=previous[current.gx][current.gy][current.gtheta];
				//reback with change function
				Dummy.change=PRIORITY_MOVEMENT*fabs(Dummy.theta-current.theta)/(2.0*BOT_M_ALPHA)+
					     PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(Dummy))/(float)(map.obs_dist_max-1)+
					     fabs(Dummy.theta)/BOT_M_ALPHA+1;
				current=Dummy;
			}
			break;
		}

		if(vis[current.gx][current.gy][current.gtheta]){
			continue;
		}

		vis[current.gx][current.gy][current.gtheta]=1;

		vector<State> next=current.getNextStates();
		//while(!getchar());
		//cout<<"ok"<<endl;
		float distance_to_start=sqrt(pow(start.x-current.x,2)+pow(start.y-current.y,2));
		//cout<<"run here 0"<<endl;
		//cout<<next.size()<<endl;
		for(int i=0;i<next.size();i++){
			//cout<<"run here 1"<<endl;
			//display.drawCar(next[i]);
			if(!map.checkCollision(next[i])){

				//cout<<"run here 2"<<endl;
				if(!vis[next[i].gx][next[i].gy][next[i].gtheta]){
					//cout<<"run here 3"<<endl;
					//display.drawCar(next[i]);
					current.next=&(next[i]);
					next[i].previous=&(current);

					if(i==1)
						next[i].cost3d=current.cost3d+5;
					else if(i<=2){
						//if (distance_to_start < 50)continue;
						next[i].cost3d = current.cost3d + 7;
					}
					else {
						if(distance_to_start<100)continue;
					    //int k_turn=(int)(100*(100.0/(distance_to_start+1)));
                        //cout<<k_turn<<endl;
                        next[i].cost3d = current.cost3d + 10;
                    }
					//next[i].cost3d=current.cost3d+1;
					pq.push(next[i]);

					previous[next[i].gx][next[i].gy][next[i].gtheta]=current;
				}
			}
		}
	}
	cout<<"Done."<<endl;
	display.show(0);

	return;
}
