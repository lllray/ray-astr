#include "include/Planner.hpp"

State getStartState();
State getTargetState();

int main(){

	Map map;
	State start=getStartState();
	State target=getTargetState();

	Planner astar;
	
	astar.plan(start, target, map);
}

State getStartState()
{
	//to do: read from yml file

	//return State(700, 100, 126);
    return State(600, 600, 18);
}

State getTargetState()
{
	//to do: read from yml file
	//return State(100, 600, 18);
    return State(700, 200, 72);
}