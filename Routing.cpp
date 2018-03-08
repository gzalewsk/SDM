#include "StdAfx.h"
#include "Routing.h"

#include <queue>

Routing::Routing(SimulationData &simData) {

	this->simData = simData;
}

Routing::~Routing(void) {}

vector<int> Routing::ShortestPath(int s, int t, vector<double> linkWeight, double limit) {
  
	int N = simData.net.nodes;

	vector<int> prev(N, -1);
	vector<double> bestCost(N, 1e20);

	priority_queue<NodeState> pq;
	pq.push(NodeState(s, 0.0));
	bestCost[s] = 0.0;
	
	while(!pq.empty()) {

	   NodeState curNode=pq.top();
	   pq.pop();

	   if(curNode.node == t)
		   break;

	   if(curNode.cost > bestCost[curNode.node])
		   continue;
	   
	   for(int i = 0; i < simData.net.neigh[curNode.node].size(); i++) {
		   
		   // noExpensiveLinks
		   if (linkWeight[simData.net.neigh[curNode.node][i]] < 0)
			   continue;
		   // noExpensiveLinks end
		   
		   int newNode = simData.net.dijkstraNeigh[curNode.node][i];
		   double newCost = curNode.cost + linkWeight[simData.net.neigh[curNode.node][i]];
		   if(newCost >= bestCost[newNode] || newCost >= limit)
			   continue;
		   
		   bestCost[newNode] = newCost;
		   pq.push(NodeState(newNode, newCost));
		   prev[newNode] = curNode.node;
	   }
	}
	
	if(prev[t] == -1)
		return vector<int>();
	
	vector<int> res;
	while(t != s) {
		res.push_back(simData.net.linkIndex[prev[t]][t]);
		t = prev[t];
	}
	return res;
}
