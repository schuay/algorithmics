#ifndef __K_MST_ILP__H__
#define __K_MST_ILP__H__

#include <ilcplex/ilocplex.h>

#include "Instance.h"
#include "Tools.h"

using namespace std;

ILOSTLBEGIN

class Variables;

class kMST_ILP
{

private:

	// input data
	Instance& instance;
	string model_type;
	int k;
	// number of edges and nodes including root node and root edges
	unsigned int m, n;

	IloEnv env;
	IloModel model;
	IloCplex cplex;

	Variables *modelSCF();
	Variables *modelMCF();
	Variables *modelMTZ();

public:

	kMST_ILP( Instance& _instance, string _model_type, int _k );
	~kMST_ILP();
	void solve();

private:

	void setCPLEXParameters();

};
// kMST_ILP

#endif //__K_MST_ILP__H__
/* vim: set noet ts=4 sw=4: */
