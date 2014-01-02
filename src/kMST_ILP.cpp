#include "kMST_ILP.h"

class Variables
{
public:
	virtual ~Variables() { }
	virtual void print(IloCplex &cplex) = 0;
};

class MTZVariables : public Variables
{
public:
	~MTZVariables();
	void print(IloCplex &cplex);

	IloBoolVarArray xs;
	IloIntVarArray us;
};

kMST_ILP::kMST_ILP( Instance& _instance, string _model_type, int _k ) :
	instance( _instance ), model_type( _model_type ), k( _k )
{
	n = instance.n_nodes;
	m = instance.n_edges;
	if( k == 0 ) k = n;
}

void kMST_ILP::solve()
{
	try {
		// initialize CPLEX
		env = IloEnv();
		model = IloModel( env );

		Variables *vars;

		// add model-specific constraints
		if( model_type == "scf" ) vars = modelSCF();
		else if( model_type == "mcf" ) vars = modelMCF();
		else if( model_type == "mtz" ) vars = modelMTZ();
		else {
			cerr << "No existing model chosen\n";
			exit( -1 );
		}

		// build model
		cplex = IloCplex( model );
		// export model to a text file
		//cplex.exportModel( "model.lp" );
		// set parameters
		setCPLEXParameters();

		// solve model
		cout << "Calling CPLEX solve ...\n";
		cplex.solve();
		cout << "CPLEX finished.\n\n";
		cout << "CPLEX status: " << cplex.getStatus() << "\n";
		cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << "\n";
		cout << "Objective value: " << cplex.getObjValue() << "\n";
		cout << "CPU time: " << Tools::CPUtime() << "\n\n";

		vars->print(cplex);
		delete vars;
	}
	catch( IloException& e ) {
		cerr << "kMST_ILP: exception " << e << "\n";
		exit( -1 );
	}
	catch( ... ) {
		cerr << "kMST_ILP: unknown exception.\n";
		exit( -1 );
	}
}

// ----- private methods -----------------------------------------------

void kMST_ILP::setCPLEXParameters()
{
	// print every x-th line of node-log and give more details
	cplex.setParam( IloCplex::MIPInterval, 1 );
	cplex.setParam( IloCplex::MIPDisplay, 2 );
	// only use a single thread
	cplex.setParam( IloCplex::Threads, 1 );
}

Variables *kMST_ILP::modelSCF()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build single commodity flow model
	// ++++++++++++++++++++++++++++++++++++++++++
	return NULL;
}

Variables *kMST_ILP::modelMCF()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build multi commodity flow model
	// ++++++++++++++++++++++++++++++++++++++++++
	return NULL;
}

/* Turns the given edge vector into a vector containing both the original
 * edges and their reverse directions. */
static vector<Instance::Edge> directed_edges(const vector<Instance::Edge> &es)
{
	vector<Instance::Edge> des;
	des.resize(es.size() * 2);

	auto it = copy(es.cbegin(), es.cend(), des.begin());
	transform(es.cbegin(), es.cend(), it, [](const Instance::Edge &e) {
			Instance::Edge f = {e.v2, e.v1, e.weight}; return f; });

	return des;
}

Variables *kMST_ILP::modelMTZ()
{
	MTZVariables *v = new MTZVariables();

	const vector<Instance::Edge> edges = directed_edges(instance.edges);
	const u_int n_edges = edges.size();

	/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active. */
	v->xs = IloBoolVarArray(env, n_edges);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		v->xs[k] = IloBoolVar(env, Tools::indicesToString("x", i, j).c_str());
	}

	/* $u_i \in [0, k]$ variables are used to impose an order on nodes. */
	v->us = IloIntVarArray(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->us[i] = IloIntVar(env, 0, k, Tools::indicesToString("u", i).c_str());
	}

	/* $\sum_{i, j > 0} x_{ij} = k - 1$. There are exactly k - 1 edges not
	 * counting edges from the artificial root node 0. 
	 * $\sum_j x_{0j} = 1$. Exactly one node is chosen as the tree root. 
	 * $\sum_i x_{i0} = 0$. No edge leads back to the artificial root node 0. */
	IloExpr e0(env);
	IloExpr e1(env);
	IloExpr e2(env);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		if (i == 0) {
			e1 += v->xs[k];
		} else if (j == 0) {
			e2 += v->xs[k];
		} else {
			e0 += v->xs[k];
		}
	}
	model.add(e0 == k - 1);
	model.add(e1 == 1);
	model.add(e2 == 0);
	e0.end();
	e1.end();
	e2.end();

	/* $u_0 = 0$. Set level of artificial root 0 to 0. */
	IloExpr e3(env);
	e3 += v->us[0];
	model.add(e3 == 0);
	e3.end();

	/* $\forall i, j: u_i + x_{ij} \leq u_j + (1 - x_{ij})k$.
	 * Enforce order hierarchy on nodes. Note the v->usage of instance.n_edges here. */
	for (u_int k = 0; k < instance.n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		IloExpr e4(env);
		e4 = v->us[i] + v->xs[k] - v->us[j] - (-v->xs[k + instance.n_edges] + 1) * this->k;
		model.add(e4 <= 0);
		e4.end();
	}

	/* $\sum_{i, j} c_{ij} x_{ij}$ is our minimization function. */
	IloExpr e5(env);
	for (u_int k = 0; k < n_edges; k++) {
		e5 += v->xs[k] * edges[k].weight;
	}
	model.add(IloMinimize(env, e5));
	e5.end();

	return v;
}

kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	cplex.end();
	model.end();
	env.end();
}

MTZVariables::~MTZVariables()
{
	xs.end();
	us.end();
}

void MTZVariables::print(IloCplex &cplex)
{
	for (u_int i = 0; i < xs.getSize(); i++) {
		const int v = cplex.getValue(xs[i]);
		if (v == 0) {
			continue;
		}
		cout << xs[i] << " = " << v << endl;
	}

	for (u_int i = 0; i < us.getSize(); i++) {
		const int v = cplex.getValue(us[i]);
		if (v == 0) {
			continue;
		}
		cout << us[i] << " = " << v << endl;
	}
}

/* vim: set noet ts=4 sw=4: */
