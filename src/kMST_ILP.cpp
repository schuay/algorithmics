#include "kMST_ILP.h"

#include <assert.h>

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
	IloBoolVarArray vs;
	IloIntVarArray us;
};

class SCFVariables : public Variables
{
public:
	~SCFVariables();
	void print(IloCplex &cplex);

	IloBoolVarArray xs;
	IloBoolVarArray vs;
	IloIntVarArray fs;
};

class MCFVariables : public Variables
{
public:
	~MCFVariables();
	void print(IloCplex &cplex);

	IloBoolVarArray xs;
	IloBoolVarArray vs;
	vector<IloIntVarArray> fss;
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

		// vars->print(cplex);
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
	cplex.setParam( IloCplex::Threads, 0 );
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

Variables *kMST_ILP::modelSCF()
{
	SCFVariables *v = new SCFVariables();

	const vector<Instance::Edge> edges = directed_edges(instance.edges);
	const u_int n_edges = edges.size();

	/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active.
	 * $f_{ij} \in [0, k - 1]$ variables denote the number of goods on edge (i, j). */
	v->xs = IloBoolVarArray(env, n_edges);
	v->fs = IloIntVarArray(env, n_edges);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		v->xs[k] = IloBoolVar(env, Tools::indicesToString("x", i, j).c_str());
		v->fs[k] = IloIntVar(env, 0, k - 1, Tools::indicesToString("f", i, j).c_str());
	}

	/* $v_i \in \{0, 1\}$ variables denote whether node i is active. */
	v->vs = IloBoolVarArray(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->vs[i] = IloBoolVar(env, Tools::indicesToString("v", i).c_str());
	}

	/* $\sum_{i, j > 0} x_{ij} = k - 1$. There are exactly k - 1 edges not
	 * counting edges from the artificial root node 0. 
	 * $\sum_j x_{0j} = 1$. Exactly one node is chosen as the tree root. 
	 * $\sum_i x_{i0} = 0$. No edge leads back to the artificial root node 0. */
	IloExpr e_num_edges(env);
	IloExpr e_single_root(env);
	IloExpr e_avoid_v0(env);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		if (i == 0) {
			e_single_root += v->xs[k];
		} else if (j == 0) {
			e_avoid_v0 += v->xs[k];
		} else {
			e_num_edges += v->xs[k];
		}
	}
	model.add(e_num_edges == k - 1);
	model.add(e_single_root == 1);
	model.add(e_avoid_v0 == 0);
	e_num_edges.end();
	e_single_root.end();
	e_avoid_v0.end();

	/* $\forall i: (k - 1)v_i \geq \sum_j (x_{ij})$. Inactive nodes have no outgoing active edges,
	 * active ones at most k - 1. TODO: A tighter bound is to take the sum of incoming goods - 1.
	 * $\forall i:  v_i \leq \sum_j (x_{ij} + x{ji})$. Active nodes have at least one active edge.
	 * $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active.
	 * $\forall j>0: \sum_i x_{ij} = v_j$. Exactly one incoming edge for an
	 *  active node and none for an inactive node (omitting artificial root). */

	IloExprArray e_in_degree(env, instance.n_nodes);
	IloExprArray e_out_degree(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		e_in_degree[i] = IloExpr(env);
		e_out_degree[i] = IloExpr(env);
	}

	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		e_out_degree[i] += v->xs[k];
		e_in_degree[j] += v->xs[k];
	}

	for (u_int i = 0; i < instance.n_nodes; i++) {
		model.add(v->vs[i] * (k - 1) >= e_out_degree[i]);
		model.add(v->vs[i] <= e_out_degree[i] + e_in_degree[i]); 
		if (i == 0){
			//do not add a constraint for in-degree of artificial root node (that's handled elsewhere)
			//model.add(e_in_degree[i] == 0);
		} else if (i > 0) {
			model.add(e_in_degree[i] == v->vs[i]);
		}
		e_in_degree[i].end();
		e_out_degree[i].end();
	}

	IloExpr e_num_nodes(env);
	for (u_int i = 1; i < instance.n_nodes; i++) {
		e_num_nodes += v->vs[i];
	}
	model.add(k == e_num_nodes);
	e_num_nodes.end();

	/* $\forall i, j \neq 0: f_{ij} \leq kx_{ij}$. Only active edges transport goods.
	 * TODO: bound sum of incoming goods per node.
	 * $\forall i, j s.t. i or j is 0: f_{ij} = kx_{ij}$. Only a single edge
	 * incident on the artificial root transports goods. */

	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		IloExpr e_active_goods(env);
		e_active_goods = v->fs[k] - v->xs[k] * this->k;
		if (i == 0 || j == 0) {
			model.add(e_active_goods == 0);
		} else {
			model.add(e_active_goods <= 0);
		}
	}

	/* $\forall j: \sum_i f_{ij} - \sum_k f_{jk} = v_i$.
	 * Each active node consumes one unit. */

	for (u_int x = 1; x < instance.n_nodes; x++) {
		IloExpr e_node_consumption(env);
		for (const u_int ind : instance.incidentEdges[x]) {
			const u_int i = edges[ind].v1;
			const u_int j = edges[ind].v2;

			if (i == x) {
				e_node_consumption += v->fs[ind + instance.n_edges];
				e_node_consumption -= v->fs[ind];
			} else if (j == x) {
				e_node_consumption += v->fs[ind];
				e_node_consumption -= v->fs[ind + instance.n_edges];
			} else {
				assert(false);
			}
		}
		e_node_consumption -= v->vs[x];
		model.add(e_node_consumption == 0);
	}

	/* $\sum_{i, j} c_{ij} x_{ij}$ is our minimization function. */
	IloExpr e_objective(env);
	for (u_int k = 0; k < n_edges; k++) {
		e_objective += v->xs[k] * edges[k].weight;
	}
	model.add(IloMinimize(env, e_objective));
	e_objective.end();

	return v;
}

Variables *kMST_ILP::modelMCF()
{
	MCFVariables *v = new MCFVariables();

	const vector<Instance::Edge> edges = directed_edges(instance.edges);
	const u_int n_edges = edges.size();

	/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active.
	 * $f^k_{ij} \in \{0, 1\}$ variables denote the flow on edge (i, j) for commodity k. */
	v->xs = IloBoolVarArray(env, n_edges);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->fss.push_back(IloBoolVarArray(env, n_edges));
	}
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		v->xs[k] = IloBoolVar(env, Tools::indicesToString("x", i, j).c_str());
		for (u_int l = 0; l < instance.n_nodes; l++) {
			v->fss[l][k] = IloBoolVar(env, Tools::indicesToString("f", l, i, j).c_str());
		}
	}

	/* $v_i \in \{0, 1\}$ variables denote whether node i is active. */
	v->vs = IloBoolVarArray(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->vs[i] = IloBoolVar(env, Tools::indicesToString("v", i).c_str());
	}

	/* $\sum_{i, j > 0} x_{ij} = k - 1$. There are exactly k - 1 edges not
	 * counting edges from the artificial root node 0. 
	 * $\sum_j x_{0j} = 1$. Exactly one node is chosen as the tree root. 
	 * $\sum_i x_{i0} = 0$. No edge leads back to the artificial root node 0. */
	IloExpr e_num_edges(env);
	IloExpr e_single_root(env);
	IloExpr e_avoid_v0(env);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		if (i == 0) {
			e_single_root += v->xs[k];
		} else if (j == 0) {
			e_avoid_v0 += v->xs[k];
		} else {
			e_num_edges += v->xs[k];
		}
	}
	model.add(e_num_edges == k - 1);
	model.add(e_single_root == 1);
	model.add(e_avoid_v0 == 0);
	e_num_edges.end();
	e_single_root.end();
	e_avoid_v0.end();

	/* $\forall i: (k - 1)v_i \geq \sum_j (x_{ij})$. Inactive nodes have no outgoing active edges,
	 * active ones at most k - 1. TODO: A tighter bound is to take the sum of incoming goods - 1.
	 * $\forall i:  v_i \leq \sum_j (x_{ij} + x{ji})$. Active nodes have at least one active edge.
	 * $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active.
	 * $\forall j>0: \sum_i x_{ij} = v_j$. Exactly one incoming edge for an
	 *  active node and none for an inactive node (omitting artificial root). */

	IloExprArray e_in_degree(env, instance.n_nodes);
	IloExprArray e_out_degree(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		e_in_degree[i] = IloExpr(env);
		e_out_degree[i] = IloExpr(env);
	}

	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		e_out_degree[i] += v->xs[k];
		e_in_degree[j] += v->xs[k];
	}

	for (u_int i = 0; i < instance.n_nodes; i++) {
		model.add(v->vs[i] * (k - 1) >= e_out_degree[i]);
		model.add(v->vs[i] <= e_out_degree[i] + e_in_degree[i]); 
		if (i == 0){
			//do not add a constraint for in-degree of artificial root node (that's handled elsewhere)
			//model.add(e_in_degree[i] == 0);
		} else if (i > 0) {
			model.add(e_in_degree[i] == v->vs[i]);
		}
		e_in_degree[i].end();
		e_out_degree[i].end();
	}

	IloExpr e_num_nodes(env);
	for (u_int i = 1; i < instance.n_nodes; i++) {
		e_num_nodes += v->vs[i];
	}
	model.add(k == e_num_nodes);
	e_num_nodes.end();

	return v;
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

	/* $u_i \in [0, k]$ variables are used to impose an order on nodes.
	 * $v_i \in \{0, 1\}$ variables denote whether node i is active. */
	v->us = IloIntVarArray(env, instance.n_nodes);
	v->vs = IloBoolVarArray(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->us[i] = IloIntVar(env, 0, k, Tools::indicesToString("u", i).c_str());
		v->vs[i] = IloBoolVar(env, Tools::indicesToString("v", i).c_str());
	}

	/* $\sum_{i, j > 0} x_{ij} = k - 1$. There are exactly k - 1 edges not
	 * counting edges from the artificial root node 0. 
	 * $\sum_j x_{0j} = 1$. Exactly one node is chosen as the tree root. 
	 * $\sum_i x_{i0} = 0$. No edge leads back to the artificial root node 0. */
	IloExpr e_num_edges(env);
	IloExpr e_single_root(env);
	IloExpr e_avoid_v0(env);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		if (i == 0) {
			e_single_root += v->xs[k];
		} else if (j == 0) {
			e_avoid_v0 += v->xs[k];
		} else {
			e_num_edges += v->xs[k];
		}
	}
	model.add(e_num_edges == k - 1);
	model.add(e_single_root == 1);
	model.add(e_avoid_v0 == 0);
	e_num_edges.end();
	e_single_root.end();
	e_avoid_v0.end();

	/* $u_0 = 0$. Set level of artificial root 0 to 0. */
	IloExpr e3(env);
	e3 += v->us[0];
	model.add(e3 == 0);
	e3.end();

	/* $\forall i, j: u_i + x_{ij} \leq u_j + (1 - x_{ij})k$.
	 * Enforce order hierarchy on nodes. */
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		IloExpr e4(env);
		e4 = v->us[i] + v->xs[k] - v->us[j] - (-v->xs[k] + 1) * this->k;
		model.add(e4 <= 0);
		e4.end();
	}

	/* $\forall i: u_i <= nv_i$ force order of inactive nodes to 0 */
	for (u_int i = 0; i < instance.n_nodes; i++) {
		model.add(v->us[i] <= v->vs[i] * (int) instance.n_nodes);
	}	

	/* $\forall i: (k - 1)v_i \geq \sum_j (x_{ij})$. Inactive nodes have no outgoing active edges,
	 * active ones at most k - 1.
	 * $\forall i:  v_i \leq \sum_j (x_{ij} + x{ji})$. Active nodes have at least one active edge.
	 * $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active.
	 * $\forall j>0: \sum_i x_{ij} = v_j$. Exactly one incoming edge for an
	 *  active node and none for an inactive node (omitting artificial root). */

	IloExprArray e_in_degree(env, instance.n_nodes);
	IloExprArray e_out_degree(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		e_in_degree[i] = IloExpr(env);
		e_out_degree[i] = IloExpr(env);
	}

	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		e_out_degree[i] += v->xs[k];
		e_in_degree[j] += v->xs[k];
	}

	for (u_int i = 0; i < instance.n_nodes; i++) {
		//model.add(v->vs[i] * k - v->us[i] >= e_out_degree[i] ); //strange: performs better with small k, worse with higher k
		model.add(v->vs[i] * (k - 1) >= e_out_degree[i] ); //more stable than the above
		model.add(v->vs[i] <= e_out_degree[i] + e_in_degree[i]); 
		if (i == 0){
			//do not add a constraint for in-degree of artificial root node (that's handled elsewhere)
			//model.add(e_in_degree[i] == 0);
		} else if (i > 0) {
			model.add(e_in_degree[i] == v->vs[i]);
		}
		e_in_degree[i].end();
		e_out_degree[i].end();
	}

	IloExpr e_num_nodes(env);
	for (u_int i = 1; i < instance.n_nodes; i++) {
		e_num_nodes += v->vs[i];
	}
	model.add(k == e_num_nodes);
	e_num_nodes.end();

	/* $min \sum_{i, j} c_{ij} x_{ij}$ is our objective function. */
	IloExpr e_objective(env);
	for (u_int k = 0; k < n_edges; k++) {
		e_objective += v->xs[k] * edges[k].weight;
	}
	model.add(IloMinimize(env, e_objective));
	e_objective.end();

	return v;
}

kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	cplex.end();
	model.end();
	env.end();
}

static void print_values(IloCplex &cplex, const IloIntVarArray *xs)
{
	for (u_int i = 0; i < xs->getSize(); i++) {
		const int v = cplex.getValue((*xs)[i]);
		if (v == 0) {
			continue;
		}
		cout << (*xs)[i] << " = " << v << endl;
	}
}

MTZVariables::~MTZVariables()
{
	xs.end();
	vs.end();
	us.end();
}

void MTZVariables::print(IloCplex &cplex)
{
	print_values(cplex, &xs);
	print_values(cplex, &vs);
	print_values(cplex, &us);
}

SCFVariables::~SCFVariables()
{
	xs.end();
	vs.end();
	fs.end();
}

void SCFVariables::print(IloCplex &cplex)
{
	print_values(cplex, &xs);
	print_values(cplex, &vs);
	print_values(cplex, &fs);
}

MCFVariables::~MCFVariables()
{
	xs.end();
	vs.end();

	for (auto &fs : fss) {
		fs.end();
	}
}

void MCFVariables::print(IloCplex &cplex)
{
	print_values(cplex, &xs);
	print_values(cplex, &vs);

	for (auto &fs : fss) {
		print_values(cplex, &fs);
	}
}

/* vim: set noet ts=4 sw=4: */
