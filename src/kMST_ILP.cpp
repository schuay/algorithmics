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

/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active. */
static IloBoolVarArray createVarArrayXs(IloEnv env, vector<Instance::Edge> edges, u_int n_edges)
{
	IloBoolVarArray xs = IloBoolVarArray(env, n_edges);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		xs[k] = IloBoolVar(env, Tools::indicesToString("x", i, j).c_str());
	}
	return xs;
}


/* $f_{ij} \in [0, k]$ variables denote the number of goods on edge (i, j). */
static IloIntVarArray createVarArrayFs(IloEnv env,vector<Instance::Edge> edges, u_int n_edges)

{
	IloIntVarArray fs = IloIntVarArray(env, n_edges);
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		fs[k] = IloIntVar(env, 0, k, Tools::indicesToString("f", i, j).c_str());
	}
	return fs;
}


/* 
 * $v_i \in \{0, 1\}$ variables denote whether node i is active. 
 */
static IloBoolVarArray createVarArrayVs(IloEnv env, u_int n_nodes)
{
	IloBoolVarArray vs = IloBoolVarArray(env, n_nodes);
	for (u_int i = 0; i < n_nodes; i++) {
		vs[i] = IloBoolVar(env, Tools::indicesToString("v", i).c_str());
	}
	return vs;
}


/**
 * Objective function:
 * $\sum_{i, j} c_{ij} x_{ij}$ 
 */ 
static void addObjectiveFunction(IloEnv env, IloModel model, IloBoolVarArray xs, vector<Instance::Edge> edges, u_int n_edges)
{
	IloExpr e_objective(env);
	for (u_int m = 0; m < n_edges; m++) {
		e_objective += xs[m] * edges[m].weight;
	}
	model.add(IloMinimize(env, e_objective));
	e_objective.end();
}

/* 
 * $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active. 
 */
static void addConstraint_k_nodes_active(IloEnv env, IloModel model, IloBoolVarArray vs, Instance& instance, u_int k)
{
	IloExpr e_num_nodes(env);
	for (u_int i = 1; i < instance.n_nodes; i++) {
		e_num_nodes += vs[i];
	}
	model.add(k == e_num_nodes);
	e_num_nodes.end();
}

/* 
 * There are exactly k - 1 arcs not counting edges from the artificial root node 0.
 * $\sum_{i, j > 0} x_{ij} = k - 1$.  
 */
static void addConstraint_k_minus_one_active_edges(IloEnv env, IloModel model, IloBoolVarArray xs, vector<Instance::Edge> edges, u_int n_edges, u_int k)
{
	IloExpr e_num_edges(env);
	for (u_int m = 0; m < n_edges; m++) {
		const u_int i = edges[m].v1;
		const u_int j = edges[m].v2;
		if (i > 0 && j > 0) {
			e_num_edges += xs[m];
		}
	}
	model.add(e_num_edges == k - 1);
	e_num_edges.end();
}

/* 
 * Exactly one node is chosen as the tree root. 
 * $\sum_j x_{0j} = 1$. 
 */
static void addConstraint_one_active_outgoing_arc_for_node_zero(IloEnv env, IloModel model, IloBoolVarArray xs, vector<Instance::Edge> edges, u_int n_edges)
{
	IloExpr e_single_root(env);
	for (u_int m = 0; m < n_edges; m++) {
		const u_int i = edges[m].v1;
		if (i == 0) {
			e_single_root += xs[m];
		}
	}
	model.add(e_single_root == 1);
	e_single_root.end();
}

/* 
 * No arc leads back to the artificial root node 0. 
 * $\sum_i x_{i0} = 0$. 
 */
static void addConstraint_no_active_incoming_arc_for_node_zero(IloEnv env, IloModel model, IloBoolVarArray xs, vector<Instance::Edge> edges, u_int n_edges)
{
	IloExpr e_single_root(env);
	for (u_int m = 0; m < n_edges; m++) {
		const u_int j = edges[m].v2;
		if (j == 0) {
			e_single_root += xs[m];
		}
	}
	model.add(e_single_root == 0);
	e_single_root.end();
}

/**
 * Inactive nodes have no outgoing active arcs, active ones at most k - 1. 
 * TODO: A tighter bound is to take the sum of incoming goods - 1.
 * $\forall i: (k - 1)v_i \geq \sum_j (x_{ij})$. 
 */
static void addConstraint_bound_on_outgoing_arcs(IloModel model, IloBoolVarArray vs, IloExprArray& e_out_degree, Instance& instance, int k)
{
	for (u_int i = 0; i < instance.n_nodes; i++) {
		model.add(vs[i] * (k - 1) >= e_out_degree[i]);
	}
}

/**
 * Active nodes have at least one active arc.
 * $\forall i:  v_i \leq \sum_j (x_{ij} + x{ji})$.
 */
static void addConstraint_active_node_at_least_one_active_arc(IloModel model, IloBoolVarArray vs, IloExprArray& e_in_degree, IloExprArray& e_out_degree, Instance& instance)
{
	for (u_int i = 0; i < instance.n_nodes; i++) {
		model.add(vs[i] <= e_out_degree[i] + e_in_degree[i]); 
	}
}

/**
 * Exactly one incoming edge for an active node and none for an inactive node (omitting artificial root). 
 * $\forall j>0: \sum_i x_{ij} = v_j$. 
 */
static void addConstraint_in_degree_one_for_active_node_zero_for_inactive(IloModel model, IloBoolVarArray vs, IloExprArray& e_in_degree, Instance& instance)
{
	for (u_int i = 1; i < instance.n_nodes; i++) {
		model.add(e_in_degree[i] == vs[i]);
	}
}


/**
 * Create expression for in-degree for each node.
 */  
static IloExprArray createExprArray_in_degree(IloEnv env, vector<Instance::Edge> edges, u_int n_edges, IloBoolVarArray xs,  Instance& instance)
{
	IloExprArray e_in_degree(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		e_in_degree[i] = IloExpr(env);
	}
	for (u_int m = 0; m < n_edges; m++) {
		const u_int j = edges[m].v2;
		e_in_degree[j] += xs[m];
	}
	return e_in_degree;
}

/**
 * Create expression for out-degree for each node.
 */  
static IloExprArray createExprArray_out_degree(IloEnv env, vector<Instance::Edge> edges, u_int n_edges, IloBoolVarArray xs, Instance& instance)
{
	IloExprArray e_out_degree(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		e_out_degree[i] = IloExpr(env);
	}
	for (u_int m = 0; m < n_edges; m++) {
		const u_int i = edges[m].v1;
		e_out_degree[i] += xs[m];
	}
	return e_out_degree;
}


/********************************** SCF specific methods ********************************/

/**
 * Create expression for in-flow for each node.
 */  
static IloExprArray createExprArray_in_flow(IloEnv env, vector<Instance::Edge> edges, u_int n_edges, IloIntVarArray fs,  Instance& instance)
{
	IloExprArray expr(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		expr[i] = IloExpr(env);
	}
	for (u_int m = 0; m < n_edges; m++) {
		const u_int j = edges[m].v2;
		expr[j] += fs[m];
	}
	return expr;
}

/**
 * Create expression for out-flow for each node.
 */
static IloExprArray createExprArray_out_flow(IloEnv env, vector<Instance::Edge> edges, u_int n_edges, IloIntVarArray fs, Instance& instance)
{
	IloExprArray expr(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		expr[i] = IloExpr(env);
	}
	for (u_int m = 0; m < n_edges; m++) {
		const u_int i = edges[m].v1;
		expr[i] += fs[m];
	}
	return expr;
}



Variables *kMST_ILP::modelSCF()
{
	SCFVariables *v = new SCFVariables();

	const vector<Instance::Edge> edges = directed_edges(instance.edges);
	const u_int n_edges = edges.size();

	/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active. */
	v->xs = createVarArrayXs(env, edges, n_edges);

	/* $v_i \in \{0, 1\}$ variables denote whether node i is active. */
	v->vs = createVarArrayVs(env, instance.n_nodes);

	/* add objective function */
	addObjectiveFunction(env, model, v->xs, edges, n_edges);

	/* There are exactly k - 1 edges not counting edges from the artificial root node 0. */
	addConstraint_k_minus_one_active_edges(env,model,v->xs,edges,n_edges,this->k);

    /* Exactly one node is chosen as the tree root. */
	addConstraint_one_active_outgoing_arc_for_node_zero(env,model,v->xs,edges,n_edges);
 
    /* No edge leads back to the artificial root node 0. */
	addConstraint_no_active_incoming_arc_for_node_zero(env,model,v->xs,edges,n_edges);

	IloExprArray e_in_degree = createExprArray_in_degree(env, edges, n_edges, v->xs, instance);
	IloExprArray e_out_degree = createExprArray_out_degree(env, edges, n_edges, v->xs, instance);

	/* Inactive nodes have no outgoing active edges, active ones at most k - 1. TODO: A tighter bound is to take the sum of incoming goods - 1.*/
	addConstraint_bound_on_outgoing_arcs(model,v->vs,e_out_degree,instance,this->k);

	/* Active nodes have at least one active arc.*/
	addConstraint_active_node_at_least_one_active_arc(model,v->vs,e_in_degree, e_out_degree,instance);
	
	/* Exactly one incoming edge for an active node and none for an inactive node (omitting artificial root). */
 	addConstraint_in_degree_one_for_active_node_zero_for_inactive(model,v->vs,e_in_degree,instance);
	
	//note: position matters. Tried worse positions than this one 
	/* $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active. */
	addConstraint_k_nodes_active(env, model, v->vs, instance, this->k);
	e_in_degree.endElements();
	e_out_degree.endElements();


	/* TODO: Missing formulation of constraints in next block. */

	/* $f_{ij} \in [0, k - 1]$ variables denote the number of goods on edge (i, j). */
	v->fs = createVarArrayFs(env, edges, n_edges);

	IloExprArray e_in_flow = createExprArray_in_flow(env, edges, n_edges, v->fs, instance);
	IloExprArray e_out_flow = createExprArray_out_flow(env, edges, n_edges, v->fs, instance);

	for (u_int i = 0; i < instance.n_nodes; i++) {
		if (i == 0){
			/* Don't add a constraint for the artificial root. */
		} else if (i > 0) {
			/* outflow = inflow -1 for active nodes, same for inactive nodes. */
			model.add(v->vs[i] == e_in_flow[i] - e_out_flow[i]); 
		}
	}
	e_in_flow.endElements();
	e_out_flow.endElements();


	/* $\forall i, j \neq 0: f_{ij} \leq kx_{ij}$. Only active edges transport goods.
	 * TODO: bound sum of incoming goods per node.
	 * $\forall i, j s.t. i or j is 0: f_{ij} = kx_{ij}$. Only a single edge
	 * incident on the artificial root transports goods. */

	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		if (i == 0 || j == 0) {
			model.add(v->fs[k] == this->k * v->xs[k]);
		} else {  
			model.add(v->fs[k] <= (this->k) * v->xs[k]);
		}
	}

	return v;
}

Variables *kMST_ILP::modelMCF()
{
	MCFVariables *v = new MCFVariables();

	/***** generic part ***/

	const vector<Instance::Edge> edges = directed_edges(instance.edges);
	const u_int n_edges = edges.size();

	/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active. */
	v->xs = createVarArrayXs(env, edges, n_edges);

	/* $v_i \in \{0, 1\}$ variables denote whether node i is active. */
	v->vs = createVarArrayVs(env, instance.n_nodes);

	/* add objective function */
	addObjectiveFunction(env, model, v->xs, edges, n_edges);

	/* There are exactly k - 1 edges not counting edges from the artificial root node 0. */
	addConstraint_k_minus_one_active_edges(env,model,v->xs,edges,n_edges,this->k);

    /* Exactly one node is chosen as the tree root. */
	addConstraint_one_active_outgoing_arc_for_node_zero(env,model,v->xs,edges,n_edges);
 
    /* No edge leads back to the artificial root node 0. */
	addConstraint_no_active_incoming_arc_for_node_zero(env,model,v->xs,edges,n_edges);

	IloExprArray e_in_degree = createExprArray_in_degree(env, edges, n_edges, v->xs, instance);
	IloExprArray e_out_degree = createExprArray_out_degree(env, edges, n_edges, v->xs, instance);

	/* Inactive nodes have no outgoing active edges, active ones at most k - 1. TODO: A tighter bound is to take the sum of incoming goods - 1.*/
	addConstraint_bound_on_outgoing_arcs(model,v->vs,e_out_degree,instance,this->k);

	/* Active nodes have at least one active arc.*/
	addConstraint_active_node_at_least_one_active_arc(model,v->vs,e_in_degree, e_out_degree,instance);
	
	/* Exactly one incoming edge for an active node and none for an inactive node (omitting artificial root). */
 	addConstraint_in_degree_one_for_active_node_zero_for_inactive(model,v->vs,e_in_degree,instance);
	
	//note: position matters. Tried worse positions than this one 
	/* $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active. */
	addConstraint_k_nodes_active(env, model, v->vs, instance, this->k);
	e_in_degree.endElements();
	e_out_degree.endElements();


    /***** MCF specific part ***/

	/* $f^k_{ij} \in \{0, 1\}$ variables denote the flow on edge (i, j) for commodity k. */
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->fss.push_back(IloBoolVarArray(env, n_edges));
	}
	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;
		for (u_int l = 0; l < (u_int) instance.n_nodes; l++) {
			v->fss[l][k] = IloBoolVar(env, Tools::indicesToString("f", l, i, j).c_str());
			model.add(v->fss[l][k] >= 0);
		}
	}
	
	/* 
     * Each commodity l is generated once by the artificial root node:
	 * $\forall l>0: \sum_j f^l_{0j} == 1$ if node l is active, 0 otherwise
     */
	for (u_int c = 1; c < instance.n_nodes; c++){
		IloExpr e_one_commodity(env);		
		for (u_int m = 0; m < n_edges; m++) {
			const u_int i = edges[m].v1;
			const u_int j = edges[m].v2;
			if (i == 0 && j > 0){
				e_one_commodity += v->fss[c][m];	
			}
		} 
		model.add(e_one_commodity == v->vs[c]);
		e_one_commodity.end();
	}

	/* 
     * The artifical root generates k commodities:
     * $\sum_{l, j} f^l_{0j} = k$. 
     */
    IloExpr e_root_generates_k(env);		
	for (u_int c = 0; c < instance.n_nodes; c++){
		for (u_int m = 0; m < n_edges; m++) {
			const u_int i = edges[m].v1;
			const u_int j = edges[m].v2;
			if (i == 0 && j > 0){
				e_root_generates_k += v->fss[c][m];	
			}
		} 
	}
	model.add(e_root_generates_k == this->k);   
	e_root_generates_k.end();


	/*
     * No commodity is generated for the artificial root:
     * $\forall i, j: f^0_{ij} = 0$. 
     */
    IloExpr e_none_gen_for_root(env);		
	for (u_int m = 0; m < n_edges; m++) {
		e_none_gen_for_root += v->fss[0][m];	
	} 
	model.add(e_none_gen_for_root == 0);
	e_none_gen_for_root.end();

	/* 
	 * Transmitted commodities end up at the target node:
	 * $\forall l>0: \sum_i f^l_{il} = \sum_j f^l_{0j}$. (here: = v_l) 
     */
	for (u_int c = 1; c < (u_int) instance.n_nodes; c++){
		IloExpr e_commodity_reaches_target(env);		
		for (u_int m = 0; m < n_edges; m++) {
			const u_int i = edges[m].v1;
			const u_int j = edges[m].v2;
			if (i != c && j == c){
				e_commodity_reaches_target += v->fss[c][m];	
			}
		} 
		model.add(e_commodity_reaches_target == v->vs[c]);
		e_commodity_reaches_target.end();
	}


	/* 
	 * Once reached, the commodity never leaves the target node:
	 * $\forall l>0: \sum_j f^l_{lj} = 0$.  
     */
	for (u_int c = 1; c < (u_int) instance.n_nodes; c++){
		IloExpr e_commodity_stays_at_target(env);		
		for (u_int m = 0; m < n_edges; m++) {
			const u_int i = edges[m].v1;
			const u_int j = edges[m].v2;
			if (i == c && j != c){
				e_commodity_stays_at_target += v->fss[c][m];	
			}
		} 
		model.add(e_commodity_stays_at_target == 0);
		e_commodity_stays_at_target.end();
	}


	/*
	 * Flow is conserved when not at target node. 
	 * $\forall j, l s.t. j \neq l: \sum_i f^l_{ij} = \sum_i f^l_{ji}$. 
	 */
	 for (u_int c = 0; c < (u_int) instance.n_nodes; c++){
		IloExprArray e_in_flow(env, instance.n_nodes);		
		IloExprArray e_out_flow(env, instance.n_nodes);		
		for (u_int m = 0; m < instance.n_nodes; m++){
			e_in_flow[m] = IloExpr(env);
			e_out_flow[m] = IloExpr(env);
		}
		for (u_int m = 0; m < n_edges; m++) {
			const u_int i = edges[m].v1;
			const u_int j = edges[m].v2;
			e_out_flow[i] += v->fss[c][m];	
			e_in_flow[j] += v->fss[c][m];	
		} 
		for (u_int m = 1; m < instance.n_nodes; m++){
			if (m != c) {
				model.add(e_in_flow[m] == e_out_flow[m]);
			}
		}
		e_in_flow.endElements();
		e_out_flow.endElements();
	 }

	/* 
	 * Commodities may only be transmitted on active edges:
	 * $\forall l, i, j: f^l_{ij} \leq x_{ij}$. 
     */
	for (u_int c = 0; c < (u_int) instance.n_nodes; c++){
		for (u_int m = 0; m < n_edges; m++) {
			model.add(v->fss[c][m] <= v->xs[m]);
		} 
	}

	/* 
	 * For each commodity l , the total flow is <= k if node l is active, 0 otherwise
	 * (works well for all before g05, k=n/2 which is a bit slower with this)
     */
	for (u_int c = 1; c < (u_int) instance.n_nodes; c++){
		IloExpr e_total_flow(env);		
		for (u_int m = 0; m < n_edges; m++) {
			e_total_flow += v->fss[c][m];	
		} 
		model.add(e_total_flow <= this->k * v->vs[c]);
		e_total_flow.end();
	}

	return v;
}

Variables *kMST_ILP::modelMTZ()
{
	MTZVariables *v = new MTZVariables();

    /***** generic part ***/

	const vector<Instance::Edge> edges = directed_edges(instance.edges);
	const u_int n_edges = edges.size();

	/* $x_{ij} \in \{0, 1\}$ variables denote whether edge (i, j) is active. */
	v->xs = createVarArrayXs(env, edges, n_edges);

	/* $v_i \in \{0, 1\}$ variables denote whether node i is active. */
	v->vs = createVarArrayVs(env, instance.n_nodes);

	/* add objective function */
	addObjectiveFunction(env, model, v->xs, edges, n_edges);

	/* There are exactly k - 1 edges not counting edges from the artificial root node 0. */
	addConstraint_k_minus_one_active_edges(env,model,v->xs,edges,n_edges,this->k);

    /* Exactly one node is chosen as the tree root. */
	addConstraint_one_active_outgoing_arc_for_node_zero(env,model,v->xs,edges,n_edges);
 
    /* No edge leads back to the artificial root node 0. */
	addConstraint_no_active_incoming_arc_for_node_zero(env,model,v->xs,edges,n_edges);

	IloExprArray e_in_degree = createExprArray_in_degree(env, edges, n_edges, v->xs, instance);
	IloExprArray e_out_degree = createExprArray_out_degree(env, edges, n_edges, v->xs, instance);

	/* Inactive nodes have no outgoing active edges, active ones at most k - 1. TODO: A tighter bound is to take the sum of incoming goods - 1.*/
	addConstraint_bound_on_outgoing_arcs(model,v->vs,e_out_degree,instance,this->k);

	/* Active nodes have at least one active arc.*/
	addConstraint_active_node_at_least_one_active_arc(model,v->vs,e_in_degree, e_out_degree,instance);
	
	/* Exactly one incoming edge for an active node and none for an inactive node (omitting artificial root). */
 	addConstraint_in_degree_one_for_active_node_zero_for_inactive(model,v->vs,e_in_degree,instance);
	
	//note: position matters. Tried worse positions than this one 
	/* $\sum_{i > 0} v_i = k$. Ensure that exactly k nodes are active. */
	addConstraint_k_nodes_active(env, model, v->vs, instance, this->k);
	e_in_degree.endElements();
	e_out_degree.endElements();


    /***** MTZ specific part ***/

	/* $u_i \in [0, k]$ variables are used to impose an order on nodes. */
	v->us = IloIntVarArray(env, instance.n_nodes);
	for (u_int i = 0; i < instance.n_nodes; i++) {
		v->us[i] = IloIntVar(env, 0, k, Tools::indicesToString("u", i).c_str());
	}

	
	IloExpr e3(env);
	e3 += v->us[0];
	/* $u_0 = 0$. Set level of artificial root 0 to 0. */
	model.add(e3 == 0);
	e3.end();

	for (u_int k = 0; k < n_edges; k++) {
		const u_int i = edges[k].v1;
		const u_int j = edges[k].v2;

		IloExpr e4(env);
		e4 = v->us[i] + v->xs[k] - v->us[j] - (-v->xs[k] + 1) * this->k;
		/* $\forall i, j: u_i + x_{ij} \leq u_j + (1 - x_{ij})k$. 
		 * Enforce order hierarchy on nodes. */
		model.add(e4 <= 0);
		e4.end();
	}

	for (u_int i = 0; i < instance.n_nodes; i++) {
		/* $\forall i: u_i <= nv_i$ force order of inactive nodes to 0 */
		/* helps with big instances 6,7,8 */
		model.add(v->us[i] <= v->vs[i] * (int) instance.n_nodes);
	}	
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
