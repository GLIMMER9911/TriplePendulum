#include "qpsolver.hpp"


// QP solver

// x^T * P * x + q * x ;
// l << A * x << u

/*
	QP Solver
	min || x^T P x || + q * x
		l < A * x < u

*/

// n is the size of output, m is the size of input, p is P, q is Q, a is A, l and u
Eigen::MatrixXd qpSolver( const Eigen::MatrixXd& p, const Eigen::MatrixXd& q, const Eigen::MatrixXd& a,
	const Eigen::MatrixXd& l, const Eigen::MatrixXd& u, const int n, const int m ) {

	// P matrix
	const int size = n + m;
	int printflag = 0;

	// P 5x5 diagonal matrix
	OSQPFloat* P_x = new OSQPFloat[size]{ 0.0 };
	for (int i = 0; i < (size); i++) {
		if (p(i, i)) {
			P_x[i] = p(i, i);
		}
		else {
			P_x[i] = 0.0;
		}
	}
	OSQPInt P_nnx = size;
	OSQPInt* P_i = new OSQPInt[size]{ 0 };
	for (int i = 0; i < size; i++) {
		P_i[i] = i;
	}
	OSQPInt* P_p = new OSQPInt[size + 1]{ 0 };
	for (int i = 0; i < size + 1; i++) {
		P_p[i] = i;
	}

	// q linear matrix
	OSQPFloat* Q = new OSQPFloat[size]{ 0.0 };
	for (int i = 0; i < size; i++) {
		Q[i] = q(i, 0);
	}

	// A matrix
	std::vector<double> vec_A_x;
	std::vector<int> vec_A_i;
	std::vector<int> vec_A_p;

	// A_p
	int nn = 0;
	for (int i = 0; i < a.cols(); ++i) {
		vec_A_p.push_back(nn);
		for (int j = 0; j < a.rows(); ++j) {
			if (a(j, i)) {
				vec_A_x.push_back(a(j, i));
				vec_A_i.push_back(j);
				++nn;
			}
		}
	}
	vec_A_p.push_back(nn);

	// A_x   A_i   A_p
	const int A_size = vec_A_x.size();
	OSQPFloat* A_x = new OSQPFloat[A_size];
	for (int i = 0; i < A_size; ++i) {
		A_x[i] = vec_A_x[i];
	}
	OSQPInt A_nnx = A_size;
	OSQPInt* A_i = new OSQPInt[vec_A_i.size()];
	for (int i = 0; i < vec_A_i.size(); ++i) {
		A_i[i] = vec_A_i[i];
	}
	OSQPInt* A_p = new OSQPInt[vec_A_p.size()];
	for (int i = 0; i < vec_A_p.size(); ++i) {
		A_p[i] = vec_A_p[i];
	}

	// l <= A*x <= u
	OSQPFloat* L = new OSQPFloat[l.rows()];
	for (int i = 0; i < l.rows(); ++i) {
		L[i] = l(i, 0);
	}

	OSQPFloat* U = new OSQPFloat[u.rows()];
	for (int i = 0; i < u.rows(); ++i) {
		U[i] = u(i, 0);
	}

	if (printflag) {
		std::cout << "A :" << std::endl;
		std::cout << a << std::endl;

		// print A
		std::cout << "vec_A_x :" << std::endl;
		for (int i = 0; i < A_size; i++) {
			std::cout << A_x[i] << "  ";
		}
		std::cout << std::endl;

		// print A_p
		std::cout << "vec_A_p :" << std::endl;
		for (int i = 0; i < vec_A_i.size(); i++) {
			std::cout << A_i[i] << "  ";
		}
		std::cout << std::endl;

		// print A_i
		std::cout << "vec_A_i :" << std::endl;
		for (int i = 0; i < vec_A_p.size(); i++) {
			std::cout << A_p[i] << "  ";
		}
		std::cout << std::endl;

		// L
		std::cout << "L :" << std::endl;
		for (int i = 0; i < l.size(); ++i) {
			std::cout << l(i, 0) << " ";
		}
		std::cout << std::endl;

		// U
		std::cout << "U :" << std::endl;
		for (int i = 0; i < u.size(); ++i) {
			std::cout << u(i, 0) << " ";
		}
		std::cout << std::endl;
	}

	// flag
	OSQPInt exitflag = 0;

	/* Solver, settings, matrix */
	OSQPSolver* solver = NULL;
	OSQPSettings* settings{ (OSQPSettings*)malloc(sizeof(OSQPSettings)) };
	OSQPCscMatrix* P = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
	OSQPCscMatrix* A = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));

	/* Populate matrices */
	csc_set_data(P, p.rows(), p.cols(), P_nnx, P_x, P_i, P_p);
	csc_set_data(A, a.rows(), a.cols(), A_nnx, A_x, A_i, A_p);

	if (settings) {
		osqp_set_default_settings(settings);
		settings->alpha = 0.8; /* Change alpha parameter */
		settings->verbose = 0;
	}

	/* Setup workspace */
	exitflag = osqp_setup(&solver, P, Q, A, L, U, a.rows(), a.cols(), settings);

	/* Solve problem */
	if (!exitflag) exitflag = osqp_solve(solver);

	OSQPFloat* optimal_solution = NULL;
	if (!exitflag) {
		optimal_solution = solver->solution->x;
	}
	else {
		throw std::runtime_error("error : acc data not correct");
	}

	Eigen::MatrixXd x(size, 1);
	for (int i = 0; i < size; ++i) {
		x(i, 0) = optimal_solution[i];
	}

	/* Cleanup */
	osqp_cleanup(solver);
	if (A) free(A);
	if (P) free(P);
	if (settings) free(settings);

	delete[] P_x;
	delete[] P_i;
	delete[] P_p;

	delete[] Q;

	delete[] A_x;
	delete[] A_i;
	delete[] A_p;
	delete[] L;
	delete[] U;

	return x;

}
