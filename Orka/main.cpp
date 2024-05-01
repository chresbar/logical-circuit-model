#include <iostream>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

int main() {
    IloEnv env;
    try {
        IloModel model(env);
        IloInt nGates = 7, nRows = 4;

 
        IloBoolVarArray s(env, nGates);
        IloBoolVarArray t1(env, nGates);
        IloBoolVarArray t2(env, nGates);
        IloArray<IloBoolVarArray> x(env, nGates);

        for (IloInt i = 0; i < nGates; ++i) {
            x[i] = IloBoolVarArray(env, nRows);
        }

        // Constants alpha representing external input signals in truth table rows
        IloIntArray alpha1(env, nRows, 0, 1, 0, 1);
        IloIntArray alpha2(env, nRows, 0, 0, 1, 1);

        // Constraints
        for (IloInt i = 0; i < nGates; ++i) {
            model.add(s[i] - t1[i] >= 0);
            model.add(s[i] - t2[i] >= 0);

            for (IloInt l = 0; l < nRows; ++l) {
                model.add(s[i] - x[i][l] >= 0);
                model.add((alpha1[l] * t1[i]) + x[i][l] <= 1);
                model.add((alpha2[l] * t2[i]) + x[i][l] <= 1);
                if (i >= 3) {
                    model.add((alpha1[l] * t1[i]) + (alpha2[l] * t2[i]) + x[i][l] - s[i] >= 0);
                }
            }
        }

        IloInt feed[3][2] = { {2, 1}, {4, 3}, {6, 5} };
        for (IloInt i = 0; i < 3; ++i) {
            IloInt j = feed[i][0];
            IloInt k = feed[i][1];
            for (IloInt l = 0; l < nRows; ++l) {
                model.add(x[j][l] + x[i][l] <= 1);
                model.add(x[k][l] + x[i][l] <= 1);
                model.add(alpha1[l] * t1[i] + alpha2[l] * t2[i] + x[j][l] + x[k][l] + x[i][l] - s[i] >= 0);
            }
            model.add(s[j] + s[k] + t1[i] + t2[i] <= 2);
        }

        // Predefined outputs for gate 1
        model.add(x[0][0] == 0);
        model.add(x[0][1] == 1);
        model.add(x[0][2] == 1);
        model.add(x[0][3] == 0);

        // Gate 1 must exist
        model.add(s[0] >= 1);

        // Objective: Minimize the number of gates used
        model.add(IloMinimize(env, IloSum(s)));

        // Create Cplex solver
        IloCplex cplex(model);

        // Find all solutions
        int solutionCount = 0;
        while (cplex.solve()) {
            ++solutionCount;

            // Output results
            std::cout << "Solution " << solutionCount << ":\n";
            std::cout << "Solution status: " << cplex.getStatus() << std::endl;
            std::cout << "Minimum number of gates: " << cplex.getObjValue() << std::endl;

            for (IloInt i = 0; i < nGates; ++i) {
                std::cout << "Gate " << i + 1 << " is used: " << cplex.getValue(s[i]) << std::endl;
            }

            // Add constraint to prevent finding the same solution
            IloExpr uniqueSolution(env);
            for (IloInt i = 0; i < nGates; ++i) {
                uniqueSolution += s[i] * (1 - cplex.getValue(s[i]));
            }
            model.add(uniqueSolution >= 1);
            uniqueSolution.end();
        }

        if (solutionCount == 0) {
            std::cout << "No feasible solution found." << std::endl;
        }
        else {
            std::cout << "Total number of solutions: " << solutionCount << std::endl;
        }

    }
    catch (IloException& ex) {
        std::cerr << "Error: " << ex.getMessage() << std::endl;
    }
    env.end();
    return 0;
}
