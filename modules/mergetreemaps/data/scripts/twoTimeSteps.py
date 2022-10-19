from minizinc import Instance, Model, Solver
from pathlib import Path
import datetime

# Variables available from from C++
# firstFixed, modelFile, modelVariables, solverTimeoutEnabled, solverTimeoutSeconds

model = Model()
model.add_file(modelFile)

# Add constraint of first decision is not fixed
if (not firstFixed):
	model.add_string("constraint forall(idx in 1..numSubTrees_0)" \
		+ " (noChildren(tree_0, decisions_0, idx));\n")

# There is only one the decision for the second time step to search for
if(firstFixed):
	model.add_string("solve::bool_search(decisions_1, " \
		+ "input_order, indomain_max) minimize obj;\n")
else:
	model.add_string("""
		array[1..numSubTrees_1+numSubTrees_0] of var bool: decisions_all = 
			interleaveDecisions(decisions_0, decisions_1);

		solve::bool_search( decisions_all, input_order, indomain_max) minimize obj;
		""")

#print(modelVariables)
model.add_string(modelVariables)

# Find the MiniZinc solver configuration for Gecode
gecode = Solver.lookup("gecode")
# Create an Instance of the model for Gecode
instance = Instance(gecode, model)

# Solve
if (solverTimeoutEnabled):
	# solver-time-limit is the time limit for the solver in miliseconds
	solverTimout = datetime.timedelta(seconds=solverTimeoutSeconds).total_seconds() * 1000
	# solver-time-limit is not a proper python argument due to dashes
	# needs to be passed through dict
	result = instance.solve(**{"solver-time-limit": solverTimout})
else:
	result = instance.solve()

# If no solution was found, return and print status
if (not result.status.has_solution()):
	print("Optimization returned status:", result.status)
	# Can we exit more gracefully here?
	import sys
	sys.exit(0)

# Prepare output for C++
obj = result.objective
#print(obj)

time = result.statistics["time"].total_seconds()
flatTime = result.statistics["flatTime"].total_seconds()
#initTime = result.statistics["initTime"].total_seconds()
solveTime = result.statistics["solveTime"].total_seconds()
nodes = result.statistics["nodes"]

numSubTrees_0 = len(result["decisions_0"])
decisions_0 = [None]*numSubTrees_0
numSubTrees_1 = len(result["decisions_1"])
decisions_1 = [None]*numSubTrees_1

for i in range(numSubTrees_1):
	decisions_1[i] = int(result["decisions_1"][i])
	#print(result["limits_0"][i])
for i in range(numSubTrees_0):
	decisions_0[i] = int(result["decisions_0"][i])
	#print(result["limits_1"][i])

#print(decisions_1)
#print(decisions_0)
#print("decisions_0 =", result["decisions_0"])
#print("decisions_1 =", result["decisions_1"])
