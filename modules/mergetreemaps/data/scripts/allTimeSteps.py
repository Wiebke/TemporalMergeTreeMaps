from minizinc import Instance, Model, Solver
from pathlib import Path
import datetime

# Variables available from from C++
# modelFile, solverTimeoutEnabled, solverTimeoutSeconds

model = Model()
model.add_file(modelFile)

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

numSubTrees_all = len(result["decisions_all"])
decisions_all = [None]*numSubTrees_all

for i in range(numSubTrees_all):
	decisions_all[i] = int(result["decisions_all"][i])