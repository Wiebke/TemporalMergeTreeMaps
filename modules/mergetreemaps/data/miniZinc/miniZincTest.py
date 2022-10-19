from minizinc import Instance, Model, Solver
from pathlib import Path
import datetime

# Load the full model from file (all variables set)
#model = Model("./leftrightdecision.mzn")

firstFixed = False
# 0: free, 1: in_order, 2: interleaved
searchOrderMode = 2

model = Model()
model.add_file("./twoTimeSteps.mzn")

# Add constraint of first decision is not fixed
if (not firstFixed):
	model.add_string("constraint forall(idx in 1..numSubTrees_0)" \
		+ " (noChildren(tree_0, decisions_0, idx));\n")
else:
	model.add_string("decisions_0 = [false | i in 1..numSubTrees_0];\n")

if (searchOrderMode == 0):
	model.add_string("solve minimize obj;\n")
else:
	# There is only one the decision for the second time step to search for
	if(firstFixed):
		model.add_string("solve::bool_search(decisions_1, " \
			+ "input_order, indomain_max) minimize obj;\n")
	# Concatenate decision
	elif(searchOrderMode == 1):
		model.add_string("solve::bool_search(decisions_0 ++ decisions_1, " \
			+ "input_order, indomain_max) minimize obj;\n")
	else:
		model.add_string("""
			array[1..numSubTrees_1+numSubTrees_0] of var bool: decisions_all = 
				interleaveDecisions(decisions_0, decisions_1);

			solve::bool_search( decisions_all, input_order, indomain_max) minimize obj;
			""")

print("Starting solve")

# Find the MiniZinc solver configuration for Gecode
gecode = Solver.lookup("gecode")
# Create an Instance of the model for Gecode
instance = Instance(gecode, model)

instance["numTimeSteps"] = 2
instance["domainSize"] = 4096

instance["rootArcIdx_0"] = 1;
instance["numSubTrees_0"] = 19;
instance["tree_0"] = [
set({19, 2}),
set({18, 3}),
set({13, 4}),
set({10, 5}),
set({17, 6}),
set({16, 7}),
set({8, 9}),
set(),
set(),
set({11, 12}),
set(),
set(),
set({14, 15}),
set(),
set(),
set(),
set(),
set(),
set()
]
instance["arcSizes_0"] = [
955,
1,
1073,
1,
5,
1,
5,
397,
396,
3,
240,
239,
3,
239,
240,
144,
144,
5,
5
]
instance["subTreeSizes_0"] = [
4096,
3136,
3130,
1575,
1092,
943,
798,
397,
396,
482,
240,
239,
482,
239,
240,
144,
144,
5,
5
]

instance["rootArcIdx_1"] = 1;
instance["numSubTrees_1"] = 19;
instance["tree_1"] = [
set({19, 2}),
set({18, 3}),
set({16, 4}),
set({17, 5}),
set({10, 6}),
set({12, 7}),
set({8, 9}),
set(),
set(),
set({11, 14}),
set(),
set({13, 15}),
set(),
set(),
set(),
set(),
set(),
set(),
set()
]
instance["arcSizes_1"] = [
1027,
1,
1015,
1,
3,
1,
15,
391,
390,
1,
239,
1,
239,
238,
238,
145,
145,
3,
3
]
instance["subTreeSizes_1"] = [
4096,
3066,
3062,
1902,
1756,
1275,
796,
391,
390,
478,
239,
478,
239,
238,
238,
145,
145,
3,
3
]
instance["overlapND_0_1"] = [
4096,3066,3062,1902,1756,1275,796,391,390,478,239,478,239,238,238,145,145,3,3,
3136,3042,3038,1902,1756,1275,796,391,390,478,239,478,239,238,238,145,145,3,0,
3130,3038,3036,1901,1755,1275,796,391,390,477,239,478,239,237,238,145,145,1,0,
1575,1575,1573,1386,1254,800,796,391,390,454,227,3,2,227,1,134,132,1,0,
1092,1092,1091,932,800,798,794,390,390,2,1,3,2,1,1,132,132,0,0,
943,943,943,798,798,796,793,390,389,2,1,2,2,1,0,132,0,0,0,
798,798,798,795,795,795,793,390,389,0,0,2,2,0,0,0,0,0,0,
397,397,397,396,396,396,395,389,0,0,0,1,1,0,0,0,0,0,0,
396,396,396,395,395,395,394,0,388,0,0,1,1,0,0,0,0,0,0,
482,482,481,454,454,2,2,1,0,452,226,0,0,226,0,2,0,1,0,
240,240,239,226,226,0,0,0,0,226,0,0,0,226,0,1,0,1,0,
239,239,239,227,227,1,1,1,0,226,226,0,0,0,0,1,0,0,0,
482,482,482,458,456,454,0,0,0,2,1,454,226,0,227,0,2,0,0,
239,239,239,228,227,226,0,0,0,1,1,226,226,0,0,0,1,0,0,
240,240,240,228,227,227,0,0,0,0,0,227,0,0,227,0,1,0,0,
144,144,144,2,2,0,0,0,0,2,1,0,0,1,0,132,0,0,0,
144,144,144,134,2,2,1,0,1,0,0,1,0,0,1,0,132,0,0,
5,3,1,1,1,0,0,0,0,1,0,0,0,1,0,0,0,2,0,
5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3
]

# Solve
#result = instance.solve()
solvertimelimit = datetime.timedelta(seconds=20).total_seconds() * 1000
result = instance.solve(**{"solver-time-limit": solvertimelimit})
# Output the array

if (not result.status.has_solution()):
	print("Optimization returned status:", result.status)
	import sys
	sys.exit()

print("Objective=", result.objective)

print("decisions_0 =", result["decisions_0"])
print("decisions_1 =", result["decisions_1"])

print("Timing:", 
	result.statistics["flatTime"].total_seconds(), "(flat)",
	result.statistics["initTime"].total_seconds(), "(init)", 
 	result.statistics["solveTime"].total_seconds(), "(solve)") 

#print key and values
for (key, value) in result.statistics.items():
    print(key, value)

#instance.solve(verbose=True, debug_output=Path("./debug_output.txt"))
#solve(timeout: Optional[datetime.timedelta] = None, nr_solutions: Optional[int] = None, 
#	processes: Optional[int] = None, random_seed: Optional[int] = None, 
#	all_solutions: bool = False, intermediate_solutions: bool = False, 
#	free_search: bool = False, optimisation_level: Optional[int] = None, 
#	**kwargs)
# times: initTime, solveTime, flatTime in the statistics part of the solution