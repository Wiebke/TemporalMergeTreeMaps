include "globals.mzn";
include "traversalOrder.mzn";

int: numTimeSteps;
int: domainSize;
set of int: DomainSize = 0..domainSize;

int: rootArcIdx_0;
int: numSubTrees_0;
array[1..numSubTrees_0] of set of 1..numSubTrees_0: tree_0;
array[1..numSubTrees_0] of DomainSize : arcSizes_0;
array[1..numSubTrees_0] of DomainSize : subTreeSizes_0;
array[1..numSubTrees_0, 1..4] of var DomainSize : limits_0;
array[1..numSubTrees_0] of var bool: decisions_0;
constraint treeLimits(tree_0, arcSizes_0, subTreeSizes_0, limits_0, decisions_0, rootArcIdx_0, numSubTrees_0);
% Children constraints can be added depending on whether initial tree is set as fixed or not
% constraint forall(idx in 1..numSubTrees_0) (noChildren(tree_0, decisions_0, idx));

int: rootArcIdx_1;
int: numSubTrees_1;
array[1..numSubTrees_1] of set of 1..numSubTrees_1: tree_1;
array[1..numSubTrees_1] of DomainSize : arcSizes_1;
array[1..numSubTrees_1] of DomainSize : subTreeSizes_1;
array[1..numSubTrees_1, 1..4] of var DomainSize : limits_1;
array[1..numSubTrees_1] of var bool: decisions_1;
constraint treeLimits(tree_1, arcSizes_1, subTreeSizes_1, limits_1, decisions_1, rootArcIdx_1, numSubTrees_1);
constraint forall(idx in 1..numSubTrees_1) (noChildren(tree_1, decisions_1, idx));

array[1..numTimeSteps-1] of var 0.0..domainSize: objPerTimeStep;

array[1..numSubTrees_0 , 1..numSubTrees_1] of DomainSize: overlapND_0_1;
array[1..numSubTrees_0 , 1..numSubTrees_1] of var DomainSize: overlap1D_0_1 = 
  array2d(1..numSubTrees_0 , 1..numSubTrees_1, [
    overlap1DCounts(limits_0[i, 1], limits_0[i, 2], limits_1[j, 1], limits_1[j, 2]) | 
      i in 1..numSubTrees_0, j in 1..numSubTrees_1]);
constraint objPerTimeStep[1] = normOfDiffCounts(numSubTrees_0, numSubTrees_1, overlapND_0_1, overlap1D_0_1);

var float: obj = sum(i in 1..numTimeSteps-1)(objPerTimeStep[i]);

%solve::bool_search(decisions_1, input_order, indomain_min) minimize obj;