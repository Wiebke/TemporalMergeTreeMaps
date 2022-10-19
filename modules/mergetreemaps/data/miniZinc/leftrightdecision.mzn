include "globals.mzn";
include "traversalOrder.mzn";

%%% Important: 
%%% All of this has 1-based indices, whereas C++ uses 0-based indices
%%% If you want integer division, explicitly use "div", otherwise model will say its unsatisfiable
%%% Sets are always ordered, keep this in mind when translating back!!

% Input parameter: Size of domain, number of time steps
int: numTimeSteps = 3;
int: domainSize = 4096;
set of int: DomainSize = 0..domainSize;
set of int: DiffDomain = -domainSize..domainSize;

% Input parameters: Tree structure per time step
int: rootArcIdx_0 = 3;
int: numSubTrees_0 = 3;
array[1..numSubTrees_0] of set of 1..numSubTrees_0: tree_0 = [
{},
{},
{2, 1}
];
array[1..numSubTrees_0] of DomainSize : arcSizes_0 = [
246,
876,
2974
];
array[1..numSubTrees_0] of DomainSize : subTreeSizes_0 = [
246,
876,
4096
];
array[1..numSubTrees_0, 1..4] of var DomainSize : limits_0;
array[1..numSubTrees_0] of var bool: decisions_0;
constraint treeLimits(tree_0, arcSizes_0, subTreeSizes_0, limits_0, decisions_0, rootArcIdx_0, numSubTrees_0);
constraint forall(idx in 1..numSubTrees_0) (noChildren(tree_0, decisions_0, idx));

int: rootArcIdx_1 = 3;
int: numSubTrees_1 = 3;
array[1..numSubTrees_1] of set of 1..numSubTrees_1: tree_1 = [
{},
{},
{2, 1}
];
array[1..numSubTrees_1] of DomainSize : arcSizes_1 = [
344,
604,
3148
];
array[1..numSubTrees_1] of DomainSize : subTreeSizes_1 = [
344,
604,
4096
];
array[1..numSubTrees_1, 1..4] of var DomainSize : limits_1;
array[1..numSubTrees_1] of var bool: decisions_1;
constraint treeLimits(tree_1, arcSizes_1, subTreeSizes_1, limits_1, decisions_1, rootArcIdx_1, numSubTrees_1);
constraint forall(idx in 1..numSubTrees_1) (noChildren(tree_1, decisions_1, idx));

int: rootArcIdx_2 = 3;
int: numSubTrees_2 = 3;
array[1..numSubTrees_2] of set of 1..numSubTrees_2: tree_2 = [
{},
{},
{2, 1}
];
array[1..numSubTrees_2] of DomainSize : arcSizes_2 = [
344,
604,
3148
];
array[1..numSubTrees_2] of DomainSize : subTreeSizes_2 = [
344,
604,
4096
];
array[1..numSubTrees_2, 1..4] of var DomainSize : limits_2;
array[1..numSubTrees_2] of var bool: decisions_2;
constraint treeLimits(tree_2, arcSizes_2, subTreeSizes_2, limits_2, decisions_2, rootArcIdx_2, numSubTrees_2);
constraint forall(idx in 1..numSubTrees_2) (noChildren(tree_2, decisions_2, idx));

array[1..numTimeSteps-1] of var 0.0..domainSize: objPerTimeStep;

array[1..numSubTrees_0 , 1..numSubTrees_1] of DomainSize: overlapND_0_1 = array2d(1..numSubTrees_0 , 1..numSubTrees_1, [
246,0,246,
4,604,876,
344,604,4096
]);
array[1..numSubTrees_0 , 1..numSubTrees_1] of var DomainSize: overlap1D_0_1 = array2d(1..numSubTrees_0 , 1..numSubTrees_1, [
overlap1DCounts(limits_0[i, 1], limits_0[i, 2], limits_1[j, 1], limits_1[j, 2]) |
i in 1..numSubTrees_0,
j in 1..numSubTrees_1
]);
constraint objPerTimeStep[1] = normOfDiffCounts(numSubTrees_0, numSubTrees_1, overlapND_0_1, overlap1D_0_1);

array[1..numSubTrees_1 , 1..numSubTrees_2] of DomainSize: overlapND_1_2 = array2d(1..numSubTrees_1 , 1..numSubTrees_2, [
0,343,344,
344,12,604,
345,603,4096
]);
array[1..numSubTrees_1 , 1..numSubTrees_2] of var DomainSize: overlap1D_1_2 = array2d(1..numSubTrees_1 , 1..numSubTrees_2, [
overlap1DCounts(limits_1[i, 1], limits_1[i, 2], limits_2[j, 1], limits_2[j, 2]) |
i in 1..numSubTrees_1,
j in 1..numSubTrees_2
]);
constraint objPerTimeStep[2] = normOfDiffCounts(numSubTrees_1, numSubTrees_2, overlapND_1_2, overlap1D_1_2);

var float: obj = sum(i in 1..numTimeSteps-1)(objPerTimeStep[i]);

solve minimize obj;
%output [ show(decisionFirst) ]
%output [ show(diff1D[i,j]) ++ if j = numSubTreesFirst then "\n" else ", \t" endif | i in 1..numSubTreesSecond, j in 1..numSubTreesFirst ] 
%  ++ ["objective value: " ++ show(obj) ];
