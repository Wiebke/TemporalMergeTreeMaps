include "globals.mzn";
include "traversalOrder.mzn";
int: numTimeSteps = 2;
int: domainSize = 4096;
set of int: DomainSize = 0..domainSize;

int: rootArcIdx_0 = 1;
int: numSubTrees_0 = 19;
array[1..numSubTrees_0] of set of 1..numSubTrees_0: tree_0 = [
{19, 2},
{18, 3},
{13, 4},
{10, 5},
{17, 6},
{16, 7},
{8, 9},
{},
{},
{11, 12},
{},
{},
{14, 15},
{},
{},
{},
{},
{},
{}
];
array[1..numSubTrees_0] of DomainSize : arcSizes_0 = [
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
];
array[1..numSubTrees_0] of DomainSize : subTreeSizes_0 = [
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
];
array[1..numSubTrees_0, 1..4] of var DomainSize : limits_0;
% Must not be commented for Test 1
array[1..numSubTrees_0] of var bool: decisions_0 = [true | i in 1..numSubTrees_0];
% Must not be commented for Test 2
%array[1..numSubTrees_0] of var bool: decisions_0;
% Must not be commented for Test 3
%decisions_0=[true, true, false, true, true, true, false, false, false, false, false, false, false, false, false, false, false, false, false];
constraint treeLimits(tree_0, arcSizes_0, subTreeSizes_0, limits_0, decisions_0, rootArcIdx_0, numSubTrees_0);
% Must be commented out for Test 1 and 3
%constraint forall(idx in 1..numSubTrees_0) (noChildren(tree_0, decisions_0, idx));

int: rootArcIdx_1 = 1;
int: numSubTrees_1 = 19;
array[1..numSubTrees_1] of set of 1..numSubTrees_1: tree_1 = [
{19, 2},
{18, 3},
{16, 4},
{17, 5},
{10, 6},
{12, 7},
{8, 9},
{},
{},
{11, 14},
{},
{13, 15},
{},
{},
{},
{},
{},
{},
{}
];
array[1..numSubTrees_1] of DomainSize : arcSizes_1 = [
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
];
array[1..numSubTrees_1] of DomainSize : subTreeSizes_1 = [
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
];
array[1..numSubTrees_1, 1..4] of var DomainSize : limits_1;
array[1..numSubTrees_1] of var bool: decisions_1;
constraint treeLimits(tree_1, arcSizes_1, subTreeSizes_1, limits_1, decisions_1, rootArcIdx_1, numSubTrees_1);
constraint forall(idx in 1..numSubTrees_1) (noChildren(tree_1, decisions_1, idx));

array[1..numTimeSteps-1] of var 0.0..domainSize: objPerTimeStep;

array[1..numSubTrees_0 , 1..numSubTrees_1] of DomainSize: overlapND_0_1 = array2d(1..numSubTrees_0 , 1..numSubTrees_1, [
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
]);
array[1..numSubTrees_0 , 1..numSubTrees_1] of var DomainSize: overlap1D_0_1 = array2d(1..numSubTrees_0 , 1..numSubTrees_1, [
overlap1DCounts(limits_0[i, 1], limits_0[i, 2], limits_1[j, 1], limits_1[j, 2]) |
i in 1..numSubTrees_0, j in 1..numSubTrees_1

]);
constraint objPerTimeStep[1] = normOfDiffCounts(numSubTrees_0, numSubTrees_1, overlapND_0_1, overlap1D_0_1);

array[1..numSubTrees_1+numSubTrees_0] of var bool: decisions_all = interleaveDecisions(decisions_0, decisions_1);

var float: obj = sum(i in 1..numTimeSteps-1)(objPerTimeStep[i]);
%solve ::bool_search( decisions_0 ++ decisions_1, input_order, indomain_min) minimize obj;
solve::bool_search( decisions_all, input_order, indomain_min) minimize obj;
%solve minimize obj;
%  ++ ["objective value: " ++ show(obj) ];
%output [ show2d(overlap1D_0_1)] ;
 

%% Test 1: Fix first tree to all true
% Finds solution with obj 0.180071771144867 in ~13 seconds with "solve minimize obj"
% decisions_1 = array1d(1..19, [false, false, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true]);
% objPerTimeStep = array1d(1..1, [0.180071771144867]);
% Same solution is found with ::bool_search in ~3 seconds

%% Test 2: Can a better solution be found if we do not fix the first tree?
% Yes!, but it takes ~5min with bool_search, ~1min with interleaved bool_search, >60min with free search
% decisions_0 = array1d(1..19, [true, true, false, true, true, true, false, true, true, false, true, true, false, true, true, true, true, true, true]);
% decisions_1 = array1d(1..19, [false, false, true, true, true, false, false, true, true, false, true, false, true, true, true, true, true, true, true]);
% objPerTimeStep = array1d(1..1, [0.0594942569732666]);

%% Test 3: Can we find the second optimal decision if we have the optimal first given?
% Expected solution with objective 0.0594942569732666 when decision_0 is set before hand to:
% decisions_0=[true, true, false, true, true, true, false, false, false, false, false, false, false, false, false, false, false, false, false];
% => decisions_1 = array1d(1..19, [false, false, true, true, true, false, false, true, true, false, true, false, true, true, true, true, true, true, true]);
%% Yes!
% This takes ~12 seconds to find with "solve minimize obj" and ~3 seconds to find with "solve ::bool_search( decisions_0 ++ decisions_1, input_order, indomain_min) minimize obj;"