predicate noChildren(array[int] of set of int: tree, array[int] of var bool: decision, int: idx) =
let {
  set of int: children = tree[idx];
  } in
  if card(children) == 0 then
    decision[idx] = true
  else
    true
  endif;

predicate sumOfChildren(array[int] of set of int: tree, array[int] of int: arcSizes, array[int] of var DomainSize: subTreeSizes, int: idx, int: rootIdx) =
let {
  set of int: children = tree[idx];
  int: arcSize = arcSizes[idx];
  } in
  if card(children) == 0 then
    subTreeSizes[idx] = arcSize + 1
  else
    if idx == rootIdx then
      subTreeSizes[idx] = arcSize + 2 + subTreeSizes[children[1]] + subTreeSizes[children[2]]    
    else 
      subTreeSizes[idx] = arcSize + 1 + subTreeSizes[children[1]] + subTreeSizes[children[2]]
    endif
  endif;
  
predicate innerLimits(var bool: first, int: arcSize, int: treeSize, array[int, 1..4] of var DomainSize: limits, int: parentIdx, int: idx) =
let {
  array[1..4] of var DomainSize: parentLimit = limits[parentIdx,..];
  array[1..4] of var DomainSize: thisLimit = limits[idx,..];
} in
  if first then
    %trace(show(parentLimit)++" " ++ show(idx) ++ "\n",
    thisLimit[1] = parentLimit[3] + 1 /\ 
    thisLimit[2] = thisLimit[1] + (treeSize - 1) /\ 
    thisLimit[3] = thisLimit[1] + (arcSize div 2 - 1) /\ 
    thisLimit[4] = thisLimit[2] - ((arcSize + 1) div 2 - 1)%)
  else
    %trace(show(parentLimit)++" " ++ show(idx)  ++ "\n",
    thisLimit[2] = parentLimit[4] - 1 /\ 
    thisLimit[1] = thisLimit[2] - (treeSize - 1) /\
    thisLimit[3] = thisLimit[1] + (arcSize div 2 - 1) /\ 
    thisLimit[4] = thisLimit[2] - ((arcSize + 1) div 2 - 1)%)
  endif;
  
predicate rootLimits(array[int] of set of int: tree, array[int] of int: arcSizes, 
  array[int] of int: subTreeSizes, array[int, 1..4] of var DomainSize: limits, int: rootIdx) = 
let {
  int: arcSize = arcSizes[rootIdx];
  int: treeSize = subTreeSizes[rootIdx];
} in
  limits[rootIdx, 1] = 0 /\ 
  limits[rootIdx, 2] = treeSize - 1 /\ 
  limits[rootIdx, 3] = limits[rootIdx, 1] + ((arcSize + 1) div 2 - 1) /\ 
  limits[rootIdx, 4] = limits[rootIdx, 2] - (arcSize div 2 - 1);

predicate treeLimits(array[int] of set of int: tree, array[int] of int: arcSizes, 
  array[int] of int: subTreeSizes, array[int, 1..4] of var DomainSize: limits, array[int] of var bool: decisions, int: rootIdx, int: numSubTrees) =
  rootLimits(tree, arcSizes, subTreeSizes, limits, rootIdx) /\ forall(idx in 1..numSubTrees) (
let {
  set of int: children = tree[idx];
} in 
  if card(children) != 0 then
    innerLimits(decisions[idx], arcSizes[children[1]], subTreeSizes[children[1]], limits, idx, children[1]) /\ 
    innerLimits(not decisions[idx], arcSizes[children[2]], subTreeSizes[children[2]], limits, idx, children[2])
  endif
);

function var DomainSize: overlap1DCounts(var int: minA, var int: maxA, var int: minB, var int: maxB) = max(0, min(maxA, maxB) - max(minA, minB) + 1);
%let {
%  var int: overlapStart = max(minA, minB);
%  var int: overlapEnd = min(maxA, maxB); 
%} in max(0, overlapEnd - overlapStart + 1);

%% Using sqrt here, leads to errors in Geocode
function var float: normOfDiffCounts(int: numSubTreesFirst, int: numSubTreesSecond, array[int,int] of DomainSize: overlapND, array[int,int] of var DomainSize: overlap1D) = 
  sum(i in 1..numSubTreesFirst, j in 1..numSubTreesSecond)
  ( let { var -1.0..1.0: difference = (overlapND[i,j] - overlap1D[i,j])/domainSize;
  } in 
  difference * difference);
  

function array[int] of var bool: interleaveDecisions(array[int] of var bool: a, array[int] of var bool: b) = let {
        array[int] of int: ai = [ i | i in index_set(a)];
        array[int] of int: bi = [ i | i in index_set(b)];
        int: share = min(length(a), length(b));
    } in [ if x = 1 then a[ai[i]] else b[bi[i]] endif | i in 1..share, x in 1..2]
        ++ [ a[ai[i]] | i in share+1..length(a) ]
        ++ [ b[bi[i]] | i in share+1..length(b) ];