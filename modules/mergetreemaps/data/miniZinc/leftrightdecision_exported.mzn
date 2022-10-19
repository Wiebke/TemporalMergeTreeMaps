include "globals.mzn";
include "traversalOrder.mzn";
int: numTimeSteps = 96;
int: domainSize = 4096;
set of int: DomainSize = 0..domainSize;

int: rootArcIdx_0 = 19;
int: numSubTrees_0 = 19;
array[1..numSubTrees_0] of set of 1..numSubTrees_0: tree_0 = [
{},
{},
{},
{},
{},
{},
{6, 5},
{},
{},
{9, 8},
{},
{},
{12, 11},
{4, 13},
{3, 14},
{10, 15},
{7, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_0] of DomainSize : arcSizes_0 = [
5,
5,
144,
144,
240,
239,
3,
239,
240,
3,
396,
397,
5,
1,
5,
1,
1073,
1,
955
];
array[1..numSubTrees_0] of DomainSize : subTreeSizes_0 = [
5,
5,
144,
144,
240,
239,
482,
239,
240,
482,
396,
397,
798,
943,
1092,
1575,
3130,
3136,
4096
];
array[1..numSubTrees_0, 1..4] of var DomainSize : limits_0;
array[1..numSubTrees_0] of var bool: decisions_0;
constraint treeLimits(tree_0, arcSizes_0, subTreeSizes_0, limits_0, decisions_0, rootArcIdx_0, numSubTrees_0);
constraint forall(idx in 1..numSubTrees_0) (noChildren(tree_0, decisions_0, idx));
int: rootArcIdx_1 = 19;
int: numSubTrees_1 = 19;
array[1..numSubTrees_1] of set of 1..numSubTrees_1: tree_1 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{},
{9, 6},
{},
{},
{12, 11},
{8, 13},
{10, 14},
{3, 15},
{4, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_1] of DomainSize : arcSizes_1 = [
3,
3,
145,
145,
238,
238,
239,
1,
239,
1,
390,
391,
15,
1,
3,
1,
1015,
1,
1027
];
array[1..numSubTrees_1] of DomainSize : subTreeSizes_1 = [
3,
3,
145,
145,
238,
238,
239,
478,
239,
478,
390,
391,
796,
1275,
1756,
1902,
3062,
3066,
4096
];
array[1..numSubTrees_1, 1..4] of var DomainSize : limits_1;
array[1..numSubTrees_1] of var bool: decisions_1;
constraint treeLimits(tree_1, arcSizes_1, subTreeSizes_1, limits_1, decisions_1, rootArcIdx_1, numSubTrees_1);
constraint forall(idx in 1..numSubTrees_1) (noChildren(tree_1, decisions_1, idx));
int: rootArcIdx_2 = 17;
int: numSubTrees_2 = 17;
array[1..numSubTrees_2] of set of 1..numSubTrees_2: tree_2 = [
{},
{},
{},
{},
{},
{},
{6, 5},
{},
{8, 4},
{},
{},
{11, 10},
{3, 12},
{2, 13},
{7, 14},
{9, 15},
{1, 16}
];
array[1..numSubTrees_2] of DomainSize : arcSizes_2 = [
2,
142,
142,
229,
229,
247,
13,
247,
13,
390,
389,
13,
1,
21,
1,
933,
1084
];
array[1..numSubTrees_2] of DomainSize : subTreeSizes_2 = [
2,
142,
142,
229,
229,
247,
489,
247,
489,
390,
389,
792,
935,
1098,
1588,
3010,
4096
];
array[1..numSubTrees_2, 1..4] of var DomainSize : limits_2;
array[1..numSubTrees_2] of var bool: decisions_2;
constraint treeLimits(tree_2, arcSizes_2, subTreeSizes_2, limits_2, decisions_2, rootArcIdx_2, numSubTrees_2);
constraint forall(idx in 1..numSubTrees_2) (noChildren(tree_2, decisions_2, idx));
int: rootArcIdx_3 = 17;
int: numSubTrees_3 = 17;
array[1..numSubTrees_3] of set of 1..numSubTrees_3: tree_3 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{4, 10},
{5, 11},
{6, 12},
{7, 13},
{2, 14},
{3, 15},
{1, 16}
];
array[1..numSubTrees_3] of DomainSize : arcSizes_3 = [
1,
141,
141,
228,
228,
254,
254,
392,
391,
13,
1,
1,
1,
1,
1,
887,
1161
];
array[1..numSubTrees_3] of DomainSize : subTreeSizes_3 = [
1,
141,
141,
228,
228,
254,
254,
392,
391,
796,
1025,
1254,
1509,
1764,
1906,
2934,
4096
];
array[1..numSubTrees_3, 1..4] of var DomainSize : limits_3;
array[1..numSubTrees_3] of var bool: decisions_3;
constraint treeLimits(tree_3, arcSizes_3, subTreeSizes_3, limits_3, decisions_3, rootArcIdx_3, numSubTrees_3);
constraint forall(idx in 1..numSubTrees_3) (noChildren(tree_3, decisions_3, idx));
int: rootArcIdx_4 = 17;
int: numSubTrees_4 = 17;
array[1..numSubTrees_4] of set of 1..numSubTrees_4: tree_4 = [
{},
{},
{},
{},
{},
{},
{6, 4},
{},
{8, 5},
{},
{},
{11, 10},
{3, 12},
{2, 13},
{9, 14},
{7, 15},
{1, 16}
];
array[1..numSubTrees_4] of DomainSize : arcSizes_4 = [
1,
138,
138,
262,
262,
220,
11,
220,
11,
385,
386,
19,
1,
17,
1,
783,
1241
];
array[1..numSubTrees_4] of DomainSize : subTreeSizes_4 = [
1,
138,
138,
262,
262,
220,
493,
220,
493,
385,
386,
790,
929,
1084,
1578,
2854,
4096
];
array[1..numSubTrees_4, 1..4] of var DomainSize : limits_4;
array[1..numSubTrees_4] of var bool: decisions_4;
constraint treeLimits(tree_4, arcSizes_4, subTreeSizes_4, limits_4, decisions_4, rootArcIdx_4, numSubTrees_4);
constraint forall(idx in 1..numSubTrees_4) (noChildren(tree_4, decisions_4, idx));
int: rootArcIdx_5 = 15;
int: numSubTrees_5 = 15;
array[1..numSubTrees_5] of set of 1..numSubTrees_5: tree_5 = [
{},
{},
{},
{},
{},
{5, 4},
{},
{7, 3},
{},
{},
{10, 9},
{1, 11},
{2, 12},
{8, 13},
{6, 14}
];
array[1..numSubTrees_5] of DomainSize : arcSizes_5 = [
134,
134,
267,
267,
220,
3,
220,
3,
388,
389,
19,
1,
1,
1,
2049
];
array[1..numSubTrees_5] of DomainSize : subTreeSizes_5 = [
134,
134,
267,
267,
220,
490,
220,
490,
388,
389,
796,
931,
1066,
1557,
4096
];
array[1..numSubTrees_5, 1..4] of var DomainSize : limits_5;
array[1..numSubTrees_5] of var bool: decisions_5;
constraint treeLimits(tree_5, arcSizes_5, subTreeSizes_5, limits_5, decisions_5, rootArcIdx_5, numSubTrees_5);
constraint forall(idx in 1..numSubTrees_5) (noChildren(tree_5, decisions_5, idx));
int: rootArcIdx_6 = 15;
int: numSubTrees_6 = 15;
array[1..numSubTrees_6] of set of 1..numSubTrees_6: tree_6 = [
{},
{},
{},
{},
{},
{5, 3},
{},
{7, 4},
{},
{},
{10, 9},
{2, 11},
{1, 12},
{8, 13},
{6, 14}
];
array[1..numSubTrees_6] of DomainSize : arcSizes_6 = [
130,
130,
215,
215,
277,
7,
277,
7,
388,
389,
15,
1,
7,
1,
2037
];
array[1..numSubTrees_6] of DomainSize : subTreeSizes_6 = [
130,
130,
215,
215,
277,
499,
277,
499,
388,
389,
792,
923,
1060,
1560,
4096
];
array[1..numSubTrees_6, 1..4] of var DomainSize : limits_6;
array[1..numSubTrees_6] of var bool: decisions_6;
constraint treeLimits(tree_6, arcSizes_6, subTreeSizes_6, limits_6, decisions_6, rootArcIdx_6, numSubTrees_6);
constraint forall(idx in 1..numSubTrees_6) (noChildren(tree_6, decisions_6, idx));
int: rootArcIdx_7 = 15;
int: numSubTrees_7 = 15;
array[1..numSubTrees_7] of set of 1..numSubTrees_7: tree_7 = [
{},
{},
{},
{},
{},
{5, 4},
{},
{7, 3},
{},
{},
{10, 9},
{1, 11},
{2, 12},
{6, 13},
{8, 14}
];
array[1..numSubTrees_7] of DomainSize : arcSizes_7 = [
124,
124,
282,
282,
213,
4,
213,
4,
387,
388,
17,
1,
5,
1,
2051
];
array[1..numSubTrees_7] of DomainSize : subTreeSizes_7 = [
124,
124,
282,
282,
213,
499,
213,
499,
387,
388,
792,
917,
1046,
1546,
4096
];
array[1..numSubTrees_7, 1..4] of var DomainSize : limits_7;
array[1..numSubTrees_7] of var bool: decisions_7;
constraint treeLimits(tree_7, arcSizes_7, subTreeSizes_7, limits_7, decisions_7, rootArcIdx_7, numSubTrees_7);
constraint forall(idx in 1..numSubTrees_7) (noChildren(tree_7, decisions_7, idx));
int: rootArcIdx_8 = 15;
int: numSubTrees_8 = 15;
array[1..numSubTrees_8] of set of 1..numSubTrees_8: tree_8 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 7},
{3, 9},
{4, 10},
{5, 11},
{6, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_8] of DomainSize : arcSizes_8 = [
121,
121,
292,
292,
213,
213,
389,
388,
9,
1,
5,
1,
5,
1,
2045
];
array[1..numSubTrees_8] of DomainSize : subTreeSizes_8 = [
121,
121,
292,
292,
213,
213,
389,
388,
786,
1079,
1376,
1590,
1808,
1930,
4096
];
array[1..numSubTrees_8, 1..4] of var DomainSize : limits_8;
array[1..numSubTrees_8] of var bool: decisions_8;
constraint treeLimits(tree_8, arcSizes_8, subTreeSizes_8, limits_8, decisions_8, rootArcIdx_8, numSubTrees_8);
constraint forall(idx in 1..numSubTrees_8) (noChildren(tree_8, decisions_8, idx));
int: rootArcIdx_9 = 15;
int: numSubTrees_9 = 15;
array[1..numSubTrees_9] of set of 1..numSubTrees_9: tree_9 = [
{},
{},
{},
{},
{},
{5, 3},
{},
{7, 4},
{},
{},
{10, 9},
{2, 11},
{1, 12},
{6, 13},
{8, 14}
];
array[1..numSubTrees_9] of DomainSize : arcSizes_9 = [
112,
112,
211,
211,
298,
3,
298,
3,
388,
387,
15,
1,
7,
1,
2049
];
array[1..numSubTrees_9] of DomainSize : subTreeSizes_9 = [
112,
112,
211,
211,
298,
512,
298,
512,
388,
387,
790,
903,
1022,
1535,
4096
];
array[1..numSubTrees_9, 1..4] of var DomainSize : limits_9;
array[1..numSubTrees_9] of var bool: decisions_9;
constraint treeLimits(tree_9, arcSizes_9, subTreeSizes_9, limits_9, decisions_9, rootArcIdx_9, numSubTrees_9);
constraint forall(idx in 1..numSubTrees_9) (noChildren(tree_9, decisions_9, idx));
int: rootArcIdx_10 = 15;
int: numSubTrees_10 = 15;
array[1..numSubTrees_10] of set of 1..numSubTrees_10: tree_10 = [
{},
{},
{},
{},
{},
{5, 3},
{},
{7, 4},
{},
{},
{10, 9},
{2, 11},
{1, 12},
{8, 13},
{6, 14}
];
array[1..numSubTrees_10] of DomainSize : arcSizes_10 = [
102,
102,
314,
314,
211,
1,
211,
1,
390,
391,
11,
1,
7,
1,
2039
];
array[1..numSubTrees_10] of DomainSize : subTreeSizes_10 = [
102,
102,
314,
314,
211,
526,
211,
526,
390,
391,
792,
895,
1004,
1531,
4096
];
array[1..numSubTrees_10, 1..4] of var DomainSize : limits_10;
array[1..numSubTrees_10] of var bool: decisions_10;
constraint treeLimits(tree_10, arcSizes_10, subTreeSizes_10, limits_10, decisions_10, rootArcIdx_10, numSubTrees_10);
constraint forall(idx in 1..numSubTrees_10) (noChildren(tree_10, decisions_10, idx));
int: rootArcIdx_11 = 15;
int: numSubTrees_11 = 15;
array[1..numSubTrees_11] of set of 1..numSubTrees_11: tree_11 = [
{},
{},
{},
{},
{},
{5, 3},
{},
{7, 4},
{},
{},
{10, 9},
{1, 11},
{2, 12},
{6, 13},
{8, 14}
];
array[1..numSubTrees_11] of DomainSize : arcSizes_11 = [
97,
97,
206,
206,
318,
7,
318,
7,
389,
390,
17,
1,
1,
1,
2041
];
array[1..numSubTrees_11] of DomainSize : subTreeSizes_11 = [
97,
97,
206,
206,
318,
531,
318,
531,
389,
390,
796,
894,
992,
1524,
4096
];
array[1..numSubTrees_11, 1..4] of var DomainSize : limits_11;
array[1..numSubTrees_11] of var bool: decisions_11;
constraint treeLimits(tree_11, arcSizes_11, subTreeSizes_11, limits_11, decisions_11, rootArcIdx_11, numSubTrees_11);
constraint forall(idx in 1..numSubTrees_11) (noChildren(tree_11, decisions_11, idx));
int: rootArcIdx_12 = 15;
int: numSubTrees_12 = 15;
array[1..numSubTrees_12] of set of 1..numSubTrees_12: tree_12 = [
{},
{},
{},
{},
{},
{5, 3},
{},
{7, 4},
{},
{},
{10, 9},
{1, 11},
{2, 12},
{8, 13},
{6, 14}
];
array[1..numSubTrees_12] of DomainSize : arcSizes_12 = [
85,
85,
208,
208,
328,
7,
328,
7,
388,
389,
9,
1,
23,
1,
2029
];
array[1..numSubTrees_12] of DomainSize : subTreeSizes_12 = [
85,
85,
208,
208,
328,
543,
328,
543,
388,
389,
786,
872,
980,
1524,
4096
];
array[1..numSubTrees_12, 1..4] of var DomainSize : limits_12;
array[1..numSubTrees_12] of var bool: decisions_12;
constraint treeLimits(tree_12, arcSizes_12, subTreeSizes_12, limits_12, decisions_12, rootArcIdx_12, numSubTrees_12);
constraint forall(idx in 1..numSubTrees_12) (noChildren(tree_12, decisions_12, idx));
int: rootArcIdx_13 = 15;
int: numSubTrees_13 = 15;
array[1..numSubTrees_13] of set of 1..numSubTrees_13: tree_13 = [
{},
{},
{},
{},
{},
{5, 3},
{},
{7, 4},
{},
{},
{10, 9},
{8, 11},
{6, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_13] of DomainSize : arcSizes_13 = [
73,
73,
204,
204,
342,
3,
342,
3,
382,
383,
43,
1,
3,
1,
2039
];
array[1..numSubTrees_13] of DomainSize : subTreeSizes_13 = [
73,
73,
204,
204,
342,
549,
342,
549,
382,
383,
808,
1358,
1910,
1984,
4096
];
array[1..numSubTrees_13, 1..4] of var DomainSize : limits_13;
array[1..numSubTrees_13] of var bool: decisions_13;
constraint treeLimits(tree_13, arcSizes_13, subTreeSizes_13, limits_13, decisions_13, rootArcIdx_13, numSubTrees_13);
constraint forall(idx in 1..numSubTrees_13) (noChildren(tree_13, decisions_13, idx));
int: rootArcIdx_14 = 19;
int: numSubTrees_14 = 19;
array[1..numSubTrees_14] of set of 1..numSubTrees_14: tree_14 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 2},
{10, 5},
{},
{},
{13, 12},
{8, 14},
{11, 15},
{3, 16},
{4, 17},
{1, 18}
];
array[1..numSubTrees_14] of DomainSize : arcSizes_14 = [
3,
1,
65,
65,
208,
208,
350,
3,
343,
6,
3,
389,
390,
17,
1,
3,
1,
31,
2009
];
array[1..numSubTrees_14] of DomainSize : subTreeSizes_14 = [
3,
1,
65,
65,
208,
208,
350,
561,
343,
350,
561,
389,
390,
796,
1358,
1922,
1988,
2084,
4096
];
array[1..numSubTrees_14, 1..4] of var DomainSize : limits_14;
array[1..numSubTrees_14] of var bool: decisions_14;
constraint treeLimits(tree_14, arcSizes_14, subTreeSizes_14, limits_14, decisions_14, rootArcIdx_14, numSubTrees_14);
constraint forall(idx in 1..numSubTrees_14) (noChildren(tree_14, decisions_14, idx));
int: rootArcIdx_15 = 19;
int: numSubTrees_15 = 19;
array[1..numSubTrees_15] of set of 1..numSubTrees_15: tree_15 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{},
{9, 6},
{},
{},
{12, 11},
{10, 13},
{8, 14},
{4, 15},
{3, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_15] of DomainSize : arcSizes_15 = [
7,
7,
56,
56,
356,
356,
210,
1,
210,
1,
389,
390,
15,
1,
3,
1,
101,
1,
1935
];
array[1..numSubTrees_15] of DomainSize : subTreeSizes_15 = [
7,
7,
56,
56,
356,
356,
210,
567,
210,
567,
389,
390,
794,
1362,
1932,
1989,
2146,
2154,
4096
];
array[1..numSubTrees_15, 1..4] of var DomainSize : limits_15;
array[1..numSubTrees_15] of var bool: decisions_15;
constraint treeLimits(tree_15, arcSizes_15, subTreeSizes_15, limits_15, decisions_15, rootArcIdx_15, numSubTrees_15);
constraint forall(idx in 1..numSubTrees_15) (noChildren(tree_15, decisions_15, idx));
int: rootArcIdx_16 = 19;
int: numSubTrees_16 = 19;
array[1..numSubTrees_16] of set of 1..numSubTrees_16: tree_16 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{},
{12, 11},
{4, 13},
{3, 14},
{10, 15},
{8, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_16] of DomainSize : arcSizes_16 = [
12,
12,
48,
48,
208,
208,
361,
1,
361,
1,
392,
391,
11,
1,
3,
1,
183,
1,
1853
];
array[1..numSubTrees_16] of DomainSize : subTreeSizes_16 = [
12,
12,
48,
48,
208,
208,
361,
570,
361,
570,
392,
391,
794,
843,
894,
1465,
2218,
2231,
4096
];
array[1..numSubTrees_16, 1..4] of var DomainSize : limits_16;
array[1..numSubTrees_16] of var bool: decisions_16;
constraint treeLimits(tree_16, arcSizes_16, subTreeSizes_16, limits_16, decisions_16, rootArcIdx_16, numSubTrees_16);
constraint forall(idx in 1..numSubTrees_16) (noChildren(tree_16, decisions_16, idx));
int: rootArcIdx_17 = 19;
int: numSubTrees_17 = 19;
array[1..numSubTrees_17] of set of 1..numSubTrees_17: tree_17 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{},
{12, 11},
{8, 13},
{10, 14},
{4, 15},
{3, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_17] of DomainSize : arcSizes_17 = [
19,
19,
42,
42,
361,
361,
213,
4,
213,
4,
391,
392,
11,
1,
1,
1,
265,
1,
1755
];
array[1..numSubTrees_17] of DomainSize : subTreeSizes_17 = [
19,
19,
42,
42,
361,
361,
213,
578,
213,
578,
391,
392,
794,
1373,
1952,
1995,
2302,
2322,
4096
];
array[1..numSubTrees_17, 1..4] of var DomainSize : limits_17;
array[1..numSubTrees_17] of var bool: decisions_17;
constraint treeLimits(tree_17, arcSizes_17, subTreeSizes_17, limits_17, decisions_17, rootArcIdx_17, numSubTrees_17);
constraint forall(idx in 1..numSubTrees_17) (noChildren(tree_17, decisions_17, idx));
int: rootArcIdx_18 = 19;
int: numSubTrees_18 = 19;
array[1..numSubTrees_18] of set of 1..numSubTrees_18: tree_18 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{6, 11},
{5, 12},
{8, 13},
{7, 14},
{4, 15},
{3, 16},
{1, 17},
{2, 18}
];
array[1..numSubTrees_18] of DomainSize : arcSizes_18 = [
25,
25,
36,
36,
215,
215,
364,
364,
392,
393,
3,
1,
9,
1,
1,
1,
355,
1,
1659
];
array[1..numSubTrees_18] of DomainSize : subTreeSizes_18 = [
25,
25,
36,
36,
215,
215,
364,
364,
392,
393,
788,
1004,
1228,
1593,
1958,
1995,
2386,
2412,
4096
];
array[1..numSubTrees_18, 1..4] of var DomainSize : limits_18;
array[1..numSubTrees_18] of var bool: decisions_18;
constraint treeLimits(tree_18, arcSizes_18, subTreeSizes_18, limits_18, decisions_18, rootArcIdx_18, numSubTrees_18);
constraint forall(idx in 1..numSubTrees_18) (noChildren(tree_18, decisions_18, idx));
int: rootArcIdx_19 = 19;
int: numSubTrees_19 = 19;
array[1..numSubTrees_19] of set of 1..numSubTrees_19: tree_19 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{3, 11},
{4, 12},
{6, 13},
{5, 14},
{8, 15},
{7, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_19] of DomainSize : arcSizes_19 = [
36,
36,
33,
33,
368,
368,
212,
212,
392,
391,
11,
1,
7,
1,
1,
1,
455,
1,
1537
];
array[1..numSubTrees_19] of DomainSize : subTreeSizes_19 = [
36,
36,
33,
33,
368,
368,
212,
212,
392,
391,
794,
828,
868,
1237,
1606,
1819,
2486,
2523,
4096
];
array[1..numSubTrees_19, 1..4] of var DomainSize : limits_19;
array[1..numSubTrees_19] of var bool: decisions_19;
constraint treeLimits(tree_19, arcSizes_19, subTreeSizes_19, limits_19, decisions_19, rootArcIdx_19, numSubTrees_19);
constraint forall(idx in 1..numSubTrees_19) (noChildren(tree_19, decisions_19, idx));
int: rootArcIdx_20 = 19;
int: numSubTrees_20 = 19;
array[1..numSubTrees_20] of set of 1..numSubTrees_20: tree_20 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{},
{9, 6},
{},
{},
{12, 11},
{4, 13},
{3, 14},
{10, 15},
{8, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_20] of DomainSize : arcSizes_20 = [
30,
30,
28,
28,
219,
219,
360,
2,
360,
2,
393,
394,
5,
1,
5,
1,
371,
1,
1647
];
array[1..numSubTrees_20] of DomainSize : subTreeSizes_20 = [
30,
30,
28,
28,
219,
219,
360,
581,
360,
581,
393,
394,
792,
821,
854,
1436,
2388,
2419,
4096
];
array[1..numSubTrees_20, 1..4] of var DomainSize : limits_20;
array[1..numSubTrees_20] of var bool: decisions_20;
constraint treeLimits(tree_20, arcSizes_20, subTreeSizes_20, limits_20, decisions_20, rootArcIdx_20, numSubTrees_20);
constraint forall(idx in 1..numSubTrees_20) (noChildren(tree_20, decisions_20, idx));
int: rootArcIdx_21 = 19;
int: numSubTrees_21 = 19;
array[1..numSubTrees_21] of set of 1..numSubTrees_21: tree_21 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{4, 11},
{3, 12},
{5, 13},
{6, 14},
{7, 15},
{8, 16},
{1, 17},
{2, 18}
];
array[1..numSubTrees_21] of DomainSize : arcSizes_21 = [
28,
28,
25,
25,
360,
360,
223,
223,
392,
393,
11,
1,
1,
1,
1,
1,
285,
1,
1737
];
array[1..numSubTrees_21] of DomainSize : subTreeSizes_21 = [
28,
28,
25,
25,
360,
360,
223,
223,
392,
393,
796,
822,
848,
1209,
1570,
1794,
2302,
2331,
4096
];
array[1..numSubTrees_21, 1..4] of var DomainSize : limits_21;
array[1..numSubTrees_21] of var bool: decisions_21;
constraint treeLimits(tree_21, arcSizes_21, subTreeSizes_21, limits_21, decisions_21, rootArcIdx_21, numSubTrees_21);
constraint forall(idx in 1..numSubTrees_21) (noChildren(tree_21, decisions_21, idx));
int: rootArcIdx_22 = 19;
int: numSubTrees_22 = 19;
array[1..numSubTrees_22] of set of 1..numSubTrees_22: tree_22 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{5, 11},
{6, 12},
{7, 13},
{8, 14},
{2, 15},
{1, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_22] of DomainSize : arcSizes_22 = [
25,
25,
27,
27,
225,
225,
355,
355,
393,
394,
9,
1,
1,
1,
5,
1,
209,
1,
1817
];
array[1..numSubTrees_22] of DomainSize : subTreeSizes_22 = [
25,
25,
27,
27,
225,
225,
355,
355,
393,
394,
796,
1022,
1248,
1604,
1964,
1990,
2224,
2252,
4096
];
array[1..numSubTrees_22, 1..4] of var DomainSize : limits_22;
array[1..numSubTrees_22] of var bool: decisions_22;
constraint treeLimits(tree_22, arcSizes_22, subTreeSizes_22, limits_22, decisions_22, rootArcIdx_22, numSubTrees_22);
constraint forall(idx in 1..numSubTrees_22) (noChildren(tree_22, decisions_22, idx));
int: rootArcIdx_23 = 19;
int: numSubTrees_23 = 19;
array[1..numSubTrees_23] of set of 1..numSubTrees_23: tree_23 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{},
{12, 11},
{8, 13},
{10, 14},
{1, 15},
{2, 16},
{3, 17},
{4, 18}
];
array[1..numSubTrees_23] of DomainSize : arcSizes_23 = [
22,
22,
26,
26,
231,
231,
348,
3,
348,
3,
392,
393,
9,
1,
19,
1,
125,
1,
1895
];
array[1..numSubTrees_23] of DomainSize : subTreeSizes_23 = [
22,
22,
26,
26,
231,
231,
348,
582,
348,
582,
392,
393,
794,
1377,
1978,
2001,
2148,
2175,
4096
];
array[1..numSubTrees_23, 1..4] of var DomainSize : limits_23;
array[1..numSubTrees_23] of var bool: decisions_23;
constraint treeLimits(tree_23, arcSizes_23, subTreeSizes_23, limits_23, decisions_23, rootArcIdx_23, numSubTrees_23);
constraint forall(idx in 1..numSubTrees_23) (noChildren(tree_23, decisions_23, idx));
int: rootArcIdx_24 = 19;
int: numSubTrees_24 = 19;
array[1..numSubTrees_24] of set of 1..numSubTrees_24: tree_24 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{8, 11},
{7, 12},
{1, 13},
{2, 14},
{5, 15},
{6, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_24] of DomainSize : arcSizes_24 = [
21,
21,
25,
25,
239,
239,
342,
342,
395,
396,
3,
1,
5,
1,
1,
1,
67,
1,
1971
];
array[1..numSubTrees_24] of DomainSize : subTreeSizes_24 = [
21,
21,
25,
25,
239,
239,
342,
342,
395,
396,
794,
1137,
1484,
1506,
1528,
1768,
2074,
2100,
4096
];
array[1..numSubTrees_24, 1..4] of var DomainSize : limits_24;
array[1..numSubTrees_24] of var bool: decisions_24;
constraint treeLimits(tree_24, arcSizes_24, subTreeSizes_24, limits_24, decisions_24, rootArcIdx_24, numSubTrees_24);
constraint forall(idx in 1..numSubTrees_24) (noChildren(tree_24, decisions_24, idx));
int: rootArcIdx_25 = 19;
int: numSubTrees_25 = 19;
array[1..numSubTrees_25] of set of 1..numSubTrees_25: tree_25 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{1, 11},
{2, 12},
{7, 13},
{8, 14},
{5, 15},
{6, 16},
{3, 17},
{4, 18}
];
array[1..numSubTrees_25] of DomainSize : arcSizes_25 = [
18,
18,
25,
25,
341,
341,
242,
242,
394,
395,
5,
1,
1,
1,
5,
1,
37,
1,
2003
];
array[1..numSubTrees_25] of DomainSize : subTreeSizes_25 = [
18,
18,
25,
25,
341,
341,
242,
242,
394,
395,
794,
813,
832,
1075,
1322,
1664,
2042,
2068,
4096
];
array[1..numSubTrees_25, 1..4] of var DomainSize : limits_25;
array[1..numSubTrees_25] of var bool: decisions_25;
constraint treeLimits(tree_25, arcSizes_25, subTreeSizes_25, limits_25, decisions_25, rootArcIdx_25, numSubTrees_25);
constraint forall(idx in 1..numSubTrees_25) (noChildren(tree_25, decisions_25, idx));
int: rootArcIdx_26 = 19;
int: numSubTrees_26 = 19;
array[1..numSubTrees_26] of set of 1..numSubTrees_26: tree_26 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{8, 3},
{},
{10, 5},
{11, 4},
{},
{},
{14, 13},
{1, 15},
{2, 16},
{9, 17},
{12, 18}
];
array[1..numSubTrees_26] of DomainSize : arcSizes_26 = [
17,
17,
24,
24,
331,
331,
248,
2,
6,
248,
2,
6,
396,
397,
5,
1,
3,
1,
2037
];
array[1..numSubTrees_26] of DomainSize : subTreeSizes_26 = [
17,
17,
24,
24,
331,
331,
248,
581,
611,
248,
581,
611,
396,
397,
798,
816,
836,
1448,
4096
];
array[1..numSubTrees_26, 1..4] of var DomainSize : limits_26;
array[1..numSubTrees_26] of var bool: decisions_26;
constraint treeLimits(tree_26, arcSizes_26, subTreeSizes_26, limits_26, decisions_26, rootArcIdx_26, numSubTrees_26);
constraint forall(idx in 1..numSubTrees_26) (noChildren(tree_26, decisions_26, idx));
int: rootArcIdx_27 = 19;
int: numSubTrees_27 = 19;
array[1..numSubTrees_27] of set of 1..numSubTrees_27: tree_27 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{},
{10, 9},
{2, 11},
{1, 12},
{6, 13},
{5, 14},
{3, 15},
{4, 16},
{8, 17},
{7, 18}
];
array[1..numSubTrees_27] of DomainSize : arcSizes_27 = [
16,
16,
25,
25,
258,
258,
328,
328,
392,
393,
3,
1,
3,
1,
3,
1,
1,
1,
2043
];
array[1..numSubTrees_27] of DomainSize : subTreeSizes_27 = [
16,
16,
25,
25,
258,
258,
328,
328,
392,
393,
788,
805,
824,
1083,
1344,
1370,
1396,
1725,
4096
];
array[1..numSubTrees_27, 1..4] of var DomainSize : limits_27;
array[1..numSubTrees_27] of var bool: decisions_27;
constraint treeLimits(tree_27, arcSizes_27, subTreeSizes_27, limits_27, decisions_27, rootArcIdx_27, numSubTrees_27);
constraint forall(idx in 1..numSubTrees_27) (noChildren(tree_27, decisions_27, idx));
int: rootArcIdx_28 = 21;
int: numSubTrees_28 = 21;
array[1..numSubTrees_28] of set of 1..numSubTrees_28: tree_28 = [
{},
{},
{},
{},
{},
{},
{6, 4},
{},
{8, 5},
{},
{},
{},
{},
{13, 12},
{3, 14},
{2, 15},
{11, 16},
{10, 17},
{1, 18},
{9, 19},
{7, 20}
];
array[1..numSubTrees_28] of DomainSize : arcSizes_28 = [
1,
14,
14,
25,
25,
265,
2,
265,
2,
317,
317,
396,
397,
1,
1,
5,
1,
1,
7,
1,
2039
];
array[1..numSubTrees_28] of DomainSize : subTreeSizes_28 = [
1,
14,
14,
25,
25,
265,
292,
265,
292,
317,
317,
396,
397,
794,
809,
828,
1146,
1464,
1472,
1765,
4096
];
array[1..numSubTrees_28, 1..4] of var DomainSize : limits_28;
array[1..numSubTrees_28] of var bool: decisions_28;
constraint treeLimits(tree_28, arcSizes_28, subTreeSizes_28, limits_28, decisions_28, rootArcIdx_28, numSubTrees_28);
constraint forall(idx in 1..numSubTrees_28) (noChildren(tree_28, decisions_28, idx));
int: rootArcIdx_29 = 21;
int: numSubTrees_29 = 21;
array[1..numSubTrees_29] of set of 1..numSubTrees_29: tree_29 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 7},
{9, 1},
{},
{11, 6},
{},
{},
{14, 13},
{12, 15},
{10, 16},
{5, 17},
{4, 18},
{3, 19},
{2, 20}
];
array[1..numSubTrees_29] of DomainSize : arcSizes_29 = [
2,
15,
15,
25,
25,
309,
309,
271,
2,
1,
271,
5,
395,
396,
3,
1,
13,
4,
2,
1,
2031
];
array[1..numSubTrees_29] of DomainSize : subTreeSizes_29 = [
2,
15,
15,
25,
25,
309,
309,
271,
582,
585,
271,
585,
395,
396,
794,
1380,
1978,
2007,
2034,
2050,
4096
];
array[1..numSubTrees_29, 1..4] of var DomainSize : limits_29;
array[1..numSubTrees_29] of var bool: decisions_29;
constraint treeLimits(tree_29, arcSizes_29, subTreeSizes_29, limits_29, decisions_29, rootArcIdx_29, numSubTrees_29);
constraint forall(idx in 1..numSubTrees_29) (noChildren(tree_29, decisions_29, idx));
int: rootArcIdx_30 = 23;
int: numSubTrees_30 = 23;
array[1..numSubTrees_30] of set of 1..numSubTrees_30: tree_30 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 3},
{},
{11, 4},
{},
{},
{14, 13},
{12, 15},
{10, 16},
{6, 17},
{5, 18},
{7, 19},
{8, 20},
{2, 21},
{1, 22}
];
array[1..numSubTrees_30] of DomainSize : arcSizes_30 = [
6,
6,
22,
22,
15,
15,
302,
302,
277,
1,
277,
1,
397,
398,
1,
1,
3,
1,
1,
1,
9,
1,
2037
];
array[1..numSubTrees_30] of DomainSize : subTreeSizes_30 = [
6,
6,
22,
22,
15,
15,
302,
302,
277,
300,
277,
300,
397,
398,
796,
1097,
1400,
1416,
1432,
1735,
2046,
2053,
4096
];
array[1..numSubTrees_30, 1..4] of var DomainSize : limits_30;
array[1..numSubTrees_30] of var bool: decisions_30;
constraint treeLimits(tree_30, arcSizes_30, subTreeSizes_30, limits_30, decisions_30, rootArcIdx_30, numSubTrees_30);
constraint forall(idx in 1..numSubTrees_30) (noChildren(tree_30, decisions_30, idx));
int: rootArcIdx_31 = 23;
int: numSubTrees_31 = 23;
array[1..numSubTrees_31] of set of 1..numSubTrees_31: tree_31 = [
{},
{},
{},
{},
{},
{},
{},
{7, 4},
{},
{9, 3},
{},
{},
{},
{13, 11},
{14, 5},
{},
{16, 12},
{6, 17},
{15, 18},
{1, 19},
{2, 20},
{10, 21},
{8, 22}
];
array[1..numSubTrees_31] of DomainSize : arcSizes_31 = [
9,
9,
19,
19,
15,
15,
289,
1,
289,
1,
292,
292,
396,
1,
1,
396,
1,
2,
1,
1,
7,
1,
2039
];
array[1..numSubTrees_31] of DomainSize : subTreeSizes_31 = [
9,
9,
19,
19,
15,
15,
289,
309,
289,
309,
292,
292,
396,
689,
705,
396,
689,
706,
1412,
1422,
1438,
1748,
4096
];
array[1..numSubTrees_31, 1..4] of var DomainSize : limits_31;
array[1..numSubTrees_31] of var bool: decisions_31;
constraint treeLimits(tree_31, arcSizes_31, subTreeSizes_31, limits_31, decisions_31, rootArcIdx_31, numSubTrees_31);
constraint forall(idx in 1..numSubTrees_31) (noChildren(tree_31, decisions_31, idx));
int: rootArcIdx_32 = 23;
int: numSubTrees_32 = 23;
array[1..numSubTrees_32] of set of 1..numSubTrees_32: tree_32 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{},
{9, 4},
{},
{},
{},
{13, 12},
{14, 10},
{},
{16, 11},
{8, 17},
{15, 18},
{6, 19},
{5, 20},
{1, 21},
{2, 22}
];
array[1..numSubTrees_32] of DomainSize : arcSizes_32 = [
12,
12,
15,
15,
13,
13,
282,
1,
282,
1,
291,
291,
403,
5,
2,
403,
5,
2,
4,
1,
9,
1,
2033
];
array[1..numSubTrees_32] of DomainSize : subTreeSizes_32 = [
12,
12,
15,
15,
13,
13,
282,
298,
282,
298,
291,
291,
403,
699,
999,
403,
699,
999,
2002,
2016,
2038,
2051,
4096
];
array[1..numSubTrees_32, 1..4] of var DomainSize : limits_32;
array[1..numSubTrees_32] of var bool: decisions_32;
constraint treeLimits(tree_32, arcSizes_32, subTreeSizes_32, limits_32, decisions_32, rootArcIdx_32, numSubTrees_32);
constraint forall(idx in 1..numSubTrees_32) (noChildren(tree_32, decisions_32, idx));
int: rootArcIdx_33 = 23;
int: numSubTrees_33 = 23;
array[1..numSubTrees_33] of set of 1..numSubTrees_33: tree_33 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 1},
{10, 7},
{},
{12, 2},
{13, 8},
{},
{11, 15},
{},
{14, 17},
{16, 18},
{6, 19},
{5, 20},
{4, 21},
{3, 22}
];
array[1..numSubTrees_33] of DomainSize : arcSizes_33 = [
8,
8,
19,
19,
12,
12,
282,
282,
303,
1,
2,
303,
1,
2,
396,
1,
396,
1,
8,
1,
3,
1,
2035
];
array[1..numSubTrees_33] of DomainSize : subTreeSizes_33 = [
8,
8,
19,
19,
12,
12,
282,
282,
303,
312,
596,
303,
312,
596,
396,
993,
396,
993,
1994,
2007,
2022,
2042,
4096
];
array[1..numSubTrees_33, 1..4] of var DomainSize : limits_33;
array[1..numSubTrees_33] of var bool: decisions_33;
constraint treeLimits(tree_33, arcSizes_33, subTreeSizes_33, limits_33, decisions_33, rootArcIdx_33, numSubTrees_33);
constraint forall(idx in 1..numSubTrees_33) (noChildren(tree_33, decisions_33, idx));
int: rootArcIdx_34 = 23;
int: numSubTrees_34 = 23;
array[1..numSubTrees_34] of set of 1..numSubTrees_34: tree_34 = [
{},
{},
{},
{},
{},
{},
{},
{7, 2},
{},
{9, 1},
{},
{},
{},
{13, 11},
{14, 3},
{15, 5},
{16, 8},
{},
{18, 12},
{4, 19},
{6, 20},
{10, 21},
{17, 22}
];
array[1..numSubTrees_34] of DomainSize : arcSizes_34 = [
4,
4,
13,
13,
24,
24,
305,
2,
305,
2,
275,
275,
395,
3,
4,
3,
1,
395,
3,
4,
3,
1,
2038
];
array[1..numSubTrees_34] of DomainSize : subTreeSizes_34 = [
4,
4,
13,
13,
24,
24,
305,
311,
305,
311,
275,
275,
395,
673,
690,
717,
1029,
395,
673,
690,
717,
1029,
4096
];
array[1..numSubTrees_34, 1..4] of var DomainSize : limits_34;
array[1..numSubTrees_34] of var bool: decisions_34;
constraint treeLimits(tree_34, arcSizes_34, subTreeSizes_34, limits_34, decisions_34, rootArcIdx_34, numSubTrees_34);
constraint forall(idx in 1..numSubTrees_34) (noChildren(tree_34, decisions_34, idx));
int: rootArcIdx_35 = 21;
int: numSubTrees_35 = 21;
array[1..numSubTrees_35] of set of 1..numSubTrees_35: tree_35 = [
{},
{},
{},
{},
{},
{},
{6, 5},
{},
{8, 4},
{},
{},
{},
{12, 2},
{13, 10},
{14, 9},
{},
{16, 3},
{11, 17},
{1, 18},
{7, 19},
{15, 20}
];
array[1..numSubTrees_35] of DomainSize : arcSizes_35 = [
2,
15,
15,
25,
26,
266,
3,
266,
4,
314,
314,
396,
2,
8,
2,
396,
2,
4,
2,
2,
2032
];
array[1..numSubTrees_35] of DomainSize : subTreeSizes_35 = [
2,
15,
15,
25,
26,
266,
295,
266,
295,
314,
314,
396,
413,
735,
1032,
396,
413,
731,
735,
1032,
4096
];
array[1..numSubTrees_35, 1..4] of var DomainSize : limits_35;
array[1..numSubTrees_35] of var bool: decisions_35;
constraint treeLimits(tree_35, arcSizes_35, subTreeSizes_35, limits_35, decisions_35, rootArcIdx_35, numSubTrees_35);
constraint forall(idx in 1..numSubTrees_35) (noChildren(tree_35, decisions_35, idx));
int: rootArcIdx_36 = 21;
int: numSubTrees_36 = 21;
array[1..numSubTrees_36] of set of 1..numSubTrees_36: tree_36 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 4},
{},
{10, 5},
{},
{12, 6},
{13, 9},
{},
{15, 7},
{11, 16},
{1, 17},
{14, 18},
{3, 19},
{2, 20}
];
array[1..numSubTrees_36] of DomainSize : arcSizes_36 = [
1,
16,
16,
26,
26,
322,
322,
261,
1,
261,
1,
397,
1,
3,
397,
1,
1,
1,
6,
1,
2035
];
array[1..numSubTrees_36] of DomainSize : subTreeSizes_36 = [
1,
16,
16,
26,
26,
322,
322,
261,
288,
261,
288,
397,
720,
1011,
397,
720,
1009,
1011,
2028,
2045,
4096
];
array[1..numSubTrees_36, 1..4] of var DomainSize : limits_36;
array[1..numSubTrees_36] of var bool: decisions_36;
constraint treeLimits(tree_36, arcSizes_36, subTreeSizes_36, limits_36, decisions_36, rootArcIdx_36, numSubTrees_36);
constraint forall(idx in 1..numSubTrees_36) (noChildren(tree_36, decisions_36, idx));
int: rootArcIdx_37 = 19;
int: numSubTrees_37 = 19;
array[1..numSubTrees_37] of set of 1..numSubTrees_37: tree_37 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 1},
{10, 5},
{11, 3},
{12, 7},
{},
{14, 2},
{6, 15},
{4, 16},
{8, 17},
{13, 18}
];
array[1..numSubTrees_37] of DomainSize : arcSizes_37 = [
15,
15,
25,
25,
254,
254,
330,
330,
397,
2,
1,
1,
5,
397,
2,
1,
1,
5,
2036
];
array[1..numSubTrees_37] of DomainSize : subTreeSizes_37 = [
15,
15,
25,
25,
254,
254,
330,
330,
397,
414,
669,
695,
1030,
397,
414,
669,
695,
1030,
4096
];
array[1..numSubTrees_37, 1..4] of var DomainSize : limits_37;
array[1..numSubTrees_37] of var bool: decisions_37;
constraint treeLimits(tree_37, arcSizes_37, subTreeSizes_37, limits_37, decisions_37, rootArcIdx_37, numSubTrees_37);
constraint forall(idx in 1..numSubTrees_37) (noChildren(tree_37, decisions_37, idx));
int: rootArcIdx_38 = 19;
int: numSubTrees_38 = 19;
array[1..numSubTrees_38] of set of 1..numSubTrees_38: tree_38 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 6},
{10, 1},
{11, 8},
{},
{13, 5},
{2, 14},
{7, 15},
{12, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_38] of DomainSize : arcSizes_38 = [
18,
18,
25,
25,
244,
244,
337,
337,
395,
4,
4,
4,
395,
4,
4,
4,
6,
1,
2027
];
array[1..numSubTrees_38] of DomainSize : subTreeSizes_38 = [
18,
18,
25,
25,
244,
244,
337,
337,
395,
643,
665,
1006,
395,
643,
665,
1006,
2018,
2044,
4096
];
array[1..numSubTrees_38, 1..4] of var DomainSize : limits_38;
array[1..numSubTrees_38] of var bool: decisions_38;
constraint treeLimits(tree_38, arcSizes_38, subTreeSizes_38, limits_38, decisions_38, rootArcIdx_38, numSubTrees_38);
constraint forall(idx in 1..numSubTrees_38) (noChildren(tree_38, decisions_38, idx));
int: rootArcIdx_39 = 19;
int: numSubTrees_39 = 19;
array[1..numSubTrees_39] of set of 1..numSubTrees_39: tree_39 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{10, 11},
{},
{8, 13},
{12, 14},
{1, 15},
{2, 16},
{3, 17},
{4, 18}
];
array[1..numSubTrees_39] of DomainSize : arcSizes_39 = [
20,
20,
24,
24,
342,
342,
238,
3,
238,
3,
398,
1,
398,
1,
8,
1,
53,
1,
1981
];
array[1..numSubTrees_39] of DomainSize : subTreeSizes_39 = [
20,
20,
24,
24,
342,
342,
238,
583,
238,
583,
398,
982,
398,
982,
1972,
1993,
2066,
2091,
4096
];
array[1..numSubTrees_39, 1..4] of var DomainSize : limits_39;
array[1..numSubTrees_39] of var bool: decisions_39;
constraint treeLimits(tree_39, arcSizes_39, subTreeSizes_39, limits_39, decisions_39, rootArcIdx_39, numSubTrees_39);
constraint forall(idx in 1..numSubTrees_39) (noChildren(tree_39, decisions_39, idx));
int: rootArcIdx_40 = 19;
int: numSubTrees_40 = 19;
array[1..numSubTrees_40] of set of 1..numSubTrees_40: tree_40 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 7},
{10, 6},
{11, 1},
{},
{13, 8},
{5, 14},
{2, 15},
{12, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_40] of DomainSize : arcSizes_40 = [
22,
22,
26,
26,
231,
231,
348,
348,
396,
4,
3,
6,
396,
4,
3,
6,
98,
1,
1925
];
array[1..numSubTrees_40] of DomainSize : subTreeSizes_40 = [
22,
22,
26,
26,
231,
231,
348,
348,
396,
748,
982,
1010,
396,
748,
982,
1010,
2118,
2145,
4096
];
array[1..numSubTrees_40, 1..4] of var DomainSize : limits_40;
array[1..numSubTrees_40] of var bool: decisions_40;
constraint treeLimits(tree_40, arcSizes_40, subTreeSizes_40, limits_40, decisions_40, rootArcIdx_40, numSubTrees_40);
constraint forall(idx in 1..numSubTrees_40) (noChildren(tree_40, decisions_40, idx));
int: rootArcIdx_41 = 19;
int: numSubTrees_41 = 19;
array[1..numSubTrees_41] of set of 1..numSubTrees_41: tree_41 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 6},
{10, 8},
{11, 2},
{},
{13, 5},
{7, 14},
{1, 15},
{12, 16},
{3, 17},
{4, 18}
];
array[1..numSubTrees_41] of DomainSize : arcSizes_41 = [
22,
22,
26,
26,
229,
229,
354,
354,
396,
3,
1,
5,
396,
3,
1,
5,
172,
1,
1851
];
array[1..numSubTrees_41] of DomainSize : subTreeSizes_41 = [
22,
22,
26,
26,
229,
229,
354,
354,
396,
628,
983,
1010,
396,
628,
983,
1010,
2192,
2219,
4096
];
array[1..numSubTrees_41, 1..4] of var DomainSize : limits_41;
array[1..numSubTrees_41] of var bool: decisions_41;
constraint treeLimits(tree_41, arcSizes_41, subTreeSizes_41, limits_41, decisions_41, rootArcIdx_41, numSubTrees_41);
constraint forall(idx in 1..numSubTrees_41) (noChildren(tree_41, decisions_41, idx));
int: rootArcIdx_42 = 19;
int: numSubTrees_42 = 19;
array[1..numSubTrees_42] of set of 1..numSubTrees_42: tree_42 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 7},
{10, 6},
{11, 2},
{},
{13, 8},
{5, 14},
{1, 15},
{12, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_42] of DomainSize : arcSizes_42 = [
25,
25,
27,
27,
225,
225,
354,
354,
395,
6,
4,
3,
395,
6,
4,
3,
238,
1,
1779
];
array[1..numSubTrees_42] of DomainSize : subTreeSizes_42 = [
25,
25,
27,
27,
225,
225,
354,
354,
395,
755,
984,
1012,
395,
755,
984,
1012,
2262,
2290,
4096
];
array[1..numSubTrees_42, 1..4] of var DomainSize : limits_42;
array[1..numSubTrees_42] of var bool: decisions_42;
constraint treeLimits(tree_42, arcSizes_42, subTreeSizes_42, limits_42, decisions_42, rootArcIdx_42, numSubTrees_42);
constraint forall(idx in 1..numSubTrees_42) (noChildren(tree_42, decisions_42, idx));
int: rootArcIdx_43 = 19;
int: numSubTrees_43 = 19;
array[1..numSubTrees_43] of set of 1..numSubTrees_43: tree_43 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{10, 11},
{},
{8, 13},
{12, 14},
{4, 15},
{3, 16},
{1, 17},
{2, 18}
];
array[1..numSubTrees_43] of DomainSize : arcSizes_43 = [
29,
29,
28,
28,
360,
360,
219,
1,
219,
1,
395,
9,
395,
9,
2,
1,
325,
1,
1685
];
array[1..numSubTrees_43] of DomainSize : subTreeSizes_43 = [
29,
29,
28,
28,
360,
360,
219,
580,
219,
580,
395,
984,
395,
984,
1970,
1999,
2352,
2382,
4096
];
array[1..numSubTrees_43, 1..4] of var DomainSize : limits_43;
array[1..numSubTrees_43] of var bool: decisions_43;
constraint treeLimits(tree_43, arcSizes_43, subTreeSizes_43, limits_43, decisions_43, rootArcIdx_43, numSubTrees_43);
constraint forall(idx in 1..numSubTrees_43) (noChildren(tree_43, decisions_43, idx));
int: rootArcIdx_44 = 19;
int: numSubTrees_44 = 19;
array[1..numSubTrees_44] of set of 1..numSubTrees_44: tree_44 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 4},
{10, 5},
{11, 8},
{},
{13, 3},
{6, 14},
{7, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_44] of DomainSize : arcSizes_44 = [
32,
32,
29,
29,
218,
218,
368,
368,
394,
2,
4,
3,
394,
2,
4,
3,
392,
1,
1603
];
array[1..numSubTrees_44] of DomainSize : subTreeSizes_44 = [
32,
32,
29,
29,
218,
218,
368,
368,
394,
425,
647,
1018,
394,
425,
647,
1018,
2428,
2461,
4096
];
array[1..numSubTrees_44, 1..4] of var DomainSize : limits_44;
array[1..numSubTrees_44] of var bool: decisions_44;
constraint treeLimits(tree_44, arcSizes_44, subTreeSizes_44, limits_44, decisions_44, rootArcIdx_44, numSubTrees_44);
constraint forall(idx in 1..numSubTrees_44) (noChildren(tree_44, decisions_44, idx));
int: rootArcIdx_45 = 19;
int: numSubTrees_45 = 19;
array[1..numSubTrees_45] of set of 1..numSubTrees_45: tree_45 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 5},
{10, 7},
{},
{12, 6},
{8, 13},
{11, 14},
{3, 15},
{4, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_45] of DomainSize : arcSizes_45 = [
29,
29,
36,
36,
357,
357,
217,
217,
397,
1,
1,
397,
1,
1,
22,
1,
399,
1,
1597
];
array[1..numSubTrees_45] of DomainSize : subTreeSizes_45 = [
29,
29,
36,
36,
357,
357,
217,
217,
397,
755,
973,
397,
755,
973,
1968,
2005,
2440,
2470,
4096
];
array[1..numSubTrees_45, 1..4] of var DomainSize : limits_45;
array[1..numSubTrees_45] of var bool: decisions_45;
constraint treeLimits(tree_45, arcSizes_45, subTreeSizes_45, limits_45, decisions_45, rootArcIdx_45, numSubTrees_45);
constraint forall(idx in 1..numSubTrees_45) (noChildren(tree_45, decisions_45, idx));
int: rootArcIdx_46 = 19;
int: numSubTrees_46 = 19;
array[1..numSubTrees_46] of set of 1..numSubTrees_46: tree_46 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{8, 11},
{},
{10, 13},
{12, 14},
{4, 15},
{3, 16},
{1, 17},
{2, 18}
];
array[1..numSubTrees_46] of DomainSize : arcSizes_46 = [
23,
23,
40,
40,
363,
363,
211,
1,
211,
1,
397,
5,
397,
5,
2,
1,
305,
1,
1707
];
array[1..numSubTrees_46] of DomainSize : subTreeSizes_46 = [
23,
23,
40,
40,
363,
363,
211,
575,
211,
575,
397,
977,
397,
977,
1956,
1997,
2342,
2366,
4096
];
array[1..numSubTrees_46, 1..4] of var DomainSize : limits_46;
array[1..numSubTrees_46] of var bool: decisions_46;
constraint treeLimits(tree_46, arcSizes_46, subTreeSizes_46, limits_46, decisions_46, rootArcIdx_46, numSubTrees_46);
constraint forall(idx in 1..numSubTrees_46) (noChildren(tree_46, decisions_46, idx));
int: rootArcIdx_47 = 19;
int: numSubTrees_47 = 19;
array[1..numSubTrees_47] of set of 1..numSubTrees_47: tree_47 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 5},
{10, 3},
{11, 7},
{},
{13, 6},
{4, 14},
{8, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_47] of DomainSize : arcSizes_47 = [
15,
15,
45,
45,
363,
363,
210,
210,
397,
1,
1,
2,
397,
1,
1,
2,
218,
1,
1809
];
array[1..numSubTrees_47] of DomainSize : subTreeSizes_47 = [
15,
15,
45,
45,
363,
363,
210,
210,
397,
761,
807,
1019,
397,
761,
807,
1019,
2256,
2272,
4096
];
array[1..numSubTrees_47, 1..4] of var DomainSize : limits_47;
array[1..numSubTrees_47] of var bool: decisions_47;
constraint treeLimits(tree_47, arcSizes_47, subTreeSizes_47, limits_47, decisions_47, rootArcIdx_47, numSubTrees_47);
constraint forall(idx in 1..numSubTrees_47) (noChildren(tree_47, decisions_47, idx));
int: rootArcIdx_48 = 19;
int: numSubTrees_48 = 19;
array[1..numSubTrees_48] of set of 1..numSubTrees_48: tree_48 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 7},
{10, 5},
{11, 3},
{},
{13, 8},
{6, 14},
{4, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_48] of DomainSize : arcSizes_48 = [
10,
10,
53,
53,
208,
208,
361,
361,
398,
1,
7,
2,
398,
1,
7,
2,
124,
1,
1891
];
array[1..numSubTrees_48] of DomainSize : subTreeSizes_48 = [
10,
10,
53,
53,
208,
208,
361,
361,
398,
760,
975,
1030,
398,
760,
975,
1030,
2184,
2195,
4096
];
array[1..numSubTrees_48, 1..4] of var DomainSize : limits_48;
array[1..numSubTrees_48] of var bool: decisions_48;
constraint treeLimits(tree_48, arcSizes_48, subTreeSizes_48, limits_48, decisions_48, rootArcIdx_48, numSubTrees_48);
constraint forall(idx in 1..numSubTrees_48) (noChildren(tree_48, decisions_48, idx));
int: rootArcIdx_49 = 19;
int: numSubTrees_49 = 19;
array[1..numSubTrees_49] of set of 1..numSubTrees_49: tree_49 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 7},
{10, 3},
{11, 5},
{},
{13, 8},
{4, 14},
{6, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_49] of DomainSize : arcSizes_49 = [
5,
5,
61,
61,
209,
209,
349,
349,
392,
7,
4,
3,
392,
7,
4,
3,
50,
1,
1985
];
array[1..numSubTrees_49] of DomainSize : subTreeSizes_49 = [
5,
5,
61,
61,
209,
209,
349,
349,
392,
748,
813,
1025,
392,
748,
813,
1025,
2100,
2106,
4096
];
array[1..numSubTrees_49, 1..4] of var DomainSize : limits_49;
array[1..numSubTrees_49] of var bool: decisions_49;
constraint treeLimits(tree_49, arcSizes_49, subTreeSizes_49, limits_49, decisions_49, rootArcIdx_49, numSubTrees_49);
constraint forall(idx in 1..numSubTrees_49) (noChildren(tree_49, decisions_49, idx));
int: rootArcIdx_50 = 17;
int: numSubTrees_50 = 17;
array[1..numSubTrees_50] of set of 1..numSubTrees_50: tree_50 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 6},
{9, 2},
{10, 5},
{},
{12, 7},
{3, 13},
{4, 14},
{11, 15},
{1, 16}
];
array[1..numSubTrees_50] of DomainSize : arcSizes_50 = [
1,
69,
69,
347,
347,
209,
209,
396,
2,
2,
3,
396,
2,
2,
3,
4,
2035
];
array[1..numSubTrees_50] of DomainSize : subTreeSizes_50 = [
1,
69,
69,
347,
347,
209,
209,
396,
607,
678,
1028,
396,
607,
678,
1028,
2060,
4096
];
array[1..numSubTrees_50, 1..4] of var DomainSize : limits_50;
array[1..numSubTrees_50] of var bool: decisions_50;
constraint treeLimits(tree_50, arcSizes_50, subTreeSizes_50, limits_50, decisions_50, rootArcIdx_50, numSubTrees_50);
constraint forall(idx in 1..numSubTrees_50) (noChildren(tree_50, decisions_50, idx));
int: rootArcIdx_51 = 15;
int: numSubTrees_51 = 15;
array[1..numSubTrees_51] of set of 1..numSubTrees_51: tree_51 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{8, 3},
{9, 1},
{},
{11, 5},
{4, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_51] of DomainSize : arcSizes_51 = [
87,
87,
328,
328,
204,
204,
391,
2,
27,
1,
391,
2,
27,
1,
2016
];
array[1..numSubTrees_51] of DomainSize : subTreeSizes_51 = [
87,
87,
328,
328,
204,
204,
391,
597,
952,
1040,
391,
597,
952,
1040,
4096
];
array[1..numSubTrees_51, 1..4] of var DomainSize : limits_51;
array[1..numSubTrees_51] of var bool: decisions_51;
constraint treeLimits(tree_51, arcSizes_51, subTreeSizes_51, limits_51, decisions_51, rootArcIdx_51, numSubTrees_51);
constraint forall(idx in 1..numSubTrees_51) (noChildren(tree_51, decisions_51, idx));
int: rootArcIdx_52 = 15;
int: numSubTrees_52 = 15;
array[1..numSubTrees_52] of set of 1..numSubTrees_52: tree_52 = [
{},
{},
{},
{},
{},
{},
{},
{7, 4},
{8, 1},
{9, 6},
{},
{11, 3},
{2, 12},
{5, 13},
{10, 14}
];
array[1..numSubTrees_52] of DomainSize : arcSizes_52 = [
92,
92,
206,
206,
325,
325,
396,
4,
2,
4,
396,
4,
2,
4,
2038
];
array[1..numSubTrees_52] of DomainSize : subTreeSizes_52 = [
92,
92,
206,
206,
325,
325,
396,
606,
700,
1029,
396,
606,
700,
1029,
4096
];
array[1..numSubTrees_52, 1..4] of var DomainSize : limits_52;
array[1..numSubTrees_52] of var bool: decisions_52;
constraint treeLimits(tree_52, arcSizes_52, subTreeSizes_52, limits_52, decisions_52, rootArcIdx_52, numSubTrees_52);
constraint forall(idx in 1..numSubTrees_52) (noChildren(tree_52, decisions_52, idx));
int: rootArcIdx_53 = 15;
int: numSubTrees_53 = 15;
array[1..numSubTrees_53] of set of 1..numSubTrees_53: tree_53 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{8, 6},
{},
{10, 4},
{5, 11},
{9, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_53] of DomainSize : arcSizes_53 = [
101,
101,
312,
312,
209,
209,
397,
4,
1,
397,
4,
1,
12,
1,
2035
];
array[1..numSubTrees_53] of DomainSize : subTreeSizes_53 = [
101,
101,
312,
312,
209,
209,
397,
713,
923,
397,
713,
923,
1858,
1960,
4096
];
array[1..numSubTrees_53, 1..4] of var DomainSize : limits_53;
array[1..numSubTrees_53] of var bool: decisions_53;
constraint treeLimits(tree_53, arcSizes_53, subTreeSizes_53, limits_53, decisions_53, rootArcIdx_53, numSubTrees_53);
constraint forall(idx in 1..numSubTrees_53) (noChildren(tree_53, decisions_53, idx));
int: rootArcIdx_54 = 15;
int: numSubTrees_54 = 15;
array[1..numSubTrees_54] of set of 1..numSubTrees_54: tree_54 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{8, 4},
{9, 1},
{},
{11, 6},
{3, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_54] of DomainSize : arcSizes_54 = [
110,
110,
305,
305,
209,
209,
398,
3,
2,
7,
398,
3,
2,
7,
2028
];
array[1..numSubTrees_54] of DomainSize : subTreeSizes_54 = [
110,
110,
305,
305,
209,
209,
398,
610,
917,
1034,
398,
610,
917,
1034,
4096
];
array[1..numSubTrees_54, 1..4] of var DomainSize : limits_54;
array[1..numSubTrees_54] of var bool: decisions_54;
constraint treeLimits(tree_54, arcSizes_54, subTreeSizes_54, limits_54, decisions_54, rootArcIdx_54, numSubTrees_54);
constraint forall(idx in 1..numSubTrees_54) (noChildren(tree_54, decisions_54, idx));
int: rootArcIdx_55 = 15;
int: numSubTrees_55 = 15;
array[1..numSubTrees_55] of set of 1..numSubTrees_55: tree_55 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{8, 5},
{9, 1},
{},
{11, 4},
{6, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_55] of DomainSize : arcSizes_55 = [
116,
116,
295,
295,
212,
212,
394,
6,
2,
1,
394,
6,
2,
1,
2044
];
array[1..numSubTrees_55] of DomainSize : subTreeSizes_55 = [
116,
116,
295,
295,
212,
212,
394,
695,
909,
1026,
394,
695,
909,
1026,
4096
];
array[1..numSubTrees_55, 1..4] of var DomainSize : limits_55;
array[1..numSubTrees_55] of var bool: decisions_55;
constraint treeLimits(tree_55, arcSizes_55, subTreeSizes_55, limits_55, decisions_55, rootArcIdx_55, numSubTrees_55);
constraint forall(idx in 1..numSubTrees_55) (noChildren(tree_55, decisions_55, idx));
int: rootArcIdx_56 = 15;
int: numSubTrees_56 = 15;
array[1..numSubTrees_56] of set of 1..numSubTrees_56: tree_56 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{8, 6},
{},
{10, 4},
{5, 11},
{9, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_56] of DomainSize : arcSizes_56 = [
124,
124,
289,
289,
215,
215,
397,
1,
3,
397,
1,
3,
4,
1,
2033
];
array[1..numSubTrees_56] of DomainSize : subTreeSizes_56 = [
124,
124,
289,
289,
215,
215,
397,
687,
905,
397,
687,
905,
1814,
1939,
4096
];
array[1..numSubTrees_56, 1..4] of var DomainSize : limits_56;
array[1..numSubTrees_56] of var bool: decisions_56;
constraint treeLimits(tree_56, arcSizes_56, subTreeSizes_56, limits_56, decisions_56, rootArcIdx_56, numSubTrees_56);
constraint forall(idx in 1..numSubTrees_56) (noChildren(tree_56, decisions_56, idx));
int: rootArcIdx_57 = 15;
int: numSubTrees_57 = 15;
array[1..numSubTrees_57] of set of 1..numSubTrees_57: tree_57 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{8, 6},
{9, 1},
{},
{11, 4},
{5, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_57] of DomainSize : arcSizes_57 = [
129,
129,
276,
276,
215,
215,
394,
4,
5,
6,
394,
4,
5,
6,
2038
];
array[1..numSubTrees_57] of DomainSize : subTreeSizes_57 = [
129,
129,
276,
276,
215,
215,
394,
674,
894,
1029,
394,
674,
894,
1029,
4096
];
array[1..numSubTrees_57, 1..4] of var DomainSize : limits_57;
array[1..numSubTrees_57] of var bool: decisions_57;
constraint treeLimits(tree_57, arcSizes_57, subTreeSizes_57, limits_57, decisions_57, rootArcIdx_57, numSubTrees_57);
constraint forall(idx in 1..numSubTrees_57) (noChildren(tree_57, decisions_57, idx));
int: rootArcIdx_58 = 15;
int: numSubTrees_58 = 15;
array[1..numSubTrees_58] of set of 1..numSubTrees_58: tree_58 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{8, 3},
{},
{10, 5},
{4, 11},
{9, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_58] of DomainSize : arcSizes_58 = [
135,
135,
272,
272,
217,
217,
392,
3,
6,
392,
3,
6,
10,
1,
2035
];
array[1..numSubTrees_58] of DomainSize : subTreeSizes_58 = [
135,
135,
272,
272,
217,
217,
392,
612,
890,
392,
612,
890,
1790,
1926,
4096
];
array[1..numSubTrees_58, 1..4] of var DomainSize : limits_58;
array[1..numSubTrees_58] of var bool: decisions_58;
constraint treeLimits(tree_58, arcSizes_58, subTreeSizes_58, limits_58, decisions_58, rootArcIdx_58, numSubTrees_58);
constraint forall(idx in 1..numSubTrees_58) (noChildren(tree_58, decisions_58, idx));
int: rootArcIdx_59 = 15;
int: numSubTrees_59 = 15;
array[1..numSubTrees_59] of set of 1..numSubTrees_59: tree_59 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{8, 3},
{9, 2},
{},
{11, 6},
{4, 12},
{1, 13},
{10, 14}
];
array[1..numSubTrees_59] of DomainSize : arcSizes_59 = [
138,
138,
220,
220,
263,
263,
394,
2,
8,
6,
394,
2,
8,
6,
2034
];
array[1..numSubTrees_59] of DomainSize : subTreeSizes_59 = [
138,
138,
220,
220,
263,
263,
394,
659,
887,
1031,
394,
659,
887,
1031,
4096
];
array[1..numSubTrees_59, 1..4] of var DomainSize : limits_59;
array[1..numSubTrees_59] of var bool: decisions_59;
constraint treeLimits(tree_59, arcSizes_59, subTreeSizes_59, limits_59, decisions_59, rootArcIdx_59, numSubTrees_59);
constraint forall(idx in 1..numSubTrees_59) (noChildren(tree_59, decisions_59, idx));
int: rootArcIdx_60 = 17;
int: numSubTrees_60 = 17;
array[1..numSubTrees_60] of set of 1..numSubTrees_60: tree_60 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 5},
{9, 7},
{},
{11, 4},
{6, 12},
{10, 13},
{2, 14},
{3, 15},
{1, 16}
];
array[1..numSubTrees_60] of DomainSize : arcSizes_60 = [
1,
140,
140,
224,
224,
259,
259,
391,
3,
10,
391,
3,
10,
2,
1,
849,
1189
];
array[1..numSubTrees_60] of DomainSize : subTreeSizes_60 = [
1,
140,
140,
224,
224,
259,
259,
391,
618,
887,
391,
618,
887,
1776,
1917,
2906,
4096
];
array[1..numSubTrees_60, 1..4] of var DomainSize : limits_60;
array[1..numSubTrees_60] of var bool: decisions_60;
constraint treeLimits(tree_60, arcSizes_60, subTreeSizes_60, limits_60, decisions_60, rootArcIdx_60, numSubTrees_60);
constraint forall(idx in 1..numSubTrees_60) (noChildren(tree_60, decisions_60, idx));
int: rootArcIdx_61 = 17;
int: numSubTrees_61 = 17;
array[1..numSubTrees_61] of set of 1..numSubTrees_61: tree_61 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 4},
{9, 6},
{10, 2},
{},
{12, 5},
{7, 13},
{3, 14},
{11, 15},
{1, 16}
];
array[1..numSubTrees_61] of DomainSize : arcSizes_61 = [
2,
144,
144,
250,
250,
225,
225,
395,
3,
8,
6,
395,
3,
8,
6,
903,
1129
];
array[1..numSubTrees_61] of DomainSize : subTreeSizes_61 = [
2,
144,
144,
250,
250,
225,
225,
395,
648,
881,
1031,
395,
648,
881,
1031,
2965,
4096
];
array[1..numSubTrees_61, 1..4] of var DomainSize : limits_61;
array[1..numSubTrees_61] of var bool: decisions_61;
constraint treeLimits(tree_61, arcSizes_61, subTreeSizes_61, limits_61, decisions_61, rootArcIdx_61, numSubTrees_61);
constraint forall(idx in 1..numSubTrees_61) (noChildren(tree_61, decisions_61, idx));
int: rootArcIdx_62 = 19;
int: numSubTrees_62 = 19;
array[1..numSubTrees_62] of set of 1..numSubTrees_62: tree_62 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{10, 5},
{11, 4},
{},
{13, 7},
{6, 14},
{3, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_62] of DomainSize : arcSizes_62 = [
2,
2,
143,
143,
247,
247,
234,
234,
392,
2,
9,
1,
392,
2,
9,
1,
988,
1,
1047
];
array[1..numSubTrees_62] of DomainSize : subTreeSizes_62 = [
2,
2,
143,
143,
247,
247,
234,
234,
392,
628,
884,
1028,
392,
628,
884,
1028,
3044,
3047,
4096
];
array[1..numSubTrees_62, 1..4] of var DomainSize : limits_62;
array[1..numSubTrees_62] of var bool: decisions_62;
constraint treeLimits(tree_62, arcSizes_62, subTreeSizes_62, limits_62, decisions_62, rootArcIdx_62, numSubTrees_62);
constraint forall(idx in 1..numSubTrees_62) (noChildren(tree_62, decisions_62, idx));
int: rootArcIdx_63 = 19;
int: numSubTrees_63 = 19;
array[1..numSubTrees_63] of set of 1..numSubTrees_63: tree_63 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{10, 5},
{11, 3},
{},
{13, 7},
{6, 14},
{4, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_63] of DomainSize : arcSizes_63 = [
4,
4,
144,
144,
239,
239,
239,
239,
398,
1,
3,
1,
398,
1,
3,
1,
1058,
1,
979
];
array[1..numSubTrees_63] of DomainSize : subTreeSizes_63 = [
4,
4,
144,
144,
239,
239,
239,
239,
398,
638,
880,
1025,
398,
638,
880,
1025,
3108,
3113,
4096
];
array[1..numSubTrees_63, 1..4] of var DomainSize : limits_63;
array[1..numSubTrees_63] of var bool: decisions_63;
constraint treeLimits(tree_63, arcSizes_63, subTreeSizes_63, limits_63, decisions_63, rootArcIdx_63, numSubTrees_63);
constraint forall(idx in 1..numSubTrees_63) (noChildren(tree_63, decisions_63, idx));
int: rootArcIdx_64 = 19;
int: numSubTrees_64 = 19;
array[1..numSubTrees_64] of set of 1..numSubTrees_64: tree_64 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{10, 5},
{11, 3},
{},
{13, 7},
{6, 14},
{4, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_64] of DomainSize : arcSizes_64 = [
4,
4,
144,
144,
239,
239,
239,
239,
398,
1,
3,
1,
398,
1,
3,
1,
1058,
1,
979
];
array[1..numSubTrees_64] of DomainSize : subTreeSizes_64 = [
4,
4,
144,
144,
239,
239,
239,
239,
398,
638,
880,
1025,
398,
638,
880,
1025,
3108,
3113,
4096
];
array[1..numSubTrees_64, 1..4] of var DomainSize : limits_64;
array[1..numSubTrees_64] of var bool: decisions_64;
constraint treeLimits(tree_64, arcSizes_64, subTreeSizes_64, limits_64, decisions_64, rootArcIdx_64, numSubTrees_64);
constraint forall(idx in 1..numSubTrees_64) (noChildren(tree_64, decisions_64, idx));
int: rootArcIdx_65 = 19;
int: numSubTrees_65 = 19;
array[1..numSubTrees_65] of set of 1..numSubTrees_65: tree_65 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 7},
{10, 6},
{11, 4},
{},
{13, 8},
{5, 14},
{3, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_65] of DomainSize : arcSizes_65 = [
2,
2,
143,
143,
247,
247,
234,
234,
392,
2,
9,
1,
392,
2,
9,
1,
988,
1,
1047
];
array[1..numSubTrees_65] of DomainSize : subTreeSizes_65 = [
2,
2,
143,
143,
247,
247,
234,
234,
392,
628,
884,
1028,
392,
628,
884,
1028,
3044,
3047,
4096
];
array[1..numSubTrees_65, 1..4] of var DomainSize : limits_65;
array[1..numSubTrees_65] of var bool: decisions_65;
constraint treeLimits(tree_65, arcSizes_65, subTreeSizes_65, limits_65, decisions_65, rootArcIdx_65, numSubTrees_65);
constraint forall(idx in 1..numSubTrees_65) (noChildren(tree_65, decisions_65, idx));
int: rootArcIdx_66 = 17;
int: numSubTrees_66 = 17;
array[1..numSubTrees_66] of set of 1..numSubTrees_66: tree_66 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 4},
{9, 6},
{10, 2},
{},
{12, 5},
{7, 13},
{3, 14},
{11, 15},
{1, 16}
];
array[1..numSubTrees_66] of DomainSize : arcSizes_66 = [
2,
144,
144,
250,
250,
225,
225,
395,
3,
8,
6,
395,
3,
8,
6,
900,
1132
];
array[1..numSubTrees_66] of DomainSize : subTreeSizes_66 = [
2,
144,
144,
250,
250,
225,
225,
395,
648,
881,
1031,
395,
648,
881,
1031,
2962,
4096
];
array[1..numSubTrees_66, 1..4] of var DomainSize : limits_66;
array[1..numSubTrees_66] of var bool: decisions_66;
constraint treeLimits(tree_66, arcSizes_66, subTreeSizes_66, limits_66, decisions_66, rootArcIdx_66, numSubTrees_66);
constraint forall(idx in 1..numSubTrees_66) (noChildren(tree_66, decisions_66, idx));
int: rootArcIdx_67 = 17;
int: numSubTrees_67 = 17;
array[1..numSubTrees_67] of set of 1..numSubTrees_67: tree_67 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 4},
{9, 7},
{},
{11, 5},
{6, 12},
{10, 13},
{2, 14},
{3, 15},
{1, 16}
];
array[1..numSubTrees_67] of DomainSize : arcSizes_67 = [
1,
140,
140,
224,
224,
259,
259,
391,
3,
10,
391,
3,
10,
2,
1,
847,
1191
];
array[1..numSubTrees_67] of DomainSize : subTreeSizes_67 = [
1,
140,
140,
224,
224,
259,
259,
391,
618,
887,
391,
618,
887,
1776,
1917,
2904,
4096
];
array[1..numSubTrees_67, 1..4] of var DomainSize : limits_67;
array[1..numSubTrees_67] of var bool: decisions_67;
constraint treeLimits(tree_67, arcSizes_67, subTreeSizes_67, limits_67, decisions_67, rootArcIdx_67, numSubTrees_67);
constraint forall(idx in 1..numSubTrees_67) (noChildren(tree_67, decisions_67, idx));
int: rootArcIdx_68 = 15;
int: numSubTrees_68 = 15;
array[1..numSubTrees_68] of set of 1..numSubTrees_68: tree_68 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{8, 4},
{9, 2},
{},
{11, 6},
{3, 12},
{1, 13},
{10, 14}
];
array[1..numSubTrees_68] of DomainSize : arcSizes_68 = [
138,
138,
220,
220,
263,
263,
394,
2,
8,
6,
394,
2,
8,
6,
2034
];
array[1..numSubTrees_68] of DomainSize : subTreeSizes_68 = [
138,
138,
220,
220,
263,
263,
394,
659,
887,
1031,
394,
659,
887,
1031,
4096
];
array[1..numSubTrees_68, 1..4] of var DomainSize : limits_68;
array[1..numSubTrees_68] of var bool: decisions_68;
constraint treeLimits(tree_68, arcSizes_68, subTreeSizes_68, limits_68, decisions_68, rootArcIdx_68, numSubTrees_68);
constraint forall(idx in 1..numSubTrees_68) (noChildren(tree_68, decisions_68, idx));
int: rootArcIdx_69 = 15;
int: numSubTrees_69 = 15;
array[1..numSubTrees_69] of set of 1..numSubTrees_69: tree_69 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{8, 3},
{},
{10, 5},
{4, 11},
{9, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_69] of DomainSize : arcSizes_69 = [
135,
135,
272,
272,
217,
217,
392,
3,
6,
392,
3,
6,
10,
1,
2035
];
array[1..numSubTrees_69] of DomainSize : subTreeSizes_69 = [
135,
135,
272,
272,
217,
217,
392,
612,
890,
392,
612,
890,
1790,
1926,
4096
];
array[1..numSubTrees_69, 1..4] of var DomainSize : limits_69;
array[1..numSubTrees_69] of var bool: decisions_69;
constraint treeLimits(tree_69, arcSizes_69, subTreeSizes_69, limits_69, decisions_69, rootArcIdx_69, numSubTrees_69);
constraint forall(idx in 1..numSubTrees_69) (noChildren(tree_69, decisions_69, idx));
int: rootArcIdx_70 = 15;
int: numSubTrees_70 = 15;
array[1..numSubTrees_70] of set of 1..numSubTrees_70: tree_70 = [
{},
{},
{},
{},
{},
{},
{},
{7, 4},
{8, 5},
{9, 1},
{},
{11, 3},
{6, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_70] of DomainSize : arcSizes_70 = [
129,
129,
276,
276,
215,
215,
394,
4,
5,
6,
394,
4,
5,
6,
2038
];
array[1..numSubTrees_70] of DomainSize : subTreeSizes_70 = [
129,
129,
276,
276,
215,
215,
394,
674,
894,
1029,
394,
674,
894,
1029,
4096
];
array[1..numSubTrees_70, 1..4] of var DomainSize : limits_70;
array[1..numSubTrees_70] of var bool: decisions_70;
constraint treeLimits(tree_70, arcSizes_70, subTreeSizes_70, limits_70, decisions_70, rootArcIdx_70, numSubTrees_70);
constraint forall(idx in 1..numSubTrees_70) (noChildren(tree_70, decisions_70, idx));
int: rootArcIdx_71 = 15;
int: numSubTrees_71 = 15;
array[1..numSubTrees_71] of set of 1..numSubTrees_71: tree_71 = [
{},
{},
{},
{},
{},
{},
{},
{7, 4},
{8, 5},
{},
{10, 3},
{6, 11},
{9, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_71] of DomainSize : arcSizes_71 = [
124,
124,
289,
289,
215,
215,
397,
1,
3,
397,
1,
3,
4,
1,
2033
];
array[1..numSubTrees_71] of DomainSize : subTreeSizes_71 = [
124,
124,
289,
289,
215,
215,
397,
687,
905,
397,
687,
905,
1814,
1939,
4096
];
array[1..numSubTrees_71, 1..4] of var DomainSize : limits_71;
array[1..numSubTrees_71] of var bool: decisions_71;
constraint treeLimits(tree_71, arcSizes_71, subTreeSizes_71, limits_71, decisions_71, rootArcIdx_71, numSubTrees_71);
constraint forall(idx in 1..numSubTrees_71) (noChildren(tree_71, decisions_71, idx));
int: rootArcIdx_72 = 15;
int: numSubTrees_72 = 15;
array[1..numSubTrees_72] of set of 1..numSubTrees_72: tree_72 = [
{},
{},
{},
{},
{},
{},
{},
{7, 4},
{8, 6},
{9, 1},
{},
{11, 3},
{5, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_72] of DomainSize : arcSizes_72 = [
116,
116,
295,
295,
212,
212,
394,
6,
2,
1,
394,
6,
2,
1,
2044
];
array[1..numSubTrees_72] of DomainSize : subTreeSizes_72 = [
116,
116,
295,
295,
212,
212,
394,
695,
909,
1026,
394,
695,
909,
1026,
4096
];
array[1..numSubTrees_72, 1..4] of var DomainSize : limits_72;
array[1..numSubTrees_72] of var bool: decisions_72;
constraint treeLimits(tree_72, arcSizes_72, subTreeSizes_72, limits_72, decisions_72, rootArcIdx_72, numSubTrees_72);
constraint forall(idx in 1..numSubTrees_72) (noChildren(tree_72, decisions_72, idx));
int: rootArcIdx_73 = 15;
int: numSubTrees_73 = 15;
array[1..numSubTrees_73] of set of 1..numSubTrees_73: tree_73 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{8, 3},
{9, 2},
{},
{11, 5},
{4, 12},
{1, 13},
{10, 14}
];
array[1..numSubTrees_73] of DomainSize : arcSizes_73 = [
110,
110,
305,
305,
209,
209,
398,
3,
2,
7,
398,
3,
2,
7,
2028
];
array[1..numSubTrees_73] of DomainSize : subTreeSizes_73 = [
110,
110,
305,
305,
209,
209,
398,
610,
917,
1034,
398,
610,
917,
1034,
4096
];
array[1..numSubTrees_73, 1..4] of var DomainSize : limits_73;
array[1..numSubTrees_73] of var bool: decisions_73;
constraint treeLimits(tree_73, arcSizes_73, subTreeSizes_73, limits_73, decisions_73, rootArcIdx_73, numSubTrees_73);
constraint forall(idx in 1..numSubTrees_73) (noChildren(tree_73, decisions_73, idx));
int: rootArcIdx_74 = 15;
int: numSubTrees_74 = 15;
array[1..numSubTrees_74] of set of 1..numSubTrees_74: tree_74 = [
{},
{},
{},
{},
{},
{},
{},
{7, 4},
{8, 5},
{},
{10, 3},
{6, 11},
{9, 12},
{2, 13},
{1, 14}
];
array[1..numSubTrees_74] of DomainSize : arcSizes_74 = [
101,
101,
312,
312,
209,
209,
397,
4,
1,
397,
4,
1,
12,
1,
2035
];
array[1..numSubTrees_74] of DomainSize : subTreeSizes_74 = [
101,
101,
312,
312,
209,
209,
397,
713,
923,
397,
713,
923,
1858,
1960,
4096
];
array[1..numSubTrees_74, 1..4] of var DomainSize : limits_74;
array[1..numSubTrees_74] of var bool: decisions_74;
constraint treeLimits(tree_74, arcSizes_74, subTreeSizes_74, limits_74, decisions_74, rootArcIdx_74, numSubTrees_74);
constraint forall(idx in 1..numSubTrees_74) (noChildren(tree_74, decisions_74, idx));
int: rootArcIdx_75 = 15;
int: numSubTrees_75 = 15;
array[1..numSubTrees_75] of set of 1..numSubTrees_75: tree_75 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{8, 1},
{9, 5},
{},
{11, 4},
{2, 12},
{6, 13},
{10, 14}
];
array[1..numSubTrees_75] of DomainSize : arcSizes_75 = [
92,
92,
206,
206,
325,
325,
396,
4,
2,
4,
396,
4,
2,
4,
2038
];
array[1..numSubTrees_75] of DomainSize : subTreeSizes_75 = [
92,
92,
206,
206,
325,
325,
396,
606,
700,
1029,
396,
606,
700,
1029,
4096
];
array[1..numSubTrees_75, 1..4] of var DomainSize : limits_75;
array[1..numSubTrees_75] of var bool: decisions_75;
constraint treeLimits(tree_75, arcSizes_75, subTreeSizes_75, limits_75, decisions_75, rootArcIdx_75, numSubTrees_75);
constraint forall(idx in 1..numSubTrees_75) (noChildren(tree_75, decisions_75, idx));
int: rootArcIdx_76 = 15;
int: numSubTrees_76 = 15;
array[1..numSubTrees_76] of set of 1..numSubTrees_76: tree_76 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{8, 4},
{9, 1},
{},
{11, 6},
{3, 12},
{2, 13},
{10, 14}
];
array[1..numSubTrees_76] of DomainSize : arcSizes_76 = [
87,
87,
328,
328,
204,
204,
391,
2,
27,
1,
391,
2,
27,
1,
2016
];
array[1..numSubTrees_76] of DomainSize : subTreeSizes_76 = [
87,
87,
328,
328,
204,
204,
391,
597,
952,
1040,
391,
597,
952,
1040,
4096
];
array[1..numSubTrees_76, 1..4] of var DomainSize : limits_76;
array[1..numSubTrees_76] of var bool: decisions_76;
constraint treeLimits(tree_76, arcSizes_76, subTreeSizes_76, limits_76, decisions_76, rootArcIdx_76, numSubTrees_76);
constraint forall(idx in 1..numSubTrees_76) (noChildren(tree_76, decisions_76, idx));
int: rootArcIdx_77 = 17;
int: numSubTrees_77 = 17;
array[1..numSubTrees_77] of set of 1..numSubTrees_77: tree_77 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 6},
{9, 2},
{10, 4},
{},
{12, 7},
{3, 13},
{5, 14},
{11, 15},
{1, 16}
];
array[1..numSubTrees_77] of DomainSize : arcSizes_77 = [
1,
69,
69,
347,
347,
209,
209,
396,
2,
2,
3,
396,
2,
2,
3,
4,
2035
];
array[1..numSubTrees_77] of DomainSize : subTreeSizes_77 = [
1,
69,
69,
347,
347,
209,
209,
396,
607,
678,
1028,
396,
607,
678,
1028,
2060,
4096
];
array[1..numSubTrees_77, 1..4] of var DomainSize : limits_77;
array[1..numSubTrees_77] of var bool: decisions_77;
constraint treeLimits(tree_77, arcSizes_77, subTreeSizes_77, limits_77, decisions_77, rootArcIdx_77, numSubTrees_77);
constraint forall(idx in 1..numSubTrees_77) (noChildren(tree_77, decisions_77, idx));
int: rootArcIdx_78 = 19;
int: numSubTrees_78 = 19;
array[1..numSubTrees_78] of set of 1..numSubTrees_78: tree_78 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 7},
{10, 3},
{11, 5},
{},
{13, 8},
{4, 14},
{6, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_78] of DomainSize : arcSizes_78 = [
5,
5,
61,
61,
209,
209,
349,
349,
392,
7,
4,
3,
392,
7,
4,
3,
50,
1,
1985
];
array[1..numSubTrees_78] of DomainSize : subTreeSizes_78 = [
5,
5,
61,
61,
209,
209,
349,
349,
392,
748,
813,
1025,
392,
748,
813,
1025,
2100,
2106,
4096
];
array[1..numSubTrees_78, 1..4] of var DomainSize : limits_78;
array[1..numSubTrees_78] of var bool: decisions_78;
constraint treeLimits(tree_78, arcSizes_78, subTreeSizes_78, limits_78, decisions_78, rootArcIdx_78, numSubTrees_78);
constraint forall(idx in 1..numSubTrees_78) (noChildren(tree_78, decisions_78, idx));
int: rootArcIdx_79 = 19;
int: numSubTrees_79 = 19;
array[1..numSubTrees_79] of set of 1..numSubTrees_79: tree_79 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{10, 5},
{11, 3},
{},
{13, 7},
{6, 14},
{4, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_79] of DomainSize : arcSizes_79 = [
10,
10,
53,
53,
208,
208,
361,
361,
398,
1,
7,
2,
398,
1,
7,
2,
124,
1,
1891
];
array[1..numSubTrees_79] of DomainSize : subTreeSizes_79 = [
10,
10,
53,
53,
208,
208,
361,
361,
398,
760,
975,
1030,
398,
760,
975,
1030,
2184,
2195,
4096
];
array[1..numSubTrees_79, 1..4] of var DomainSize : limits_79;
array[1..numSubTrees_79] of var bool: decisions_79;
constraint treeLimits(tree_79, arcSizes_79, subTreeSizes_79, limits_79, decisions_79, rootArcIdx_79, numSubTrees_79);
constraint forall(idx in 1..numSubTrees_79) (noChildren(tree_79, decisions_79, idx));
int: rootArcIdx_80 = 19;
int: numSubTrees_80 = 19;
array[1..numSubTrees_80] of set of 1..numSubTrees_80: tree_80 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 6},
{10, 3},
{11, 7},
{},
{13, 5},
{4, 14},
{8, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_80] of DomainSize : arcSizes_80 = [
15,
15,
45,
45,
363,
363,
210,
210,
397,
1,
1,
2,
397,
1,
1,
2,
218,
1,
1809
];
array[1..numSubTrees_80] of DomainSize : subTreeSizes_80 = [
15,
15,
45,
45,
363,
363,
210,
210,
397,
761,
807,
1019,
397,
761,
807,
1019,
2256,
2272,
4096
];
array[1..numSubTrees_80, 1..4] of var DomainSize : limits_80;
array[1..numSubTrees_80] of var bool: decisions_80;
constraint treeLimits(tree_80, arcSizes_80, subTreeSizes_80, limits_80, decisions_80, rootArcIdx_80, numSubTrees_80);
constraint forall(idx in 1..numSubTrees_80) (noChildren(tree_80, decisions_80, idx));
int: rootArcIdx_81 = 19;
int: numSubTrees_81 = 19;
array[1..numSubTrees_81] of set of 1..numSubTrees_81: tree_81 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{},
{9, 6},
{},
{8, 11},
{},
{10, 13},
{12, 14},
{4, 15},
{3, 16},
{1, 17},
{2, 18}
];
array[1..numSubTrees_81] of DomainSize : arcSizes_81 = [
23,
23,
40,
40,
363,
363,
211,
1,
211,
1,
397,
5,
397,
5,
2,
1,
305,
1,
1707
];
array[1..numSubTrees_81] of DomainSize : subTreeSizes_81 = [
23,
23,
40,
40,
363,
363,
211,
575,
211,
575,
397,
977,
397,
977,
1956,
1997,
2342,
2366,
4096
];
array[1..numSubTrees_81, 1..4] of var DomainSize : limits_81;
array[1..numSubTrees_81] of var bool: decisions_81;
constraint treeLimits(tree_81, arcSizes_81, subTreeSizes_81, limits_81, decisions_81, rootArcIdx_81, numSubTrees_81);
constraint forall(idx in 1..numSubTrees_81) (noChildren(tree_81, decisions_81, idx));
int: rootArcIdx_82 = 19;
int: numSubTrees_82 = 19;
array[1..numSubTrees_82] of set of 1..numSubTrees_82: tree_82 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 5},
{10, 7},
{},
{12, 6},
{8, 13},
{11, 14},
{3, 15},
{4, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_82] of DomainSize : arcSizes_82 = [
29,
29,
36,
36,
357,
357,
217,
217,
397,
1,
1,
397,
1,
1,
22,
1,
399,
1,
1597
];
array[1..numSubTrees_82] of DomainSize : subTreeSizes_82 = [
29,
29,
36,
36,
357,
357,
217,
217,
397,
755,
973,
397,
755,
973,
1968,
2005,
2440,
2470,
4096
];
array[1..numSubTrees_82, 1..4] of var DomainSize : limits_82;
array[1..numSubTrees_82] of var bool: decisions_82;
constraint treeLimits(tree_82, arcSizes_82, subTreeSizes_82, limits_82, decisions_82, rootArcIdx_82, numSubTrees_82);
constraint forall(idx in 1..numSubTrees_82) (noChildren(tree_82, decisions_82, idx));
int: rootArcIdx_83 = 19;
int: numSubTrees_83 = 19;
array[1..numSubTrees_83] of set of 1..numSubTrees_83: tree_83 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 4},
{10, 5},
{11, 8},
{},
{13, 3},
{6, 14},
{7, 15},
{12, 16},
{2, 17},
{1, 18}
];
array[1..numSubTrees_83] of DomainSize : arcSizes_83 = [
32,
32,
29,
29,
218,
218,
368,
368,
394,
2,
4,
3,
394,
2,
4,
3,
392,
1,
1603
];
array[1..numSubTrees_83] of DomainSize : subTreeSizes_83 = [
32,
32,
29,
29,
218,
218,
368,
368,
394,
425,
647,
1018,
394,
425,
647,
1018,
2428,
2461,
4096
];
array[1..numSubTrees_83, 1..4] of var DomainSize : limits_83;
array[1..numSubTrees_83] of var bool: decisions_83;
constraint treeLimits(tree_83, arcSizes_83, subTreeSizes_83, limits_83, decisions_83, rootArcIdx_83, numSubTrees_83);
constraint forall(idx in 1..numSubTrees_83) (noChildren(tree_83, decisions_83, idx));
int: rootArcIdx_84 = 19;
int: numSubTrees_84 = 19;
array[1..numSubTrees_84] of set of 1..numSubTrees_84: tree_84 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{10, 11},
{},
{8, 13},
{12, 14},
{4, 15},
{3, 16},
{1, 17},
{2, 18}
];
array[1..numSubTrees_84] of DomainSize : arcSizes_84 = [
29,
29,
28,
28,
360,
360,
219,
1,
219,
1,
395,
9,
395,
9,
2,
1,
325,
1,
1685
];
array[1..numSubTrees_84] of DomainSize : subTreeSizes_84 = [
29,
29,
28,
28,
360,
360,
219,
580,
219,
580,
395,
984,
395,
984,
1970,
1999,
2352,
2382,
4096
];
array[1..numSubTrees_84, 1..4] of var DomainSize : limits_84;
array[1..numSubTrees_84] of var bool: decisions_84;
constraint treeLimits(tree_84, arcSizes_84, subTreeSizes_84, limits_84, decisions_84, rootArcIdx_84, numSubTrees_84);
constraint forall(idx in 1..numSubTrees_84) (noChildren(tree_84, decisions_84, idx));
int: rootArcIdx_85 = 19;
int: numSubTrees_85 = 19;
array[1..numSubTrees_85] of set of 1..numSubTrees_85: tree_85 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{10, 6},
{11, 2},
{},
{13, 7},
{5, 14},
{1, 15},
{12, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_85] of DomainSize : arcSizes_85 = [
25,
25,
27,
27,
225,
225,
354,
354,
395,
6,
4,
3,
395,
6,
4,
3,
238,
1,
1779
];
array[1..numSubTrees_85] of DomainSize : subTreeSizes_85 = [
25,
25,
27,
27,
225,
225,
354,
354,
395,
755,
984,
1012,
395,
755,
984,
1012,
2262,
2290,
4096
];
array[1..numSubTrees_85, 1..4] of var DomainSize : limits_85;
array[1..numSubTrees_85] of var bool: decisions_85;
constraint treeLimits(tree_85, arcSizes_85, subTreeSizes_85, limits_85, decisions_85, rootArcIdx_85, numSubTrees_85);
constraint forall(idx in 1..numSubTrees_85) (noChildren(tree_85, decisions_85, idx));
int: rootArcIdx_86 = 19;
int: numSubTrees_86 = 19;
array[1..numSubTrees_86] of set of 1..numSubTrees_86: tree_86 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 6},
{10, 8},
{11, 2},
{},
{13, 5},
{7, 14},
{1, 15},
{12, 16},
{3, 17},
{4, 18}
];
array[1..numSubTrees_86] of DomainSize : arcSizes_86 = [
22,
22,
26,
26,
229,
229,
354,
354,
396,
3,
1,
5,
396,
3,
1,
5,
172,
1,
1851
];
array[1..numSubTrees_86] of DomainSize : subTreeSizes_86 = [
22,
22,
26,
26,
229,
229,
354,
354,
396,
628,
983,
1010,
396,
628,
983,
1010,
2192,
2219,
4096
];
array[1..numSubTrees_86, 1..4] of var DomainSize : limits_86;
array[1..numSubTrees_86] of var bool: decisions_86;
constraint treeLimits(tree_86, arcSizes_86, subTreeSizes_86, limits_86, decisions_86, rootArcIdx_86, numSubTrees_86);
constraint forall(idx in 1..numSubTrees_86) (noChildren(tree_86, decisions_86, idx));
int: rootArcIdx_87 = 19;
int: numSubTrees_87 = 19;
array[1..numSubTrees_87] of set of 1..numSubTrees_87: tree_87 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 8},
{10, 5},
{11, 1},
{},
{13, 7},
{6, 14},
{2, 15},
{12, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_87] of DomainSize : arcSizes_87 = [
22,
22,
26,
26,
231,
231,
348,
348,
396,
4,
3,
6,
396,
4,
3,
6,
98,
1,
1925
];
array[1..numSubTrees_87] of DomainSize : subTreeSizes_87 = [
22,
22,
26,
26,
231,
231,
348,
348,
396,
748,
982,
1010,
396,
748,
982,
1010,
2118,
2145,
4096
];
array[1..numSubTrees_87, 1..4] of var DomainSize : limits_87;
array[1..numSubTrees_87] of var bool: decisions_87;
constraint treeLimits(tree_87, arcSizes_87, subTreeSizes_87, limits_87, decisions_87, rootArcIdx_87, numSubTrees_87);
constraint forall(idx in 1..numSubTrees_87) (noChildren(tree_87, decisions_87, idx));
int: rootArcIdx_88 = 19;
int: numSubTrees_88 = 19;
array[1..numSubTrees_88] of set of 1..numSubTrees_88: tree_88 = [
{},
{},
{},
{},
{},
{},
{},
{7, 5},
{},
{9, 6},
{},
{10, 11},
{},
{8, 13},
{12, 14},
{1, 15},
{2, 16},
{3, 17},
{4, 18}
];
array[1..numSubTrees_88] of DomainSize : arcSizes_88 = [
20,
20,
24,
24,
342,
342,
238,
3,
238,
3,
398,
1,
398,
1,
8,
1,
53,
1,
1981
];
array[1..numSubTrees_88] of DomainSize : subTreeSizes_88 = [
20,
20,
24,
24,
342,
342,
238,
583,
238,
583,
398,
982,
398,
982,
1972,
1993,
2066,
2091,
4096
];
array[1..numSubTrees_88, 1..4] of var DomainSize : limits_88;
array[1..numSubTrees_88] of var bool: decisions_88;
constraint treeLimits(tree_88, arcSizes_88, subTreeSizes_88, limits_88, decisions_88, rootArcIdx_88, numSubTrees_88);
constraint forall(idx in 1..numSubTrees_88) (noChildren(tree_88, decisions_88, idx));
int: rootArcIdx_89 = 19;
int: numSubTrees_89 = 19;
array[1..numSubTrees_89] of set of 1..numSubTrees_89: tree_89 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 6},
{10, 1},
{11, 8},
{},
{13, 5},
{2, 14},
{7, 15},
{12, 16},
{4, 17},
{3, 18}
];
array[1..numSubTrees_89] of DomainSize : arcSizes_89 = [
18,
18,
25,
25,
244,
244,
337,
337,
395,
4,
4,
4,
395,
4,
4,
4,
6,
1,
2027
];
array[1..numSubTrees_89] of DomainSize : subTreeSizes_89 = [
18,
18,
25,
25,
244,
244,
337,
337,
395,
643,
665,
1006,
395,
643,
665,
1006,
2018,
2044,
4096
];
array[1..numSubTrees_89, 1..4] of var DomainSize : limits_89;
array[1..numSubTrees_89] of var bool: decisions_89;
constraint treeLimits(tree_89, arcSizes_89, subTreeSizes_89, limits_89, decisions_89, rootArcIdx_89, numSubTrees_89);
constraint forall(idx in 1..numSubTrees_89) (noChildren(tree_89, decisions_89, idx));
int: rootArcIdx_90 = 19;
int: numSubTrees_90 = 19;
array[1..numSubTrees_90] of set of 1..numSubTrees_90: tree_90 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 1},
{10, 5},
{11, 3},
{12, 8},
{},
{14, 2},
{6, 15},
{4, 16},
{7, 17},
{13, 18}
];
array[1..numSubTrees_90] of DomainSize : arcSizes_90 = [
15,
15,
25,
25,
254,
254,
330,
330,
397,
2,
1,
1,
5,
397,
2,
1,
1,
5,
2036
];
array[1..numSubTrees_90] of DomainSize : subTreeSizes_90 = [
15,
15,
25,
25,
254,
254,
330,
330,
397,
414,
669,
695,
1030,
397,
414,
669,
695,
1030,
4096
];
array[1..numSubTrees_90, 1..4] of var DomainSize : limits_90;
array[1..numSubTrees_90] of var bool: decisions_90;
constraint treeLimits(tree_90, arcSizes_90, subTreeSizes_90, limits_90, decisions_90, rootArcIdx_90, numSubTrees_90);
constraint forall(idx in 1..numSubTrees_90) (noChildren(tree_90, decisions_90, idx));
int: rootArcIdx_91 = 21;
int: numSubTrees_91 = 21;
array[1..numSubTrees_91] of set of 1..numSubTrees_91: tree_91 = [
{},
{},
{},
{},
{},
{},
{},
{},
{8, 4},
{},
{10, 5},
{},
{12, 6},
{13, 9},
{},
{15, 7},
{11, 16},
{1, 17},
{14, 18},
{3, 19},
{2, 20}
];
array[1..numSubTrees_91] of DomainSize : arcSizes_91 = [
1,
16,
16,
26,
26,
322,
322,
261,
1,
261,
1,
397,
1,
3,
397,
1,
1,
1,
6,
1,
2035
];
array[1..numSubTrees_91] of DomainSize : subTreeSizes_91 = [
1,
16,
16,
26,
26,
322,
322,
261,
288,
261,
288,
397,
720,
1011,
397,
720,
1009,
1011,
2028,
2045,
4096
];
array[1..numSubTrees_91, 1..4] of var DomainSize : limits_91;
array[1..numSubTrees_91] of var bool: decisions_91;
constraint treeLimits(tree_91, arcSizes_91, subTreeSizes_91, limits_91, decisions_91, rootArcIdx_91, numSubTrees_91);
constraint forall(idx in 1..numSubTrees_91) (noChildren(tree_91, decisions_91, idx));
int: rootArcIdx_92 = 23;
int: numSubTrees_92 = 23;
array[1..numSubTrees_92] of set of 1..numSubTrees_92: tree_92 = [
{},
{},
{},
{},
{},
{},
{},
{7, 6},
{},
{9, 5},
{},
{},
{},
{13, 3},
{14, 12},
{15, 2},
{16, 10},
{},
{18, 4},
{11, 19},
{1, 20},
{8, 21},
{17, 22}
];
array[1..numSubTrees_92] of DomainSize : arcSizes_92 = [
1,
1,
15,
15,
25,
26,
266,
3,
266,
4,
314,
314,
396,
2,
4,
3,
2,
396,
2,
4,
3,
2,
2032
];
array[1..numSubTrees_92] of DomainSize : subTreeSizes_92 = [
1,
1,
15,
15,
25,
26,
266,
295,
266,
295,
314,
314,
396,
413,
731,
735,
1032,
396,
413,
731,
735,
1032,
4096
];
array[1..numSubTrees_92, 1..4] of var DomainSize : limits_92;
array[1..numSubTrees_92] of var bool: decisions_92;
constraint treeLimits(tree_92, arcSizes_92, subTreeSizes_92, limits_92, decisions_92, rootArcIdx_92, numSubTrees_92);
constraint forall(idx in 1..numSubTrees_92) (noChildren(tree_92, decisions_92, idx));
int: rootArcIdx_93 = 23;
int: numSubTrees_93 = 23;
array[1..numSubTrees_93] of set of 1..numSubTrees_93: tree_93 = [
{},
{},
{},
{},
{},
{},
{},
{7, 2},
{},
{9, 1},
{},
{},
{},
{13, 11},
{14, 3},
{15, 5},
{16, 8},
{},
{18, 12},
{4, 19},
{6, 20},
{10, 21},
{17, 22}
];
array[1..numSubTrees_93] of DomainSize : arcSizes_93 = [
4,
4,
13,
13,
24,
24,
305,
2,
305,
2,
275,
275,
395,
3,
4,
3,
1,
395,
3,
4,
3,
1,
2038
];
array[1..numSubTrees_93] of DomainSize : subTreeSizes_93 = [
4,
4,
13,
13,
24,
24,
305,
311,
305,
311,
275,
275,
395,
673,
690,
717,
1029,
395,
673,
690,
717,
1029,
4096
];
array[1..numSubTrees_93, 1..4] of var DomainSize : limits_93;
array[1..numSubTrees_93] of var bool: decisions_93;
constraint treeLimits(tree_93, arcSizes_93, subTreeSizes_93, limits_93, decisions_93, rootArcIdx_93, numSubTrees_93);
constraint forall(idx in 1..numSubTrees_93) (noChildren(tree_93, decisions_93, idx));
int: rootArcIdx_94 = 23;
int: numSubTrees_94 = 23;
array[1..numSubTrees_94] of set of 1..numSubTrees_94: tree_94 = [
{},
{},
{},
{},
{},
{},
{},
{},
{},
{9, 1},
{10, 7},
{},
{12, 2},
{13, 8},
{},
{11, 15},
{},
{14, 17},
{16, 18},
{6, 19},
{5, 20},
{4, 21},
{3, 22}
];
array[1..numSubTrees_94] of DomainSize : arcSizes_94 = [
8,
8,
19,
19,
12,
12,
282,
282,
303,
1,
2,
303,
1,
2,
396,
1,
396,
1,
8,
1,
3,
1,
2035
];
array[1..numSubTrees_94] of DomainSize : subTreeSizes_94 = [
8,
8,
19,
19,
12,
12,
282,
282,
303,
312,
596,
303,
312,
596,
396,
993,
396,
993,
1994,
2007,
2022,
2042,
4096
];
array[1..numSubTrees_94, 1..4] of var DomainSize : limits_94;
array[1..numSubTrees_94] of var bool: decisions_94;
constraint treeLimits(tree_94, arcSizes_94, subTreeSizes_94, limits_94, decisions_94, rootArcIdx_94, numSubTrees_94);
constraint forall(idx in 1..numSubTrees_94) (noChildren(tree_94, decisions_94, idx));
int: rootArcIdx_95 = 23;
int: numSubTrees_95 = 23;
array[1..numSubTrees_95] of set of 1..numSubTrees_95: tree_95 = [
{},
{},
{},
{},
{},
{},
{},
{7, 3},
{},
{9, 4},
{},
{},
{},
{13, 12},
{14, 10},
{},
{16, 11},
{8, 17},
{15, 18},
{6, 19},
{5, 20},
{2, 21},
{1, 22}
];
array[1..numSubTrees_95] of DomainSize : arcSizes_95 = [
12,
12,
15,
15,
13,
13,
282,
1,
282,
1,
291,
291,
403,
5,
2,
403,
5,
2,
4,
1,
9,
1,
2033
];
array[1..numSubTrees_95] of DomainSize : subTreeSizes_95 = [
12,
12,
15,
15,
13,
13,
282,
298,
282,
298,
291,
291,
403,
699,
999,
403,
699,
999,
2002,
2016,
2038,
2051,
4096
];
array[1..numSubTrees_95, 1..4] of var DomainSize : limits_95;
array[1..numSubTrees_95] of var bool: decisions_95;
constraint treeLimits(tree_95, arcSizes_95, subTreeSizes_95, limits_95, decisions_95, rootArcIdx_95, numSubTrees_95);
constraint forall(idx in 1..numSubTrees_95) (noChildren(tree_95, decisions_95, idx));

array[1..numTimeSteps-1] of var 0.0..domainSize: objPerTimeStep;

array[1..numSubTrees_1 , 1..numSubTrees_0] of DomainSize: overlapND_0_1 = array2d(1..numSubTrees_1 , 1..numSubTrees_0, [
3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
0,2,0,0,0,1,0,0,0,1,0,0,0,0,1,1,1,3,5,
0,0,132,0,1,0,0,1,0,0,1,0,1,2,2,134,144,144,144,
0,0,0,132,0,1,0,0,1,2,0,0,0,0,2,2,144,144,144,
0,0,1,0,227,0,0,227,0,0,0,0,0,227,227,228,240,240,240,
0,0,1,0,0,0,226,226,1,1,0,0,0,226,227,228,239,239,239,
0,0,2,0,227,0,226,454,1,2,0,0,0,454,456,458,482,482,482,
0,0,0,1,0,0,0,0,226,226,0,1,1,1,227,227,239,239,239,
0,1,0,1,0,226,0,0,0,226,0,0,0,0,226,226,239,240,240,
0,1,0,2,0,226,0,0,226,452,0,1,2,2,454,454,481,482,482,
0,0,0,0,0,0,1,1,0,0,388,0,394,395,395,395,396,396,396,
0,0,0,0,0,0,1,1,0,0,0,389,395,396,396,396,397,397,397,
0,0,0,0,0,0,2,2,0,0,389,390,793,795,795,795,798,798,798,
0,0,0,132,0,1,2,2,1,2,389,390,793,796,798,798,943,943,943,
0,0,132,132,1,1,2,3,1,2,390,390,794,798,800,932,1091,1092,1092,
0,1,132,134,1,227,2,3,227,454,390,391,796,800,1254,1386,1573,1575,1575,
0,1,145,145,238,237,239,478,239,477,390,391,796,1275,1755,1901,3036,3038,3130,
0,3,145,145,238,238,239,478,239,478,390,391,796,1275,1756,1902,3038,3042,3136,
3,3,145,145,238,238,239,478,239,478,390,391,796,1275,1756,1902,3062,3066,4096
]);
array[1..numSubTrees_1 , 1..numSubTrees_0] of var DomainSize: overlap1D_0_1 = array2d(1..numSubTrees_1 , 1..numSubTrees_0, [
overlap1DCounts(limits_1[i, 1], limits_1[i, 2], limits_0[j, 1], limits_0[j, 2]) |
i in 1..numSubTrees_1,
j in 1..numSubTrees_0
]);
constraint objPerTimeStep[1] = normOfDiffCounts(numSubTrees_0, numSubTrees_1, overlapND_0_1, overlap1D_0_1);

array[1..numSubTrees_2 , 1..numSubTrees_1] of DomainSize: overlapND_1_2 = array2d(1..numSubTrees_2 , 1..numSubTrees_1, [
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,
1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,3,
0,139,0,0,0,2,2,0,0,0,0,0,0,141,143,145,145,
1,0,139,0,0,0,0,0,0,1,0,1,140,142,142,144,145,
0,0,0,0,0,233,235,0,0,0,1,1,1,1,236,237,238,
0,0,2,0,0,0,0,233,235,0,0,0,2,2,2,238,238,
0,0,0,0,226,0,230,0,0,0,1,1,1,1,231,239,239,
0,0,0,0,226,233,466,0,0,0,2,2,2,2,468,477,478,
0,0,0,227,0,0,0,0,231,0,0,0,0,0,0,239,239,
0,0,2,227,0,1,1,233,466,0,0,0,2,2,3,478,478,
0,1,0,0,1,0,1,0,0,0,373,376,376,379,380,390,390,
0,0,0,0,0,0,0,1,1,375,0,378,378,380,380,391,391,
0,1,0,0,1,0,1,2,2,375,373,754,754,759,760,795,796,
0,1,0,0,227,233,467,2,2,375,375,756,757,762,1229,1273,1275,
0,1,2,227,228,235,470,236,469,375,375,756,759,764,1234,1754,1756,
0,140,2,228,228,237,472,236,470,375,375,756,759,905,1377,1900,1902,
1,141,142,229,229,247,489,246,488,390,389,792,935,1096,1586,2979,3062,
2,141,142,229,229,247,489,247,489,390,389,792,935,1097,1587,2981,3066,
3,141,142,229,229,247,489,247,489,390,389,792,935,1097,1587,3009,4096
]);
array[1..numSubTrees_2 , 1..numSubTrees_1] of var DomainSize: overlap1D_1_2 = array2d(1..numSubTrees_2 , 1..numSubTrees_1, [
overlap1DCounts(limits_2[i, 1], limits_2[i, 2], limits_1[j, 1], limits_1[j, 2]) |
i in 1..numSubTrees_2,
j in 1..numSubTrees_1
]);
constraint objPerTimeStep[2] = normOfDiffCounts(numSubTrees_1, numSubTrees_2, overlapND_1_2, overlap1D_1_2);

array[1..numSubTrees_3 , 1..numSubTrees_2] of DomainSize: overlapND_2_3 = array2d(1..numSubTrees_3 , 1..numSubTrees_2, [
0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,3,
0,130,0,0,0,1,0,0,0,0,0,0,1,1,131,141,141,
1,0,130,0,0,0,1,0,0,0,0,0,0,1,1,141,142,
0,0,0,1,216,0,0,1,0,1,2,218,218,218,218,229,229,
0,0,0,216,1,0,0,0,1,1,217,218,218,218,218,229,229,
0,1,0,0,0,236,0,0,1,1,1,1,237,237,238,247,247,
0,1,0,218,1,240,0,0,2,3,221,222,462,463,464,489,489,
0,0,1,0,0,0,236,1,0,1,1,1,1,237,237,247,247,
0,0,1,1,218,1,240,2,0,2,3,221,222,462,463,489,489,
0,0,0,0,1,0,1,380,0,385,385,386,386,387,387,390,390,
0,0,0,1,0,1,0,0,379,384,385,385,386,386,386,389,389,
0,0,0,1,1,1,2,383,382,777,778,779,780,782,782,792,792,
1,0,130,1,1,1,3,383,382,777,779,780,781,784,784,934,935,
1,133,133,1,1,2,3,384,383,779,781,782,784,787,920,1095,1097,
1,134,133,219,2,242,3,384,385,782,1002,1004,1246,1250,1384,1585,1587,
1,140,140,228,228,254,254,392,391,796,1025,1254,1509,1764,1905,2909,3009,
2,140,141,228,228,254,254,392,391,796,1025,1254,1509,1764,1905,2933,4096
]);
array[1..numSubTrees_3 , 1..numSubTrees_2] of var DomainSize: overlap1D_2_3 = array2d(1..numSubTrees_3 , 1..numSubTrees_2, [
overlap1DCounts(limits_3[i, 1], limits_3[i, 2], limits_2[j, 1], limits_2[j, 2]) |
i in 1..numSubTrees_3,
j in 1..numSubTrees_2
]);
constraint objPerTimeStep[3] = normOfDiffCounts(numSubTrees_2, numSubTrees_3, overlapND_2_3, overlap1D_2_3);

array[1..numSubTrees_4 , 1..numSubTrees_3] of DomainSize: overlapND_3_4 = array2d(1..numSubTrees_4 , 1..numSubTrees_3, [
1,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1,2,
0,130,0,1,0,0,1,0,0,0,0,0,0,131,131,140,140,
1,0,130,0,1,0,0,0,1,0,0,0,130,131,132,140,141,
0,0,0,0,0,212,217,1,2,0,0,0,0,0,2,228,228,
0,0,0,0,0,1,1,211,216,0,1,2,2,2,218,228,228,
0,1,0,245,0,0,246,0,0,1,0,1,1,2,2,254,254,
0,0,1,0,246,0,0,0,247,0,0,0,1,1,248,254,254,
0,0,0,0,0,0,0,1,1,0,379,383,383,385,386,392,392,
0,0,0,1,0,0,1,0,0,378,0,382,382,384,384,391,391,
0,0,0,1,0,0,2,1,1,378,379,765,765,769,770,796,796,
0,0,0,1,0,212,219,2,3,378,379,765,766,770,773,1025,1025,
0,0,0,1,0,214,221,213,219,378,380,767,768,772,991,1254,1254,
0,1,0,246,0,214,467,214,220,379,380,768,769,774,994,1509,1509,
0,1,1,247,246,214,468,214,467,379,380,768,770,775,1242,1764,1764,
0,131,1,248,247,214,469,214,468,379,380,768,770,906,1374,1905,1905,
1,137,137,262,262,220,493,220,493,385,386,790,928,1081,1575,2829,2933,
2,137,138,262,262,220,493,220,493,385,386,790,929,1083,1577,2853,4096
]);
array[1..numSubTrees_4 , 1..numSubTrees_3] of var DomainSize: overlap1D_3_4 = array2d(1..numSubTrees_4 , 1..numSubTrees_3, [
overlap1DCounts(limits_4[i, 1], limits_4[i, 2], limits_3[j, 1], limits_3[j, 2]) |
i in 1..numSubTrees_4,
j in 1..numSubTrees_3
]);
constraint objPerTimeStep[4] = normOfDiffCounts(numSubTrees_3, numSubTrees_4, overlapND_3_4, overlap1D_3_4);

array[1..numSubTrees_5 , 1..numSubTrees_4] of DomainSize: overlapND_4_5 = array2d(1..numSubTrees_5 , 1..numSubTrees_4, [
1,1,0,0,0,0,0,0,0,0,0,1,2,2,2,
126,0,1,0,0,0,0,1,0,0,0,126,126,127,137,
0,126,0,1,0,2,0,0,0,0,0,0,126,126,138,
1,0,251,0,0,0,0,251,1,0,1,2,2,253,262,
0,1,0,252,0,252,0,0,0,0,0,0,1,1,262,
0,0,0,0,1,1,210,210,0,0,0,0,0,210,220,
1,0,254,0,1,1,211,465,1,0,2,3,3,468,493,
0,0,0,0,209,209,1,1,0,1,1,1,1,2,220,
0,1,0,255,210,465,1,2,0,1,1,1,2,4,493,
0,0,1,0,0,0,0,1,375,0,379,379,379,380,385,
0,0,0,0,1,1,0,0,0,376,380,380,380,380,386,
0,0,1,0,1,1,0,1,380,381,773,773,773,774,790,
0,126,1,1,1,3,0,1,380,381,773,774,900,901,929,
127,127,2,1,1,3,0,2,382,383,777,905,1033,1036,1083,
127,128,2,256,211,468,1,4,382,384,778,906,1035,1040,1577,
133,133,267,267,220,490,220,490,388,389,796,930,1064,1555,2853,
134,134,267,267,220,490,220,490,388,389,796,931,1066,1557,4096
]);
array[1..numSubTrees_5 , 1..numSubTrees_4] of var DomainSize: overlap1D_4_5 = array2d(1..numSubTrees_5 , 1..numSubTrees_4, [
overlap1DCounts(limits_5[i, 1], limits_5[i, 2], limits_4[j, 1], limits_4[j, 2]) |
i in 1..numSubTrees_5,
j in 1..numSubTrees_4
]);
constraint objPerTimeStep[5] = normOfDiffCounts(numSubTrees_4, numSubTrees_5, overlapND_4_5, overlap1D_4_5);

array[1..numSubTrees_6 , 1..numSubTrees_5] of DomainSize: overlapND_5_6 = array2d(1..numSubTrees_6 , 1..numSubTrees_5, [
123,0,0,0,1,1,0,0,0,0,0,0,123,123,134,
0,123,0,0,0,1,1,1,0,0,0,123,123,124,134,
1,0,0,0,258,259,0,0,1,0,1,1,2,2,267,
0,1,0,0,0,0,258,259,0,1,1,2,2,261,267,
0,0,0,208,0,0,0,211,0,0,1,1,1,212,220,
0,2,0,209,0,0,259,472,0,1,2,4,4,476,490,
0,0,207,0,0,210,0,1,1,0,1,1,1,2,220,
1,0,208,0,259,471,1,2,2,0,2,2,3,5,490,
0,0,1,0,1,2,0,0,378,0,380,380,380,380,388,
0,0,0,0,0,0,1,1,0,380,382,382,382,383,389,
0,0,1,0,2,3,1,1,381,383,770,770,772,773,796,
123,0,1,0,3,4,1,1,381,383,770,771,896,897,931,
123,123,1,0,3,5,2,2,381,383,770,894,1019,1021,1066,
124,123,209,0,262,476,3,4,383,383,772,896,1022,1026,1557,
130,130,215,215,277,499,277,499,388,389,792,923,1060,1560,4096
]);
array[1..numSubTrees_6 , 1..numSubTrees_5] of var DomainSize: overlap1D_5_6 = array2d(1..numSubTrees_6 , 1..numSubTrees_5, [
overlap1DCounts(limits_6[i, 1], limits_6[i, 2], limits_5[j, 1], limits_5[j, 2]) |
i in 1..numSubTrees_6,
j in 1..numSubTrees_5
]);
constraint objPerTimeStep[6] = normOfDiffCounts(numSubTrees_5, numSubTrees_6, overlapND_5_6, overlap1D_5_6);

array[1..numSubTrees_7 , 1..numSubTrees_6] of DomainSize: overlapND_6_7 = array2d(1..numSubTrees_7 , 1..numSubTrees_6, [
118,0,0,1,0,1,0,0,0,0,0,118,119,120,130,
0,117,1,0,0,1,0,1,0,0,0,0,118,119,130,
0,0,0,0,205,205,1,1,0,0,0,0,0,205,215,
0,0,0,0,1,1,204,204,0,1,1,1,1,2,215,
1,0,0,267,0,267,0,0,1,0,1,2,2,269,277,
1,1,0,267,206,474,1,2,1,0,1,2,3,477,499,
0,1,268,0,0,0,0,268,0,0,0,0,1,1,277,
0,1,268,0,1,1,205,474,0,1,2,2,3,4,499,
0,0,0,1,0,1,0,0,377,0,381,381,381,382,388,
0,0,0,0,0,0,1,1,0,378,382,382,382,382,389,
0,0,0,1,0,1,1,1,380,381,771,771,771,772,792,
0,117,1,1,0,2,1,2,380,381,771,772,890,892,923,
119,118,1,2,0,3,1,2,380,381,773,893,1013,1016,1060,
119,119,269,2,1,4,206,476,380,382,775,895,1016,1020,1560,
124,124,282,282,213,499,213,499,387,388,792,917,1046,1546,4096
]);
array[1..numSubTrees_7 , 1..numSubTrees_6] of var DomainSize: overlap1D_6_7 = array2d(1..numSubTrees_7 , 1..numSubTrees_6, [
overlap1DCounts(limits_7[i, 1], limits_7[i, 2], limits_6[j, 1], limits_6[j, 2]) |
i in 1..numSubTrees_7,
j in 1..numSubTrees_6
]);
constraint objPerTimeStep[7] = normOfDiffCounts(numSubTrees_6, numSubTrees_7, overlapND_6_7, overlap1D_6_7);

array[1..numSubTrees_8 , 1..numSubTrees_7] of DomainSize: overlapND_7_8 = array2d(1..numSubTrees_8 , 1..numSubTrees_7, [
114,0,0,0,0,0,0,0,0,0,0,0,0,0,124,
0,114,0,1,0,0,0,0,0,0,1,1,1,115,124,
0,1,0,272,0,0,1,0,1,1,273,273,273,274,282,
0,0,273,0,0,0,0,1,1,274,274,274,274,274,282,
0,0,0,0,201,1,0,1,1,1,1,202,204,204,213,
0,0,273,0,203,1,0,2,3,276,277,480,482,482,499,
0,0,0,0,1,201,1,0,1,1,1,2,203,204,213,
0,1,1,272,1,203,2,0,2,3,275,277,480,482,499,
0,0,1,0,1,0,0,377,377,378,378,379,379,379,387,
0,0,0,1,0,1,378,0,378,378,379,379,380,380,388,
0,0,1,2,1,1,381,380,763,764,766,767,770,770,792,
114,0,1,2,1,1,381,380,763,765,767,768,771,771,917,
114,114,1,3,1,1,382,381,765,767,770,771,774,888,1046,
114,114,274,3,204,2,382,383,768,1043,1047,1251,1256,1370,1546,
121,121,292,292,213,213,389,388,786,1079,1376,1590,1808,1930,4096
]);
array[1..numSubTrees_8 , 1..numSubTrees_7] of var DomainSize: overlap1D_7_8 = array2d(1..numSubTrees_8 , 1..numSubTrees_7, [
overlap1DCounts(limits_8[i, 1], limits_8[i, 2], limits_7[j, 1], limits_7[j, 2]) |
i in 1..numSubTrees_8,
j in 1..numSubTrees_7
]);
constraint objPerTimeStep[8] = normOfDiffCounts(numSubTrees_7, numSubTrees_8, overlapND_7_8, overlap1D_7_8);

array[1..numSubTrees_9 , 1..numSubTrees_8] of DomainSize: overlapND_8_9 = array2d(1..numSubTrees_9 , 1..numSubTrees_8, [
107,0,0,0,1,1,0,0,0,0,0,0,107,108,121,
0,107,0,0,0,1,1,1,0,0,0,107,107,108,121,
1,0,0,0,282,282,0,0,0,1,1,1,2,284,292,
0,1,0,0,0,0,283,283,0,0,0,1,1,1,292,
0,0,202,1,0,202,0,1,0,0,0,0,0,202,213,
0,0,1,201,0,1,0,201,1,0,1,1,1,2,213,
0,0,0,1,0,0,0,1,378,0,382,382,382,382,389,
0,0,0,0,1,1,0,0,0,377,381,381,381,382,388,
0,1,0,1,1,1,0,1,380,379,769,770,770,771,786,
1,1,0,1,283,283,0,1,380,380,770,772,773,1056,1079,
1,2,0,1,283,283,283,284,382,382,774,777,778,1061,1376,
1,2,202,2,283,485,283,285,382,382,774,777,778,1263,1590,
1,2,203,203,283,486,283,487,383,382,777,780,781,1267,1808,
1,109,203,203,283,487,284,488,383,382,778,888,889,1376,1930,
112,112,211,211,298,512,298,512,388,387,790,903,1022,1535,4096
]);
array[1..numSubTrees_9 , 1..numSubTrees_8] of var DomainSize: overlap1D_8_9 = array2d(1..numSubTrees_9 , 1..numSubTrees_8, [
overlap1DCounts(limits_9[i, 1], limits_9[i, 2], limits_8[j, 1], limits_8[j, 2]) |
i in 1..numSubTrees_9,
j in 1..numSubTrees_8
]);
constraint objPerTimeStep[9] = normOfDiffCounts(numSubTrees_8, numSubTrees_9, overlapND_8_9, overlap1D_8_9);

array[1..numSubTrees_10 , 1..numSubTrees_9] of DomainSize: overlapND_9_10 = array2d(1..numSubTrees_10 , 1..numSubTrees_9, [
95,0,1,0,0,1,0,0,0,0,0,0,98,98,112,
0,95,0,1,0,1,0,1,0,0,0,95,98,99,112,
0,0,0,0,202,202,0,1,1,0,1,1,1,2,211,
0,0,0,0,0,0,203,203,0,0,1,1,1,204,211,
1,0,289,0,0,289,0,0,1,0,1,1,2,2,298,
1,1,291,0,202,493,0,1,2,0,2,3,4,5,512,
0,1,0,289,0,0,0,289,0,1,1,2,2,291,298,
0,1,1,291,0,1,203,494,0,1,2,3,3,497,512,
0,0,0,1,0,0,0,1,0,380,382,382,382,383,388,
0,0,1,0,1,2,0,0,378,0,380,380,380,380,387,
0,0,1,2,1,2,0,2,380,382,768,768,768,770,790,
0,95,1,3,1,3,0,3,380,382,768,864,867,870,903,
96,96,2,3,1,4,0,3,382,384,773,870,972,975,1022,
97,97,293,3,203,497,0,4,384,384,776,874,977,981,1535,
102,102,314,314,211,526,211,526,390,391,792,895,1004,1531,4096
]);
array[1..numSubTrees_10 , 1..numSubTrees_9] of var DomainSize: overlap1D_9_10 = array2d(1..numSubTrees_10 , 1..numSubTrees_9, [
overlap1DCounts(limits_10[i, 1], limits_10[i, 2], limits_9[j, 1], limits_9[j, 2]) |
i in 1..numSubTrees_10,
j in 1..numSubTrees_9
]);
constraint objPerTimeStep[10] = normOfDiffCounts(numSubTrees_9, numSubTrees_10, overlapND_9_10, overlap1D_9_10);

array[1..numSubTrees_11 , 1..numSubTrees_10] of DomainSize: overlapND_10_11 = array2d(1..numSubTrees_11 , 1..numSubTrees_10, [
91,0,0,0,1,1,0,0,0,0,0,91,91,92,102,
0,90,0,0,0,1,1,1,0,0,0,0,90,91,102,
1,0,0,0,301,302,0,0,1,0,1,2,2,304,314,
0,1,0,0,0,0,302,303,0,0,0,0,1,1,314,
0,0,196,1,0,198,0,2,0,0,0,0,0,198,211,
1,1,196,1,301,500,0,2,1,0,1,2,3,503,526,
0,0,1,195,0,1,0,197,0,1,2,2,2,3,211,
0,1,1,195,1,2,302,500,0,1,2,2,3,5,526,
0,0,0,0,1,1,0,0,377,0,382,382,382,383,390,
0,0,0,1,0,0,0,1,0,378,383,383,383,383,391,
0,0,0,1,1,1,1,2,379,380,771,771,771,772,792,
0,90,0,1,1,2,2,3,379,380,771,772,862,864,895,
91,90,0,1,2,3,2,3,379,380,771,863,953,956,1004,
91,91,1,196,3,5,304,503,379,381,773,865,956,961,1531,
97,97,206,206,318,531,318,531,389,390,796,894,992,1524,4096
]);
array[1..numSubTrees_11 , 1..numSubTrees_10] of var DomainSize: overlap1D_10_11 = array2d(1..numSubTrees_11 , 1..numSubTrees_10, [
overlap1DCounts(limits_11[i, 1], limits_11[i, 2], limits_10[j, 1], limits_10[j, 2]) |
i in 1..numSubTrees_11,
j in 1..numSubTrees_10
]);
constraint objPerTimeStep[11] = normOfDiffCounts(numSubTrees_10, numSubTrees_11, overlapND_10_11, overlap1D_10_11);

array[1..numSubTrees_12 , 1..numSubTrees_11] of DomainSize: overlapND_11_12 = array2d(1..numSubTrees_12 , 1..numSubTrees_11, [
81,0,0,0,1,1,0,0,0,0,0,81,83,83,97,
0,80,0,0,0,1,1,1,0,0,0,0,82,83,97,
0,0,198,1,0,199,0,1,0,0,0,0,0,1,206,
0,0,1,197,0,1,0,198,0,1,1,1,1,199,206,
1,0,0,0,308,309,0,0,1,0,1,2,2,2,318,
1,1,199,1,310,512,0,2,1,0,1,2,3,5,531,
0,1,0,0,0,0,309,310,0,0,0,0,1,311,318,
0,1,1,198,0,1,311,512,0,1,2,2,3,515,531,
0,0,0,0,1,1,0,0,380,0,382,382,383,383,389,
0,0,0,1,0,0,0,1,0,381,383,383,384,385,390,
0,0,0,1,1,1,0,1,382,383,769,769,775,776,796,
81,0,0,1,2,2,0,1,382,383,769,851,859,860,894,
81,80,0,1,2,3,1,2,382,383,769,851,942,944,992,
82,81,199,2,312,515,1,4,383,383,770,853,946,950,1524,
85,85,208,208,328,543,328,543,388,389,786,872,980,1524,4096
]);
array[1..numSubTrees_12 , 1..numSubTrees_11] of var DomainSize: overlap1D_11_12 = array2d(1..numSubTrees_12 , 1..numSubTrees_11, [
overlap1DCounts(limits_12[i, 1], limits_12[i, 2], limits_11[j, 1], limits_11[j, 2]) |
i in 1..numSubTrees_12,
j in 1..numSubTrees_11
]);
constraint objPerTimeStep[12] = normOfDiffCounts(numSubTrees_11, numSubTrees_12, overlapND_11_12, overlap1D_11_12);

array[1..numSubTrees_13 , 1..numSubTrees_12] of DomainSize: overlapND_12_13 = array2d(1..numSubTrees_13 , 1..numSubTrees_12, [
67,0,0,0,0,0,1,1,0,0,0,1,1,1,85,
0,67,0,0,1,2,0,0,0,0,0,0,2,69,85,
0,0,0,195,0,0,0,195,1,1,3,198,198,198,208,
0,0,195,2,0,195,0,3,0,0,0,3,198,198,208,
1,0,0,0,0,0,319,320,1,0,1,321,321,321,328,
1,1,0,196,0,0,321,518,2,1,4,522,522,523,543,
0,1,0,0,319,320,0,0,0,1,1,1,321,322,328,
0,1,196,2,321,518,1,4,0,1,1,5,523,524,543,
0,0,1,0,0,1,1,1,373,0,384,385,386,386,388,
0,0,1,0,1,2,0,0,0,374,385,385,387,387,389,
0,0,2,0,2,4,1,1,375,376,773,774,778,778,786,
67,0,2,0,2,4,2,2,375,376,773,776,780,780,872,
69,69,2,0,3,6,2,2,376,377,784,787,793,862,980,
69,70,198,2,324,524,3,6,376,378,786,793,1317,1387,1524,
73,73,204,204,342,549,342,549,382,383,808,1358,1910,1984,4096
]);
array[1..numSubTrees_13 , 1..numSubTrees_12] of var DomainSize: overlap1D_12_13 = array2d(1..numSubTrees_13 , 1..numSubTrees_12, [
overlap1DCounts(limits_13[i, 1], limits_13[i, 2], limits_12[j, 1], limits_12[j, 2]) |
i in 1..numSubTrees_13,
j in 1..numSubTrees_12
]);
constraint objPerTimeStep[13] = normOfDiffCounts(numSubTrees_12, numSubTrees_13, overlapND_12_13, overlap1D_12_13);

array[1..numSubTrees_14 , 1..numSubTrees_13] of DomainSize: overlapND_13_14 = array2d(1..numSubTrees_14 , 1..numSubTrees_13, [
3,0,59,0,0,0,0,0,0,0,0,0,0,0,0,0,59,60,73,
0,1,0,59,0,0,0,0,1,2,2,0,0,0,0,2,2,65,73,
0,0,0,0,196,0,0,0,0,0,197,1,0,1,1,198,198,198,204,
0,0,0,0,0,197,0,198,0,0,0,0,0,0,198,198,198,198,204,
0,0,0,1,0,0,0,0,328,329,330,0,1,1,1,331,331,332,342,
0,0,0,2,196,0,0,1,328,329,527,1,1,2,3,530,530,532,549,
0,0,1,0,0,0,330,330,0,0,0,1,0,1,331,331,332,332,342,
0,0,1,0,0,197,331,529,0,0,0,1,0,2,531,531,532,532,549,
0,0,0,0,1,0,1,1,0,0,1,375,0,377,378,379,379,381,382,
0,0,0,0,0,0,0,0,1,1,1,0,377,379,379,380,380,382,383,
0,0,0,0,1,0,1,1,1,2,3,380,382,772,773,776,776,786,808,
0,0,1,0,1,197,332,530,1,2,3,381,382,774,1305,1308,1309,1319,1358,
0,0,1,2,197,198,332,532,329,331,530,383,384,778,1311,1841,1842,1854,1910,
0,1,1,61,198,198,332,532,330,333,533,383,384,778,1311,1844,1845,1920,1984,
4,1,64,65,208,208,350,561,343,350,561,389,390,796,1358,1922,1987,2083,4096
]);
array[1..numSubTrees_14 , 1..numSubTrees_13] of var DomainSize: overlap1D_13_14 = array2d(1..numSubTrees_14 , 1..numSubTrees_13, [
overlap1DCounts(limits_14[i, 1], limits_14[i, 2], limits_13[j, 1], limits_13[j, 2]) |
i in 1..numSubTrees_14,
j in 1..numSubTrees_13
]);
constraint objPerTimeStep[14] = normOfDiffCounts(numSubTrees_13, numSubTrees_14, overlapND_13_14, overlap1D_13_14);

array[1..numSubTrees_15 , 1..numSubTrees_14] of DomainSize: overlapND_14_15 = array2d(1..numSubTrees_15 , 1..numSubTrees_14, [
2,0,1,0,0,1,0,0,0,1,0,0,0,1,1,1,2,2,4,
0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,
1,0,50,0,0,0,0,0,0,0,0,0,0,0,0,0,55,55,64,
0,0,0,50,1,0,0,2,0,0,0,0,0,0,2,52,57,57,65,
0,0,0,0,0,0,199,199,1,1,0,0,0,1,200,200,205,205,208,
0,0,0,0,0,0,1,1,198,198,0,1,1,199,200,200,205,205,208,
1,0,0,0,0,335,0,0,0,335,1,0,1,336,336,336,341,341,350,
1,0,0,0,0,336,1,1,198,534,1,1,3,537,538,538,549,549,561,
0,1,0,0,332,0,0,332,0,0,0,0,0,0,332,332,335,336,343,
0,1,0,1,336,1,0,336,0,1,0,0,0,1,337,338,341,342,350,
0,1,0,1,337,1,199,536,1,3,0,0,0,3,539,540,549,550,561,
0,0,0,0,0,1,0,0,0,1,379,0,382,383,383,383,387,387,389,
0,0,0,0,0,0,0,0,1,1,0,380,383,384,384,384,388,388,390,
0,0,0,0,1,1,0,1,1,2,382,383,775,777,780,780,790,790,796,
1,0,0,0,1,337,1,2,199,536,383,384,778,1315,1319,1319,1340,1340,1358,
1,1,0,1,339,339,200,539,200,540,383,384,778,1319,1861,1862,1892,1893,1922,
2,1,50,1,339,339,200,539,200,540,383,384,778,1319,1861,1863,1948,1949,1987,
2,4,52,54,342,341,201,544,201,543,384,385,780,1324,1871,1926,2024,2029,2083,
8,7,55,56,356,356,210,567,210,567,389,390,794,1362,1932,1989,2145,2153,4096
]);
array[1..numSubTrees_15 , 1..numSubTrees_14] of var DomainSize: overlap1D_14_15 = array2d(1..numSubTrees_15 , 1..numSubTrees_14, [
overlap1DCounts(limits_15[i, 1], limits_15[i, 2], limits_14[j, 1], limits_14[j, 2]) |
i in 1..numSubTrees_15,
j in 1..numSubTrees_14
]);
constraint objPerTimeStep[15] = normOfDiffCounts(numSubTrees_14, numSubTrees_15, overlapND_14_15, overlap1D_14_15);

array[1..numSubTrees_16 , 1..numSubTrees_15] of DomainSize: overlapND_15_16 = array2d(1..numSubTrees_16 , 1..numSubTrees_15, [
6,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,8,
0,7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,7,
1,0,42,0,0,0,0,0,0,0,0,0,0,0,42,43,51,51,55,
0,0,0,43,0,0,0,1,0,0,0,0,0,43,44,44,53,53,56,
0,0,0,0,0,0,0,0,344,344,0,0,0,0,0,344,352,352,356,
0,0,0,0,0,0,344,344,0,0,0,0,0,0,0,0,352,352,356,
0,0,0,0,198,1,0,1,0,198,0,0,1,1,1,199,209,209,210,
0,0,0,1,198,1,0,1,344,542,0,0,1,2,2,544,562,562,567,
0,0,0,0,1,197,0,197,0,2,1,0,1,1,1,3,209,209,210,
0,0,0,0,1,197,344,541,1,3,1,0,1,1,1,4,562,562,567,
0,0,0,0,0,0,0,0,0,0,0,380,383,383,383,383,389,389,389,
0,0,0,0,0,1,0,1,0,0,380,0,383,383,383,383,390,390,390,
0,0,0,0,0,1,1,2,0,0,384,384,774,774,774,774,792,792,794,
0,0,0,0,1,198,345,543,1,3,385,384,775,776,776,779,1355,1355,1362,
0,0,0,1,199,199,345,544,345,545,385,384,776,778,778,1323,1919,1920,1932,
0,0,0,44,199,199,345,545,345,545,385,384,776,821,822,1367,1973,1974,1989,
2,1,44,46,201,201,353,555,353,555,389,388,788,835,880,1436,2103,2105,2145,
2,8,44,46,201,201,353,555,353,555,389,388,788,835,880,1436,2104,2113,2153,
13,12,47,48,208,208,361,570,361,570,392,391,794,843,893,1464,2217,2230,4096
]);
array[1..numSubTrees_16 , 1..numSubTrees_15] of var DomainSize: overlap1D_15_16 = array2d(1..numSubTrees_16 , 1..numSubTrees_15, [
overlap1DCounts(limits_16[i, 1], limits_16[i, 2], limits_15[j, 1], limits_15[j, 2]) |
i in 1..numSubTrees_16,
j in 1..numSubTrees_15
]);
constraint objPerTimeStep[16] = normOfDiffCounts(numSubTrees_15, numSubTrees_16, overlapND_15_16, overlap1D_15_16);

array[1..numSubTrees_17 , 1..numSubTrees_16] of DomainSize: overlapND_16_17 = array2d(1..numSubTrees_17 , 1..numSubTrees_16, [
11,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,13,
0,11,0,0,0,1,0,1,0,0,0,0,0,1,1,1,1,12,12,
1,0,38,0,0,0,0,0,0,0,0,0,0,0,0,0,46,46,47,
0,0,0,39,0,0,0,1,0,0,0,0,0,1,1,40,48,48,48,
0,0,0,0,0,0,201,201,0,0,1,0,1,202,202,202,208,208,208,
0,0,0,0,0,0,0,0,202,202,0,0,0,0,202,202,208,208,208,
0,0,0,0,347,0,0,0,0,348,1,0,1,1,349,349,359,359,361,
0,0,0,1,347,0,0,0,202,550,1,0,1,1,551,552,568,568,570,
0,1,0,0,0,346,0,347,0,0,0,1,1,348,348,348,358,359,361,
0,1,0,0,0,346,201,548,0,1,1,1,2,550,551,551,567,568,570,
0,0,0,0,0,1,0,1,0,0,0,382,385,386,386,386,392,392,392,
0,0,0,0,1,0,1,1,0,1,380,0,383,384,385,385,391,391,391,
0,0,0,0,1,1,1,3,0,1,381,383,770,773,774,774,794,794,794,
0,0,0,39,1,1,1,4,0,1,381,383,770,775,776,815,843,843,843,
1,0,38,39,1,1,1,4,0,1,382,384,772,777,778,817,892,892,893,
1,1,38,39,1,347,202,552,0,2,383,385,774,1327,1329,1368,1460,1461,1464,
1,1,40,42,359,358,208,570,208,570,390,391,790,1361,1932,1975,2184,2186,2217,
1,12,40,42,359,359,208,571,208,571,390,391,790,1362,1934,1977,2186,2199,2230,
20,19,41,42,361,361,213,578,213,578,391,392,794,1373,1952,1995,2301,2321,4096
]);
array[1..numSubTrees_17 , 1..numSubTrees_16] of var DomainSize: overlap1D_16_17 = array2d(1..numSubTrees_17 , 1..numSubTrees_16, [
overlap1DCounts(limits_17[i, 1], limits_17[i, 2], limits_16[j, 1], limits_16[j, 2]) |
i in 1..numSubTrees_17,
j in 1..numSubTrees_16
]);
constraint objPerTimeStep[17] = normOfDiffCounts(numSubTrees_16, numSubTrees_17, overlapND_16_17, overlap1D_16_17);

array[1..numSubTrees_18 , 1..numSubTrees_17] of DomainSize: overlapND_17_18 = array2d(1..numSubTrees_18 , 1..numSubTrees_17, [
19,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,20,20,
0,19,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,19,
1,0,31,0,0,0,0,0,0,0,0,0,0,0,0,0,40,41,41,
0,0,0,32,0,0,0,0,0,0,1,1,1,1,1,33,42,42,42,
0,0,0,0,0,0,349,0,0,0,0,0,0,0,349,349,361,361,361,
0,0,0,0,0,0,0,349,0,0,0,0,0,349,349,349,361,361,361,
0,0,0,0,203,1,0,0,0,0,0,1,206,206,207,207,213,213,213,
0,0,0,1,203,1,0,350,0,0,0,1,206,556,557,558,578,578,578,
0,0,0,0,1,202,0,0,0,1,1,203,206,206,206,207,213,213,213,
0,0,0,0,1,202,350,1,0,1,1,203,206,207,557,558,578,578,578,
0,0,0,0,0,0,0,0,382,0,383,383,383,384,384,384,391,391,391,
0,0,0,0,0,1,0,0,0,382,383,384,385,385,385,385,392,392,392,
0,0,0,0,0,1,1,0,384,384,770,771,774,775,776,776,794,794,794,
0,0,0,1,203,2,1,350,384,384,770,773,981,1332,1334,1335,1373,1373,1373,
0,0,0,1,204,204,351,351,384,385,771,976,1188,1540,1892,1894,1952,1952,1952,
0,0,0,33,204,204,351,351,384,385,772,977,1190,1542,1894,1928,1995,1995,1995,
1,0,34,36,212,212,364,364,392,393,788,1001,1222,1587,1952,1989,2269,2271,2301,
1,19,34,36,212,212,364,364,392,393,788,1001,1222,1587,1952,1989,2270,2272,2321,
26,25,35,36,215,215,364,364,392,393,788,1004,1228,1593,1958,1995,2385,2412,4096
]);
array[1..numSubTrees_18 , 1..numSubTrees_17] of var DomainSize: overlap1D_17_18 = array2d(1..numSubTrees_18 , 1..numSubTrees_17, [
overlap1DCounts(limits_18[i, 1], limits_18[i, 2], limits_17[j, 1], limits_17[j, 2]) |
i in 1..numSubTrees_18,
j in 1..numSubTrees_17
]);
constraint objPerTimeStep[18] = normOfDiffCounts(numSubTrees_17, numSubTrees_18, overlapND_17_18, overlap1D_17_18);

array[1..numSubTrees_19 , 1..numSubTrees_18] of DomainSize: overlapND_18_19 = array2d(1..numSubTrees_19 , 1..numSubTrees_18, [
24,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,26,
0,24,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,25,25,
1,0,27,0,0,0,0,0,0,0,0,27,27,27,27,27,34,34,35,
0,0,0,28,0,0,0,0,0,0,1,1,29,29,29,29,36,36,36,
0,0,0,0,0,0,203,1,0,0,0,0,0,0,0,1,214,215,215,
0,0,0,0,0,0,1,202,1,0,1,1,1,1,1,203,215,215,215,
0,0,0,0,353,0,0,0,0,1,1,1,1,1,354,354,364,364,364,
0,1,0,0,0,353,0,0,0,0,0,0,0,353,353,353,363,364,364,
0,0,0,0,1,0,0,0,0,380,383,383,384,384,385,385,392,392,392,
0,0,0,0,0,0,0,1,381,0,384,384,385,385,385,386,393,393,393,
0,0,0,1,1,0,0,1,381,380,767,767,770,770,771,772,788,788,788,
0,0,0,1,1,0,1,203,382,380,768,769,772,772,773,976,1004,1004,1004,
0,0,0,1,1,0,204,204,382,380,770,771,776,776,777,981,1227,1228,1228,
0,1,0,1,1,353,204,204,382,380,770,771,776,1129,1130,1334,1591,1593,1593,
0,1,0,1,354,354,204,204,382,381,771,772,777,1131,1485,1689,1956,1958,1958,
0,1,0,29,355,354,204,204,382,381,772,773,806,1160,1515,1719,1993,1995,1995,
1,1,31,33,368,367,212,212,392,391,794,826,866,1234,1603,1816,2360,2362,2385,
25,1,32,33,368,367,212,212,392,391,794,827,867,1235,1604,1817,2362,2364,2412,
37,36,32,33,368,368,212,212,392,391,794,827,867,1236,1605,1818,2485,2522,4096
]);
array[1..numSubTrees_19 , 1..numSubTrees_18] of var DomainSize: overlap1D_18_19 = array2d(1..numSubTrees_19 , 1..numSubTrees_18, [
overlap1DCounts(limits_19[i, 1], limits_19[i, 2], limits_18[j, 1], limits_18[j, 2]) |
i in 1..numSubTrees_19,
j in 1..numSubTrees_18
]);
constraint objPerTimeStep[19] = normOfDiffCounts(numSubTrees_18, numSubTrees_19, overlapND_18_19, overlap1D_18_19);

array[1..numSubTrees_20 , 1..numSubTrees_19] of DomainSize: overlapND_19_20 = array2d(1..numSubTrees_20 , 1..numSubTrees_19, [
29,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,37,
0,30,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,30,36,
1,0,24,0,0,0,0,0,0,0,0,0,0,0,25,25,31,31,32,
0,0,0,25,0,0,0,1,0,0,0,0,0,25,26,26,33,33,33,
0,0,0,0,0,0,0,0,351,351,0,0,0,0,0,351,368,368,368,
0,0,0,0,0,0,350,350,0,0,0,1,1,1,1,1,368,368,368,
0,0,0,0,205,1,0,205,0,1,1,0,1,1,1,2,212,212,212,
0,0,0,0,1,205,0,1,0,205,0,1,1,1,1,206,212,212,212,
0,0,0,0,0,1,1,1,0,1,0,383,384,384,384,385,392,392,392,
0,0,0,0,1,0,0,1,0,0,383,0,384,384,384,384,391,391,391,
0,0,0,1,1,1,1,2,0,1,385,385,772,773,773,774,794,794,794,
1,0,24,1,1,1,1,2,0,1,385,385,772,774,799,800,826,826,827,
1,0,24,26,1,1,1,3,0,1,385,385,772,799,826,828,865,866,867,
1,0,24,26,1,1,351,353,0,1,385,386,773,800,827,829,1234,1235,1236,
1,0,24,26,1,1,351,353,351,352,385,386,773,800,827,1180,1603,1604,1605,
1,0,24,26,2,206,351,354,351,557,385,387,774,801,828,1386,1816,1817,1818,
1,0,26,28,219,219,360,581,360,581,393,394,792,821,852,1434,2359,2360,2485,
1,30,26,28,219,219,360,581,360,581,393,394,792,821,852,1434,2360,2391,2522,
31,30,27,28,219,219,360,581,360,581,393,394,792,821,853,1435,2387,2418,4096
]);
array[1..numSubTrees_20 , 1..numSubTrees_19] of var DomainSize: overlap1D_19_20 = array2d(1..numSubTrees_20 , 1..numSubTrees_19, [
overlap1DCounts(limits_20[i, 1], limits_20[i, 2], limits_19[j, 1], limits_19[j, 2]) |
i in 1..numSubTrees_20,
j in 1..numSubTrees_19
]);
constraint objPerTimeStep[20] = normOfDiffCounts(numSubTrees_19, numSubTrees_20, overlapND_19_20, overlap1D_19_20);

array[1..numSubTrees_21 , 1..numSubTrees_20] of DomainSize: overlapND_20_21 = array2d(1..numSubTrees_21 , 1..numSubTrees_20, [
28,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,29,31,
0,27,0,0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,30,
1,0,21,0,0,0,0,0,0,0,0,0,21,21,21,21,26,27,27,
0,0,0,22,0,0,0,0,0,0,1,23,23,23,23,23,28,28,28,
0,0,0,0,0,0,0,211,0,0,0,0,0,0,0,0,219,219,219,
0,0,0,0,0,0,211,0,0,0,0,0,0,0,0,211,219,219,219,
0,1,0,0,0,347,0,0,0,0,0,0,0,0,347,347,358,358,360,
0,1,0,1,0,347,0,212,0,0,0,1,1,1,348,348,579,579,581,
0,0,0,0,347,0,0,0,1,0,1,1,1,348,348,348,359,359,360,
0,0,0,0,347,1,212,0,1,0,1,1,1,348,349,561,580,580,581,
0,0,0,0,1,0,0,0,383,0,385,385,385,386,386,386,393,393,393,
0,0,0,0,0,0,0,0,0,385,387,387,387,387,387,387,394,394,394,
0,0,0,0,2,0,0,0,383,385,774,774,774,776,776,776,792,792,792,
0,0,0,22,2,0,0,0,383,385,775,798,798,800,800,800,821,821,821,
1,0,21,22,2,0,0,0,384,386,777,800,821,823,823,823,852,853,853,
1,0,21,22,349,1,212,0,385,386,778,801,822,1171,1172,1384,1433,1434,1435,
1,1,23,25,360,359,223,223,392,393,796,822,846,1207,1567,1791,2273,2274,2387,
1,28,23,25,360,360,223,223,392,393,796,822,846,1207,1568,1792,2274,2276,2418,
29,28,24,25,360,360,223,223,392,393,796,822,847,1208,1569,1793,2301,2331,4096
]);
array[1..numSubTrees_21 , 1..numSubTrees_20] of var DomainSize: overlap1D_20_21 = array2d(1..numSubTrees_21 , 1..numSubTrees_20, [
overlap1DCounts(limits_21[i, 1], limits_21[i, 2], limits_20[j, 1], limits_20[j, 2]) |
i in 1..numSubTrees_21,
j in 1..numSubTrees_20
]);
constraint objPerTimeStep[21] = normOfDiffCounts(numSubTrees_20, numSubTrees_21, overlapND_20_21, overlap1D_20_21);

array[1..numSubTrees_22 , 1..numSubTrees_21] of DomainSize: overlapND_21_22 = array2d(1..numSubTrees_22 , 1..numSubTrees_21, [
2,0,26,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,29,
0,0,0,27,0,0,0,0,0,0,0,0,0,0,0,0,0,27,28,
19,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,22,22,24,
0,20,0,0,0,0,0,0,0,0,1,1,1,1,1,21,24,24,25,
0,0,0,0,0,0,0,346,0,0,0,0,0,0,346,346,356,356,360,
0,0,0,0,0,0,346,0,0,0,0,0,0,346,346,346,356,356,360,
0,0,0,0,1,212,0,0,0,1,1,2,214,214,214,215,223,223,223,
0,0,0,0,213,1,0,0,0,0,0,213,214,214,215,215,223,223,223,
0,0,0,0,0,0,0,0,383,0,384,384,384,384,385,385,392,392,392,
0,0,0,0,0,1,0,0,0,383,384,384,385,385,386,386,393,393,393,
0,1,0,0,0,1,0,0,385,385,772,772,773,773,775,776,796,796,796,
0,21,0,0,0,1,0,0,385,385,773,774,775,775,777,798,821,821,822,
19,21,1,0,0,1,0,0,385,385,773,774,775,775,777,798,844,844,847,
19,21,1,0,0,1,0,346,385,385,773,774,775,775,1123,1144,1201,1201,1208,
19,21,1,0,0,1,346,346,385,385,774,775,776,1122,1470,1491,1558,1558,1569,
19,21,1,0,1,213,346,346,385,386,776,778,991,1337,1685,1707,1782,1782,1793,
23,25,1,0,224,224,355,355,393,394,796,1021,1246,1602,1962,1988,2193,2194,2301,
25,25,27,0,224,224,355,355,393,394,796,1021,1246,1602,1962,1988,2196,2197,2331,
25,25,27,27,225,225,355,355,393,394,796,1022,1248,1604,1964,1990,2224,2252,4096
]);
array[1..numSubTrees_22 , 1..numSubTrees_21] of var DomainSize: overlap1D_21_22 = array2d(1..numSubTrees_22 , 1..numSubTrees_21, [
overlap1DCounts(limits_22[i, 1], limits_22[i, 2], limits_21[j, 1], limits_21[j, 2]) |
i in 1..numSubTrees_22,
j in 1..numSubTrees_21
]);
constraint objPerTimeStep[22] = normOfDiffCounts(numSubTrees_21, numSubTrees_22, overlapND_21_22, overlap1D_21_22);

array[1..numSubTrees_23 , 1..numSubTrees_22] of DomainSize: overlapND_22_23 = array2d(1..numSubTrees_23 , 1..numSubTrees_22, [
18,0,1,0,0,0,0,0,0,0,0,0,0,0,0,18,21,22,25,
0,19,0,0,0,0,0,1,0,0,0,0,0,1,1,1,23,23,25,
1,0,25,0,0,0,0,0,0,0,0,0,0,0,0,1,1,26,27,
0,0,0,25,0,0,0,0,1,1,0,0,0,0,1,1,1,1,27,
0,0,0,0,216,1,0,1,0,216,1,0,1,2,220,220,224,224,225,
0,0,0,0,1,216,0,216,0,1,0,1,1,217,220,220,224,224,225,
0,0,0,1,0,0,0,0,336,337,0,1,1,1,340,340,348,348,355,
0,0,0,0,0,0,337,338,0,0,1,0,1,339,341,341,349,349,355,
0,0,0,0,1,0,1,1,0,1,382,0,382,383,386,386,392,392,393,
0,0,0,0,0,1,0,1,1,1,0,383,383,384,387,387,393,393,394,
0,1,0,0,1,1,1,2,1,2,384,385,771,773,779,779,792,792,796,
0,1,0,0,217,2,1,3,1,218,385,385,772,776,1000,1000,1017,1017,1022,
0,1,0,0,219,218,1,219,1,220,385,386,773,993,1221,1221,1242,1242,1248,
0,1,0,1,219,219,1,220,337,557,385,387,774,995,1562,1562,1591,1591,1604,
0,1,0,1,219,219,339,559,338,558,386,387,776,1336,1906,1906,1943,1943,1964,
0,20,0,1,219,219,339,560,338,559,386,387,776,1337,1908,1908,1967,1967,1990,
20,22,1,1,229,229,347,579,346,578,392,393,794,1374,1971,1992,2106,2108,2224,
20,22,1,26,229,229,347,579,347,579,392,393,794,1374,1972,1993,2108,2110,2252,
22,22,26,26,231,231,348,582,348,582,392,393,794,1377,1978,2001,2148,2175,4096
]);
array[1..numSubTrees_23 , 1..numSubTrees_22] of var DomainSize: overlap1D_22_23 = array2d(1..numSubTrees_23 , 1..numSubTrees_22, [
overlap1DCounts(limits_23[i, 1], limits_23[i, 2], limits_22[j, 1], limits_22[j, 2]) |
i in 1..numSubTrees_23,
j in 1..numSubTrees_22
]);
constraint objPerTimeStep[23] = normOfDiffCounts(numSubTrees_22, numSubTrees_23, overlapND_22_23, overlap1D_22_23);

array[1..numSubTrees_24 , 1..numSubTrees_23] of DomainSize: overlapND_23_24 = array2d(1..numSubTrees_24 , 1..numSubTrees_23, [
17,0,1,0,0,0,0,0,0,0,0,0,0,17,17,17,17,17,22,
0,17,0,0,0,0,0,0,0,0,1,1,1,1,18,18,18,18,22,
1,0,24,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,26,
0,0,0,25,0,0,0,0,0,0,0,0,0,0,0,0,0,25,26,
0,0,0,0,1,223,0,0,0,0,0,0,0,0,0,1,226,227,231,
0,0,0,0,222,1,0,0,0,1,1,1,1,1,1,223,227,227,231,
0,0,0,0,0,0,333,0,0,0,0,0,334,335,335,335,340,340,348,
0,1,0,0,222,1,334,0,0,1,1,1,336,337,338,560,569,569,582,
0,0,0,0,0,0,0,333,0,0,0,333,335,335,335,335,340,340,348,
0,0,0,0,1,223,1,334,0,0,0,334,337,337,337,338,568,569,582,
0,0,0,0,0,0,0,0,384,0,385,385,386,386,386,386,387,387,392,
0,0,0,0,1,0,0,0,0,384,385,385,386,386,386,387,388,388,393,
0,0,0,0,1,0,0,1,388,388,778,779,781,781,781,782,784,784,794,
0,1,0,0,223,1,334,1,388,389,779,781,1118,1119,1120,1343,1354,1354,1377,
0,1,0,0,226,226,336,336,389,389,780,1117,1458,1459,1460,1686,1933,1934,1978,
17,1,1,0,226,226,336,336,389,390,781,1118,1459,1477,1478,1704,1951,1952,2001,
19,20,1,0,231,231,341,341,395,396,794,1136,1482,1502,1523,1755,2036,2037,2148,
20,20,25,0,231,231,341,341,395,396,794,1136,1482,1503,1524,1756,2038,2039,2175,
21,21,25,25,239,239,342,342,395,396,794,1137,1484,1506,1528,1768,2074,2100,4096
]);
array[1..numSubTrees_24 , 1..numSubTrees_23] of var DomainSize: overlap1D_23_24 = array2d(1..numSubTrees_24 , 1..numSubTrees_23, [
overlap1DCounts(limits_24[i, 1], limits_24[i, 2], limits_23[j, 1], limits_23[j, 2]) |
i in 1..numSubTrees_24,
j in 1..numSubTrees_23
]);
constraint objPerTimeStep[24] = normOfDiffCounts(numSubTrees_23, numSubTrees_24, overlapND_23_24, overlap1D_23_24);

array[1..numSubTrees_25 , 1..numSubTrees_24] of DomainSize: overlapND_24_25 = array2d(1..numSubTrees_25 , 1..numSubTrees_24, [
15,0,1,0,0,0,0,0,0,0,0,15,15,15,15,15,16,17,21,
0,15,0,0,0,0,0,0,0,0,1,1,16,16,16,16,17,17,21,
1,0,23,0,0,0,0,0,0,0,0,1,1,1,1,1,1,24,25,
0,0,0,23,1,0,0,0,0,0,0,0,0,0,0,1,1,1,25,
0,0,0,0,0,0,0,230,0,0,0,0,0,0,231,231,234,234,239,
0,0,0,0,0,0,230,0,0,0,0,0,0,230,231,231,234,234,239,
0,0,0,0,0,328,0,0,1,0,1,1,1,1,1,1,332,332,342,
0,0,0,1,328,0,0,0,0,0,0,0,0,0,0,328,331,331,342,
0,0,0,0,0,1,0,0,383,0,385,385,385,385,385,385,388,388,395,
0,0,0,0,0,0,0,0,0,385,387,387,387,387,387,387,389,389,396,
0,1,0,0,0,1,0,0,383,385,772,772,773,773,773,773,778,778,794,
0,1,0,1,328,1,0,0,383,385,772,773,774,774,774,1102,1110,1110,1137,
0,1,0,1,328,329,0,0,384,385,773,774,775,775,775,1103,1443,1443,1484,
15,1,1,1,328,329,0,0,384,385,773,789,790,790,790,1118,1459,1461,1506,
15,16,1,1,328,329,0,0,384,385,774,790,806,806,806,1134,1476,1478,1528,
15,16,1,1,328,329,0,230,384,385,774,790,806,806,1037,1365,1710,1712,1768,
16,17,1,1,334,336,232,232,388,389,782,799,817,1050,1285,1620,1985,1987,2074,
16,17,1,24,336,336,232,232,388,389,782,799,817,1050,1285,1622,1987,1989,2100,
18,18,25,25,341,341,242,242,394,395,794,813,832,1075,1322,1664,2042,2068,4096
]);
array[1..numSubTrees_25 , 1..numSubTrees_24] of var DomainSize: overlap1D_24_25 = array2d(1..numSubTrees_25 , 1..numSubTrees_24, [
overlap1DCounts(limits_25[i, 1], limits_25[i, 2], limits_24[j, 1], limits_24[j, 2]) |
i in 1..numSubTrees_25,
j in 1..numSubTrees_24
]);
constraint objPerTimeStep[25] = normOfDiffCounts(numSubTrees_24, numSubTrees_25, overlapND_24_25, overlap1D_24_25);

array[1..numSubTrees_26 , 1..numSubTrees_25] of DomainSize: overlapND_25_26 = array2d(1..numSubTrees_26 , 1..numSubTrees_25, [
12,0,1,0,0,0,0,0,1,0,0,0,0,0,0,12,12,13,18,
0,13,0,0,0,0,0,1,1,0,0,0,0,0,0,0,13,14,18,
1,0,23,0,0,0,0,0,23,0,1,1,0,0,0,1,1,24,25,
0,0,0,23,0,1,0,1,1,0,0,23,0,0,1,1,1,2,25,
0,0,0,1,0,321,0,321,324,0,0,1,0,1,1,1,1,325,341,
0,0,0,0,322,0,0,0,0,0,322,325,1,0,1,1,1,1,341,
0,0,0,0,0,0,231,231,233,1,1,1,1,0,1,1,1,234,242,
0,0,0,0,0,0,1,1,1,231,231,233,0,1,1,1,1,2,242,
0,0,0,0,1,0,1,1,1,0,1,1,384,0,384,384,384,385,394,
0,0,0,0,0,1,0,1,1,1,1,1,0,385,385,385,385,386,395,
0,1,0,0,1,1,1,2,2,1,2,2,384,385,771,771,772,774,794,
12,1,1,0,1,1,1,2,3,1,2,2,384,385,771,784,785,788,813,
12,14,1,0,1,1,1,3,4,1,2,2,384,385,771,784,798,802,832,
12,14,1,0,1,1,232,234,237,2,3,3,385,385,772,785,799,1036,1075,
13,15,1,0,1,1,234,236,239,233,234,236,385,386,773,787,802,1041,1322,
13,15,1,1,1,322,234,557,563,234,235,238,385,387,774,788,803,1366,1664,
13,15,1,1,326,325,236,562,568,236,562,568,389,390,781,795,810,1378,2042,
14,15,24,1,326,325,236,562,591,236,563,569,389,390,781,796,811,1402,2068,
17,17,24,24,331,331,248,581,611,248,581,611,396,397,798,816,836,1448,4096
]);
array[1..numSubTrees_26 , 1..numSubTrees_25] of var DomainSize: overlap1D_25_26 = array2d(1..numSubTrees_26 , 1..numSubTrees_25, [
overlap1DCounts(limits_26[i, 1], limits_26[i, 2], limits_25[j, 1], limits_25[j, 2]) |
i in 1..numSubTrees_26,
j in 1..numSubTrees_25
]);
constraint objPerTimeStep[26] = normOfDiffCounts(numSubTrees_25, numSubTrees_26, overlapND_25_26, overlap1D_25_26);

array[1..numSubTrees_27 , 1..numSubTrees_26] of DomainSize: overlapND_26_27 = array2d(1..numSubTrees_27 , 1..numSubTrees_26, [
1,12,0,0,0,0,0,0,0,0,0,12,13,13,13,13,13,13,17,
13,0,0,0,0,0,0,0,0,0,0,0,13,13,13,13,13,13,17,
0,0,24,0,0,0,0,0,0,0,0,0,0,0,0,24,24,24,24,
0,0,0,24,0,0,0,0,0,0,0,0,0,0,0,0,24,24,24,
0,0,0,0,0,0,317,0,0,0,0,0,0,0,0,0,0,0,331,
0,0,0,0,0,0,0,317,0,0,0,0,0,0,0,0,0,317,331,
0,0,0,0,242,1,0,0,0,0,0,0,0,1,243,243,243,243,248,
0,0,0,0,243,1,0,317,0,0,1,1,1,2,245,245,245,562,581,
0,0,25,0,243,1,0,318,0,0,1,1,1,2,245,270,270,588,611,
0,0,0,0,1,241,0,0,0,1,1,1,1,242,243,243,243,243,248,
0,0,0,0,1,242,317,0,0,1,1,1,1,243,244,244,245,245,581,
0,0,0,25,1,242,318,0,0,1,1,1,1,243,244,244,270,270,611,
0,0,0,0,0,0,0,0,384,0,385,385,386,386,387,387,387,387,396,
0,0,0,0,0,1,0,0,0,384,385,385,386,387,388,388,388,388,397,
0,0,0,0,0,1,0,0,385,385,772,772,774,775,778,779,779,780,798,
1,12,0,0,0,1,0,0,385,385,772,785,788,789,792,793,793,794,816,
15,13,0,0,0,1,0,0,385,385,772,786,804,805,808,809,809,810,836,
15,13,25,0,243,2,0,318,385,385,773,787,805,808,1054,1080,1080,1399,1448,
17,15,25,25,258,258,328,328,392,393,788,804,824,1083,1344,1370,1396,1725,4096
]);
array[1..numSubTrees_27 , 1..numSubTrees_26] of var DomainSize: overlap1D_26_27 = array2d(1..numSubTrees_27 , 1..numSubTrees_26, [
overlap1DCounts(limits_27[i, 1], limits_27[i, 2], limits_26[j, 1], limits_26[j, 2]) |
i in 1..numSubTrees_27,
j in 1..numSubTrees_26
]);
constraint objPerTimeStep[27] = normOfDiffCounts(numSubTrees_26, numSubTrees_27, overlapND_26_27, overlap1D_26_27);

array[1..numSubTrees_28 , 1..numSubTrees_27] of DomainSize: overlapND_27_28 = array2d(1..numSubTrees_28 , 1..numSubTrees_27, [
2,0,10,0,1,0,0,0,1,0,0,0,0,0,10,11,11,11,13,14,17,
0,10,0,1,0,0,1,0,0,0,0,0,0,0,0,11,11,11,11,11,15,
0,1,0,22,0,0,22,0,1,1,0,0,0,0,0,1,1,2,2,3,25,
0,0,1,0,22,0,0,0,23,0,1,0,0,0,1,1,2,2,2,25,25,
0,0,0,0,0,248,248,1,1,0,0,0,0,0,0,0,0,0,0,1,258,
0,0,0,0,0,1,1,247,247,0,0,0,1,1,1,1,1,1,1,248,258,
0,0,0,1,0,0,1,0,0,308,0,1,0,1,1,1,1,309,312,312,328,
0,0,0,0,1,0,0,0,1,0,309,0,0,0,0,0,309,309,311,313,328,
0,0,0,0,0,0,0,0,0,1,0,385,0,385,385,385,385,386,386,386,392,
0,0,0,0,0,0,0,1,1,0,0,0,386,386,386,386,386,386,386,387,393,
0,0,1,0,0,0,0,1,1,1,0,385,386,771,772,772,772,773,773,774,788,
0,10,1,1,0,0,1,1,1,1,0,385,386,771,773,784,784,785,785,786,804,
2,10,11,1,1,0,1,1,2,1,0,385,386,771,783,795,795,796,798,800,824,
2,10,11,1,1,1,2,248,249,1,0,385,387,772,784,796,796,797,799,1048,1083,
2,10,11,1,1,249,250,249,250,1,0,385,387,772,784,796,796,797,799,1049,1344,
2,11,11,23,1,249,272,249,251,2,0,385,387,772,784,797,797,799,801,1052,1370,
2,11,12,23,23,250,273,249,274,2,1,385,387,772,785,798,799,801,803,1077,1396,
2,11,12,23,24,250,273,250,276,2,310,385,387,772,785,798,1108,1110,1114,1391,1725,
2,13,14,25,25,265,292,265,292,317,317,396,397,794,809,827,1145,1463,1472,1765,4096
]);
array[1..numSubTrees_28 , 1..numSubTrees_27] of var DomainSize: overlap1D_27_28 = array2d(1..numSubTrees_28 , 1..numSubTrees_27, [
overlap1DCounts(limits_28[i, 1], limits_28[i, 2], limits_27[j, 1], limits_27[j, 2]) |
i in 1..numSubTrees_28,
j in 1..numSubTrees_27
]);
constraint objPerTimeStep[28] = normOfDiffCounts(numSubTrees_27, numSubTrees_28, overlapND_27_28, overlap1D_27_28);

array[1..numSubTrees_29 , 1..numSubTrees_28] of DomainSize: overlapND_28_29 = array2d(1..numSubTrees_29 , 1..numSubTrees_28, [
0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,
0,11,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,13,
1,0,12,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,13,14,
0,1,0,22,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,24,25,
0,0,0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,24,25,25,25,
0,0,0,0,0,0,0,256,256,257,1,1,1,0,1,2,259,259,259,259,265,
1,1,0,22,0,0,0,257,257,259,1,1,1,0,1,2,261,261,284,285,292,
0,0,0,0,0,0,0,1,1,1,256,256,0,1,2,258,259,259,259,259,265,
0,0,0,0,23,0,0,1,1,1,256,256,0,1,2,258,259,285,286,286,292,
0,0,0,0,0,301,0,0,0,0,0,301,0,0,0,301,303,303,303,303,317,
0,0,0,0,0,0,300,0,300,300,0,0,0,1,1,1,303,303,303,303,317,
0,0,0,0,0,0,0,1,1,1,0,0,384,0,384,384,386,386,386,386,396,
0,0,0,0,0,0,1,0,1,1,1,1,0,384,384,385,387,387,387,387,397,
0,0,0,0,0,0,1,1,2,2,2,2,384,384,768,770,774,774,774,774,794,
1,0,12,0,0,0,1,1,2,3,2,2,384,384,768,771,776,776,776,788,809,
1,11,12,1,0,0,1,1,2,3,2,2,385,385,770,773,778,778,779,791,827,
1,11,12,1,0,0,301,1,302,303,2,2,385,386,771,774,1081,1081,1082,1094,1145,
1,11,12,1,0,301,301,1,302,303,2,303,385,386,772,1076,1385,1385,1386,1398,1463,
1,11,13,1,0,301,301,1,302,303,2,304,385,386,773,1078,1387,1387,1388,1401,1472,
1,11,13,1,23,301,301,2,303,304,258,560,385,387,775,1336,1646,1672,1674,1687,1765,
3,14,15,25,25,309,309,271,582,586,271,585,395,396,794,1380,1979,2008,2035,2051,4096
]);
array[1..numSubTrees_29 , 1..numSubTrees_28] of var DomainSize: overlap1D_28_29 = array2d(1..numSubTrees_29 , 1..numSubTrees_28, [
overlap1DCounts(limits_29[i, 1], limits_29[i, 2], limits_28[j, 1], limits_28[j, 2]) |
i in 1..numSubTrees_29,
j in 1..numSubTrees_28
]);
constraint objPerTimeStep[29] = normOfDiffCounts(numSubTrees_28, numSubTrees_29, overlapND_28_29, overlap1D_28_29);

array[1..numSubTrees_30 , 1..numSubTrees_29] of DomainSize: overlapND_29_30 = array2d(1..numSubTrees_30 , 1..numSubTrees_29, [
1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,3,
0,0,1,0,10,0,0,0,0,1,0,0,0,0,0,0,1,1,11,11,11,11,14,
0,1,0,1,0,10,0,0,0,0,0,1,0,0,0,1,1,11,11,11,11,12,15,
0,0,20,0,1,0,0,1,0,20,0,0,0,0,0,0,20,20,21,21,22,22,25,
0,0,0,20,0,1,1,0,0,0,0,20,0,0,0,20,20,21,21,22,22,22,25,
0,0,1,0,0,0,0,293,0,1,0,0,1,0,1,1,2,2,2,2,295,295,309,
0,0,0,1,0,0,294,0,0,0,0,1,0,0,0,1,1,1,1,295,295,295,309,
0,0,0,0,0,0,0,0,261,261,1,1,0,0,0,1,262,262,262,262,264,264,271,
0,0,0,1,0,0,295,0,261,262,1,2,0,0,0,2,264,264,264,559,561,561,582,
1,1,0,1,0,1,295,0,261,262,1,2,0,0,0,2,264,265,265,560,563,564,586,
0,0,0,0,0,0,0,0,1,1,260,260,0,1,1,261,262,262,262,262,264,264,271,
2,1,1,0,1,0,0,294,1,2,260,260,1,1,2,262,264,264,265,265,561,562,585,
0,0,0,0,0,0,0,1,0,0,0,0,386,0,386,386,386,387,387,387,388,388,395,
0,0,0,0,0,0,0,0,0,0,1,1,0,387,387,388,389,389,389,389,389,389,396,
0,0,0,0,0,0,0,1,0,0,1,1,387,388,775,776,777,778,778,778,779,780,794,
2,1,1,0,1,0,0,295,1,2,261,261,388,389,777,1039,1042,1043,1044,1044,1341,1343,1380,
4,3,1,1,1,1,296,296,263,265,264,265,388,389,777,1043,1309,1311,1312,1608,1909,1913,1979,
4,3,1,21,1,2,297,296,264,266,264,285,388,389,777,1063,1330,1333,1334,1631,1932,1936,2008,
4,3,21,21,2,2,297,297,264,286,264,285,388,389,777,1063,1350,1353,1355,1652,1954,1958,2035,
4,4,21,22,2,12,297,297,264,286,264,286,388,389,777,1064,1351,1364,1366,1663,1965,1970,2051,
6,6,22,22,15,15,302,302,277,300,277,300,397,398,796,1097,1400,1416,1432,1735,2046,2053,4096
]);
array[1..numSubTrees_30 , 1..numSubTrees_29] of var DomainSize: overlap1D_29_30 = array2d(1..numSubTrees_30 , 1..numSubTrees_29, [
overlap1DCounts(limits_30[i, 1], limits_30[i, 2], limits_29[j, 1], limits_29[j, 2]) |
i in 1..numSubTrees_30,
j in 1..numSubTrees_29
]);
constraint objPerTimeStep[30] = normOfDiffCounts(numSubTrees_29, numSubTrees_30, overlapND_29_30, overlap1D_29_30);

array[1..numSubTrees_31 , 1..numSubTrees_30] of DomainSize: overlapND_30_31 = array2d(1..numSubTrees_31 , 1..numSubTrees_30, [
1,4,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,2,6,6,6,
4,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1,5,5,5,6,
0,0,17,0,1,0,0,0,0,17,1,0,0,1,2,0,0,0,2,2,2,19,22,
0,0,0,17,0,1,0,17,0,0,0,1,0,0,0,0,1,2,2,2,2,2,22,
0,1,1,0,10,0,0,0,0,1,0,0,0,0,10,0,0,0,10,10,11,12,15,
1,0,0,1,0,10,0,1,0,0,0,0,0,0,0,0,0,10,10,11,11,11,15,
0,0,0,1,0,0,0,1,0,0,0,282,0,0,0,0,282,282,282,282,284,284,302,
0,0,1,0,0,0,0,0,0,1,281,0,1,282,282,0,0,0,282,282,283,285,302,
0,0,0,0,0,0,1,1,269,269,0,0,0,0,0,0,0,0,0,0,0,269,277,
1,0,17,0,1,0,1,1,269,286,1,0,0,1,2,0,0,0,2,3,3,289,300,
0,0,0,0,0,0,268,268,1,1,0,0,0,0,0,1,1,1,1,1,1,2,277,
0,0,0,17,0,1,268,285,2,2,0,1,0,0,0,1,2,3,3,3,3,5,300,
0,0,0,0,0,0,0,0,0,0,1,0,389,390,390,0,0,0,390,390,390,390,397,
0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,389,389,390,390,390,390,390,398,
0,0,0,0,0,0,2,2,0,0,1,0,389,390,390,389,389,390,780,780,780,780,796,
0,0,0,17,0,1,270,287,2,2,1,1,389,390,390,390,391,393,783,784,784,786,1097,
1,0,17,17,1,1,271,288,271,288,2,1,390,392,393,391,392,394,787,789,789,1077,1400,
2,0,17,18,1,11,271,289,271,288,2,1,390,392,393,391,392,404,797,800,800,1088,1416,
2,1,18,18,11,11,271,289,271,289,2,1,390,392,403,391,392,404,807,810,811,1100,1432,
2,1,18,19,11,11,271,290,271,289,2,283,390,392,403,391,674,686,1089,1092,1095,1384,1735,
2,1,19,19,11,11,271,290,271,290,283,284,391,674,686,391,675,688,1374,1377,1383,1674,2046,
6,1,19,19,11,12,271,291,271,290,284,284,391,675,687,391,675,689,1376,1383,1389,1680,2053,
10,8,19,19,15,15,289,309,289,309,292,292,396,689,705,396,689,706,1412,1423,1438,1748,4096
]);
array[1..numSubTrees_31 , 1..numSubTrees_30] of var DomainSize: overlap1D_30_31 = array2d(1..numSubTrees_31 , 1..numSubTrees_30, [
overlap1DCounts(limits_31[i, 1], limits_31[i, 2], limits_30[j, 1], limits_30[j, 2]) |
i in 1..numSubTrees_31,
j in 1..numSubTrees_30
]);
constraint objPerTimeStep[31] = normOfDiffCounts(numSubTrees_30, numSubTrees_31, overlapND_30_31, overlap1D_30_31);

array[1..numSubTrees_32 , 1..numSubTrees_31] of DomainSize: overlapND_31_32 = array2d(1..numSubTrees_32 , 1..numSubTrees_31, [
0,0,0,7,0,1,0,1,0,7,0,0,0,0,7,0,0,1,8,9,9,9,10,
0,0,7,0,1,0,0,7,0,0,0,0,0,0,0,0,0,7,7,7,8,8,8,
0,10,0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,2,2,19,
11,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,12,19,
0,1,1,0,10,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,11,11,15,
1,0,0,1,0,10,0,0,0,1,0,0,0,0,1,0,0,0,1,11,11,12,15,
0,0,0,0,0,0,0,0,0,0,275,1,0,1,1,1,278,278,281,281,281,281,289,
11,0,0,1,0,1,0,0,0,1,275,1,0,1,2,1,278,278,282,283,283,294,309,
0,0,0,0,0,0,0,0,0,0,1,275,1,278,278,0,2,2,281,281,281,281,289,
0,10,0,0,1,0,1,1,0,0,2,275,1,278,278,0,3,4,283,283,284,284,309,
0,1,0,0,0,0,277,277,0,0,0,0,0,0,0,0,0,277,277,277,277,277,292,
0,0,0,0,0,0,0,0,277,277,0,0,0,0,277,1,1,1,278,278,278,278,292,
0,0,0,0,0,0,0,0,0,0,0,1,381,383,383,0,0,0,383,383,384,384,396,
0,1,0,0,0,0,277,277,0,0,0,2,381,384,384,0,0,277,661,661,662,662,689,
0,2,1,0,10,0,277,278,0,0,0,2,382,385,385,0,0,278,663,663,674,674,705,
0,0,0,0,0,0,0,0,1,1,1,0,0,0,1,380,382,382,383,383,384,384,396,
0,0,0,0,0,0,0,0,278,278,1,0,0,0,278,381,383,383,661,661,662,662,689,
1,0,0,1,0,10,0,0,278,279,1,0,0,0,279,382,384,384,663,674,675,676,706,
1,2,1,1,10,10,277,278,278,279,1,2,382,385,664,382,384,662,1326,1337,1349,1350,1412,
1,2,1,8,10,11,277,279,278,286,1,2,382,385,671,382,384,663,1335,1347,1359,1360,1423,
1,2,9,9,11,11,277,287,278,287,1,2,382,385,673,382,384,672,1346,1358,1371,1372,1438,
1,12,9,9,12,11,278,288,278,287,3,277,383,663,951,382,387,676,1629,1641,1655,1656,1748,
13,12,14,15,13,13,282,297,282,298,291,291,403,699,999,403,699,998,2001,2015,2037,2051,4096
]);
array[1..numSubTrees_32 , 1..numSubTrees_31] of var DomainSize: overlap1D_31_32 = array2d(1..numSubTrees_32 , 1..numSubTrees_31, [
overlap1DCounts(limits_32[i, 1], limits_32[i, 2], limits_31[j, 1], limits_31[j, 2]) |
i in 1..numSubTrees_32,
j in 1..numSubTrees_31
]);
constraint objPerTimeStep[32] = normOfDiffCounts(numSubTrees_31, numSubTrees_32, overlapND_31_32, overlap1D_31_32);

array[1..numSubTrees_33 , 1..numSubTrees_32] of DomainSize: overlapND_32_33 = array2d(1..numSubTrees_33 , 1..numSubTrees_32, [
1,7,0,0,0,0,1,0,0,1,2,0,7,8,0,2,0,8,10,10,10,10,13,
7,0,0,0,1,0,0,0,0,7,8,0,0,0,0,8,0,0,8,8,9,9,12,
0,0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,14,
0,0,0,14,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,15,15,
1,0,0,0,9,0,0,0,0,1,1,0,0,0,0,1,0,0,1,1,10,10,13,
0,0,0,1,0,9,0,0,0,0,0,0,0,0,0,0,0,0,0,9,9,10,13,
0,0,0,0,0,0,1,267,1,1,2,0,0,268,0,2,0,268,270,270,270,270,282,
0,0,14,0,0,0,1,267,1,2,3,0,0,268,0,3,0,268,271,271,271,271,297,
0,0,0,0,0,0,267,0,0,0,268,1,1,1,0,268,1,2,270,270,270,270,282,
0,0,1,14,0,1,267,0,0,0,268,1,1,1,0,268,1,2,270,271,271,285,298,
0,1,0,0,0,0,0,0,1,1,1,288,289,289,0,1,0,289,291,291,291,291,291,
0,0,0,0,0,0,0,1,288,288,288,0,0,1,1,289,0,1,291,291,291,291,291,
0,0,0,0,0,0,0,1,0,0,0,0,0,1,392,392,0,2,394,394,394,394,403,
0,0,0,1,0,0,0,2,288,288,288,0,0,2,394,682,0,3,686,686,686,687,699,
0,0,1,15,0,1,267,2,289,289,557,1,1,3,395,952,1,5,958,959,959,974,999,
0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,392,393,394,394,394,394,403,
0,1,0,0,0,0,0,0,1,1,1,289,291,291,0,1,393,684,687,687,687,687,699,
0,1,14,0,0,0,1,267,2,3,4,290,292,560,0,4,394,954,960,960,960,960,998,
0,1,15,15,0,1,268,269,291,292,561,291,293,563,395,957,395,959,1920,1921,1921,1936,2001,
0,1,15,16,0,10,268,269,291,292,561,291,293,563,395,957,395,959,1921,1931,1931,1947,2015,
1,1,15,16,11,12,269,270,291,293,563,291,293,564,395,959,395,960,1924,1936,1948,1964,2037,
2,8,15,16,11,12,270,270,291,294,565,291,300,572,395,961,395,968,1934,1946,1958,1975,2051,
9,8,18,19,12,12,282,282,303,313,597,303,312,596,396,994,396,993,1995,2008,2023,2043,4096
]);
array[1..numSubTrees_33 , 1..numSubTrees_32] of var DomainSize: overlap1D_32_33 = array2d(1..numSubTrees_33 , 1..numSubTrees_32, [
overlap1DCounts(limits_33[i, 1], limits_33[i, 2], limits_32[j, 1], limits_32[j, 2]) |
i in 1..numSubTrees_33,
j in 1..numSubTrees_32
]);
constraint objPerTimeStep[33] = normOfDiffCounts(numSubTrees_32, numSubTrees_33, overlapND_32_33, overlap1D_32_33);

array[1..numSubTrees_34 , 1..numSubTrees_33] of DomainSize: overlapND_33_34 = array2d(1..numSubTrees_34 , 1..numSubTrees_33, [
4,0,0,0,0,0,0,0,1,5,0,0,0,0,0,0,0,0,0,0,0,5,9,
0,2,0,1,0,0,1,3,0,0,0,0,0,0,0,0,3,0,0,1,1,1,8,
0,0,1,0,17,0,0,0,0,0,0,0,0,0,1,18,18,0,0,0,0,0,18,
0,0,0,0,0,18,0,1,0,0,0,0,0,0,0,0,1,0,0,0,18,18,19,
0,0,9,0,1,0,0,0,0,0,0,0,0,0,10,11,11,0,0,0,0,0,12,
0,1,0,9,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,10,10,10,12,
0,0,0,0,0,0,0,0,0,0,1,267,0,1,1,1,1,1,268,269,269,270,282,
0,0,0,0,0,0,0,0,0,0,268,1,0,268,269,269,270,0,1,1,1,1,282,
1,0,0,0,0,0,0,0,291,292,0,0,1,1,1,1,1,0,0,0,0,292,303,
5,0,0,0,0,1,0,0,292,297,0,0,1,1,1,1,1,0,0,0,1,298,313,
5,0,0,0,0,1,0,0,292,297,1,267,1,2,2,2,2,1,268,269,270,568,597,
0,1,0,0,0,0,292,293,0,0,0,0,0,0,0,0,293,0,1,1,1,1,303,
0,3,0,1,0,0,293,296,0,0,0,1,0,0,0,0,296,0,2,3,3,3,312,
0,3,0,1,0,0,293,296,0,0,268,2,0,268,269,269,566,0,3,4,4,4,596,
0,0,0,0,0,0,0,0,1,1,0,0,384,385,385,386,386,0,0,0,0,1,396,
5,0,0,0,0,1,0,0,293,298,2,267,385,388,388,389,389,1,268,269,270,569,994,
0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,384,386,386,387,387,396,
0,3,0,1,0,0,293,296,0,0,268,3,0,268,269,269,566,384,389,390,391,391,993,
5,3,0,1,0,1,294,297,294,299,270,270,385,656,658,659,957,385,657,659,661,961,1995,
5,4,0,10,0,1,294,298,294,299,270,270,385,656,658,659,958,385,657,670,672,972,2008,
5,4,9,10,1,1,294,298,294,299,271,271,385,657,669,671,970,385,658,671,673,973,2023,
5,4,9,10,1,19,294,299,294,299,271,271,385,657,669,671,971,385,658,671,691,991,2043,
5,4,13,13,23,24,305,311,305,312,275,275,395,673,690,716,1028,395,673,690,717,1030,4096
]);
array[1..numSubTrees_34 , 1..numSubTrees_33] of var DomainSize: overlap1D_33_34 = array2d(1..numSubTrees_34 , 1..numSubTrees_33, [
overlap1DCounts(limits_34[i, 1], limits_34[i, 2], limits_33[j, 1], limits_33[j, 2]) |
i in 1..numSubTrees_34,
j in 1..numSubTrees_33
]);
constraint objPerTimeStep[34] = normOfDiffCounts(numSubTrees_33, numSubTrees_34, overlapND_33_34, overlap1D_33_34);

array[1..numSubTrees_35 , 1..numSubTrees_34] of DomainSize: overlapND_34_35 = array2d(1..numSubTrees_35 , 1..numSubTrees_34, [
0,1,0,0,0,0,0,0,0,0,0,0,1,2,2,0,0,0,0,0,5,
0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,2,2,2,4,
0,10,0,1,0,0,0,0,1,1,0,0,10,11,12,0,0,0,0,0,13,
1,0,10,0,1,0,1,0,0,0,0,0,0,0,0,0,10,10,11,12,13,
0,1,0,21,0,0,0,0,21,0,0,0,1,1,22,0,0,0,0,0,23,
0,0,1,0,21,0,22,0,0,0,0,0,0,0,0,0,1,1,1,23,24,
1,0,0,0,0,0,0,0,0,0,294,0,0,0,0,1,1,296,297,297,305,
1,0,1,0,1,0,1,0,0,0,296,0,0,0,0,1,2,299,300,301,311,
0,0,0,0,0,0,0,0,0,294,0,1,1,297,297,0,0,0,0,0,305,
0,1,0,0,0,0,0,0,0,295,1,1,2,300,300,0,0,1,1,1,312,
0,0,0,0,0,0,0,258,258,0,0,1,2,2,260,0,0,0,0,0,275,
0,0,0,0,0,259,260,0,0,0,0,0,0,0,0,0,0,0,0,260,275,
0,0,0,0,0,0,0,1,1,1,0,384,384,386,389,0,0,0,0,0,395,
0,0,0,0,0,0,0,260,260,2,0,386,387,390,652,0,0,0,0,0,673,
0,10,0,1,0,0,0,260,261,3,0,388,399,403,666,0,0,0,0,0,690,
0,11,0,22,0,0,0,261,283,3,0,388,400,404,689,0,0,0,0,0,716,
1,11,1,22,1,0,1,261,283,3,296,388,400,404,689,1,2,299,300,301,1028,
0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,385,385,387,388,389,395,
0,0,0,0,0,260,261,1,1,0,1,0,0,0,1,386,386,388,389,651,673,
1,0,10,0,1,260,262,1,1,0,1,0,0,0,1,388,398,400,402,665,690,
1,0,11,0,22,261,285,1,1,0,1,0,0,0,1,388,399,401,403,689,717,
1,1,11,0,22,261,285,1,1,295,2,1,2,300,301,388,399,402,404,690,1030,
3,15,15,24,26,266,295,266,294,314,314,396,413,735,1031,396,413,731,736,1033,4096
]);
array[1..numSubTrees_35 , 1..numSubTrees_34] of var DomainSize: overlap1D_34_35 = array2d(1..numSubTrees_35 , 1..numSubTrees_34, [
overlap1DCounts(limits_35[i, 1], limits_35[i, 2], limits_34[j, 1], limits_34[j, 2]) |
i in 1..numSubTrees_35,
j in 1..numSubTrees_34
]);
constraint objPerTimeStep[35] = normOfDiffCounts(numSubTrees_34, numSubTrees_35, overlapND_34_35, overlap1D_34_35);

array[1..numSubTrees_36 , 1..numSubTrees_35] of DomainSize: overlapND_35_36 = array2d(1..numSubTrees_36 , 1..numSubTrees_35, [
1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,1,2,2,2,3,
0,10,0,1,0,1,0,0,1,0,0,0,1,2,0,0,0,0,2,2,15,
0,0,10,0,1,0,0,0,0,0,1,0,0,0,0,0,1,1,2,12,15,
0,1,0,22,0,0,0,0,22,0,0,0,0,22,0,0,0,0,22,22,24,
0,0,1,0,23,0,0,0,1,0,23,0,0,1,0,0,23,23,24,25,26,
0,0,0,0,0,0,0,1,1,249,249,0,1,2,1,1,250,250,252,252,266,
0,0,1,0,24,0,0,1,2,250,274,0,1,3,1,1,275,275,278,279,295,
0,0,0,0,0,0,0,250,250,1,2,0,0,250,0,0,2,2,252,252,266,
0,1,0,24,1,0,0,251,275,1,3,0,0,275,0,0,3,3,278,278,294,
0,1,0,0,0,304,0,0,0,0,0,1,305,305,0,0,0,0,307,307,314,
1,0,0,0,0,0,305,0,0,0,0,0,0,0,0,305,305,306,307,308,314,
0,0,0,0,0,1,0,0,0,0,0,385,386,386,0,0,0,0,387,387,396,
0,10,0,1,0,2,0,0,1,0,0,386,388,389,0,0,0,0,390,390,413,
0,11,0,1,0,307,0,0,1,0,0,388,695,698,0,0,0,0,701,701,735,
0,12,0,25,1,307,0,251,276,1,3,388,695,974,0,0,3,3,980,980,1031,
0,0,0,0,0,0,0,0,0,1,1,0,0,0,385,385,386,386,387,387,396,
0,0,10,0,1,0,0,0,0,1,2,0,0,0,386,387,389,389,391,401,413,
1,0,10,0,1,0,305,0,0,1,2,0,0,0,387,693,695,696,699,710,731,
2,0,11,0,1,0,306,0,0,1,2,0,0,0,387,694,697,699,702,714,736,
2,0,12,0,25,0,307,1,2,251,276,0,1,3,388,696,973,975,981,994,1033,
2,16,16,25,26,322,322,261,287,261,288,397,720,1010,397,720,1009,1012,2028,2045,4096
]);
array[1..numSubTrees_36 , 1..numSubTrees_35] of var DomainSize: overlap1D_35_36 = array2d(1..numSubTrees_36 , 1..numSubTrees_35, [
overlap1DCounts(limits_36[i, 1], limits_36[i, 2], limits_35[j, 1], limits_35[j, 2]) |
i in 1..numSubTrees_36,
j in 1..numSubTrees_35
]);
constraint objPerTimeStep[36] = normOfDiffCounts(numSubTrees_35, numSubTrees_36, overlapND_35_36, overlap1D_35_36);

array[1..numSubTrees_37 , 1..numSubTrees_36] of DomainSize: overlapND_36_37 = array2d(1..numSubTrees_37 , 1..numSubTrees_36, [
1,0,0,0,0,0,0,1,0,1,1,1,1,0,0,0,0,1,2,
11,0,0,0,0,0,1,0,0,12,13,13,14,0,0,0,0,0,16,
0,10,0,1,0,0,0,1,0,0,0,0,0,0,11,12,13,14,16,
0,0,24,0,0,0,0,0,0,0,0,24,24,0,0,0,0,0,25,
0,1,0,24,0,0,0,0,0,0,0,0,0,0,1,1,25,25,26,
1,0,0,0,0,0,312,0,1,2,2,2,314,0,0,0,0,0,322,
0,1,0,0,0,0,0,313,0,0,0,0,0,0,1,1,1,314,322,
0,0,0,0,246,1,0,0,0,0,246,246,248,0,0,1,1,1,261,
0,0,24,0,246,1,0,0,0,1,247,271,273,0,0,1,1,1,287,
0,0,0,0,1,245,0,0,0,0,1,1,1,1,1,246,246,248,261,
0,1,0,24,1,245,0,0,0,0,1,1,2,1,2,247,271,273,288,
0,0,0,0,0,0,1,0,387,387,387,387,388,0,0,0,0,0,397,
1,0,0,0,0,0,313,0,388,389,389,389,702,0,0,0,0,1,720,
1,0,24,0,246,1,314,0,388,390,636,660,976,0,1,2,2,3,1010,
0,0,0,0,0,1,0,0,0,0,0,0,0,387,387,388,388,388,397,
0,1,0,0,0,1,0,313,0,0,0,1,1,387,388,389,389,702,720,
0,2,0,24,1,246,0,313,0,0,1,2,3,388,390,636,660,975,1009,
1,2,0,24,1,246,0,314,0,1,2,3,4,388,390,636,661,977,1012,
2,2,24,24,247,247,314,314,388,391,638,663,980,388,391,638,663,980,2028,
2,12,24,25,247,247,314,315,388,391,638,663,980,388,402,650,676,994,2045,
16,15,24,25,254,254,330,330,397,415,670,695,1030,397,414,669,695,1030,4096
]);
array[1..numSubTrees_37 , 1..numSubTrees_36] of var DomainSize: overlap1D_36_37 = array2d(1..numSubTrees_37 , 1..numSubTrees_36, [
overlap1DCounts(limits_37[i, 1], limits_37[i, 2], limits_36[j, 1], limits_36[j, 2]) |
i in 1..numSubTrees_37,
j in 1..numSubTrees_36
]);
constraint objPerTimeStep[37] = normOfDiffCounts(numSubTrees_36, numSubTrees_37, overlapND_36_37, overlap1D_36_37);

array[1..numSubTrees_38 , 1..numSubTrees_37] of DomainSize: overlapND_37_38 = array2d(1..numSubTrees_38 , 1..numSubTrees_37, [
13,0,0,0,0,0,1,0,0,0,13,13,0,0,0,1,14,14,16,
0,12,0,1,0,0,0,1,0,0,0,1,0,0,12,12,13,14,15,
0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,24,
0,1,0,23,0,0,0,0,0,0,0,0,0,0,1,1,1,24,25,
0,0,0,0,0,236,0,0,1,238,240,240,0,0,0,0,240,240,254,
0,0,0,0,237,0,0,0,0,0,0,0,0,238,240,240,240,240,254,
1,0,0,0,0,0,319,0,1,1,2,2,0,0,0,319,323,323,330,
0,1,0,0,0,0,0,319,0,0,0,319,1,2,3,3,323,323,330,
0,0,0,0,0,1,1,0,384,386,387,387,0,0,0,1,389,389,397,
13,0,0,0,0,1,2,0,384,387,401,401,0,0,0,2,404,404,415,
13,0,0,0,0,237,2,0,385,625,641,641,0,0,0,2,644,644,670,
13,0,23,0,0,237,2,0,385,626,642,642,0,0,0,2,645,645,695,
15,0,24,0,0,238,321,0,386,628,646,646,0,0,0,321,970,970,1030,
0,0,0,0,0,0,0,1,0,0,0,1,385,386,387,387,389,389,397,
0,12,0,1,0,0,0,2,0,0,0,2,385,386,399,400,403,404,414,
0,12,0,1,237,0,0,2,0,0,0,2,385,624,639,640,643,644,669,
0,13,0,24,237,0,0,2,0,0,0,2,385,625,641,642,645,669,695,
0,15,0,25,238,0,0,321,0,0,0,321,386,628,646,647,970,995,1030,
19,18,24,25,244,244,337,337,395,643,666,1007,395,643,665,1006,2019,2045,4096
]);
array[1..numSubTrees_38 , 1..numSubTrees_37] of var DomainSize: overlap1D_37_38 = array2d(1..numSubTrees_38 , 1..numSubTrees_37, [
overlap1DCounts(limits_38[i, 1], limits_38[i, 2], limits_37[j, 1], limits_37[j, 2]) |
i in 1..numSubTrees_38,
j in 1..numSubTrees_37
]);
constraint objPerTimeStep[38] = normOfDiffCounts(numSubTrees_37, numSubTrees_38, overlapND_37_38, overlap1D_37_38);

array[1..numSubTrees_39 , 1..numSubTrees_38] of DomainSize: overlapND_38_39 = array2d(1..numSubTrees_39 , 1..numSubTrees_38, [
16,0,0,0,1,0,0,0,0,1,0,1,0,0,1,17,17,17,19,
0,14,0,1,0,1,0,1,0,0,0,0,0,1,1,1,15,15,18,
0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,24,
0,1,0,23,0,0,0,0,0,0,0,0,0,0,0,0,1,1,25,
0,0,0,0,0,0,1,1,229,230,0,230,1,2,232,232,235,235,244,
0,0,0,0,0,0,230,231,1,1,0,1,0,231,232,232,235,235,244,
1,0,0,0,326,0,0,0,0,326,1,327,0,0,327,328,333,333,337,
0,1,0,0,0,327,0,327,0,0,0,0,0,327,327,327,333,333,337,
0,0,0,0,1,0,0,0,0,1,387,388,0,0,388,388,390,390,395,
0,0,0,0,1,0,230,232,1,2,388,390,0,232,622,622,627,627,643,
16,0,0,0,2,0,231,233,1,3,388,391,0,233,624,641,647,647,666,
16,1,0,0,2,328,232,562,1,3,388,391,0,562,953,970,984,984,1007,
0,0,0,0,0,0,0,0,1,1,0,1,387,387,388,388,390,390,395,
0,0,0,0,0,0,1,1,230,232,0,232,389,390,622,622,627,627,643,
0,14,0,1,0,1,1,2,231,233,0,233,389,391,625,625,645,645,665,
1,14,0,1,327,1,1,2,232,561,1,562,389,391,954,955,981,982,1006,
17,15,0,1,329,329,233,564,233,564,389,954,389,954,1909,1927,1967,1968,2019,
17,16,0,24,329,329,233,564,233,564,389,954,389,954,1910,1928,1969,1970,2045,
21,20,23,24,342,342,238,583,238,583,398,982,398,982,1972,1994,2067,2091,4096
]);
array[1..numSubTrees_39 , 1..numSubTrees_38] of var DomainSize: overlap1D_38_39 = array2d(1..numSubTrees_39 , 1..numSubTrees_38, [
overlap1DCounts(limits_39[i, 1], limits_39[i, 2], limits_38[j, 1], limits_38[j, 2]) |
i in 1..numSubTrees_39,
j in 1..numSubTrees_38
]);
constraint objPerTimeStep[39] = normOfDiffCounts(numSubTrees_38, numSubTrees_39, overlapND_38_39, overlap1D_38_39);

array[1..numSubTrees_40 , 1..numSubTrees_39] of DomainSize: overlapND_39_40 = array2d(1..numSubTrees_40 , 1..numSubTrees_39, [
17,0,0,0,0,0,1,0,0,1,1,18,0,0,0,0,20,20,21,
0,15,0,1,0,0,0,1,0,0,0,0,0,1,1,16,18,19,20,
0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,
0,1,0,23,0,0,0,0,0,0,0,0,0,0,0,1,1,24,24,
1,0,0,0,0,0,330,0,1,332,332,335,0,0,0,0,338,338,342,
0,1,0,0,0,0,0,331,0,0,0,0,0,332,332,335,338,338,342,
0,0,0,0,223,1,0,0,0,0,1,1,0,0,224,224,232,232,238,
0,1,0,0,224,1,0,331,0,1,2,2,0,332,557,560,572,572,583,
0,0,0,0,1,222,0,0,0,0,223,223,1,1,2,2,232,232,238,
1,0,0,0,2,223,330,0,1,332,556,559,1,1,3,3,572,572,583,
0,0,0,0,0,0,1,0,388,391,391,391,0,0,0,0,393,393,398,
1,0,0,0,2,224,331,0,389,723,948,951,1,1,3,3,966,966,982,
0,0,0,0,0,1,0,0,0,0,1,1,388,390,390,390,393,393,398,
0,1,0,0,224,2,0,331,0,1,4,4,388,722,947,950,966,966,982,
1,1,0,0,226,226,333,333,390,727,955,958,390,726,954,957,1940,1940,1972,
18,1,0,0,226,226,334,333,390,728,956,976,390,726,954,957,1960,1961,1994,
20,18,0,1,229,229,337,337,392,733,964,986,392,733,964,984,2017,2019,2067,
20,18,23,1,229,229,337,337,392,733,964,986,392,733,964,984,2018,2020,2091,
23,22,25,26,231,231,348,348,396,748,982,1011,396,748,982,1010,2119,2146,4096
]);
array[1..numSubTrees_40 , 1..numSubTrees_39] of var DomainSize: overlap1D_39_40 = array2d(1..numSubTrees_40 , 1..numSubTrees_39, [
overlap1DCounts(limits_40[i, 1], limits_40[i, 2], limits_39[j, 1], limits_39[j, 2]) |
i in 1..numSubTrees_40,
j in 1..numSubTrees_39
]);
constraint objPerTimeStep[40] = normOfDiffCounts(numSubTrees_39, numSubTrees_40, overlapND_39_40, overlap1D_39_40);

array[1..numSubTrees_41 , 1..numSubTrees_40] of DomainSize: overlapND_40_41 = array2d(1..numSubTrees_41 , 1..numSubTrees_40, [
0,17,0,0,0,0,1,0,0,0,0,18,0,0,1,1,22,22,23,
16,0,0,1,0,0,0,1,0,0,1,1,0,0,0,17,21,21,22,
0,0,25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,25,25,
1,0,0,25,0,0,0,0,0,0,0,0,0,0,0,1,1,1,26,
0,0,0,0,1,217,0,0,1,219,219,220,0,1,1,1,228,228,231,
0,0,0,0,217,1,0,0,0,1,1,1,1,219,219,220,228,228,231,
0,1,0,0,0,0,337,0,1,1,1,2,0,0,337,338,348,348,348,
1,0,0,0,0,0,0,337,0,0,337,338,1,1,1,2,348,348,348,
0,0,0,0,0,1,1,0,384,385,385,385,0,0,1,1,393,393,396,
0,1,0,0,0,1,338,0,385,387,387,388,0,0,338,339,745,745,748,
0,1,0,0,217,2,340,0,385,388,388,389,1,219,559,561,976,976,982,
0,18,0,0,220,2,341,0,387,390,390,409,1,222,563,565,1004,1004,1011,
0,0,0,0,1,0,0,1,0,0,1,1,384,385,385,385,392,393,396,
1,0,0,0,1,0,0,338,0,0,338,339,385,386,386,387,744,745,748,
1,0,0,0,2,217,0,340,1,219,559,561,385,387,387,388,975,976,982,
17,0,0,1,2,220,0,341,1,222,563,565,387,389,389,407,1002,1003,1010,
19,20,0,1,228,228,346,346,395,625,972,995,395,625,972,994,2080,2081,2119,
20,20,0,26,228,228,346,346,395,625,972,995,395,625,972,995,2082,2083,2146,
23,22,25,26,229,229,354,354,396,628,983,1010,396,628,983,1011,2193,2219,4096
]);
array[1..numSubTrees_41 , 1..numSubTrees_40] of var DomainSize: overlap1D_40_41 = array2d(1..numSubTrees_41 , 1..numSubTrees_40, [
overlap1DCounts(limits_41[i, 1], limits_41[i, 2], limits_40[j, 1], limits_40[j, 2]) |
i in 1..numSubTrees_41,
j in 1..numSubTrees_40
]);
constraint objPerTimeStep[41] = normOfDiffCounts(numSubTrees_40, numSubTrees_41, overlapND_40_41, overlap1D_40_41);

array[1..numSubTrees_42 , 1..numSubTrees_41] of DomainSize: overlapND_41_42 = array2d(1..numSubTrees_42 , 1..numSubTrees_41, [
20,0,0,1,0,0,0,0,0,0,0,0,0,0,0,20,22,23,23,
0,19,0,0,0,0,1,0,0,1,1,20,0,0,0,0,22,22,22,
0,0,25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,25,
1,0,0,25,0,0,0,0,0,0,0,0,0,0,0,1,1,26,26,
0,0,0,0,1,215,0,0,0,0,215,215,1,1,2,2,228,228,229,
0,0,0,0,216,1,0,0,0,0,1,1,0,0,216,216,228,228,229,
0,1,0,0,0,0,341,0,0,342,343,346,0,0,0,0,354,354,354,
0,0,0,0,0,0,0,342,0,0,0,0,0,343,344,346,354,354,354,
0,0,0,0,0,0,0,0,385,386,387,387,0,0,0,0,396,396,396,
0,0,0,0,217,1,0,0,385,387,389,389,0,0,217,217,627,627,628,
0,0,0,0,217,1,0,342,386,388,390,390,0,343,561,563,982,982,983,
0,19,0,0,218,1,1,343,387,390,392,411,0,344,563,565,1008,1008,1010,
0,0,0,0,0,1,0,0,0,0,1,1,384,385,386,386,396,396,396,
0,0,0,0,1,217,0,0,0,0,217,217,385,386,388,388,627,627,628,
0,1,0,0,1,217,341,0,0,342,560,563,386,387,389,389,982,982,983,
20,1,0,1,1,218,342,0,0,343,562,565,387,388,390,410,1008,1009,1011,
24,24,0,1,225,225,349,349,395,750,979,1005,395,750,979,1005,2155,2157,2193,
24,24,25,1,225,225,349,349,395,750,979,1005,395,750,979,1005,2156,2158,2219,
26,25,26,27,225,225,354,354,395,755,984,1012,395,755,984,1013,2263,2291,4096
]);
array[1..numSubTrees_42 , 1..numSubTrees_41] of var DomainSize: overlap1D_41_42 = array2d(1..numSubTrees_42 , 1..numSubTrees_41, [
overlap1DCounts(limits_42[i, 1], limits_42[i, 2], limits_41[j, 1], limits_41[j, 2]) |
i in 1..numSubTrees_42,
j in 1..numSubTrees_41
]);
constraint objPerTimeStep[42] = normOfDiffCounts(numSubTrees_41, numSubTrees_42, overlapND_41_42, overlap1D_41_42);

array[1..numSubTrees_43 , 1..numSubTrees_42] of DomainSize: overlapND_42_43 = array2d(1..numSubTrees_43 , 1..numSubTrees_42, [
1,1,21,0,0,1,0,1,0,0,0,0,0,1,1,1,24,25,26,
0,0,0,22,1,0,0,0,0,1,0,1,0,0,1,23,25,25,25,
26,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,26,26,
0,26,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,27,
0,0,0,0,0,0,211,211,1,1,0,1,0,213,214,214,225,225,225,
0,0,0,0,0,0,1,1,210,210,0,212,1,2,214,214,225,225,225,
0,0,0,1,343,0,0,0,0,343,1,344,0,0,344,345,354,354,354,
0,0,1,0,0,344,0,344,0,0,0,0,0,344,344,344,354,354,354,
0,0,0,0,1,0,0,0,0,1,384,387,0,0,387,387,395,395,395,
0,0,0,1,346,0,0,1,0,346,386,734,0,2,736,737,755,755,755,
0,0,0,1,347,0,1,2,211,558,386,948,1,4,952,953,984,984,984,
0,0,0,23,348,0,1,2,211,559,387,951,1,4,955,978,1012,1012,1012,
0,0,0,0,0,0,0,0,1,1,0,1,384,386,387,387,395,395,395,
0,0,1,0,0,346,0,346,1,1,0,1,385,733,735,735,755,755,755,
0,0,1,0,0,347,212,559,2,2,0,2,385,948,951,951,984,984,984,
1,1,22,0,0,348,212,560,2,3,0,3,386,950,954,954,1011,1012,1013,
1,1,26,27,358,358,219,578,219,578,395,981,395,981,1964,1991,2232,2234,2263,
1,27,27,27,358,358,219,578,219,578,395,981,395,981,1964,1992,2234,2236,2291,
29,29,28,28,360,360,219,580,219,580,395,984,395,984,1970,1999,2352,2382,4096
]);
array[1..numSubTrees_43 , 1..numSubTrees_42] of var DomainSize: overlap1D_42_43 = array2d(1..numSubTrees_43 , 1..numSubTrees_42, [
overlap1DCounts(limits_43[i, 1], limits_43[i, 2], limits_42[j, 1], limits_42[j, 2]) |
i in 1..numSubTrees_43,
j in 1..numSubTrees_42
]);
constraint objPerTimeStep[43] = normOfDiffCounts(numSubTrees_42, numSubTrees_43, overlapND_42_43, overlap1D_42_43);

array[1..numSubTrees_44 , 1..numSubTrees_43] of DomainSize: overlapND_43_44 = array2d(1..numSubTrees_44 , 1..numSubTrees_43, [
29,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,29,
0,28,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,29,29,
0,1,23,0,0,0,0,0,0,0,0,0,0,23,23,23,27,28,28,
0,0,0,23,0,0,0,1,0,23,23,24,0,0,0,0,28,28,28,
0,0,0,1,0,0,0,350,0,1,1,351,0,0,0,0,360,360,360,
0,0,0,0,0,0,351,0,0,0,0,0,0,0,0,351,359,360,360,
0,0,0,0,207,1,0,0,0,0,208,208,0,0,1,1,219,219,219,
0,0,0,0,207,1,351,0,0,1,209,209,0,0,1,352,579,580,580,
0,0,0,0,1,206,0,0,0,0,1,1,1,1,208,208,219,219,219,
0,0,0,1,2,206,0,350,0,1,3,353,1,1,208,208,580,580,580,
0,0,0,0,0,0,0,0,386,386,386,388,0,0,0,0,395,395,395,
0,0,0,1,2,207,0,352,388,389,391,745,1,1,209,209,984,984,984,
0,0,0,0,0,1,0,0,0,0,0,0,385,385,386,388,395,395,395,
0,0,0,0,207,2,353,0,0,1,209,209,387,388,390,745,983,984,984,
0,0,0,1,209,209,353,352,388,390,600,955,388,389,599,954,1969,1970,1970,
0,0,0,24,209,209,353,353,388,413,623,979,388,389,599,955,1998,1999,1999,
0,1,28,29,218,218,368,368,394,425,647,1018,394,424,646,1017,2316,2318,2352,
29,1,28,29,218,218,368,368,394,425,647,1018,394,424,646,1017,2317,2319,2382,
32,32,29,29,218,218,368,368,394,425,647,1018,394,425,647,1018,2428,2461,4096
]);
array[1..numSubTrees_44 , 1..numSubTrees_43] of var DomainSize: overlap1D_43_44 = array2d(1..numSubTrees_44 , 1..numSubTrees_43, [
overlap1DCounts(limits_44[i, 1], limits_44[i, 2], limits_43[j, 1], limits_43[j, 2]) |
i in 1..numSubTrees_44,
j in 1..numSubTrees_43
]);
constraint objPerTimeStep[44] = normOfDiffCounts(numSubTrees_43, numSubTrees_44, overlapND_43_44, overlap1D_43_44);

array[1..numSubTrees_45 , 1..numSubTrees_44] of DomainSize: overlapND_44_45 = array2d(1..numSubTrees_45 , 1..numSubTrees_44, [
27,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,
0,28,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,28,32,
0,0,0,25,0,1,0,0,0,0,0,0,1,1,1,1,29,29,29,
0,0,26,0,0,0,0,0,0,0,0,0,0,0,0,26,29,29,29,
0,0,0,0,0,0,209,0,1,1,210,0,0,0,211,211,218,218,218,
0,0,0,0,0,0,0,210,0,0,0,0,0,210,211,211,218,218,218,
0,0,0,1,0,348,0,0,0,0,0,1,349,349,349,349,368,368,368,
0,0,0,0,349,0,0,0,1,350,350,0,0,0,350,350,368,368,368,
0,0,0,0,1,0,1,0,383,384,386,0,0,0,387,387,394,394,394,
0,0,26,0,1,0,1,0,384,386,388,0,0,0,389,415,425,425,425,
0,0,27,0,1,0,210,0,386,388,599,0,0,0,602,629,647,647,647,
0,0,27,0,350,0,210,0,387,738,949,0,1,1,953,980,1018,1018,1018,
0,0,0,0,0,1,0,0,0,0,0,384,385,386,387,387,394,394,394,
0,0,0,25,0,2,0,0,0,0,0,385,387,388,390,390,425,425,425,
0,0,0,26,0,2,0,210,0,0,0,386,388,599,603,603,647,647,647,
0,0,0,27,0,350,0,210,0,0,0,387,737,948,953,953,1018,1018,1018,
0,0,36,36,357,357,217,217,397,755,973,397,755,973,1967,2004,2373,2374,2428,
0,28,36,36,357,357,217,217,397,755,973,397,755,973,1968,2005,2374,2403,2461,
29,29,36,36,357,357,217,217,397,755,973,397,755,973,1968,2005,2440,2470,4096
]);
array[1..numSubTrees_45 , 1..numSubTrees_44] of var DomainSize: overlap1D_44_45 = array2d(1..numSubTrees_45 , 1..numSubTrees_44, [
overlap1DCounts(limits_45[i, 1], limits_45[i, 2], limits_44[j, 1], limits_44[j, 2]) |
i in 1..numSubTrees_45,
j in 1..numSubTrees_44
]);
constraint objPerTimeStep[45] = normOfDiffCounts(numSubTrees_44, numSubTrees_45, overlapND_44_45, overlap1D_44_45);

array[1..numSubTrees_46 , 1..numSubTrees_45] of DomainSize: overlapND_45_46 = array2d(1..numSubTrees_46 , 1..numSubTrees_45, [
23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,29,
0,22,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,29,
0,0,33,0,1,0,0,0,0,1,0,0,0,1,1,1,36,36,36,
0,1,0,33,0,0,0,0,0,0,0,0,0,0,0,33,35,35,36,
0,0,1,0,346,0,0,0,0,346,0,1,0,347,348,348,357,357,357,
0,0,0,0,0,347,0,347,0,1,0,348,0,1,349,349,357,357,357,
0,0,0,0,0,0,203,203,1,1,0,203,0,2,205,205,215,215,217,
0,0,0,0,0,0,1,1,202,202,0,1,1,203,205,205,215,215,217,
0,0,0,0,0,0,0,0,0,0,388,389,0,0,389,389,397,397,397,
0,0,1,0,346,0,0,1,0,346,388,391,0,347,738,738,755,755,755,
0,0,1,0,346,0,203,204,1,347,388,594,0,349,943,943,971,971,973,
0,0,0,0,0,0,0,0,1,1,0,0,387,389,389,389,397,397,397,
0,0,0,0,0,347,0,347,1,2,1,349,387,390,739,739,755,755,755,
0,0,0,0,0,347,1,348,203,204,1,350,388,593,944,944,971,971,973,
0,0,1,0,348,349,207,557,206,555,390,951,390,949,1901,1901,1964,1964,1968,
0,0,34,0,349,349,207,557,207,557,390,951,390,951,1903,1903,2001,2001,2005,
0,1,40,39,363,363,211,575,211,575,397,977,397,977,1956,1996,2314,2315,2440,
0,23,40,40,363,363,211,575,211,575,397,977,397,977,1956,1997,2316,2317,2470,
23,23,40,40,363,363,211,575,211,575,397,977,397,977,1956,1997,2342,2366,4096
]);
array[1..numSubTrees_46 , 1..numSubTrees_45] of var DomainSize: overlap1D_45_46 = array2d(1..numSubTrees_46 , 1..numSubTrees_45, [
overlap1DCounts(limits_46[i, 1], limits_46[i, 2], limits_45[j, 1], limits_45[j, 2]) |
i in 1..numSubTrees_46,
j in 1..numSubTrees_45
]);
constraint objPerTimeStep[46] = normOfDiffCounts(numSubTrees_45, numSubTrees_46, overlapND_45_46, overlap1D_45_46);

array[1..numSubTrees_47 , 1..numSubTrees_46] of DomainSize: overlapND_46_47 = array2d(1..numSubTrees_47 , 1..numSubTrees_46, [
15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,23,
0,14,0,1,0,0,0,0,0,0,0,0,0,0,1,1,2,16,23,
0,0,35,0,1,0,0,0,0,1,36,36,0,0,0,0,39,39,40,
0,1,0,34,0,1,0,0,0,0,0,0,0,1,35,35,38,39,40,
0,0,1,0,347,0,0,0,1,348,349,349,0,0,0,0,361,361,363,
0,0,0,1,0,348,0,0,0,0,0,0,0,348,349,349,360,361,363,
0,0,0,0,0,0,200,1,0,0,0,200,0,0,0,1,207,207,211,
0,0,0,1,0,348,200,1,0,1,1,201,0,348,349,350,568,569,575,
0,0,0,0,0,0,1,199,0,0,0,1,1,1,1,200,207,207,211,
0,0,1,0,347,0,2,199,1,348,349,351,1,1,1,200,569,569,575,
0,0,0,0,1,0,0,0,387,388,389,389,0,0,0,0,397,397,397,
0,0,0,1,1,348,200,2,388,390,391,592,0,349,350,352,970,971,977,
0,0,0,0,0,0,0,1,0,0,0,0,387,387,388,389,397,397,397,
0,0,1,0,347,0,2,200,1,348,349,351,389,389,390,591,971,971,977,
0,0,1,1,348,349,202,202,389,738,740,943,389,739,741,944,1943,1944,1956,
0,1,1,35,349,350,202,202,389,739,741,944,389,740,776,979,1982,1984,1997,
0,1,45,44,362,362,210,210,397,760,806,1018,397,760,805,1017,2224,2226,2342,
15,1,45,44,362,362,210,210,397,760,806,1018,397,760,805,1017,2226,2228,2366,
15,15,45,45,363,363,210,210,397,761,807,1019,397,761,807,1019,2256,2272,4096
]);
array[1..numSubTrees_47 , 1..numSubTrees_46] of var DomainSize: overlap1D_46_47 = array2d(1..numSubTrees_47 , 1..numSubTrees_46, [
overlap1DCounts(limits_47[i, 1], limits_47[i, 2], limits_46[j, 1], limits_46[j, 2]) |
i in 1..numSubTrees_47,
j in 1..numSubTrees_46
]);
constraint objPerTimeStep[47] = normOfDiffCounts(numSubTrees_46, numSubTrees_47, overlapND_46_47, overlap1D_46_47);

array[1..numSubTrees_48 , 1..numSubTrees_47] of DomainSize: overlapND_47_48 = array2d(1..numSubTrees_48 , 1..numSubTrees_47, [
9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,
0,9,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,10,15,
0,0,41,0,0,0,1,0,0,1,1,42,0,0,0,0,45,45,45,
0,1,0,41,0,0,0,0,0,0,0,0,0,0,0,41,44,45,45,
0,0,1,0,0,0,348,0,0,348,348,349,0,0,0,0,358,358,363,
0,0,0,0,0,0,0,348,0,0,0,0,1,349,349,349,358,358,363,
0,0,0,0,198,1,0,0,1,1,201,201,0,0,1,1,205,205,210,
0,0,0,0,1,198,0,0,0,0,1,1,1,1,201,201,205,205,210,
0,0,0,0,1,0,0,0,387,387,388,388,0,0,0,0,394,394,397,
0,0,1,0,1,0,348,0,387,736,737,738,0,0,0,0,753,753,761,
0,0,42,0,1,0,349,0,387,737,738,780,0,0,0,0,799,799,807,
0,0,42,0,199,1,349,0,388,738,940,982,0,0,1,1,1006,1006,1019,
0,0,0,0,0,1,0,1,0,0,0,0,386,387,388,388,394,394,397,
0,0,0,0,0,1,0,349,0,0,0,1,387,736,737,737,753,753,761,
0,1,0,41,0,1,0,349,0,0,0,1,387,736,737,778,798,799,807,
0,1,0,41,1,199,0,349,0,0,1,2,388,737,939,980,1005,1006,1019,
0,1,52,51,207,208,357,357,398,756,970,1024,398,756,971,1024,2142,2144,2256,
0,10,52,52,208,208,357,357,398,756,971,1025,398,756,971,1025,2144,2155,2272,
10,10,53,53,208,208,361,361,398,760,975,1030,398,760,975,1030,2184,2195,4096
]);
array[1..numSubTrees_48 , 1..numSubTrees_47] of var DomainSize: overlap1D_47_48 = array2d(1..numSubTrees_48 , 1..numSubTrees_47, [
overlap1DCounts(limits_48[i, 1], limits_48[i, 2], limits_47[j, 1], limits_47[j, 2]) |
i in 1..numSubTrees_48,
j in 1..numSubTrees_47
]);
constraint objPerTimeStep[48] = normOfDiffCounts(numSubTrees_47, numSubTrees_48, overlapND_47_48, overlap1D_47_48);

array[1..numSubTrees_49 , 1..numSubTrees_48] of DomainSize: overlapND_48_49 = array2d(1..numSubTrees_49 , 1..numSubTrees_48, [
5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,
0,4,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,5,10,
0,0,48,0,0,0,1,0,0,1,50,50,0,0,0,0,51,51,53,
0,1,0,47,0,0,0,1,0,0,0,0,0,1,49,49,50,51,53,
0,0,0,0,199,0,0,0,0,0,0,199,0,0,0,0,202,202,208,
0,0,0,0,0,199,0,0,0,0,0,0,0,0,0,199,202,202,208,
0,0,1,0,0,0,338,0,1,341,343,343,0,0,0,0,346,347,361,
0,0,0,1,0,0,0,339,0,0,0,0,0,341,343,343,347,347,361,
0,0,0,0,0,0,1,0,383,386,387,387,0,0,0,0,391,391,398,
0,0,1,0,0,0,339,0,384,728,731,731,0,0,0,0,738,739,760,
0,0,1,0,203,0,339,0,384,728,732,935,0,0,0,0,945,946,975,
0,0,49,0,203,0,340,0,384,730,783,986,0,1,1,1,998,999,1030,
0,0,0,0,0,0,0,0,0,0,0,0,384,386,387,387,391,391,398,
0,0,0,1,0,0,0,339,0,0,0,1,384,727,730,730,739,739,760,
0,0,0,1,0,203,0,339,0,0,0,1,384,727,731,934,946,946,975,
0,1,0,48,0,203,0,340,0,0,0,1,384,729,781,984,998,999,1030,
0,1,56,55,207,206,344,344,390,741,801,1010,390,741,800,1008,2060,2062,2184,
0,5,56,56,207,207,344,344,390,741,801,1010,390,741,801,1010,2062,2068,2195,
5,5,61,61,209,209,349,349,392,748,813,1025,392,748,813,1025,2100,2106,4096
]);
array[1..numSubTrees_49 , 1..numSubTrees_48] of var DomainSize: overlap1D_48_49 = array2d(1..numSubTrees_49 , 1..numSubTrees_48, [
overlap1DCounts(limits_49[i, 1], limits_49[i, 2], limits_48[j, 1], limits_48[j, 2]) |
i in 1..numSubTrees_49,
j in 1..numSubTrees_48
]);
constraint objPerTimeStep[49] = normOfDiffCounts(numSubTrees_48, numSubTrees_49, overlapND_48_49, overlap1D_48_49);

array[1..numSubTrees_50 , 1..numSubTrees_49] of DomainSize: overlapND_49_50 = array2d(1..numSubTrees_50 , 1..numSubTrees_49, [
0,1,0,0,0,0,0,0,0,1,1,0,0,0,0,1,5,
0,0,2,0,0,0,0,0,0,0,0,0,0,2,2,2,5,
0,55,0,1,0,0,0,0,0,55,55,0,0,0,1,56,61,
1,0,56,0,0,0,0,0,0,0,0,0,0,56,56,56,61,
0,0,0,0,0,198,1,1,199,200,200,0,1,1,1,201,209,
0,0,0,0,0,1,198,0,1,1,1,1,199,200,200,201,209,
0,1,0,333,0,0,0,0,0,1,1,0,0,0,334,336,349,
0,0,0,0,333,0,0,0,0,0,334,1,2,2,2,336,349,
0,0,0,0,0,1,0,385,386,386,386,0,0,0,0,386,392,
0,1,0,335,0,1,0,386,388,389,389,0,0,0,336,726,748,
0,56,0,336,0,1,0,386,388,444,444,0,0,0,338,783,813,
0,56,0,336,0,200,1,388,589,646,646,0,1,1,339,986,1025,
0,0,0,0,1,0,1,0,0,0,1,384,385,385,385,386,392,
0,0,0,0,336,0,1,0,1,1,338,386,388,388,388,726,748,
1,0,56,0,336,0,1,0,1,1,339,386,388,444,444,784,813,
1,0,56,0,336,1,200,0,2,2,340,388,589,646,646,987,1025,
1,57,57,340,340,203,202,390,595,654,997,390,594,653,996,1997,2100,
1,57,59,340,340,203,203,390,595,654,997,390,595,656,999,2000,2106,
2,68,69,347,347,209,209,396,607,677,1027,396,607,678,1028,2059,4096
]);
array[1..numSubTrees_50 , 1..numSubTrees_49] of var DomainSize: overlap1D_49_50 = array2d(1..numSubTrees_50 , 1..numSubTrees_49, [
overlap1DCounts(limits_50[i, 1], limits_50[i, 2], limits_49[j, 1], limits_49[j, 2]) |
i in 1..numSubTrees_50,
j in 1..numSubTrees_49
]);
constraint objPerTimeStep[50] = normOfDiffCounts(numSubTrees_49, numSubTrees_50, overlapND_49_50, overlap1D_49_50);

array[1..numSubTrees_51 , 1..numSubTrees_50] of DomainSize: overlapND_50_51 = array2d(1..numSubTrees_51 , 1..numSubTrees_50, [
0,1,0,0,0,0,0,0,0,0,0,0,0,1,2,
65,0,1,0,0,0,0,0,1,66,0,0,0,0,68,
0,65,0,1,0,0,0,1,1,1,0,0,1,66,69,
1,0,319,0,0,0,1,1,328,329,0,0,0,0,347,
0,1,0,319,0,0,0,0,0,0,1,1,328,329,347,
0,0,0,0,0,196,1,197,199,199,1,1,1,1,209,
0,0,0,0,196,2,0,2,2,2,0,196,198,198,209,
0,0,1,0,1,0,379,380,385,385,0,1,1,1,396,
0,1,1,0,1,196,380,577,585,585,1,2,2,3,607,
65,1,2,0,1,196,380,577,587,652,1,2,2,3,677,
65,2,2,319,1,196,380,577,587,652,2,4,332,334,1027,
0,0,0,1,1,0,0,0,0,0,379,381,386,386,396,
0,0,0,1,197,2,0,2,3,3,379,577,585,585,607,
0,65,0,2,197,2,0,3,4,4,379,577,587,652,678,
1,65,319,2,197,2,1,4,333,334,379,577,587,652,1028,
66,67,321,321,198,198,381,581,921,987,381,581,921,988,2059,
87,87,328,328,204,204,391,597,952,1040,391,597,952,1040,4096
]);
array[1..numSubTrees_51 , 1..numSubTrees_50] of var DomainSize: overlap1D_50_51 = array2d(1..numSubTrees_51 , 1..numSubTrees_50, [
overlap1DCounts(limits_51[i, 1], limits_51[i, 2], limits_50[j, 1], limits_50[j, 2]) |
i in 1..numSubTrees_51,
j in 1..numSubTrees_50
]);
constraint objPerTimeStep[51] = normOfDiffCounts(numSubTrees_50, numSubTrees_51, overlapND_50_51, overlap1D_50_51);

array[1..numSubTrees_52 , 1..numSubTrees_51] of DomainSize: overlapND_51_52 = array2d(1..numSubTrees_52 , 1..numSubTrees_51, [
82,0,0,0,0,0,0,0,82,82,0,0,0,0,87,
0,81,0,0,0,1,0,0,0,1,0,0,81,81,87,
0,0,0,0,313,0,1,1,1,1,0,0,0,314,328,
0,1,0,0,0,313,0,0,0,314,0,1,2,2,328,
0,0,195,1,0,0,0,1,1,1,0,195,196,196,204,
0,0,1,194,0,0,0,194,195,195,1,2,2,2,204,
0,0,0,0,1,0,384,384,384,384,0,0,0,1,391,
0,0,1,194,1,0,384,579,580,581,1,2,2,3,597,
0,0,1,198,317,0,389,589,590,592,1,2,2,320,952,
82,0,1,198,318,0,389,589,672,674,1,2,2,321,1040,
0,0,0,1,0,0,0,1,1,1,384,384,384,384,391,
0,0,195,2,0,0,0,2,2,2,384,579,580,581,597,
0,1,199,2,0,316,0,2,2,319,388,589,591,593,952,
0,82,199,2,0,318,0,2,2,321,388,589,672,674,1040,
92,92,206,206,325,325,396,606,700,1029,396,606,700,1029,4096
]);
array[1..numSubTrees_52 , 1..numSubTrees_51] of var DomainSize: overlap1D_51_52 = array2d(1..numSubTrees_52 , 1..numSubTrees_51, [
overlap1DCounts(limits_52[i, 1], limits_52[i, 2], limits_51[j, 1], limits_51[j, 2]) |
i in 1..numSubTrees_52,
j in 1..numSubTrees_51
]);
constraint objPerTimeStep[52] = normOfDiffCounts(numSubTrees_51, numSubTrees_52, overlapND_51_52, overlap1D_51_52);

array[1..numSubTrees_53 , 1..numSubTrees_52] of DomainSize: overlapND_52_53 = array2d(1..numSubTrees_53 , 1..numSubTrees_52, [
86,0,1,0,0,0,0,1,1,0,0,0,1,1,92,
0,86,0,0,0,0,0,1,1,0,0,0,1,87,92,
0,0,0,0,198,0,0,0,0,0,0,198,199,199,206,
0,0,0,0,0,198,0,0,198,0,0,0,199,199,206,
1,0,304,0,0,0,0,305,305,0,0,0,307,307,325,
0,0,0,305,0,0,0,0,0,0,306,306,307,308,325,
0,0,0,0,0,0,387,388,388,0,0,0,388,388,396,
0,1,0,0,0,199,388,389,588,0,1,1,590,591,606,
86,1,1,0,0,199,389,391,590,0,1,1,592,593,700,
86,1,1,305,0,199,389,391,590,0,307,307,900,902,1029,
0,0,0,0,0,0,0,0,0,387,388,388,388,388,396,
0,0,0,0,199,1,0,0,1,388,389,588,591,591,606,
0,86,0,0,199,1,0,1,2,389,390,589,593,679,700,
1,86,304,0,199,1,0,306,307,389,390,589,901,987,1029,
101,101,312,312,209,209,397,713,923,397,713,923,1858,1960,4096
]);
array[1..numSubTrees_53 , 1..numSubTrees_52] of var DomainSize: overlap1D_52_53 = array2d(1..numSubTrees_53 , 1..numSubTrees_52, [
overlap1DCounts(limits_53[i, 1], limits_53[i, 2], limits_52[j, 1], limits_52[j, 2]) |
i in 1..numSubTrees_53,
j in 1..numSubTrees_52
]);
constraint objPerTimeStep[53] = normOfDiffCounts(numSubTrees_52, numSubTrees_53, overlapND_52_53, overlap1D_52_53);

array[1..numSubTrees_54 , 1..numSubTrees_53] of DomainSize: overlapND_53_54 = array2d(1..numSubTrees_54 , 1..numSubTrees_53, [
1,95,1,0,0,0,0,0,0,1,0,0,1,96,101,
95,0,0,1,0,0,0,1,2,97,0,1,1,1,101,
0,1,293,0,0,0,0,0,0,0,1,1,295,297,312,
1,0,0,294,0,0,0,0,295,297,0,0,0,0,312,
0,0,0,0,200,1,0,200,200,200,0,1,1,1,209,
0,0,0,0,1,199,1,2,2,2,0,199,199,199,209,
0,0,1,0,0,0,0,0,0,0,387,388,389,390,397,
1,1,295,0,0,0,0,0,0,1,388,389,685,688,713,
1,1,296,0,1,199,1,2,2,3,388,588,885,888,923,
0,0,0,0,0,1,387,388,388,389,0,1,1,1,397,
1,0,0,295,0,1,387,388,684,687,0,1,2,2,713,
1,0,0,296,200,2,387,588,885,888,0,2,3,3,923,
2,1,297,297,203,202,388,592,891,896,388,591,890,894,1858,
97,1,297,298,203,203,388,593,893,993,388,593,892,896,1960,
111,109,305,305,209,209,398,610,917,1035,398,610,917,1033,4096
]);
array[1..numSubTrees_54 , 1..numSubTrees_53] of var DomainSize: overlap1D_53_54 = array2d(1..numSubTrees_54 , 1..numSubTrees_53, [
overlap1DCounts(limits_54[i, 1], limits_54[i, 2], limits_53[j, 1], limits_53[j, 2]) |
i in 1..numSubTrees_54,
j in 1..numSubTrees_53
]);
constraint objPerTimeStep[54] = normOfDiffCounts(numSubTrees_53, numSubTrees_54, overlapND_53_54, overlap1D_53_54);

array[1..numSubTrees_55 , 1..numSubTrees_54] of DomainSize: overlapND_54_55 = array2d(1..numSubTrees_55 , 1..numSubTrees_54, [
0,103,0,0,0,0,0,1,1,1,0,0,0,103,111,
102,0,1,0,0,0,0,1,1,103,0,0,0,0,109,
1,0,286,0,0,0,0,287,287,288,0,0,0,0,305,
0,0,0,286,0,0,0,0,0,0,1,288,288,288,305,
0,0,0,0,199,1,1,1,200,200,0,0,1,1,209,
0,0,0,0,1,199,0,0,1,1,1,1,200,200,209,
0,0,0,1,0,1,0,0,0,0,384,387,390,390,398,
0,1,0,1,200,2,1,1,201,201,384,387,391,392,610,
0,1,0,287,200,2,1,1,201,201,385,675,679,680,917,
0,105,0,287,201,2,1,2,203,203,387,678,682,787,1035,
0,0,0,0,1,0,385,387,390,390,0,0,0,0,398,
0,0,0,0,2,200,385,387,391,391,1,1,201,201,610,
1,0,286,0,2,200,385,674,678,679,1,1,201,201,917,
104,0,287,0,2,201,387,678,682,786,1,1,202,202,1033,
116,116,295,295,212,212,394,695,909,1026,394,695,909,1026,4096
]);
array[1..numSubTrees_55 , 1..numSubTrees_54] of var DomainSize: overlap1D_54_55 = array2d(1..numSubTrees_55 , 1..numSubTrees_54, [
overlap1DCounts(limits_55[i, 1], limits_55[i, 2], limits_54[j, 1], limits_54[j, 2]) |
i in 1..numSubTrees_55,
j in 1..numSubTrees_54
]);
constraint objPerTimeStep[55] = normOfDiffCounts(numSubTrees_54, numSubTrees_55, overlapND_54_55, overlap1D_54_55);

array[1..numSubTrees_56 , 1..numSubTrees_55] of DomainSize: overlapND_55_56 = array2d(1..numSubTrees_56 , 1..numSubTrees_55, [
110,0,1,0,0,0,0,1,1,0,0,0,1,1,116,
0,110,0,1,0,0,0,1,1,0,1,1,2,112,116,
1,0,278,0,0,0,1,279,279,0,0,0,280,281,295,
0,1,0,278,0,0,0,0,0,1,279,279,281,282,295,
0,0,0,0,202,1,1,1,2,0,0,202,204,204,212,
0,0,0,0,1,202,0,0,202,1,1,2,204,204,212,
0,0,1,0,1,0,384,385,386,0,0,1,387,387,394,
1,1,280,0,2,0,386,666,667,0,0,2,670,672,695,
1,1,280,0,204,1,387,667,670,0,0,204,875,877,909,
111,1,281,0,205,1,387,668,671,0,0,205,877,879,1026,
0,0,0,1,0,1,0,0,1,384,385,386,387,387,394,
0,1,0,280,0,2,0,0,2,386,666,667,671,672,695,
0,1,0,280,1,204,0,0,204,387,667,670,876,877,909,
0,111,0,281,1,205,0,1,206,387,668,671,879,990,1026,
124,124,289,289,215,215,397,687,905,397,687,905,1814,1939,4096
]);
array[1..numSubTrees_56 , 1..numSubTrees_55] of var DomainSize: overlap1D_55_56 = array2d(1..numSubTrees_56 , 1..numSubTrees_55, [
overlap1DCounts(limits_56[i, 1], limits_56[i, 2], limits_55[j, 1], limits_55[j, 2]) |
i in 1..numSubTrees_56,
j in 1..numSubTrees_55
]);
constraint objPerTimeStep[56] = normOfDiffCounts(numSubTrees_55, numSubTrees_56, overlapND_55_56, overlap1D_55_56);

array[1..numSubTrees_57 , 1..numSubTrees_56] of DomainSize: overlapND_56_57 = array2d(1..numSubTrees_57 , 1..numSubTrees_56, [
118,0,1,0,0,0,0,1,1,119,0,0,0,0,124,
0,117,0,1,0,0,0,1,1,1,0,1,1,118,124,
1,0,267,0,0,0,1,269,271,274,0,0,0,0,289,
0,1,0,268,0,0,0,0,0,0,0,269,271,274,289,
0,0,0,0,206,0,0,0,0,0,0,0,206,207,215,
0,0,0,0,0,206,0,0,206,207,0,0,0,0,215,
0,0,1,0,0,0,386,389,389,389,0,0,0,0,397,
1,1,268,0,0,0,387,658,660,663,0,0,0,1,687,
1,1,268,0,0,206,387,658,866,871,0,0,0,1,905,
0,0,0,0,0,0,0,0,0,0,387,389,389,389,397,
0,1,0,268,0,0,0,0,0,1,387,658,660,663,687,
0,1,0,268,206,0,0,0,0,1,387,658,866,871,905,
1,2,268,268,206,206,387,658,866,872,387,658,866,873,1814,
1,119,268,269,206,206,387,659,867,873,387,659,867,991,1939,
129,129,276,276,215,215,394,674,894,1029,394,674,894,1029,4096
]);
array[1..numSubTrees_57 , 1..numSubTrees_56] of var DomainSize: overlap1D_56_57 = array2d(1..numSubTrees_57 , 1..numSubTrees_56, [
overlap1DCounts(limits_57[i, 1], limits_57[i, 2], limits_56[j, 1], limits_56[j, 2]) |
i in 1..numSubTrees_57,
j in 1..numSubTrees_56
]);
constraint objPerTimeStep[57] = normOfDiffCounts(numSubTrees_56, numSubTrees_57, overlapND_56_57, overlap1D_56_57);

array[1..numSubTrees_58 , 1..numSubTrees_57] of DomainSize: overlapND_57_58 = array2d(1..numSubTrees_58 , 1..numSubTrees_57, [
121,0,1,0,0,0,0,0,1,0,0,0,1,1,129,
0,122,0,0,0,0,0,1,1,0,0,0,1,123,129,
1,0,262,0,0,0,0,0,262,0,0,0,263,264,276,
0,0,0,262,0,0,0,0,0,1,1,263,265,265,276,
0,0,0,0,205,1,1,2,2,0,205,205,208,208,215,
0,0,0,0,1,205,0,205,205,1,2,2,208,208,215,
0,0,0,0,1,0,382,382,383,0,2,2,385,385,394,
1,1,262,0,1,0,382,382,645,0,2,2,648,650,674,
1,1,264,0,2,205,383,588,853,1,4,4,860,862,894,
123,1,265,0,2,205,383,588,856,1,4,5,864,866,1029,
0,0,0,1,0,1,0,1,1,381,381,383,385,385,394,
0,0,0,263,0,1,0,1,1,382,382,646,650,650,674,
0,0,0,265,205,2,1,3,3,383,588,854,862,862,894,
0,123,0,265,205,2,1,4,5,383,588,856,866,989,1029,
135,135,272,272,217,217,392,612,890,392,612,890,1790,1926,4096
]);
array[1..numSubTrees_58 , 1..numSubTrees_57] of var DomainSize: overlap1D_57_58 = array2d(1..numSubTrees_58 , 1..numSubTrees_57, [
overlap1DCounts(limits_58[i, 1], limits_58[i, 2], limits_57[j, 1], limits_57[j, 2]) |
i in 1..numSubTrees_58,
j in 1..numSubTrees_57
]);
constraint objPerTimeStep[58] = normOfDiffCounts(numSubTrees_57, numSubTrees_58, overlapND_57_58, overlap1D_57_58);

array[1..numSubTrees_59 , 1..numSubTrees_58] of DomainSize: overlapND_58_59 = array2d(1..numSubTrees_59 , 1..numSubTrees_58, [
128,0,0,0,0,0,0,0,0,0,0,0,0,128,135,
0,127,0,0,1,0,0,1,1,128,0,1,1,1,135,
0,0,0,0,0,255,0,0,0,0,1,257,257,259,272,
0,1,0,0,255,0,0,256,256,259,0,0,0,0,272,
0,0,207,1,0,0,0,0,209,209,0,0,1,1,217,
0,0,1,206,0,0,1,1,2,2,0,0,208,208,217,
0,0,0,0,0,1,0,0,0,0,383,384,386,386,392,
0,0,1,207,0,1,1,2,3,3,384,385,596,596,612,
0,0,1,209,0,257,1,2,3,3,387,645,858,860,890,
0,0,0,1,0,0,383,383,385,385,0,0,1,1,392,
0,0,208,2,0,0,384,384,596,596,0,0,2,2,612,
0,1,210,2,256,0,386,643,857,860,0,0,2,2,890,
0,1,211,211,256,257,388,646,861,864,388,646,862,864,1790,
0,128,211,211,257,257,388,647,863,993,388,647,863,865,1926,
138,138,220,220,263,263,394,659,887,1031,394,659,887,1031,4096
]);
array[1..numSubTrees_59 , 1..numSubTrees_58] of var DomainSize: overlap1D_58_59 = array2d(1..numSubTrees_59 , 1..numSubTrees_58, [
overlap1DCounts(limits_59[i, 1], limits_59[i, 2], limits_58[j, 1], limits_58[j, 2]) |
i in 1..numSubTrees_59,
j in 1..numSubTrees_58
]);
constraint objPerTimeStep[59] = normOfDiffCounts(numSubTrees_58, numSubTrees_59, overlapND_58_59, overlap1D_58_59);

array[1..numSubTrees_60 , 1..numSubTrees_59] of DomainSize: overlapND_59_60 = array2d(1..numSubTrees_60 , 1..numSubTrees_59, [
1,130,0,0,0,0,1,0,0,1,0,0,0,1,131,137,138,
0,0,130,0,0,1,0,0,1,1,0,0,1,2,2,138,138,
0,0,0,212,1,0,0,0,1,1,0,212,213,214,214,220,220,
0,0,0,1,211,0,0,0,211,212,1,2,2,214,214,220,220,
0,0,1,0,0,249,0,0,0,0,0,0,252,252,252,263,263,
0,1,0,0,0,0,248,1,1,252,0,0,0,252,253,263,263,
0,0,0,0,1,0,0,0,1,1,380,381,383,385,385,394,394,
0,0,2,0,1,249,0,0,1,1,380,381,635,637,637,659,659,
0,0,2,213,2,250,0,0,2,2,381,595,852,855,855,887,887,
0,0,132,214,2,252,0,0,3,3,383,598,857,861,861,1031,1031,
0,0,0,0,0,0,1,380,381,384,0,1,1,385,385,394,394,
0,1,0,0,0,0,249,381,382,636,0,1,1,637,638,659,659,
0,1,0,1,212,0,250,382,595,852,1,3,3,855,856,887,887,
1,131,0,1,213,0,252,384,598,857,1,3,3,860,991,1030,1031,
1,140,140,224,224,259,259,391,618,887,391,618,887,1776,1917,2906,4096
]);
array[1..numSubTrees_60 , 1..numSubTrees_59] of var DomainSize: overlap1D_59_60 = array2d(1..numSubTrees_60 , 1..numSubTrees_59, [
overlap1DCounts(limits_60[i, 1], limits_60[i, 2], limits_59[j, 1], limits_59[j, 2]) |
i in 1..numSubTrees_60,
j in 1..numSubTrees_59
]);
constraint objPerTimeStep[60] = normOfDiffCounts(numSubTrees_59, numSubTrees_60, overlapND_59_60, overlap1D_59_60);

array[1..numSubTrees_61 , 1..numSubTrees_60] of DomainSize: overlapND_60_61 = array2d(1..numSubTrees_61 , 1..numSubTrees_60, [
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
0,134,0,1,0,0,0,0,1,1,137,0,0,0,0,140,140,
0,0,134,0,0,0,0,0,1,1,1,0,0,0,136,140,140,
0,0,0,0,0,1,215,1,1,2,2,0,0,216,216,224,224,
0,0,0,0,0,215,1,0,0,216,216,1,1,2,2,224,224,
0,0,0,0,244,0,0,0,0,0,0,1,245,246,246,258,259,
0,1,0,244,0,0,0,0,244,245,246,0,0,0,0,259,259,
0,0,0,0,0,0,1,385,385,386,386,0,0,1,1,391,391,
0,0,1,0,0,215,2,386,386,603,603,1,1,3,4,618,618,
0,1,1,244,0,217,2,387,631,851,852,1,1,4,5,887,887,
0,0,0,0,1,1,0,0,0,1,1,384,385,386,386,391,391,
0,0,0,0,1,2,215,1,1,3,3,385,386,603,603,618,618,
0,0,0,0,245,2,217,1,1,4,4,387,632,852,852,886,887,
0,1,1,244,245,219,220,388,632,855,856,388,633,857,858,1775,1776,
0,135,1,245,245,220,220,388,633,857,994,388,633,857,858,1916,1917,
0,144,144,250,250,225,225,395,648,881,1031,395,648,881,1031,2878,2906,
2,144,144,250,250,225,225,395,648,881,1031,395,648,881,1031,2965,4096
]);
array[1..numSubTrees_61 , 1..numSubTrees_60] of var DomainSize: overlap1D_60_61 = array2d(1..numSubTrees_61 , 1..numSubTrees_60, [
overlap1DCounts(limits_61[i, 1], limits_61[i, 2], limits_60[j, 1], limits_60[j, 2]) |
i in 1..numSubTrees_61,
j in 1..numSubTrees_60
]);
constraint objPerTimeStep[61] = normOfDiffCounts(numSubTrees_60, numSubTrees_61, overlapND_60_61, overlap1D_60_61);

array[1..numSubTrees_62 , 1..numSubTrees_61] of DomainSize: overlapND_61_62 = array2d(1..numSubTrees_62 , 1..numSubTrees_61, [
1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,
0,0,0,135,1,0,0,0,0,0,1,136,0,0,0,0,144,144,144,
0,1,134,0,0,1,0,0,0,0,0,0,0,0,1,135,143,144,144,
0,0,0,1,235,0,0,0,1,1,238,239,0,0,0,0,250,250,250,
0,0,1,0,0,235,0,0,0,0,0,0,1,1,238,239,250,250,250,
0,0,0,0,0,0,1,215,0,215,216,216,1,2,2,2,225,225,225,
0,0,0,0,0,0,215,1,1,2,2,2,0,215,216,216,225,225,225,
0,0,0,0,1,0,1,0,379,380,383,384,0,1,1,1,395,395,395,
0,0,1,1,237,0,1,0,381,382,623,625,0,1,1,2,648,648,648,
0,0,1,1,238,0,2,218,381,600,844,846,1,3,3,4,881,881,881,
0,0,1,136,240,0,2,220,382,603,849,986,1,3,3,4,1031,1031,1031,
0,0,0,0,0,1,0,1,0,1,1,1,379,380,383,384,395,395,395,
0,0,1,0,0,237,0,1,0,1,1,1,381,382,623,625,648,648,648,
0,0,1,0,0,238,218,2,1,3,3,3,381,600,844,846,881,881,881,
0,1,135,0,0,240,220,2,1,3,3,3,382,603,849,985,1030,1031,1031,
0,2,143,143,247,247,234,234,392,628,884,1028,392,628,884,1028,2940,2942,2965,
2,2,143,143,247,247,234,234,392,628,884,1028,392,628,884,1028,3044,3047,4096
]);
array[1..numSubTrees_62 , 1..numSubTrees_61] of var DomainSize: overlap1D_61_62 = array2d(1..numSubTrees_62 , 1..numSubTrees_61, [
overlap1DCounts(limits_62[i, 1], limits_62[i, 2], limits_61[j, 1], limits_61[j, 2]) |
i in 1..numSubTrees_62,
j in 1..numSubTrees_61
]);
constraint objPerTimeStep[62] = normOfDiffCounts(numSubTrees_61, numSubTrees_62, overlapND_61_62, overlap1D_61_62);

array[1..numSubTrees_63 , 1..numSubTrees_62] of DomainSize: overlapND_62_63 = array2d(1..numSubTrees_63 , 1..numSubTrees_62, [
2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,
0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,
0,1,0,130,0,0,1,0,0,0,0,0,0,1,1,131,142,143,143,
0,0,131,0,0,0,0,1,0,1,1,132,0,0,0,0,143,143,143,
0,0,1,0,0,0,0,226,1,227,227,228,0,0,0,0,247,247,247,
0,0,0,1,0,0,227,0,0,0,0,0,0,227,227,228,246,247,247,
0,0,0,0,1,222,0,0,0,0,1,1,0,0,222,222,234,234,234,
0,0,0,0,221,1,0,0,0,0,221,221,1,1,2,2,234,234,234,
0,0,0,0,0,0,0,1,384,385,385,385,0,0,0,0,392,392,392,
0,0,0,0,221,1,0,1,384,386,607,607,1,1,2,2,628,628,628,
0,0,1,0,222,1,0,227,387,615,837,838,1,1,3,3,884,884,884,
0,0,132,0,222,1,0,228,387,616,838,970,1,1,3,3,1028,1028,1028,
0,0,0,0,1,0,0,0,0,0,1,1,384,384,384,384,392,392,392,
0,0,0,0,2,222,0,0,0,0,2,2,384,385,607,607,628,628,628,
0,0,0,1,2,223,227,0,0,0,3,3,386,614,837,838,883,884,884,
0,1,0,131,2,223,228,0,0,0,3,3,386,615,838,969,1026,1028,1028,
0,1,144,143,238,239,239,239,398,638,879,1024,398,638,880,1024,3014,3016,3044,
0,2,144,144,239,239,239,239,398,638,880,1025,398,638,880,1025,3016,3019,3047,
4,4,144,144,239,239,239,239,398,638,880,1025,398,638,880,1025,3108,3113,4096
]);
array[1..numSubTrees_63 , 1..numSubTrees_62] of var DomainSize: overlap1D_62_63 = array2d(1..numSubTrees_63 , 1..numSubTrees_62, [
overlap1DCounts(limits_63[i, 1], limits_63[i, 2], limits_62[j, 1], limits_62[j, 2]) |
i in 1..numSubTrees_63,
j in 1..numSubTrees_62
]);
constraint objPerTimeStep[63] = normOfDiffCounts(numSubTrees_62, numSubTrees_63, overlapND_62_63, overlap1D_62_63);

array[1..numSubTrees_64 , 1..numSubTrees_63] of DomainSize: overlapND_63_64 = array2d(1..numSubTrees_64 , 1..numSubTrees_63, [
3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,
0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,3,4,
0,0,142,0,1,0,0,0,1,1,2,144,0,0,0,0,144,144,144,
0,1,0,142,0,1,0,0,0,0,0,0,0,0,1,143,143,144,144,
0,0,0,0,0,0,0,238,0,238,239,239,0,0,0,0,239,239,239,
0,0,0,0,0,0,238,0,0,0,0,0,0,238,239,239,239,239,239,
0,0,0,1,0,237,0,0,0,0,0,0,0,0,238,239,239,239,239,
0,0,1,0,237,0,0,0,0,0,238,239,0,0,0,0,239,239,239,
0,0,1,0,0,0,0,0,396,396,396,398,0,0,0,0,398,398,398,
0,0,2,0,237,0,0,0,396,397,635,638,0,0,0,0,638,638,638,
0,0,2,0,238,0,0,239,396,636,877,880,0,0,0,0,880,880,880,
0,0,144,0,239,0,0,239,398,638,880,1025,0,0,0,0,1025,1025,1025,
0,0,0,0,0,0,0,0,0,0,0,0,397,397,397,398,398,398,398,
0,0,0,1,0,237,0,0,0,0,0,0,397,397,635,637,638,638,638,
0,0,0,1,0,238,239,0,0,0,0,0,397,636,877,879,880,880,880,
0,1,0,143,0,239,239,0,0,0,0,0,398,637,879,1023,1024,1025,1025,
0,1,144,143,239,239,239,239,398,638,880,1025,398,638,880,1024,3065,3066,3108,
0,3,144,144,239,239,239,239,398,638,880,1025,398,638,880,1025,3066,3070,3113,
4,4,144,144,239,239,239,239,398,638,880,1025,398,638,880,1025,3108,3113,4096
]);
array[1..numSubTrees_64 , 1..numSubTrees_63] of var DomainSize: overlap1D_63_64 = array2d(1..numSubTrees_64 , 1..numSubTrees_63, [
overlap1DCounts(limits_64[i, 1], limits_64[i, 2], limits_63[j, 1], limits_63[j, 2]) |
i in 1..numSubTrees_64,
j in 1..numSubTrees_63
]);
constraint objPerTimeStep[64] = normOfDiffCounts(numSubTrees_63, numSubTrees_64, overlapND_63_64, overlap1D_63_64);

array[1..numSubTrees_65 , 1..numSubTrees_64] of DomainSize: overlapND_64_65 = array2d(1..numSubTrees_65 , 1..numSubTrees_64, [
1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,
0,1,0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,2,4,
0,0,0,130,1,0,1,0,0,1,1,131,0,0,1,1,144,144,144,
0,0,130,0,0,1,0,1,0,0,1,1,0,1,1,131,144,144,144,
0,0,0,1,0,0,222,0,0,222,223,224,0,0,0,0,238,239,239,
0,1,1,0,0,0,0,221,0,0,0,0,0,221,222,223,238,239,239,
0,0,0,0,226,0,0,0,1,1,1,1,1,1,227,227,239,239,239,
0,0,1,0,1,226,0,0,0,0,226,226,0,0,1,2,239,239,239,
0,0,0,1,0,0,0,0,384,384,386,387,0,0,0,0,398,398,398,
0,0,1,1,1,226,0,0,384,385,613,614,0,0,1,2,638,638,638,
0,0,1,2,1,226,222,0,384,607,836,838,0,0,2,3,879,880,880,
0,0,1,132,2,226,223,0,384,608,837,969,0,0,3,4,1024,1025,1025,
0,0,0,0,0,1,0,0,0,0,1,1,384,384,386,386,398,398,398,
0,0,0,0,226,1,0,0,1,1,2,2,385,386,614,614,638,638,638,
0,1,1,0,226,1,0,221,1,1,3,3,385,607,836,837,879,880,880,
0,1,131,0,226,2,0,222,1,1,4,4,385,608,837,968,1024,1025,1025,
0,1,143,143,247,246,234,233,392,628,883,1027,392,627,883,1027,3014,3016,3108,
0,2,143,143,247,247,234,234,392,628,884,1028,392,628,884,1028,3016,3019,3113,
2,2,143,143,247,247,234,234,392,628,884,1028,392,628,884,1028,3044,3047,4096
]);
array[1..numSubTrees_65 , 1..numSubTrees_64] of var DomainSize: overlap1D_64_65 = array2d(1..numSubTrees_65 , 1..numSubTrees_64, [
overlap1DCounts(limits_65[i, 1], limits_65[i, 2], limits_64[j, 1], limits_64[j, 2]) |
i in 1..numSubTrees_65,
j in 1..numSubTrees_64
]);
constraint objPerTimeStep[65] = normOfDiffCounts(numSubTrees_64, numSubTrees_65, overlapND_64_65, overlap1D_64_65);

array[1..numSubTrees_66 , 1..numSubTrees_65] of DomainSize: overlapND_65_66 = array2d(1..numSubTrees_66 , 1..numSubTrees_65, [
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,
1,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,2,
0,0,134,1,0,0,1,0,1,1,1,0,0,1,135,143,143,
0,134,0,0,1,1,0,0,0,1,135,0,1,1,1,143,143,
0,1,0,0,235,0,0,1,1,1,2,0,236,237,238,247,247,
0,0,1,235,0,0,0,0,236,237,238,1,1,1,2,247,247,
0,1,0,0,0,215,0,0,0,218,221,0,0,0,0,233,234,
1,0,1,0,0,0,215,0,0,0,0,0,0,218,221,233,234,
0,0,0,0,1,0,0,379,380,380,381,1,2,2,2,392,392,
0,1,0,0,1,215,0,380,382,600,604,1,2,2,2,627,628,
0,1,1,237,1,216,0,382,622,843,848,2,3,3,4,883,884,
0,135,1,237,2,217,0,383,623,845,984,2,4,4,5,1027,1028,
0,0,0,1,0,0,0,1,2,2,2,379,380,380,381,392,392,
1,0,1,1,0,0,215,1,2,2,2,380,381,599,603,627,628,
1,1,1,1,237,0,216,2,3,3,4,382,621,842,847,883,884,
1,1,135,2,237,0,217,2,4,4,5,383,622,844,983,1027,1028,
1,144,144,250,250,224,224,395,648,880,1030,395,648,880,1030,2938,3044,
2,144,144,250,250,224,225,395,648,880,1030,395,648,881,1031,2939,3047,
3,144,144,250,250,224,225,395,648,880,1030,395,648,881,1031,2961,4096
]);
array[1..numSubTrees_66 , 1..numSubTrees_65] of var DomainSize: overlap1D_65_66 = array2d(1..numSubTrees_66 , 1..numSubTrees_65, [
overlap1DCounts(limits_66[i, 1], limits_66[i, 2], limits_65[j, 1], limits_65[j, 2]) |
i in 1..numSubTrees_66,
j in 1..numSubTrees_65
]);
constraint objPerTimeStep[66] = normOfDiffCounts(numSubTrees_65, numSubTrees_66, overlapND_65_66, overlap1D_65_66);

array[1..numSubTrees_67 , 1..numSubTrees_66] of DomainSize: overlapND_66_67 = array2d(1..numSubTrees_67 , 1..numSubTrees_66, [
0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,1,3,
0,133,0,1,0,1,0,0,1,1,0,0,1,2,135,144,144,
0,0,134,0,1,0,0,0,0,0,0,1,1,1,1,144,144,
0,0,0,0,0,0,244,0,0,244,1,1,1,245,245,250,250,
0,1,0,0,0,244,0,0,0,0,0,0,244,244,245,250,250,
0,1,0,215,0,0,0,0,215,217,0,0,0,217,219,224,224,
1,0,1,0,215,0,0,0,0,0,0,215,217,218,218,224,225,
0,0,0,0,0,0,0,385,386,387,1,1,1,388,388,395,395,
0,0,0,0,0,0,244,385,387,632,2,2,2,634,634,648,648,
0,1,0,216,0,0,245,386,604,852,2,2,3,855,857,880,880,
0,136,0,217,0,1,245,386,605,853,2,2,4,857,994,1030,1030,
0,0,0,0,0,0,1,1,1,2,384,385,386,388,388,395,395,
0,1,0,0,0,244,1,1,1,2,384,385,630,632,633,648,648,
1,1,1,0,216,245,1,1,1,3,385,602,850,854,855,880,881,
1,1,137,0,217,245,1,1,1,3,385,603,851,855,856,1030,1031,
1,140,140,223,223,258,259,391,617,886,391,617,885,1773,1914,2873,2961,
2,140,140,223,224,259,259,391,617,886,391,618,887,1775,1916,2903,4096
]);
array[1..numSubTrees_67 , 1..numSubTrees_66] of var DomainSize: overlap1D_66_67 = array2d(1..numSubTrees_67 , 1..numSubTrees_66, [
overlap1DCounts(limits_67[i, 1], limits_67[i, 2], limits_66[j, 1], limits_66[j, 2]) |
i in 1..numSubTrees_67,
j in 1..numSubTrees_66
]);
constraint objPerTimeStep[67] = normOfDiffCounts(numSubTrees_66, numSubTrees_67, overlapND_66_67, overlap1D_66_67);

array[1..numSubTrees_68 , 1..numSubTrees_67] of DomainSize: overlapND_67_68 = array2d(1..numSubTrees_68 , 1..numSubTrees_67, [
1,0,0,1,0,0,0,0,1,1,0,0,0,1,2,
130,0,1,0,1,0,0,1,1,1,0,0,1,131,140,
0,130,0,1,0,1,0,0,1,131,0,1,1,1,140,
1,0,211,0,0,0,0,0,0,0,0,0,212,214,223,
0,1,0,211,0,0,0,1,213,215,0,0,0,0,224,
1,0,0,0,248,0,0,248,249,250,1,1,1,2,259,
0,1,0,0,0,249,0,0,0,1,0,249,250,251,259,
0,0,0,0,1,0,0,1,1,1,380,380,381,383,391,
1,0,211,0,1,0,0,1,1,1,381,381,594,598,617,
1,1,212,0,1,252,0,1,1,2,383,635,851,856,886,
0,0,0,0,0,0,381,381,382,384,0,0,0,0,391,
0,1,0,211,0,0,382,383,596,600,1,1,1,1,618,
1,1,0,212,251,0,384,636,852,857,2,2,2,3,887,
2,2,212,212,252,252,385,638,854,860,385,637,853,859,1775,
132,2,213,212,253,252,385,639,855,861,385,637,854,990,1916,
138,138,219,219,263,263,394,659,886,1030,394,659,886,1030,2903,
139,138,219,220,263,263,394,659,887,1031,394,659,886,1031,4096
]);
array[1..numSubTrees_68 , 1..numSubTrees_67] of var DomainSize: overlap1D_67_68 = array2d(1..numSubTrees_68 , 1..numSubTrees_67, [
overlap1DCounts(limits_68[i, 1], limits_68[i, 2], limits_67[j, 1], limits_67[j, 2]) |
i in 1..numSubTrees_68,
j in 1..numSubTrees_67
]);
constraint objPerTimeStep[68] = normOfDiffCounts(numSubTrees_67, numSubTrees_68, overlapND_67_68, overlap1D_67_68);

array[1..numSubTrees_69 , 1..numSubTrees_68] of DomainSize: overlapND_68_69 = array2d(1..numSubTrees_69 , 1..numSubTrees_68, [
127,0,0,0,1,0,0,0,0,0,1,1,1,1,139,
0,127,0,1,0,0,0,0,0,0,0,1,1,128,138,
1,0,0,0,206,0,0,0,0,0,207,209,209,209,219,
0,0,0,0,0,207,0,209,211,0,0,0,211,211,220,
0,0,255,0,0,0,0,0,256,1,1,1,257,257,263,
0,1,0,255,0,0,0,0,0,0,0,256,256,257,263,
0,0,0,0,0,0,384,385,387,0,0,0,388,388,394,
0,0,256,0,0,1,384,386,645,1,1,1,647,647,659,
0,0,256,0,0,210,386,599,860,1,1,1,862,863,887,
0,127,258,1,0,210,386,599,862,1,1,2,865,993,1031,
0,0,1,0,0,0,0,0,1,383,384,386,388,388,394,
0,2,1,256,0,0,0,0,1,383,384,643,645,647,659,
1,2,1,256,208,0,0,0,1,385,595,856,859,861,886,
128,2,1,258,209,0,0,0,1,385,596,859,862,864,1031,
136,135,272,272,216,217,392,612,890,392,611,889,1789,1925,4096
]);
array[1..numSubTrees_69 , 1..numSubTrees_68] of var DomainSize: overlap1D_68_69 = array2d(1..numSubTrees_69 , 1..numSubTrees_68, [
overlap1DCounts(limits_69[i, 1], limits_69[i, 2], limits_68[j, 1], limits_68[j, 2]) |
i in 1..numSubTrees_69,
j in 1..numSubTrees_68
]);
constraint objPerTimeStep[69] = normOfDiffCounts(numSubTrees_68, numSubTrees_69, overlapND_68_69, overlap1D_68_69);

array[1..numSubTrees_70 , 1..numSubTrees_69] of DomainSize: overlapND_69_70 = array2d(1..numSubTrees_70 , 1..numSubTrees_69, [
121,0,1,0,1,0,0,0,1,123,0,1,1,1,136,
0,122,0,0,0,1,0,0,0,0,0,0,1,124,135,
1,0,262,0,0,0,0,0,0,1,0,262,264,264,272,
0,0,0,262,0,0,0,262,264,264,1,1,1,1,272,
1,0,0,0,205,0,0,0,205,206,0,0,0,0,216,
0,1,0,0,0,205,0,1,1,1,0,0,205,206,217,
0,0,0,1,0,0,1,2,2,2,381,381,382,382,392,
0,1,0,1,0,206,1,3,3,3,381,381,588,589,612,
1,1,262,1,0,206,1,3,3,5,382,644,853,856,890,
0,0,0,0,0,0,382,382,383,383,1,1,1,1,392,
1,0,0,0,205,0,382,382,588,589,2,2,2,2,611,
1,0,0,262,205,0,383,645,853,856,3,3,3,4,889,
2,1,264,264,206,207,385,650,860,865,385,649,860,864,1789,
2,123,264,265,206,208,385,651,861,866,385,649,861,988,1925,
130,129,276,276,214,215,394,674,893,1029,394,674,894,1029,4096
]);
array[1..numSubTrees_70 , 1..numSubTrees_69] of var DomainSize: overlap1D_69_70 = array2d(1..numSubTrees_70 , 1..numSubTrees_69, [
overlap1DCounts(limits_70[i, 1], limits_70[i, 2], limits_69[j, 1], limits_69[j, 2]) |
i in 1..numSubTrees_70,
j in 1..numSubTrees_69
]);
constraint objPerTimeStep[70] = normOfDiffCounts(numSubTrees_69, numSubTrees_70, overlapND_69_70, overlap1D_69_70);

array[1..numSubTrees_71 , 1..numSubTrees_70] of DomainSize: overlapND_70_71 = array2d(1..numSubTrees_71 , 1..numSubTrees_70, [
119,0,1,0,0,0,0,0,0,0,1,1,1,1,130,
0,117,0,1,0,1,0,1,1,0,0,1,2,119,129,
1,0,267,0,0,0,1,1,1,0,267,267,268,268,276,
0,1,0,268,0,0,0,268,268,0,0,0,268,269,276,
0,0,0,0,205,0,0,0,205,0,0,0,205,205,214,
0,1,0,0,0,205,0,0,0,0,0,205,205,206,215,
0,0,1,0,0,0,386,386,386,0,1,1,387,387,394,
0,1,1,269,0,0,388,658,658,0,1,1,659,660,674,
0,1,1,271,205,0,388,660,865,0,1,1,866,867,893,
119,1,2,273,206,0,388,662,869,0,3,3,872,873,1029,
0,0,0,0,0,0,0,0,0,387,387,387,387,387,394,
1,0,268,0,0,0,1,1,1,389,657,657,658,658,674,
1,1,270,0,0,205,1,1,1,389,659,864,865,866,894,
1,118,272,1,0,207,1,2,2,389,661,869,872,990,1029,
125,124,289,289,214,215,397,687,904,397,687,905,1813,1938,4096
]);
array[1..numSubTrees_71 , 1..numSubTrees_70] of var DomainSize: overlap1D_70_71 = array2d(1..numSubTrees_71 , 1..numSubTrees_70, [
overlap1DCounts(limits_71[i, 1], limits_71[i, 2], limits_70[j, 1], limits_70[j, 2]) |
i in 1..numSubTrees_71,
j in 1..numSubTrees_70
]);
constraint objPerTimeStep[71] = normOfDiffCounts(numSubTrees_70, numSubTrees_71, overlapND_70_71, overlap1D_70_71);

array[1..numSubTrees_72 , 1..numSubTrees_71] of DomainSize: overlapND_71_72 = array2d(1..numSubTrees_72 , 1..numSubTrees_71, [
110,0,1,0,1,0,0,0,0,110,0,1,2,2,125,
0,110,0,1,0,1,0,1,2,2,0,0,0,110,124,
1,0,278,0,0,0,1,1,1,2,0,279,279,279,289,
0,1,0,278,0,0,0,279,279,279,1,1,1,2,289,
1,0,0,0,202,0,0,0,0,1,0,1,203,204,214,
0,1,0,0,0,202,0,2,204,205,0,0,0,1,215,
0,0,1,0,0,0,384,385,385,385,1,2,2,2,397,
0,1,1,278,0,1,384,664,665,665,2,3,3,4,687,
1,1,1,278,202,1,385,665,667,668,2,4,206,208,904,
0,0,0,1,0,0,1,2,2,2,384,385,385,385,397,
1,0,278,1,0,0,2,3,3,4,384,664,664,664,687,
1,1,278,1,0,202,2,5,207,209,385,665,666,667,905,
2,2,281,280,202,203,387,671,875,878,387,671,874,877,1813,
2,112,281,282,202,204,387,673,878,881,387,671,874,987,1938,
117,116,295,295,211,212,394,695,909,1027,394,695,908,1025,4096
]);
array[1..numSubTrees_72 , 1..numSubTrees_71] of var DomainSize: overlap1D_71_72 = array2d(1..numSubTrees_72 , 1..numSubTrees_71, [
overlap1DCounts(limits_72[i, 1], limits_72[i, 2], limits_71[j, 1], limits_71[j, 2]) |
i in 1..numSubTrees_72,
j in 1..numSubTrees_71
]);
constraint objPerTimeStep[72] = normOfDiffCounts(numSubTrees_71, numSubTrees_72, overlapND_71_72, overlap1D_71_72);

array[1..numSubTrees_73 , 1..numSubTrees_72] of DomainSize: overlapND_72_73 = array2d(1..numSubTrees_73 , 1..numSubTrees_72, [
102,0,1,0,1,0,0,0,1,1,0,1,1,104,117,
0,103,0,0,0,1,0,1,1,105,0,0,0,0,116,
1,0,286,0,0,0,0,0,286,286,0,0,0,1,295,
0,0,0,286,0,0,1,1,1,1,0,0,286,286,295,
1,0,0,0,199,0,0,0,0,0,0,200,200,202,211,
0,1,0,0,0,199,0,201,201,203,0,0,0,0,212,
0,0,0,0,0,0,1,1,1,1,385,385,385,387,394,
0,0,0,287,0,1,2,3,3,3,387,387,674,677,695,
0,1,0,287,0,200,2,204,204,206,389,389,676,679,909,
102,1,1,287,1,200,2,204,205,207,389,390,677,783,1027,
0,0,0,1,0,0,384,384,384,386,1,1,2,2,394,
1,0,287,1,0,0,386,386,673,676,1,1,2,3,695,
2,0,287,1,199,0,388,388,675,678,1,201,202,205,908,
2,103,287,1,199,1,388,389,676,783,1,201,202,205,1025,
111,110,305,305,208,209,398,610,917,1034,398,609,916,1034,4096
]);
array[1..numSubTrees_73 , 1..numSubTrees_72] of var DomainSize: overlap1D_72_73 = array2d(1..numSubTrees_73 , 1..numSubTrees_72, [
overlap1DCounts(limits_73[i, 1], limits_73[i, 2], limits_72[j, 1], limits_72[j, 2]) |
i in 1..numSubTrees_73,
j in 1..numSubTrees_72
]);
constraint objPerTimeStep[73] = normOfDiffCounts(numSubTrees_72, numSubTrees_73, overlapND_72_73, overlap1D_72_73);

array[1..numSubTrees_74 , 1..numSubTrees_73] of DomainSize: overlapND_73_74 = array2d(1..numSubTrees_74 , 1..numSubTrees_73, [
95,0,1,0,1,0,0,0,1,0,1,1,2,2,111,
0,95,0,1,0,1,0,1,1,0,0,1,2,97,110,
1,0,293,0,0,0,1,1,1,0,294,295,297,297,305,
0,1,0,294,0,0,0,295,296,0,0,0,297,298,305,
1,0,0,0,199,0,0,0,199,0,0,0,200,201,208,
0,1,0,0,0,199,0,1,1,0,0,199,202,203,209,
0,0,0,0,0,0,0,0,0,388,388,388,388,388,398,
0,1,0,0,0,200,0,1,1,389,389,589,592,593,610,
1,1,294,0,0,200,1,2,2,389,684,885,891,892,917,
1,96,295,1,0,201,1,3,3,390,686,888,896,992,1034,
0,0,1,0,0,0,387,387,387,0,1,1,388,388,398,
1,1,1,0,199,0,388,388,587,0,1,1,589,591,609,
1,2,1,295,199,0,388,684,884,0,2,2,888,891,916,
96,2,2,296,200,0,389,686,887,0,3,3,893,896,1034,
102,101,312,312,208,209,397,713,922,397,713,923,1857,1959,4096
]);
array[1..numSubTrees_74 , 1..numSubTrees_73] of var DomainSize: overlap1D_73_74 = array2d(1..numSubTrees_74 , 1..numSubTrees_73, [
overlap1DCounts(limits_74[i, 1], limits_74[i, 2], limits_73[j, 1], limits_73[j, 2]) |
i in 1..numSubTrees_74,
j in 1..numSubTrees_73
]);
constraint objPerTimeStep[74] = normOfDiffCounts(numSubTrees_73, numSubTrees_74, overlapND_73_74, overlap1D_73_74);

array[1..numSubTrees_75 , 1..numSubTrees_74] of DomainSize: overlapND_74_75 = array2d(1..numSubTrees_75 , 1..numSubTrees_74, [
87,0,0,0,1,0,0,0,87,88,0,0,0,0,102,
0,86,0,1,0,0,0,0,0,0,0,1,87,87,101,
1,0,0,0,304,0,0,0,1,305,0,0,0,0,312,
0,0,0,0,0,305,0,0,0,0,0,0,0,305,312,
0,0,197,0,0,0,0,198,198,198,0,1,1,1,208,
0,1,0,197,0,0,0,0,0,0,0,198,199,199,209,
0,0,0,0,0,0,387,388,389,389,0,0,0,0,397,
0,0,0,0,0,306,388,390,391,391,0,0,0,306,713,
0,0,197,0,0,306,388,588,589,589,0,1,1,307,922,
0,0,0,0,0,0,0,0,0,0,387,388,389,389,397,
1,0,0,0,305,0,0,1,2,307,388,389,390,390,713,
1,1,0,197,305,0,0,1,2,307,388,587,589,589,923,
1,1,198,198,306,308,388,590,592,899,388,590,592,901,1857,
1,87,198,199,307,308,388,590,592,900,388,591,679,988,1959,
93,92,205,206,325,325,396,605,700,1029,396,606,700,1029,4096
]);
array[1..numSubTrees_75 , 1..numSubTrees_74] of var DomainSize: overlap1D_74_75 = array2d(1..numSubTrees_75 , 1..numSubTrees_74, [
overlap1DCounts(limits_75[i, 1], limits_75[i, 2], limits_74[j, 1], limits_74[j, 2]) |
i in 1..numSubTrees_75,
j in 1..numSubTrees_74
]);
constraint objPerTimeStep[75] = normOfDiffCounts(numSubTrees_74, numSubTrees_75, overlapND_74_75, overlap1D_74_75);

array[1..numSubTrees_76 , 1..numSubTrees_75] of DomainSize: overlapND_75_76 = array2d(1..numSubTrees_76 , 1..numSubTrees_75, [
82,0,0,0,1,0,0,1,1,83,0,0,0,0,93,
0,81,0,1,0,0,0,0,1,1,0,0,0,81,92,
1,0,0,0,194,0,0,194,198,199,0,0,0,0,205,
0,0,0,0,0,195,0,1,1,1,0,195,199,199,206,
0,0,313,0,0,0,1,1,1,1,0,0,316,317,325,
0,1,0,313,0,0,0,0,316,317,0,0,0,1,325,
0,0,1,0,0,0,384,384,388,388,0,0,1,1,396,
1,0,1,0,194,1,384,578,587,588,0,1,2,2,605,
83,0,1,0,196,1,384,580,589,672,0,1,2,2,700,
83,0,315,0,196,1,385,582,592,675,0,1,319,320,1029,
0,0,0,0,0,0,0,0,0,0,385,385,389,389,396,
0,0,1,0,0,195,0,1,1,1,385,580,590,590,606,
0,81,1,1,0,196,0,1,2,2,385,581,591,672,700,
0,82,1,315,0,196,0,1,319,320,385,582,593,675,1029,
88,87,328,328,203,204,391,596,951,1040,391,597,952,1040,4096
]);
array[1..numSubTrees_76 , 1..numSubTrees_75] of var DomainSize: overlap1D_75_76 = array2d(1..numSubTrees_76 , 1..numSubTrees_75, [
overlap1DCounts(limits_76[i, 1], limits_76[i, 2], limits_75[j, 1], limits_75[j, 2]) |
i in 1..numSubTrees_76,
j in 1..numSubTrees_75
]);
constraint objPerTimeStep[76] = normOfDiffCounts(numSubTrees_75, numSubTrees_76, overlapND_75_76, overlap1D_75_76);

array[1..numSubTrees_77 , 1..numSubTrees_76] of DomainSize: overlapND_76_77 = array2d(1..numSubTrees_77 , 1..numSubTrees_76, [
0,65,0,0,0,1,0,1,2,67,67,0,0,0,0,67,88,
0,0,65,0,2,0,0,0,1,1,1,0,0,65,67,68,87,
0,2,0,319,0,0,0,0,0,2,321,0,0,0,0,321,328,
0,0,0,0,319,0,1,0,0,0,0,1,2,2,321,321,328,
0,0,0,1,0,196,0,0,196,196,197,0,0,0,0,197,203,
1,0,1,0,0,0,196,0,0,0,0,0,196,197,197,197,204,
0,0,0,1,0,0,0,379,379,379,380,1,1,1,1,381,391,
0,0,0,2,0,196,1,380,576,576,578,1,2,2,2,580,596,
0,0,0,2,327,198,2,384,583,584,586,2,5,5,333,920,951,
0,65,0,2,327,199,2,385,585,651,653,2,5,5,333,987,1040,
0,0,1,0,0,0,0,1,1,1,1,379,379,380,380,381,391,
1,0,2,0,0,0,196,1,1,1,2,380,576,578,578,580,597,
1,2,2,327,0,0,198,1,1,3,332,384,583,586,586,920,952,
1,2,67,327,2,0,198,1,2,4,333,384,583,651,653,988,1040,
2,69,69,347,347,208,209,396,606,677,1027,396,607,678,1028,2059,4096
]);
array[1..numSubTrees_77 , 1..numSubTrees_76] of var DomainSize: overlap1D_76_77 = array2d(1..numSubTrees_77 , 1..numSubTrees_76, [
overlap1DCounts(limits_77[i, 1], limits_77[i, 2], limits_76[j, 1], limits_76[j, 2]) |
i in 1..numSubTrees_77,
j in 1..numSubTrees_76
]);
constraint objPerTimeStep[77] = normOfDiffCounts(numSubTrees_76, numSubTrees_77, overlapND_76_77, overlap1D_76_77);

array[1..numSubTrees_78 , 1..numSubTrees_77] of DomainSize: overlapND_77_78 = array2d(1..numSubTrees_78 , 1..numSubTrees_77, [
0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,2,
1,0,56,0,0,0,0,0,1,1,57,57,0,0,0,0,58,58,69,
0,1,0,55,0,0,1,0,0,1,1,1,1,1,56,56,58,59,69,
0,0,0,0,1,0,0,333,0,0,0,1,0,335,335,335,340,340,347,
0,0,0,1,0,0,333,0,0,335,335,335,0,0,1,1,340,340,347,
0,0,0,0,198,0,0,1,0,0,0,199,0,1,1,1,202,202,208,
0,1,0,0,0,199,0,0,0,0,0,0,0,0,0,200,201,203,209,
0,0,1,0,0,0,0,0,384,385,386,387,1,1,1,1,390,390,396,
0,0,1,0,198,0,0,1,384,386,387,587,1,3,3,3,594,594,606,
1,0,57,0,199,0,0,1,385,387,444,645,1,3,3,3,654,654,677,
1,0,57,0,200,0,0,335,385,387,444,646,1,339,340,340,997,997,1027,
0,0,0,1,0,0,0,0,1,1,1,1,384,385,386,387,390,390,396,
0,1,0,1,0,199,0,1,1,1,1,1,384,386,387,588,593,595,607,
0,2,0,56,0,200,1,1,1,2,2,2,385,387,443,645,653,656,678,
0,2,0,57,0,200,335,1,1,338,339,339,385,387,444,646,996,999,1028,
1,2,57,57,200,200,336,336,386,726,784,986,386,726,785,987,1997,2000,2059,
5,5,61,61,209,209,349,349,392,748,813,1025,392,748,813,1025,2100,2106,4096
]);
array[1..numSubTrees_78 , 1..numSubTrees_77] of var DomainSize: overlap1D_77_78 = array2d(1..numSubTrees_78 , 1..numSubTrees_77, [
overlap1DCounts(limits_78[i, 1], limits_78[i, 2], limits_77[j, 1], limits_77[j, 2]) |
i in 1..numSubTrees_78,
j in 1..numSubTrees_77
]);
constraint objPerTimeStep[78] = normOfDiffCounts(numSubTrees_77, numSubTrees_78, overlapND_77_78, overlap1D_77_78);

array[1..numSubTrees_79 , 1..numSubTrees_78] of DomainSize: overlapND_78_79 = array2d(1..numSubTrees_79 , 1..numSubTrees_78, [
5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,
0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,5,
0,0,47,0,0,0,1,0,1,1,1,48,0,1,1,1,56,56,61,
0,0,0,48,0,0,0,1,0,1,1,1,0,0,0,48,56,56,61,
0,0,0,0,199,0,0,0,0,0,203,203,0,0,0,0,207,207,209,
0,0,0,0,0,198,0,1,0,1,1,1,0,0,202,202,206,207,209,
0,0,0,1,0,1,0,338,0,338,338,338,0,0,1,2,344,344,349,
0,0,1,0,0,0,339,0,0,0,0,1,0,339,339,339,344,344,349,
0,0,1,0,0,0,0,0,383,383,383,384,0,0,0,0,390,390,392,
0,0,1,1,0,1,0,340,385,726,726,728,0,0,1,2,741,741,748,
0,0,49,1,0,1,1,341,387,729,730,780,0,1,2,3,801,801,813,
0,0,49,1,199,1,1,341,387,729,933,983,0,2,3,4,1010,1010,1025,
0,0,0,0,0,0,0,0,0,0,0,0,384,384,384,384,390,390,392,
0,0,1,0,0,0,341,0,0,0,0,2,386,727,727,728,741,741,748,
0,0,1,49,0,0,342,1,0,1,1,3,387,729,730,780,801,801,813,
0,0,1,49,0,198,342,2,0,2,2,4,387,729,932,982,1009,1010,1025,
0,0,51,51,202,202,347,346,391,738,945,998,391,739,946,999,2061,2062,2100,
0,5,51,51,202,202,347,347,391,739,946,999,391,739,946,999,2062,2068,2106,
10,10,53,53,208,208,361,361,398,760,975,1030,398,760,975,1030,2184,2195,4096
]);
array[1..numSubTrees_79 , 1..numSubTrees_78] of var DomainSize: overlap1D_78_79 = array2d(1..numSubTrees_79 , 1..numSubTrees_78, [
overlap1DCounts(limits_79[i, 1], limits_79[i, 2], limits_78[j, 1], limits_78[j, 2]) |
i in 1..numSubTrees_79,
j in 1..numSubTrees_78
]);
constraint objPerTimeStep[79] = normOfDiffCounts(numSubTrees_78, numSubTrees_79, overlapND_78_79, overlap1D_78_79);

array[1..numSubTrees_80 , 1..numSubTrees_79] of DomainSize: overlapND_79_80 = array2d(1..numSubTrees_80 , 1..numSubTrees_79, [
9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,
0,9,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,10,10,
0,0,41,0,0,0,0,0,1,1,42,42,0,0,0,0,52,52,53,
0,0,0,40,0,1,0,0,0,1,1,1,1,1,41,41,52,52,53,
0,0,0,0,1,0,199,0,0,0,0,199,0,1,1,1,207,208,208,
0,1,0,0,0,0,0,199,0,0,0,0,0,0,0,199,207,208,208,
0,0,0,0,348,0,1,0,0,0,0,1,0,348,348,348,357,357,361,
0,0,0,1,0,348,0,0,0,348,348,348,0,0,1,1,357,357,361,
0,0,1,0,0,0,0,0,386,386,387,387,1,1,1,1,398,398,398,
0,0,1,1,0,348,0,0,386,735,736,736,1,1,2,2,756,756,760,
0,0,1,1,1,348,201,0,386,735,736,938,1,2,3,3,970,971,975,
0,0,42,1,1,348,201,0,387,736,778,980,1,3,4,4,1024,1025,1030,
0,0,0,1,0,0,0,0,1,1,1,1,386,386,387,387,398,398,398,
0,0,0,1,348,0,1,0,1,1,1,2,386,734,735,735,756,756,760,
0,1,0,1,348,0,1,201,1,1,1,2,386,734,735,937,970,971,975,
0,1,0,41,348,1,1,201,1,2,2,3,387,735,776,978,1024,1025,1030,
0,1,45,45,358,358,205,204,394,753,799,1006,394,753,799,1005,2142,2144,2184,
0,10,45,45,358,358,205,205,394,753,799,1006,394,753,799,1006,2144,2155,2195,
15,15,45,45,363,363,210,210,397,761,807,1019,397,761,807,1019,2256,2272,4096
]);
array[1..numSubTrees_80 , 1..numSubTrees_79] of var DomainSize: overlap1D_79_80 = array2d(1..numSubTrees_80 , 1..numSubTrees_79, [
overlap1DCounts(limits_80[i, 1], limits_80[i, 2], limits_79[j, 1], limits_79[j, 2]) |
i in 1..numSubTrees_80,
j in 1..numSubTrees_79
]);
constraint objPerTimeStep[80] = normOfDiffCounts(numSubTrees_79, numSubTrees_80, overlapND_79_80, overlap1D_79_80);

array[1..numSubTrees_81 , 1..numSubTrees_80] of DomainSize: overlapND_80_81 = array2d(1..numSubTrees_81 , 1..numSubTrees_80, [
15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,15,
0,14,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1,15,
0,0,34,0,1,0,0,1,0,0,1,2,0,0,2,2,45,45,45,
0,0,0,35,0,1,0,0,0,1,0,0,0,1,1,36,45,45,45,
0,0,1,0,347,0,1,348,0,0,0,348,0,0,349,349,362,362,363,
0,0,0,1,0,347,0,0,1,348,0,0,0,348,348,350,362,362,363,
0,0,0,0,1,0,200,201,0,1,0,201,0,1,202,202,210,210,210,
0,1,0,0,0,1,0,0,199,200,0,1,0,200,201,201,209,209,210,
0,0,1,0,0,0,0,0,0,0,387,388,0,0,388,388,397,397,397,
0,0,1,1,0,347,0,1,1,348,387,389,0,348,737,739,760,760,761,
0,0,35,1,1,347,0,2,1,348,389,392,0,348,740,742,806,806,807,
0,0,35,1,2,347,200,203,1,349,389,594,0,349,943,945,1018,1018,1019,
0,0,0,0,0,0,0,0,0,0,0,0,388,389,389,389,397,397,397,
0,0,1,0,347,0,1,348,0,0,0,349,388,389,739,739,760,760,761,
0,0,1,35,347,1,1,348,0,1,0,349,389,391,741,776,806,806,807,
0,1,1,35,347,2,1,348,199,201,0,350,389,592,943,978,1017,1017,1019,
1,2,39,39,360,361,207,568,206,568,397,970,397,970,1942,1982,2224,2226,2256,
1,16,39,39,361,361,207,569,207,569,397,971,397,971,1944,1984,2226,2228,2272,
23,23,40,40,363,363,211,575,211,575,397,977,397,977,1956,1997,2342,2366,4096
]);
array[1..numSubTrees_81 , 1..numSubTrees_80] of var DomainSize: overlap1D_80_81 = array2d(1..numSubTrees_81 , 1..numSubTrees_80, [
overlap1DCounts(limits_81[i, 1], limits_81[i, 2], limits_80[j, 1], limits_80[j, 2]) |
i in 1..numSubTrees_81,
j in 1..numSubTrees_80
]);
constraint objPerTimeStep[81] = normOfDiffCounts(numSubTrees_80, numSubTrees_81, overlapND_80_81, overlap1D_80_81);

array[1..numSubTrees_82 , 1..numSubTrees_81] of DomainSize: overlapND_81_82 = array2d(1..numSubTrees_82 , 1..numSubTrees_81, [
23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,
0,22,0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,23,23,
0,0,33,0,0,0,0,0,1,1,1,0,0,0,1,34,40,40,40,
0,0,0,33,1,0,0,0,0,1,1,0,0,0,1,1,40,40,40,
0,0,0,0,0,346,1,0,0,0,1,0,346,346,349,349,363,363,363,
0,0,0,1,346,0,0,0,0,346,346,0,0,0,348,348,363,363,363,
0,0,0,0,0,1,203,0,0,0,203,0,1,1,207,207,211,211,211,
0,0,0,0,0,347,204,0,0,1,205,0,347,347,557,557,575,575,575,
0,1,0,0,0,0,0,203,0,0,0,0,0,203,205,206,210,211,211,
0,1,0,1,346,1,0,203,0,346,346,0,1,204,554,555,574,575,575,
0,0,1,0,0,0,0,0,387,387,387,0,1,1,389,390,397,397,397,
0,0,1,0,1,348,204,0,388,390,594,0,349,349,950,951,977,977,977,
0,0,0,0,0,0,0,0,0,0,0,388,388,388,390,390,397,397,397,
0,1,0,1,347,1,1,203,0,347,348,389,390,593,948,949,976,977,977,
0,1,1,1,348,349,205,204,388,737,942,389,739,943,1899,1901,1955,1956,1956,
0,1,1,34,349,349,205,204,388,738,943,389,739,943,1900,1902,1996,1997,1997,
0,1,36,36,357,357,215,214,397,755,971,397,755,970,1963,2000,2314,2316,2342,
23,1,36,36,357,357,215,214,397,755,971,397,755,970,1963,2000,2315,2317,2366,
29,29,36,36,357,357,217,217,397,755,973,397,755,973,1968,2005,2440,2470,4096
]);
array[1..numSubTrees_82 , 1..numSubTrees_81] of var DomainSize: overlap1D_81_82 = array2d(1..numSubTrees_82 , 1..numSubTrees_81, [
overlap1DCounts(limits_82[i, 1], limits_82[i, 2], limits_81[j, 1], limits_81[j, 2]) |
i in 1..numSubTrees_82,
j in 1..numSubTrees_81
]);
constraint objPerTimeStep[82] = normOfDiffCounts(numSubTrees_81, numSubTrees_82, overlapND_81_82, overlap1D_81_82);

array[1..numSubTrees_83 , 1..numSubTrees_82] of DomainSize: overlapND_82_83 = array2d(1..numSubTrees_83 , 1..numSubTrees_82, [
27,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,29,
0,28,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,28,29,
0,0,0,25,0,0,1,0,0,25,26,26,0,0,0,1,36,36,36,
0,0,25,0,0,0,0,0,0,0,0,0,1,26,27,27,36,36,36,
0,0,0,0,0,1,0,349,0,0,0,349,0,0,1,1,357,357,357,
0,0,0,1,0,0,349,0,0,1,1,1,0,0,0,349,357,357,357,
0,0,0,0,210,0,0,0,0,0,210,210,0,0,0,0,217,217,217,
0,0,0,0,0,209,0,1,0,0,0,1,0,0,209,209,217,217,217,
0,0,0,0,0,0,0,0,384,385,386,386,1,1,1,1,397,397,397,
0,0,0,0,0,1,0,349,384,386,387,736,1,1,2,2,755,755,755,
0,0,0,0,210,1,0,349,385,387,598,947,1,1,2,2,973,973,973,
0,0,1,0,0,0,0,0,1,1,1,1,383,385,386,386,397,397,397,
0,0,1,1,0,0,349,0,1,2,2,3,383,385,386,735,755,755,755,
0,0,1,1,0,209,349,1,1,2,2,4,384,386,596,945,973,973,973,
0,0,1,1,211,211,349,350,387,390,603,954,386,389,602,952,1967,1968,1968,
0,0,1,26,211,211,350,350,387,415,629,980,386,389,602,953,2004,2005,2005,
0,0,29,29,218,218,368,368,394,425,647,1018,394,425,647,1018,2373,2374,2440,
0,28,29,29,218,218,368,368,394,425,647,1018,394,425,647,1018,2374,2403,2470,
32,32,29,29,218,218,368,368,394,425,647,1018,394,425,647,1018,2428,2461,4096
]);
array[1..numSubTrees_83 , 1..numSubTrees_82] of var DomainSize: overlap1D_82_83 = array2d(1..numSubTrees_83 , 1..numSubTrees_82, [
overlap1DCounts(limits_83[i, 1], limits_83[i, 2], limits_82[j, 1], limits_82[j, 2]) |
i in 1..numSubTrees_83,
j in 1..numSubTrees_82
]);
constraint objPerTimeStep[83] = normOfDiffCounts(numSubTrees_82, numSubTrees_83, overlapND_82_83, overlap1D_82_83);

array[1..numSubTrees_84 , 1..numSubTrees_83] of DomainSize: overlapND_83_84 = array2d(1..numSubTrees_84 , 1..numSubTrees_83, [
29,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,29,32,
0,28,0,0,0,0,0,0,1,1,0,1,0,0,1,1,1,1,32,
0,0,23,0,1,0,0,0,0,1,0,1,0,0,1,1,29,29,29,
0,0,0,23,0,0,0,0,0,0,1,1,0,0,1,24,29,29,29,
0,0,0,0,0,1,207,208,0,1,0,1,0,208,209,209,218,218,218,
0,1,0,0,0,0,0,0,207,207,0,208,0,0,208,208,217,217,218,
0,0,0,0,0,350,1,351,0,0,0,0,0,353,353,353,368,368,368,
0,0,1,0,350,0,0,0,0,350,0,352,0,0,352,352,368,368,368,
0,0,0,1,0,0,0,0,0,0,385,387,0,0,387,388,394,394,394,
0,0,0,24,0,0,0,1,0,0,386,388,0,1,389,413,425,425,425,
0,0,0,24,0,1,208,210,0,1,386,389,0,210,599,623,647,647,647,
0,0,1,24,350,1,208,210,0,351,388,743,0,210,954,978,1018,1018,1018,
0,0,0,0,0,0,0,0,0,0,0,0,386,388,388,388,394,394,394,
0,0,23,0,1,0,0,0,0,1,0,1,386,389,390,390,425,425,425,
0,1,23,0,1,0,0,0,208,209,0,210,386,389,599,599,646,646,647,
0,1,23,0,1,350,1,351,208,209,0,210,388,744,954,955,1017,1017,1018,
0,1,28,28,360,359,219,579,218,579,395,983,395,983,1968,1997,2316,2317,2428,
0,29,28,28,360,360,219,580,219,580,395,984,395,984,1970,1999,2318,2319,2461,
29,29,28,28,360,360,219,580,219,580,395,984,395,984,1970,1999,2352,2382,4096
]);
array[1..numSubTrees_84 , 1..numSubTrees_83] of var DomainSize: overlap1D_83_84 = array2d(1..numSubTrees_84 , 1..numSubTrees_83, [
overlap1DCounts(limits_84[i, 1], limits_84[i, 2], limits_83[j, 1], limits_83[j, 2]) |
i in 1..numSubTrees_84,
j in 1..numSubTrees_83
]);
constraint objPerTimeStep[84] = normOfDiffCounts(numSubTrees_83, numSubTrees_84, overlapND_83_84, overlap1D_83_84);

array[1..numSubTrees_85 , 1..numSubTrees_84] of DomainSize: overlapND_84_85 = array2d(1..numSubTrees_85 , 1..numSubTrees_84, [
1,0,26,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,29,
0,0,0,26,0,1,0,0,0,0,1,1,0,0,0,0,1,27,29,
22,0,0,0,0,0,0,1,0,1,1,1,0,0,0,22,27,27,28,
0,21,0,0,0,0,1,0,1,1,1,22,0,1,1,1,27,27,28,
1,0,0,0,0,1,0,343,0,345,347,347,0,0,0,1,358,358,360,
0,1,0,0,1,0,343,0,0,0,0,1,0,345,347,347,358,358,360,
0,0,0,0,211,0,1,0,0,0,0,0,0,1,213,213,219,219,219,
0,1,0,0,212,0,344,0,0,1,1,2,0,346,560,560,578,578,580,
0,0,0,1,0,210,0,1,0,1,212,212,0,0,0,0,218,219,219,
1,0,0,1,0,211,0,344,0,346,559,559,0,0,0,2,577,578,580,
0,1,0,0,0,0,0,0,384,385,385,387,0,0,0,0,395,395,395,
1,1,0,1,0,213,0,344,386,733,948,951,0,0,0,2,980,981,984,
0,0,0,0,0,0,0,0,0,0,0,0,385,386,386,387,395,395,395,
0,1,0,0,214,0,344,0,0,2,2,3,387,734,950,951,981,981,984,
1,2,0,1,214,213,344,344,386,735,950,954,387,735,951,954,1963,1964,1970,
1,23,0,1,214,213,345,344,387,736,951,976,387,736,952,955,1990,1992,1999,
25,25,0,1,225,224,354,354,395,755,983,1011,395,755,984,1012,2232,2234,2352,
26,25,26,1,225,224,354,354,395,755,983,1011,395,755,984,1013,2234,2236,2382,
26,25,26,27,225,225,354,354,395,755,984,1012,395,755,984,1013,2263,2291,4096
]);
array[1..numSubTrees_85 , 1..numSubTrees_84] of var DomainSize: overlap1D_84_85 = array2d(1..numSubTrees_85 , 1..numSubTrees_84, [
overlap1DCounts(limits_85[i, 1], limits_85[i, 2], limits_84[j, 1], limits_84[j, 2]) |
i in 1..numSubTrees_85,
j in 1..numSubTrees_84
]);
constraint objPerTimeStep[85] = normOfDiffCounts(numSubTrees_84, numSubTrees_85, overlapND_84_85, overlap1D_84_85);

array[1..numSubTrees_86 , 1..numSubTrees_85] of DomainSize: overlapND_85_86 = array2d(1..numSubTrees_86 , 1..numSubTrees_85, [
20,0,0,0,0,0,1,0,0,0,0,0,0,0,1,21,25,25,26,
0,19,0,0,0,0,0,0,1,1,1,20,0,0,0,0,24,24,25,
0,0,25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,25,26,
0,0,0,25,1,0,0,0,0,0,0,0,0,1,1,1,1,1,27,
0,0,0,0,0,216,0,1,0,217,218,219,0,0,0,0,225,225,225,
0,0,0,1,216,0,0,0,0,0,0,0,0,217,217,218,224,224,225,
0,0,0,0,0,1,0,341,0,1,342,343,0,0,0,0,349,349,354,
1,0,0,0,0,0,341,0,0,0,0,0,0,0,341,343,349,349,354,
0,1,0,0,0,0,0,0,384,384,385,387,0,0,0,0,395,395,395,
1,1,0,0,0,0,342,0,385,386,387,389,0,0,342,344,750,750,755,
1,1,0,1,216,0,343,0,386,387,388,390,0,217,560,563,978,978,984,
1,20,0,1,216,0,345,0,387,388,389,410,0,217,562,565,1004,1004,1012,
0,0,0,0,0,0,0,0,0,0,0,0,385,385,386,387,395,395,395,
0,0,0,0,0,1,0,342,0,1,343,344,386,386,387,388,750,750,755,
0,0,0,0,0,217,0,344,0,218,562,564,387,387,388,389,979,979,984,
20,0,0,0,0,217,1,346,0,218,564,566,387,387,389,410,1006,1006,1013,
23,22,0,1,227,228,354,354,396,627,982,1008,396,626,981,1008,2155,2156,2263,
23,22,0,26,228,228,354,354,396,627,982,1008,396,627,982,1009,2157,2158,2291,
23,22,25,26,229,229,354,354,396,628,983,1010,396,628,983,1011,2193,2219,4096
]);
array[1..numSubTrees_86 , 1..numSubTrees_85] of var DomainSize: overlap1D_85_86 = array2d(1..numSubTrees_86 , 1..numSubTrees_85, [
overlap1DCounts(limits_86[i, 1], limits_86[i, 2], limits_85[j, 1], limits_85[j, 2]) |
i in 1..numSubTrees_86,
j in 1..numSubTrees_85
]);
constraint objPerTimeStep[86] = normOfDiffCounts(numSubTrees_85, numSubTrees_86, overlapND_85_86, overlap1D_85_86);

array[1..numSubTrees_87 , 1..numSubTrees_86] of DomainSize: overlapND_86_87 = array2d(1..numSubTrees_87 , 1..numSubTrees_86, [
0,16,0,0,0,0,0,1,0,1,1,1,1,1,1,17,20,20,23,
16,0,0,0,0,0,1,0,1,1,1,17,0,1,1,1,20,20,22,
0,0,25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,25,
0,0,0,25,1,0,0,0,0,0,1,1,0,0,0,0,1,26,26,
0,0,0,1,217,0,0,1,0,1,218,221,0,0,0,0,227,228,229,
0,0,0,0,0,218,1,0,0,0,0,0,0,1,219,222,228,228,229,
0,1,0,0,1,0,0,337,0,337,340,340,0,0,0,1,346,346,354,
1,0,0,0,0,1,337,0,0,0,0,1,0,337,340,340,346,346,354,
1,0,0,0,0,0,0,0,384,384,384,387,1,1,1,1,395,395,396,
1,0,0,0,0,219,1,0,384,385,385,388,1,2,221,224,625,625,628,
2,0,0,0,0,220,338,0,384,385,385,389,1,339,561,564,972,972,983,
19,0,0,0,0,221,340,0,385,386,386,407,1,341,564,567,995,995,1010,
0,1,0,0,0,0,0,0,1,1,1,1,384,384,384,387,395,395,396,
0,1,0,1,218,0,0,1,1,2,220,223,384,384,384,387,624,625,628,
0,2,0,1,219,0,0,338,1,339,560,563,384,384,384,388,971,972,983,
0,19,0,1,220,0,0,340,1,341,563,566,385,385,385,406,994,995,1011,
22,22,0,1,227,228,348,348,393,745,975,1003,392,744,975,1003,2080,2082,2193,
22,22,25,1,227,228,348,348,393,745,975,1003,393,745,976,1004,2081,2083,2219,
23,22,25,26,231,231,348,348,396,748,982,1011,396,748,982,1010,2119,2146,4096
]);
array[1..numSubTrees_87 , 1..numSubTrees_86] of var DomainSize: overlap1D_86_87 = array2d(1..numSubTrees_87 , 1..numSubTrees_86, [
overlap1DCounts(limits_87[i, 1], limits_87[i, 2], limits_86[j, 1], limits_86[j, 2]) |
i in 1..numSubTrees_87,
j in 1..numSubTrees_86
]);
constraint objPerTimeStep[87] = normOfDiffCounts(numSubTrees_86, numSubTrees_87, overlapND_86_87, overlap1D_86_87);

array[1..numSubTrees_88 , 1..numSubTrees_87] of DomainSize: overlapND_87_88 = array2d(1..numSubTrees_88 , 1..numSubTrees_87, [
16,0,0,0,1,0,0,1,0,0,1,1,0,1,2,18,20,20,23,
0,16,0,0,0,1,0,0,0,1,0,1,0,0,1,1,19,19,22,
0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,25,
0,0,0,23,0,0,0,0,1,1,0,1,0,0,1,1,1,1,26,
0,0,0,1,0,1,0,0,222,224,0,225,0,0,225,225,228,228,231,
0,0,0,0,1,0,223,225,0,1,0,1,0,225,226,226,229,229,231,
1,0,0,0,330,0,1,331,0,0,0,0,0,331,333,334,337,337,348,
0,1,0,0,0,330,0,0,1,331,0,331,0,0,333,333,337,337,348,
1,0,0,0,0,0,0,0,0,0,388,388,0,0,389,390,392,392,396,
1,1,0,0,0,331,0,1,1,332,390,722,0,1,726,727,733,733,748,
1,1,0,1,0,332,0,1,224,557,390,948,0,2,953,954,963,963,982,
17,1,0,1,1,334,0,2,224,559,391,951,0,3,957,974,985,985,1011,
0,0,0,0,0,0,0,0,0,0,0,0,389,389,390,390,392,392,396,
1,0,0,0,331,0,1,332,0,0,0,0,391,723,726,727,733,733,748,
1,0,0,0,332,0,225,558,0,1,0,1,391,949,954,955,964,964,982,
1,16,0,0,334,1,225,560,0,2,0,2,391,951,957,958,985,985,1010,
20,19,0,1,338,338,232,572,231,571,393,965,393,966,1939,1959,2017,2018,2119,
20,19,0,24,338,338,232,572,232,572,393,966,393,966,1940,1961,2019,2020,2146,
21,20,23,24,342,342,238,583,238,583,398,982,398,982,1972,1994,2067,2091,4096
]);
array[1..numSubTrees_88 , 1..numSubTrees_87] of var DomainSize: overlap1D_87_88 = array2d(1..numSubTrees_88 , 1..numSubTrees_87, [
overlap1DCounts(limits_88[i, 1], limits_88[i, 2], limits_87[j, 1], limits_87[j, 2]) |
i in 1..numSubTrees_88,
j in 1..numSubTrees_87
]);
constraint objPerTimeStep[88] = normOfDiffCounts(numSubTrees_87, numSubTrees_88, overlapND_87_88, overlap1D_87_88);

array[1..numSubTrees_89 , 1..numSubTrees_88] of DomainSize: overlapND_88_89 = array2d(1..numSubTrees_89 , 1..numSubTrees_88, [
15,0,0,0,0,0,0,1,1,1,16,17,0,0,0,0,17,17,21,
0,15,0,0,0,0,1,0,0,0,0,0,0,0,15,16,16,16,20,
0,0,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,
0,0,0,23,1,0,0,0,0,0,0,0,0,1,1,1,1,24,24,
1,0,0,0,0,1,0,326,0,1,2,329,0,0,0,0,329,329,342,
0,1,0,0,1,0,326,0,0,0,0,0,0,1,2,329,329,329,342,
0,0,0,0,0,230,0,1,0,230,231,233,0,0,0,0,233,233,238,
1,0,0,0,0,232,0,327,0,233,235,564,0,0,0,0,564,564,583,
0,0,0,1,229,0,1,0,0,0,0,0,0,229,230,232,232,233,238,
0,1,0,1,231,0,327,0,0,0,0,0,0,232,234,563,563,564,583,
1,0,0,0,0,0,0,0,387,388,389,389,0,0,0,0,389,389,398,
1,1,0,1,231,0,327,0,387,388,389,389,0,232,234,563,953,954,982,
0,0,0,0,0,0,0,0,0,0,0,0,388,389,389,389,389,389,398,
1,0,0,0,0,232,0,327,0,233,235,564,388,389,389,389,954,954,982,
2,1,0,1,231,232,327,327,387,621,624,953,388,621,624,953,1908,1910,1972,
17,1,0,1,231,232,327,328,388,622,641,971,388,621,624,953,1926,1928,1994,
17,16,0,1,234,235,333,333,390,627,647,984,390,626,645,981,1967,1969,2067,
17,16,23,1,234,235,333,333,390,627,647,984,390,626,645,982,1968,1970,2091,
19,18,24,25,244,244,337,337,395,643,666,1007,395,643,665,1006,2019,2045,4096
]);
array[1..numSubTrees_89 , 1..numSubTrees_88] of var DomainSize: overlap1D_88_89 = array2d(1..numSubTrees_89 , 1..numSubTrees_88, [
overlap1DCounts(limits_89[i, 1], limits_89[i, 2], limits_88[j, 1], limits_88[j, 2]) |
i in 1..numSubTrees_89,
j in 1..numSubTrees_88
]);
constraint objPerTimeStep[89] = normOfDiffCounts(numSubTrees_88, numSubTrees_89, overlapND_88_89, overlap1D_88_89);

array[1..numSubTrees_90 , 1..numSubTrees_89] of DomainSize: overlapND_89_90 = array2d(1..numSubTrees_90 , 1..numSubTrees_89, [
12,0,0,0,0,0,1,0,1,13,13,13,14,0,0,0,0,1,19,
0,12,0,0,0,0,0,1,0,0,0,0,1,1,13,13,13,14,18,
0,0,23,0,0,0,0,0,0,0,0,23,24,0,0,0,0,0,24,
0,0,0,24,0,0,0,0,0,0,0,0,0,0,0,0,24,25,25,
0,0,0,0,0,236,0,1,0,0,0,0,1,0,0,236,236,237,244,
0,0,0,0,237,0,0,0,0,0,237,237,238,0,0,0,0,0,244,
0,1,0,0,0,1,0,319,0,0,0,0,319,0,1,2,2,2,337,
1,0,0,0,0,0,320,0,0,1,1,1,1,0,0,0,0,320,337,
1,0,0,0,0,0,0,0,384,385,385,385,385,1,1,1,1,1,395,
1,0,0,0,238,0,0,0,385,387,625,626,627,1,1,1,1,1,643,
13,0,0,0,240,0,1,0,387,401,641,642,644,1,1,1,1,2,666,
14,0,0,0,240,0,321,0,387,402,642,643,645,1,1,1,1,322,1007,
0,1,0,0,0,0,0,0,1,1,1,1,1,384,385,385,385,385,395,
0,1,0,0,0,237,1,1,1,1,1,1,2,385,386,623,624,626,643,
0,13,0,0,0,239,1,2,1,1,1,1,3,387,400,639,640,643,665,
0,14,0,0,0,240,1,321,1,1,1,1,322,387,402,642,643,646,1006,
14,14,0,0,240,240,323,323,389,404,644,645,970,389,404,644,645,970,2019,
14,14,0,24,240,240,323,323,389,404,644,645,970,389,404,644,669,995,2045,
16,15,24,25,254,254,330,330,397,415,670,695,1030,397,414,669,695,1030,4096
]);
array[1..numSubTrees_90 , 1..numSubTrees_89] of var DomainSize: overlap1D_89_90 = array2d(1..numSubTrees_90 , 1..numSubTrees_89, [
overlap1DCounts(limits_90[i, 1], limits_90[i, 2], limits_89[j, 1], limits_89[j, 2]) |
i in 1..numSubTrees_90,
j in 1..numSubTrees_89
]);
constraint objPerTimeStep[90] = normOfDiffCounts(numSubTrees_89, numSubTrees_90, overlapND_89_90, overlap1D_89_90);

array[1..numSubTrees_91 , 1..numSubTrees_90] of DomainSize: overlapND_90_91 = array2d(1..numSubTrees_91 , 1..numSubTrees_90, [
1,10,0,0,0,0,1,0,0,0,0,1,1,1,0,1,1,2,3,3,16,
0,0,11,0,0,1,0,0,0,0,0,0,1,1,0,0,0,0,1,12,15,
0,0,0,24,0,0,0,0,24,0,0,0,0,24,0,0,0,0,24,24,24,
0,0,0,0,24,0,0,0,0,1,25,0,0,0,0,0,25,25,25,25,25,
0,0,0,0,0,0,1,246,246,0,0,0,0,246,0,1,1,1,247,247,254,
0,0,0,0,1,1,0,0,0,245,246,0,1,1,0,0,246,246,247,247,254,
1,1,0,0,0,0,312,1,1,0,0,0,0,1,0,312,312,313,314,314,330,
0,0,1,0,0,312,0,0,0,1,1,0,312,313,0,0,1,1,314,315,330,
0,1,0,0,0,0,0,0,0,0,0,387,387,387,0,0,0,0,387,387,397,
1,12,0,0,0,0,1,0,1,0,0,388,388,389,0,1,1,2,391,391,415,
1,13,0,0,0,0,2,246,247,0,0,388,388,635,0,2,2,3,638,638,670,
1,13,0,24,0,0,2,246,271,0,0,388,388,659,0,3,3,4,663,663,695,
1,13,1,24,0,312,2,248,273,1,2,388,700,974,0,3,5,6,980,981,1030,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,388,388,388,388,388,388,397,
0,0,12,0,0,1,0,0,0,0,0,0,1,2,388,388,388,388,390,402,414,
0,0,13,0,1,2,0,0,0,245,246,0,2,3,388,388,634,634,637,650,669,
0,0,13,0,25,2,0,0,0,246,271,0,2,3,388,388,659,660,663,676,695,
1,1,13,0,25,2,312,1,1,248,273,0,3,5,388,700,973,975,980,993,1030,
2,16,16,25,26,322,322,261,287,261,288,397,720,1010,397,720,1009,1012,2028,2045,4096
]);
array[1..numSubTrees_91 , 1..numSubTrees_90] of var DomainSize: overlap1D_90_91 = array2d(1..numSubTrees_91 , 1..numSubTrees_90, [
overlap1DCounts(limits_91[i, 1], limits_91[i, 2], limits_90[j, 1], limits_90[j, 2]) |
i in 1..numSubTrees_91,
j in 1..numSubTrees_90
]);
constraint objPerTimeStep[91] = normOfDiffCounts(numSubTrees_90, numSubTrees_91, overlapND_90_91, overlap1D_90_91);

array[1..numSubTrees_92 , 1..numSubTrees_91] of DomainSize: overlapND_91_92 = array2d(1..numSubTrees_92 , 1..numSubTrees_91, [
1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,2,2,2,
0,0,10,0,0,0,0,0,0,0,0,0,1,11,11,11,11,0,0,0,1,1,16,
0,0,0,11,0,0,0,0,0,0,0,1,0,0,1,1,1,0,11,11,11,11,16,
0,0,0,0,22,0,0,0,1,25,0,0,0,0,0,0,25,0,0,0,0,0,25,
0,0,0,0,0,23,1,25,0,1,0,0,0,0,0,0,1,0,0,0,0,25,26,
0,1,0,1,0,0,0,0,0,0,0,304,0,0,304,305,306,0,1,1,1,1,322,
1,0,0,0,0,0,0,0,1,1,304,0,0,0,0,0,1,0,0,304,305,306,322,
0,0,0,0,1,0,0,0,249,251,1,0,0,0,0,0,251,0,0,1,1,1,261,
0,0,0,0,23,1,0,1,250,276,1,0,0,0,0,0,276,0,0,1,1,2,287,
0,0,0,0,0,1,249,251,0,0,0,1,0,0,1,1,1,0,0,0,0,251,261,
0,0,0,0,0,24,250,276,1,2,0,1,0,0,1,1,3,0,0,0,0,276,288,
0,0,1,0,0,0,0,0,0,0,0,0,385,387,388,388,388,0,0,0,0,0,397,
0,1,1,1,0,0,1,1,0,0,0,304,385,387,692,693,694,0,1,1,1,2,720,
0,1,1,1,23,1,1,2,250,276,1,304,385,387,692,696,973,0,1,2,2,4,1010,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,386,387,388,388,388,397,
1,0,0,0,0,0,0,0,1,1,304,0,0,0,0,0,1,386,388,693,694,695,720,
1,0,0,0,0,24,250,276,2,3,304,1,0,0,1,1,4,386,388,693,695,972,1009,
2,0,0,0,0,24,250,276,2,3,305,1,0,0,1,1,4,386,388,694,697,974,1012,
2,1,2,1,23,25,251,278,252,279,308,306,386,389,696,700,980,387,390,699,702,981,2028,
2,1,2,12,23,25,251,278,252,279,308,308,386,389,698,702,982,387,401,710,713,992,2045,
2,1,15,15,24,26,266,295,266,294,314,314,396,413,731,735,1031,396,413,731,736,1033,4096
]);
array[1..numSubTrees_92 , 1..numSubTrees_91] of var DomainSize: overlap1D_91_92 = array2d(1..numSubTrees_92 , 1..numSubTrees_91, [
overlap1DCounts(limits_92[i, 1], limits_92[i, 2], limits_91[j, 1], limits_91[j, 2]) |
i in 1..numSubTrees_92,
j in 1..numSubTrees_91
]);
constraint objPerTimeStep[92] = normOfDiffCounts(numSubTrees_91, numSubTrees_92, overlapND_91_92, overlap1D_91_92);

array[1..numSubTrees_93 , 1..numSubTrees_92] of DomainSize: overlapND_92_93 = array2d(1..numSubTrees_93 , 1..numSubTrees_92, [
0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,2,
0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,
0,1,10,0,0,0,0,1,0,0,0,0,1,1,11,11,12,0,0,0,0,0,15,
0,0,0,10,0,0,0,0,1,1,0,0,0,0,0,0,0,1,1,11,11,12,15,
0,0,0,0,21,0,0,0,0,0,1,0,0,1,1,22,22,0,0,0,0,0,24,
0,0,0,0,0,22,0,1,0,0,0,0,0,0,0,0,1,0,0,0,22,22,26,
0,0,0,0,0,0,0,0,0,0,0,259,0,0,0,0,0,0,260,260,261,261,266,
0,0,0,0,0,23,0,1,0,0,0,260,0,0,0,0,1,0,261,261,285,285,295,
0,0,0,0,1,0,0,0,0,0,258,0,0,259,259,261,261,0,1,1,1,1,266,
0,0,0,0,22,0,0,0,0,0,259,0,0,260,260,283,283,0,1,1,1,1,294,
0,1,1,0,0,0,294,296,0,1,0,0,0,0,1,1,297,0,0,0,0,1,314,
1,0,0,1,0,0,0,0,294,296,0,0,0,1,1,1,1,0,0,1,1,297,314,
0,0,1,0,0,0,0,0,0,0,0,0,384,385,388,388,388,1,1,1,1,1,396,
0,1,11,0,0,0,0,1,0,0,1,0,385,387,400,400,401,1,1,1,1,1,413,
1,1,11,1,0,0,0,1,295,297,1,0,386,389,402,402,403,1,1,2,2,299,731,
2,1,11,1,0,0,0,1,296,299,1,0,387,390,403,403,404,1,1,2,2,301,735,
2,1,11,1,22,0,0,1,296,299,260,0,388,651,664,687,688,1,2,3,3,302,1031,
0,0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,384,385,388,388,388,396,
0,0,0,11,0,0,0,0,1,1,0,0,1,1,1,1,1,385,386,399,399,400,413,
0,1,1,11,0,0,295,297,1,2,0,0,1,1,2,2,299,386,387,400,400,402,731,
0,1,1,11,0,0,297,299,1,2,0,0,1,1,2,2,301,387,388,401,401,403,736,
0,1,1,11,0,23,297,300,1,2,0,260,1,1,2,2,302,388,650,663,687,689,1033,
5,4,13,13,23,24,305,311,305,312,275,275,395,673,690,716,1028,395,673,690,717,1030,4096
]);
array[1..numSubTrees_93 , 1..numSubTrees_92] of var DomainSize: overlap1D_92_93 = array2d(1..numSubTrees_93 , 1..numSubTrees_92, [
overlap1DCounts(limits_93[i, 1], limits_93[i, 2], limits_92[j, 1], limits_92[j, 2]) |
i in 1..numSubTrees_93,
j in 1..numSubTrees_92
]);
constraint objPerTimeStep[93] = normOfDiffCounts(numSubTrees_92, numSubTrees_93, overlapND_92_93, overlap1D_92_93);

array[1..numSubTrees_94 , 1..numSubTrees_93] of DomainSize: overlapND_93_94 = array2d(1..numSubTrees_94 , 1..numSubTrees_93, [
3,0,0,0,0,0,1,0,1,4,5,0,0,0,0,5,0,0,5,5,5,5,5,
0,2,0,0,0,0,0,1,0,0,0,1,3,4,0,0,0,4,4,4,4,4,4,
0,0,0,0,9,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,10,10,13,
0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,10,10,10,13,
0,0,17,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1,1,1,1,1,23,
0,0,0,17,0,0,1,0,0,1,2,0,0,0,0,2,0,0,2,2,2,19,24,
0,1,0,0,1,0,0,0,0,0,0,291,292,292,0,0,0,292,293,293,294,294,305,
0,3,0,1,1,0,0,1,0,0,0,292,295,296,0,0,0,296,297,297,298,299,311,
1,0,0,0,0,0,0,0,292,293,293,0,0,0,0,293,0,0,294,294,294,294,305,
4,0,0,0,0,0,1,0,293,297,298,0,0,0,0,298,0,0,299,299,299,299,312,
0,1,1,0,0,0,0,267,0,0,0,0,1,268,0,1,0,268,269,269,270,270,275,
1,0,0,1,0,0,267,0,0,1,268,0,1,1,0,268,0,1,269,269,270,271,275,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,385,385,0,0,385,385,385,385,395,
0,1,1,0,0,0,0,267,0,0,0,0,1,268,386,387,0,268,655,655,656,656,673,
0,1,1,0,10,0,0,268,0,0,0,1,2,270,386,387,0,270,658,658,669,669,690,
0,1,18,0,10,0,0,269,0,0,0,1,2,271,387,388,0,271,660,660,671,671,716,
0,4,18,1,11,0,0,271,0,0,0,293,297,568,387,388,0,568,958,958,970,971,1028,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,385,385,385,385,385,385,395,
1,0,0,1,0,0,267,0,0,1,268,1,2,2,0,268,386,388,656,656,657,658,673,
1,0,0,1,0,11,268,0,0,1,269,1,2,2,0,269,386,388,657,669,670,671,690,
1,0,0,18,0,11,269,0,0,2,271,1,2,2,0,271,387,389,660,672,673,691,717,
5,0,0,18,0,11,271,0,293,299,570,1,2,2,0,570,387,389,960,972,973,991,1030,
9,8,18,19,12,12,282,282,303,313,597,303,312,596,396,994,396,993,1995,2008,2023,2043,4096
]);
array[1..numSubTrees_94 , 1..numSubTrees_93] of var DomainSize: overlap1D_93_94 = array2d(1..numSubTrees_94 , 1..numSubTrees_93, [
overlap1DCounts(limits_94[i, 1], limits_94[i, 2], limits_93[j, 1], limits_93[j, 2]) |
i in 1..numSubTrees_94,
j in 1..numSubTrees_93
]);
constraint objPerTimeStep[94] = normOfDiffCounts(numSubTrees_93, numSubTrees_94, overlapND_93_94, overlap1D_93_94);

array[1..numSubTrees_95 , 1..numSubTrees_94] of DomainSize: overlapND_94_95 = array2d(1..numSubTrees_95 , 1..numSubTrees_94, [
1,7,0,0,0,0,0,0,0,0,0,1,0,1,1,0,0,0,1,1,1,8,9,
7,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,8,
0,0,14,0,0,0,0,14,0,1,0,0,0,0,1,0,0,14,15,15,15,15,18,
0,0,0,14,0,0,0,0,1,15,0,0,0,1,16,0,0,0,16,16,16,16,19,
0,0,0,0,8,0,0,0,0,0,1,0,1,1,1,0,1,1,2,2,12,12,12,
0,0,0,0,0,9,0,0,0,0,0,1,0,1,1,0,0,0,1,10,12,12,12,
0,1,0,1,0,0,0,0,267,268,0,0,0,0,268,0,0,0,268,268,269,270,282,
1,0,0,0,0,0,267,267,0,0,1,0,0,0,0,0,1,268,268,268,269,269,282,
0,0,0,0,0,1,0,0,1,1,0,288,0,288,290,0,0,0,290,291,291,291,303,
1,7,0,0,0,1,0,1,1,1,0,289,0,289,291,0,0,1,292,293,293,300,313,
1,9,0,1,0,1,0,1,269,270,0,289,0,289,560,0,0,1,561,562,563,572,597,
0,0,0,0,1,0,1,1,0,0,288,0,0,0,0,0,288,290,290,290,291,291,303,
7,0,0,0,1,0,2,2,0,0,288,0,0,0,0,0,289,292,292,292,293,293,312,
9,0,0,0,1,0,270,270,0,0,289,0,0,0,0,0,290,561,561,561,563,563,596,
0,0,0,0,1,0,0,0,0,0,0,0,392,393,394,0,0,0,394,394,395,395,396,
1,9,0,1,1,1,0,1,269,270,0,289,392,682,954,0,0,1,956,957,959,968,994,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,393,394,395,395,395,395,395,396,
9,0,0,0,1,0,270,270,0,0,289,0,1,1,1,393,684,956,957,957,959,959,993,
10,9,0,1,2,1,270,271,269,270,290,290,393,684,956,394,686,959,1917,1919,1923,1932,1995,
10,9,0,1,2,10,270,271,269,270,290,291,393,685,957,394,686,959,1918,1929,1935,1944,2008,
10,9,0,1,10,10,270,271,269,270,291,291,394,686,958,394,687,960,1920,1931,1948,1957,2023,
10,9,0,15,10,10,270,271,270,285,291,291,394,687,974,394,687,960,1936,1947,1964,1974,2043,
13,12,14,15,13,13,282,297,282,298,291,291,403,699,999,403,699,998,2001,2015,2037,2050,4096
]);
array[1..numSubTrees_95 , 1..numSubTrees_94] of var DomainSize: overlap1D_94_95 = array2d(1..numSubTrees_95 , 1..numSubTrees_94, [
overlap1DCounts(limits_95[i, 1], limits_95[i, 2], limits_94[j, 1], limits_94[j, 2]) |
i in 1..numSubTrees_95,
j in 1..numSubTrees_94
]);
constraint objPerTimeStep[95] = normOfDiffCounts(numSubTrees_94, numSubTrees_95, overlapND_94_95, overlap1D_94_95);

var float: obj = sum(i in 1..numTimeSteps-1)(objPerTimeStep[i]);

solve minimize obj;
