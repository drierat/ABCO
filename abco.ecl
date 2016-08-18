:-lib(ic).

% abco\0 The Ant Borg Colony Optimisation algorithm
abco:-
 parameters(Max_sol,M,Alpha,Beta,Ro),							%parameters initalisation
 problem_instance(Clients,Demands,MaxCap,Distances,PheromonesIn),			%problem instance values
 (for(I,1,Max_sol),fromto(PheromonesIn,Pheromones,PheromonesOut,_),param(M,Alpha,Beta,Ro,Clients,Demands,MaxCap,Distances) do	%Loop Max_sol times
       construct_solutions(M,Alpha,Beta,Ants_routes,Clients,Demands,MaxCap,Distances,Pheromones),	%solutions construction phase
       local_search(Ants_routes,Ants_routes_LS),				%local search phase TO DO
       update_trails(Ro,Ants_routes_LS,Distances,Pheromones, PheromonesOut),						%given the new soltions, update the pheromones
  writeln(I) %remove
 ),
 show_solution.

%parameters(-Max_sol,-M,-Alpha,-Beta,-Ro) ABCO parameters initialisation
% Should the parameters be read from a file or be in the query itself (?)
parameters(Max_sol,M,Alpha,Beta,Ro):-
 Max_sol is 10,			%Temination condition - maximum number of solutions
 M is 5, 					%Number of ants building the solutions
 Alpha is 1, 				%Weight of the pheromone trail
 Beta is 5,					%Weight of the heuristic information
 Ro is 0.5.	 				%Pheromone trail persistence

%problem_instance(-Clients,-Demands,-MaxCap,-Distances,-Pheromones) Gets the intance to be solved
problem_instance(Clients,Demands,MaxCap,Distances,Pheromones):-
 Clients = [1,2,3,4,5],															%List of clients to serve
 Demands = [10,10,10,10],														%List of clients demands
 MaxCap = 100,																	%Maximum capacity for a truck
 Distances = [[0,3,5.099019514,2.236067977,4.472135955,4],
              [3,0,2.236067977,2.828427125,4.123105626,5],
              [5.099019514,2.236067977,0,4.123105626,4.242640687,5.830951895],
              [2.236067977,2.828427125,4.123105626,0,2.236067977,2.236067977],
              [4.472135955,4.123105626,4.242640687,2.236067977,0,2],
              [4,5,5.830951895,2.236067977,2,0]
             ],
 Pheromones = [[0,0.2,0.2,0.2,0.2,0.2],[0.2,0,0.2,0.2,0.2,0.2],[0.2,0.2,0,0.2,0.2,0.2],
 			   [0.2,0.2,0.2,0,0.2,0.2],[0.2,0.2,0.2,0.2,0,0.2],[0.2,0.2,0.2,0.2,0.2,0]].
%Pheromones = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]].

%construct_solutions(+M,+Alpha,+Beta,-Ant_routes,+Clients) Construction of the solutions for a set of ants
construct_solutions(M,Alpha,Beta,Ants_routes,Clients,Demands,MaxCap,Distances,Pheromones):-
 length(Ants_routes,M), 										%A list of M routes, each corresponding to an ant
  (foreach(Ant_k,Ants_routes),param(Alpha,Beta,Clients,Demands,MaxCap,Distances,Pheromones) do 		%For each ant, her route is built
  (fromto([0],Partial_sol_0,Partial_sol_1,Ant_k_0),				
   fromto(Clients,Clients_0,Clients_1,[]),fromto(0,Pos_0,Next,_),param(Alpha,Beta,Demands,MaxCap,Distances,Pheromones) do
    choose_next_movement(Pos_0,Clients_0,Partial_sol_0,Demands,MaxCap,Distances,Pheromones,Next,Alpha,Beta),	%The ant chooses her next step
    move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Next)	%The ant moves (update the values in the loop)
  ),
  append(Ant_k_0,[0],Ant_k),
  writeln(Ant_k)
 ).

local_search(Ants_routes,Ants_routes).	%TO DO (change the second to Ant_routes_LS)

show_solution.							%TO DO

%choose_next_movement(+Pos_0,+Clients_0,_Partial_sol_0,_Demands,_MaxCap,+Distances,+Pheromones,-Next,+Alpha,+Beta)
choose_next_movement(Pos_0,Clients_0,_Partial_sol_0,_Demands,_MaxCap,Distances,Pheromones,Next,Alpha,Beta):-
 PosCorr is Pos_0+1, %Arrays are indexed from 0
 ith(PosCorr,Distances,DistL),
 ith(PosCorr,Pheromones,PheroL),
 calulate_ProbDenom(Clients_0,PheroL,DistL,Alpha,Beta,SUM_Ph_H),
 (foreach(J,Clients_0),fromto([],PIn,POut,Plist),param(_PosCorr,DistL,PheroL,SUM_Ph_H,Alpha,Beta,SUM_Ph_H) do
  Jcorr is J+1, %Arrays are indexed from 0
  ith(Jcorr,DistL,Dist_i_j),
  ith(Jcorr,PheroL,Phero_i_j),
  P_i_j is ((Phero_i_j^Alpha)*((1/Dist_i_j)^Beta))/SUM_Ph_H,
  append(PIn,[P_i_j],POut)
 ),
 random(N), 
 NProb is N/(2^31-1), 
 nextMovement(NProb,Plist,1,Chosen),
 ith(Chosen,Clients_0,Next).

%move(+Partial_sol_0,-Partial_sol_1,+Clients_0,-Clients_1,+Next)
move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Next):-
 append(Partial_sol_0,[Next],Partial_sol_1),
 delete_meu(Next,Clients_0,Clients_1).

%calulate_ProbDenom(+Clients_0,+PheroL,+DistL,+Alpha,+Beta,-SUM_Ph_H) calculates the denom. of the P_i_j^k formula
calulate_ProbDenom(Clients_0,PheroL,DistL,Alpha,Beta,SUM_Ph_H):-
 (foreach(Neighbour,Clients_0),fromto(0,SumIn,SumOut,SUM_Ph_H),param(Alpha,Beta,PheroL,DistL) do
   NCorr is Neighbour+1,
  ith(NCorr,PheroL,Ph_l),
  ith(NCorr,DistL,D_l),
  SumOut is SumIn + (Ph_l^Alpha)*((1/D_l)^Beta)
 ).

%nextMovement(+NProb,+Plist,+ChosenIn,-Chosen) 
nextMovement(NProb,[H|_],Chosen,Chosen):-
 (NProb =< H -> true).
nextMovement(NProb,[H|T],ChosenIn,Chosen):-
 (NProb>H -> NProb_1 is NProb-H,
             Chosen_1 is ChosenIn+1,
             nextMovement(NProb_1,T,Chosen_1,Chosen)
 ).

%update_trails(+Ro,+Ants_routes_LS,+Distances,+Pheromones,-PheromonesOut)
update_trails(_Ro,Ants_routes_LS,Distances,Pheromones,_PheromonesOut):-
 writeln(Pheromones),
 trails_quality(Ants_routes_LS,Distances,LQuality),
 best(LQuality,Best)
% TO DO: lower and add new pheromones
 .

%trails_quality(+Ants_routes_LS,+Distances,-LQuality) builds the list of qualities of the trails found
trails_quality(Ants_routes_LS,Distances,LQuality):-
 writeln(Distances),
 writeln(Ants_routes_LS),
 (foreach(Route,Ants_routes_LS),fromto([],LQualityIn,LQualityOut,LQuality),param(Distances) do
  append([First],Route_1,Route),
  (foreach(J,Route_1),fromto(First,I,Inext,_),fromto(0,QIn,QOut,RouteQ),param(Distances) do
   ICorr is I+1, %Arrays are indexed from 0
   JCorr is J+1, %Arrays are indexed from 0
   ith(ICorr,Distances,DistL),
   ith(JCorr,DistL,Dist_i_j),
   QOut is QIn+Dist_i_j,
   Inext is J
  ),
  append(LQualityIn,[RouteQ],LQualityOut)
 ),
 writeln(LQuality).


% ----- Auxiliay rules -----

 substr(X,Y,Z):-
    (foreach(Xi,X),fromto(Y,Yin,Yout,Z) do
        delete_meu(Xi,Yin,Yout)
    ).

%delete_meu(+El,+L,-L2) Removes element El from list L, returning L2
delete_meu(_,[],[]).
delete_meu(El,[H|T],X):-
    (El = H -> X = T;
               delete_meu(El,T,Y),
               append([H],Y,X)
    ).

%remove_ith(+N,+L,-El) List L without the Nth element 
remove_ith(N,L,L_N):-
N_1 is N-1,
length(L1,N_1),
append(L1,[_X|L2],L),
append(L1,L2,L_N).

%ith(+N,+L,-El) Returns El, which is the Nth element of list L 
ith(1,[H|_],H).
ith(N,[_|T],P):-
    ith(Naux,T,P),
    N is Naux+1.

%ith2(+N,+L,-El) Returns El, which is the Nth element of list L
%                In this case, L cannot contain unbounded variables
ith2(1,[H|_],H2):-nonvar(H),H=H2.
ith2(N,[_|T],P):-
    ith2(Naux,T,P),
    N is Naux+1.

% best(+LQuality,-Best) Returns the position of the best (lowest) element
%						in the list 
best(LQuality,Best):-
 Min is min(LQuality),
 ith(Best,LQuality,Min).