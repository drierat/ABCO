:-lib(ic).

% abco\0 The Ant Borg Colony Optimisation algorithm
%
abco:-
 parameters(Max_sol,M,Alpha,Beta,Ro),										%ABCO parameters initalisation
 problem_instance(Clients,Demands,MaxCap,Distances,PheromonesIn),			%problem instance values
 (for(I,1,Max_sol),fromto(PheromonesIn,Pheromones,PheromonesOut,_),fromto([99999,[]],SolIn,SolOut,Sol),
  param(M,Alpha,Beta,Ro,Clients,Demands,MaxCap,Distances) do				%Loop Max_sol times
  	writeln(''),write('iteration '),writeln(I),								%TO REMOVE
  	construct_solutions(M,Alpha,Beta,Ants_routes,Clients,Demands,MaxCap,Distances,Pheromones),		%solutions construction phase
  	local_search(Ants_routes,Ants_routes_LS),								%TO DO local search phase
  	update_trails(Ro,Ants_routes_LS,Distances,Pheromones,PheromonesOut,SolIn,SolOut)	%pheromones update
 ),
 show_solution(Sol).

% parameters(-Max_sol,-M,-Alpha,-Beta,-Ro) 
% ABCO parameters initialisation 
% Should the parameters be read from a file or be in the query itself (?)
%
parameters(Max_sol,M,Alpha,Beta,Ro):-
 Max_sol is 25000,															%temination condition - maximum number of solutions
 M is 50,																	%number of ants building the solutions
 Alpha is 1,																%weight of the pheromone trail
 Beta is 2,																	%weight of the heuristic information
 Ro is 0.98.																	%pheromone trail persistence

% construct_solutions(+M,+Alpha,+Beta,-Ant_routes,+Clients) 
% Construction of the solutions for a set of M ants (one iteration)
%
construct_solutions(M,Alpha,Beta,Ants_routes,Clients,Demands,MaxCap,Distances,Pheromones):-
 length(Ants_routes,M),														%a list of M routes, each corresponding to an ant
 (foreach(Ant_k,Ants_routes),param(Alpha,Beta,Clients,Demands,MaxCap,Distances,Pheromones) do	%for each ant, her route is built
 	(fromto([0],Partial_sol_0,Partial_sol_1,Ant_k_0),fromto(MaxCap,Cap_0,Cap_1,_),
 	 fromto(Clients,Clients_0,Clients_1,[]),fromto(0,Pos_0,Next,_),
   	 param(Alpha,Beta,MaxCap,Demands,Distances,Pheromones) do				%the ant chooses her next step and moves
   	 	choose_next_movement(Pos_0,Clients_0,Partial_sol_0,Demands,Cap_0,Distances,Pheromones,Next,Alpha,Beta), 
    	move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Cap_0,Cap_1,MaxCap,Demands,Next)
    ),
    append(Ant_k_0,[0],Ant_k)												%return to depot ('0') is the final movement
 ).

local_search(Ants_routes,Ants_routes).										%TO DO (change the second to Ant_routes_LS)

%show_solution(Solution) shows the best solution found
%
show_solution([Cost|Sol]):-
writeln('---------------------------------'),
write('The cost of the best solution is '),writeln(Cost),
write('corresponding to '),writeln(Sol),
writeln('---------------------------------').

% choose_next_movement(+Pos_0,+Clients_0,_Partial_sol_0,+Demands,+Cap,+Distances,+Pheromones,-Next,+Alpha,+Beta)
% Next movement choice for an ant
choose_next_movement(Pos_0,Clients_0,_Partial_sol_0,Demands,Cap,Distances,Pheromones,Next,Alpha,Beta):-
 PosCorr is Pos_0+1,														%arrays are indexed from 1
 ith(PosCorr,Distances,DistL),!,
 ith(PosCorr,Pheromones,PheroL),!,
 remove_non_feasible(Clients_0,Clients_0Corr,Demands,Cap),					%removes clients which cannot be served due to lack of capacity TO REDO
 (Clients_0Corr = [] -> Next is 0;
 						calulate_ProbDenom(Clients_0Corr,PheroL,DistL,Alpha,Beta,SUM_Ph_H),	%calculus for the denominator of the probability formula
 						(foreach(J,Clients_0Corr),fromto([],PIn,POut,Plist),
 						param(DistL,PheroL,SUM_Ph_H,Alpha,Beta,SUM_Ph_H) do
 							Jcorr is J+1,									%arrays are indexed from 1
   							ith(Jcorr,DistL,Dist_i_j),!,
   							ith(Jcorr,PheroL,Phero_i_j),!,
   							P_i_j is ((Phero_i_j^Alpha)*((1/Dist_i_j)^Beta))/SUM_Ph_H,
   							append(PIn,[P_i_j],POut)
 						),
 						random(N),
 						NProb is N/(2^31-1), 
 						nextMovement(NProb,Plist,1,Chosen),
 						ith(Chosen,Clients_0Corr,Next),!
 ).

% move(+Partial_sol_0,-Partial_sol_1,+Clients_0,-Clients_1,+CapMax,+Demands,+Next)
% the ant moves to the choosen client. The movement is included in the solution and the client revomed from
% the list of clients to visit
%
move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Cap_0,Cap_1,CapMax,Demands,Next):-
 append(Partial_sol_0,[Next],Partial_sol_1),
 (Next=0 -> Cap_1 is CapMax,
 			Clients_1 = Clients_0;
 			delete_meu(Next,Clients_0,Clients_1),
 			ith(Next,Demands,D),!,
 			Cap_1 is Cap_0-D
 ).

% remove_non_feasible(+Clients,-Clients_Corr,+Demands,+Cap)
% removes those clients which cannot be served due to lack of capacity constraint
% 
remove_non_feasible(Clients,Clients_Corr,Demands,Cap):-
 (foreach(J,Clients),fromto([],ClIn,ClOut,Clients_Corr),param(Demands,Cap) do
   	ith(J,Demands,D_J),!,
 	(D_J>Cap -> ClOut = ClIn;
 				append(ClIn,[J],ClOut)
 	)
 ).

% calulate_ProbDenom(+Clients_0,+PheroL,+DistL,+Alpha,+Beta,-SUM_Ph_H) 
% calculates the denom. of the P_i_j^k formula
%
calulate_ProbDenom(Clients_0,PheroL,DistL,Alpha,Beta,SUM_Ph_H):-
 (foreach(Neighbour,Clients_0),fromto(0,SumIn,SumOut,SUM_Ph_H),param(Alpha,Beta,PheroL,DistL) do
  NCorr is Neighbour+1,														%arrays are indexed from 1
  ith(NCorr,PheroL,Ph_l),!,
  ith(NCorr,DistL,D_l),!,
  SumOut is SumIn + (Ph_l^Alpha)*((1/D_l)^Beta)
 ).

% nextMovement(+NProb,+Plist,+ChosenIn,-Chosen) 
% returns the next movement chosen by the ant
%
nextMovement(NProb,[H|_],Chosen,Chosen):-
 (NProb =< H -> true).
nextMovement(NProb,[H|T],ChosenIn,Chosen):-
 (NProb>H -> NProb_1 is NProb-H,
             Chosen_1 is ChosenIn+1,
             nextMovement(NProb_1,T,Chosen_1,Chosen)
 ).

% update_trails(+Ro,+Ants_routes_LS,+Distances,+Pheromones,-PheromonesOut,+solIn,-SolOut)
% once the best trail for a set of ants is chosen, the pheromones of all the paths are updated
% by applying evaporation and increment of the arcs in the best solution found
%
update_trails(Ro,Ants_routes_LS,Distances,Pheromones,PheromonesOut,SolIn,SolOut):-
 trails_quality(Ants_routes_LS,Distances,LQuality),						%trails quality calculation
 best(LQuality,Best,L),													%what's the best trail found?
 ith(Best,Ants_routes_LS,Ant_best),!,
 SolIn = [Cost|_],
 (L<Cost -> append([L],[Ant_best],SolOut),
 			Lbest is L;
 			SolOut = SolIn,
 			Lbest is Cost
 ),
 evaporation(Pheromones,Ro,Lbest,Pheromones_Ev),						%pheromones evaporation
 depositPh(Ant_best,L,Lbest,Ro,Pheromones_Ev,PheromonesOut).			%ant with best trail deposits pheromones

% trails_quality(+Ants_routes_LS,+Distances,-LQuality) 
% builds the list of qualities (depending on the cost function -> length) of the trails found
%
trails_quality(Ants_routes_LS,Distances,LQuality):-
 (foreach(Route,Ants_routes_LS),fromto([],LQualityIn,LQualityOut,LQuality),param(Distances) do
  	append([First],Route_1,Route),
  	(foreach(J,Route_1),fromto(First,I,Inext,_),fromto(0,QIn,QOut,RouteQ),param(Distances) do
   		ICorr is I+1,													%arrays are indexed from 1
   		JCorr is J+1,													%arrays are indexed from 1
   		ith(ICorr,Distances,DistL),!,
   		ith(JCorr,DistL,Dist_i_j),!,
   		QOut is QIn+Dist_i_j,
   		Inext is J
  	),
  	append(LQualityIn,[RouteQ],LQualityOut)
 ).

% evaporation(+Pheromones,+Ro,+Lbest,-Pheromones_Ev) 
% applies evaporation to pheromones trails
%
evaporation(Pheromones,Ro,Lbest,Pheromones_Ev):-
 length(Pheromones,L),length(Pheromones_Ev,L),
 L_1 is L-1,
 Pbest is 0.05,
 PhMAX is (1/(1-Ro))*(1/Lbest),
 Aux is Pbest^(1/L_1),													%TO DO Pass Pbest as a parameter (MMAS)
 PhMIN is (PhMAX*(1-Aux))/(L_1/2-1)*Aux,
 (foreach(PhI,Pheromones),foreach(PhI_Ev,Pheromones_Ev),param(Ro,L,PhMIN) do
  length(PhI_Ev,L),
  (foreach(PhIJ,PhI),foreach(PhIJ_Ev,PhI_Ev),param(Ro,PhMIN) do
   PhIJ_EvAux is PhIJ*Ro,
   (PhMIN>PhIJ_EvAux -> PhIJ_Ev is PhMIN;
   						PhIJ_Ev is PhIJ_EvAux)
  )
 ).

% depositPh(+Ant_best,+L,+Lbest,+Ro,+Pheromones_Ev,-PheromonesUpdated) 
% the ant with best trail deposits pheromones
%
depositPh(Ant_best,L,Lbest,Ro,Pheromones_Ev,PheromonesUpdated):-
 length(Pheromones_Ev,PhL),length(PheromonesUpdated,PhL),
 (foreach(PhU_I,PheromonesUpdated),param(PhL) do
  length(PhU_I,PhL)
 ),
 build_pairs_list(Ant_best,Arcs),
 PhMAX is (1/(1-Ro))*(1/Lbest),											%PhMAX is the max value for the pheromones in an arc (MMAS)
 (foreach([I,J],Arcs),param(L,Pheromones_Ev,PheromonesUpdated,PhMAX) do	%for each arc in the solution, pheromones are deposited
  ICorr is I+1,															%arrays are indexed from 0
  JCorr is J+1,															%arrays are indexed from 0
  ith(ICorr,Pheromones_Ev,PhI),!,ith(ICorr,PheromonesUpdated,PhUI),!,
  ith(JCorr,PhI,PhIJ),!,ith(JCorr,PhUI,PhUIJ),!,
  PhUIJaux is PhIJ+(1/L),													%the amount deposited is the inverse of the sol. distance
  (PhUIJaux>PhMAX -> PhUIJ is PhMAX;
  					 PhUIJ is PhUIJaux
  )
  %Uncomment next 3 lines if pheromones are considered symmetric
  %ith(JCorr,Pheromones_Ev,PhJ),ith(JCorr,PheromonesUpdated,PhUJ),
  %ith(ICorr,PhJ,PhJI),!,ith(ICorr,PhUJ,PhUJI),!,
  %PhUJI is PhJI+(1/L),
 ),
 length(Pheromones_Ev,N),
 (for(I2,1,N),param(N,Pheromones_Ev,PheromonesUpdated) do				%arcs out of the sol. are copied (remain) as they were
  (for(J2,1,N),param(I2,Pheromones_Ev,PheromonesUpdated) do
   ith(I2,PheromonesUpdated,PhUI),!,ith(J2,PhUI,PhUIJ),!,
   (ground(PhUIJ)->true;
   				  ith(I2,Pheromones_Ev,PhI),!,ith(J2,PhI,PhIJ),!,
   				  PhUIJ is PhIJ)
  )
 ).
 

% ----- Auxiliay rules -----

% problem_instance(-Clients,-Demands,-MaxCap,-Distances,-Pheromones) 
% Reads the intance to be solved
%
problem_instance(Clients,Demands,MaxCap,Distances,Pheromones):-
% Clients = [1,2,3,4,5],														%list of clients to serve
% Demands = [10,20,20,10,20],												%list of clients demands
% MaxCap = 70,																%maximum capacity for a truck
% Distances = [[0,3,5.099019514,2.236067977,4.472135955,4],					%distances matrix
%              [3,0,2.236067977,2.828427125,4.123105626,5],
%              [5.099019514,2.236067977,0,4.123105626,4.242640687,5.830951895],
%              [2.236067977,2.828427125,4.123105626,0,2.236067977,2.236067977],
%              [4.472135955,4.123105626,4.242640687,2.236067977,0,2],
%              [4,5,5.830951895,2.236067977,2,0]
%             ],
%initialisationPheromones(0.2,5,Pheromones).
%
Clients = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36],
Demands = [1,23,23,5,7,18,12,20,19,19,16,2,26,13,19,17,14,8,10,5,19,12,9,18,4,20,8,3,18,26,21,21,8,19,66,21],
MaxCap = 100,
Coords = ([[86,22],[29,17],[4,50],[25,13],[67,37],[13,7],[62,15],[84,38],[34,3],[19,45],[42,76],[40,86],[25,94],[63,57],[75,24],[61,85],[87,38],
[54,39],[66,34],[46,39],[47,17],[21,54],[19,83],[1,82],[94,28],[82,72],[41,59],[100,77],[1,57],[96,7],[57,82],[47,38],[68,89],[16,36],[51,38],[83,74],[84,2]]),
coords2dists(Coords,Distances),
initialisationPheromones(1,36,Pheromones).

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

%ithB(+N,+L,-El) Returns El, which is the Nth element of list L 
ithB(N,L,El):-
	N_1 is N-1,
	length(Laux,N_1),
	append(Laux,[El|_],L).

%ith2(+N,+L,-El) Returns El, which is the Nth element of list L
%                In this case, L cannot contain unbounded variables
ith2(1,[H|_],H2):-nonvar(H),H=H2.
ith2(N,[_|T],P):-
    ith2(Naux,T,P),
    N is Naux+1.

% best(+LQuality,-Best,-L) Returns the position and weigth of the best (lowest) element
%						   in the list 
best(LQuality,Best,L):-
 L is min(LQuality),
 write('The minimum cost in this iteration is '),writeln(L),
 ith(Best,LQuality,L),!.

%build_pairs_list(+L,-PL) Returns a list with the arcs extracted from a sorted nodes list
build_pairs_list([First|T],PL):-
 (foreach(J,T),fromto(First,I,Inext,_),fromto([],PLIn,PLOut,PL) do
   append(PLIn,[[I,J]],PLOut),
   Inext is J
 ).

%coords2dists(Coords+,Distances-) transforms coordinates of clients into a distances matrix
coords2dists(Coords,Distances):-
	(foreach([C1_i,C1_j],Coords),fromto([],DIn,DOut,Distances),param(Coords) do
 		(foreach([C2_i,C2_j],Coords),fromto([],D2In,D2Out,Dist1),param(C1_i,C1_j) do
 			Aux is (C2_i-C1_i)^2+(C2_j-C1_j)^2,
 			sqrt(Aux,D2),
 			append(D2In,[D2],D2Out)
 		),
 		append(DIn,[Dist1],DOut)
 	).

%initialisationPheromones(+Value,+ClientsNum,-Pheromones) initialises the pheromones matrix
initialisationPheromones(Value,ClientsNum,Pheromones):-
	Cl is ClientsNum+1, 													%include the depot
	(for(I,1,Cl),fromto([],PhIn,PhOut,Pheromones),param(Cl,Value) do
		(for(J,1,Cl),fromto([],Ph2In,Ph2Out,Ph2),param(I,Value) do
			(I=J -> append(Ph2In,[0],Ph2Out);
					append(Ph2In,[Value],Ph2Out)
			)
		),
		append(PhIn,[Ph2],PhOut)
	).
