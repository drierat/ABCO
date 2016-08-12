:-lib(ic).

% abco\0 The Ant Borg Colony Optimisation algorithm
abco:-
 parameters(Max_sol,M,Alpha,Beta,Ro),							%parameters initalisation
 problem_instance(Clients,Demands,MaxCap,Distances),			%problem instance values
 (for(I,1,Max_sol),param(M,Alpha,Beta,Ro,Clients,Demands,MaxCap,Distances) do			%Loop Max_sol times
       construct_solutions(M,Alpha,Beta,Ants_routes,Clients,Demands,MaxCap,Distances),	%solutions construction phase
       local_search(Ants_routes,Ants_routes_LS),				%local search phase
       update_trails(Ro,Ants_routes_LS),						%given the new soltions, update the pheromones
  writeln(I) %remove
 ),
 show_solution.

%parameters(-Max_sol,-M,-Alpha,-Beta,-Ro) ABCO parameters initialisation
% Should the parameters be read from a file or be in the query itself (?)
parameters(Max_sol,M,Alpha,Beta,Ro):-
 Max_sol is 10,			%Temination condition - maximum number of solutions
 M is 10, 					%Number of ants building the solutions
 Alpha is 1, 				%Weight of the pheromone trail
 Beta is 1,					%Weight of the heuristic information
 Ro is 0.5.	 				%Pheromone trail persistence

%problem_instance(-Clients,-Demands,-MaxCap,-Distances) Gets the intance to be solved
problem_instance(Clients,Demands,MaxCap,Distances):-
 Clients = [1,2,3,4,5],															%List of clients to serve
 Demands = [10,10,10,10],														%List of clients demands
 MaxCap = 100,																	%Maximum capacity for a truck
 Distances = [[0,3,5.099019514,2.236067977,4.472135955,4],
              [3,0,2.236067977,2.828427125,4,123105626,5],
              [5.099019514,2.236067977,0,4.123105626,4.242640687,5.830951895],
              [2.236067977,2.828427125,4.123105626,0,2.236067977,2.236067977],
              [4.472135955,4,123105626,4.242640687,2.236067977,0,2],
              [4,5,5.830951895,2.236067977,2,0]
             ],
 Pheromones = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]].

%construct_solutions(+M,+Alpha,+Beta,-Ant_routes,+Clients) Construction of the solutions for a set of ants
construct_solutions(M,Alpha,Beta,Ant_routes,Clients,Demands,MaxCap,Distances):-
 length(Ants_routes,M), 										%A list of M routes, each corresponding to an ant
 (foreach(Ant_k,Ants_routes),param(Alpha,Beta,Clients,Demands,MaxCap,Distances) do 		%For each ant, her route is built
  (fromto([0],Partial_sol_0,Partial_sol_1,Ant_k),				
   fromto(Clients,Clients_0,Clients_1,[]),fromto(0,Pos_0,Next,_),param(Demands,MaxCap,Distances) do
    choose_next_movement(Pos_0,Clients_0,Partial_sol_0,Demands,MaxCap,Distances,Next),	%The ant chooses her next step
    move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Next)	%The ant moves (update the values in the loop)
  ),
 writeln(Ant_k)
 ).

% INCOMPLETE RULES (below)

local_search(Ant_routes,Ant_routes_LS).	%TO DO
update_trails(Ro,Ant_routes_LS).		%TO DO
show_solution.							%TO DO

choose_next_movement(Pos_0,Clients_0,_Partial_sol_0,Demands,MaxCap,Distances,Next):-
 length(Clients_0,L),
% random(N), Ancho is (2^31-1)/L, N1 is N/Ancho, ceiling(N1,Nextf), integer(Nextf,Next). % This is a test: The ant chooses the next movement randomly
%greedy version (chooses always the shortest path)
PosCorr is Pos_0+1, %Arrays are indexed from 0
ith(PosCorr,Distances,DistL),
(foreach(J,Clients_0),fromto(9999,Din,Dout,Dist_i_j),fromto(0,NextIn,NextOut,Next),param(DistL) do
 Jcorr is J+1, %Arrays are indexed from 0
 ith(Jcorr,DistL,Dist2check),
 (Dist2check<Din -> Dout is Dist2check, NextOut is J;
 					Dout is Din, NextOut is NextIn
 )
).

move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Next):-	% MODIFICAR
 append(Partial_sol_0,[Next],Partial_sol_1),
 delete_meu(Next,Clients_0,Clients_1).

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