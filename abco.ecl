:-lib(ic).

% abco\0 The Ant Borg Colony Optimisation algorithm
abco:-
 parameters(Max_sol,M,Alpha,Beta,Ro,Clients), 					%parameters initalisation
 (for(I,1,Max_sol),param(M,Alpha,Beta,Ro,Clients) do			%Loop Max_sol times
       construct_solutions(M,Alpha,Beta,Ants_routes,Clients),	%solutions construction phase
       local_search(Ants_routes,Ants_routes_LS),				%local search phase
       update_trails(Ro,Ants_routes_LS),						%given the new soltions, update the pheromones
  writeln(I) %remove
 ),
 show_solution.

%parameters(-Max_sol,-M,-Alpha,-Beta,-Ro,-Clients) Rule for parameters initialisation
% Should the parameters be read from a file or be in the query itself (?)
parameters(Max_sol,M,Alpha,Beta,Ro,Clients):-
 Clients = [1,2,3,4,5],		%List of clients to serve
 Max_sol is 100,			%Temination condition - maximum number of solutions
 M is 10, 					%Number of ants building the solutions
 Alpha is 1, 				%Weight of the pheromone trail
 Beta is 1,					%Weight of the heuristic information
 Ro is 0.5.	 				%Pheromone trail persistence

%construct_solutions(+M,+Alpha,+Beta,-Ant_routes,+Clients) Construction of the solutions for a set of ants
construct_solutions(M,Alpha,Beta,Ant_routes,Clients):-
 length(Ants_routes,M), 										%A list of M routes, each corresponding to an ant
 (foreach(Ant_k,Ants_routes),param(Alpha,Beta,Clients) do 		%For each ant, her route is built
  (fromto([0],Partial_sol_0,Partial_sol_1,Ant_k),				
   fromto(Clients,Clients_0,Clients_1,[]),fromto(0,Pos_0,Pos_1,_) do
    choose_next_movement(Pos_0,Clients_0,Partial_sol_0,Next),	%The ant chooses her next step
    move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Next,Pos_1)	%The ant moves (update the values in the loop)
  ),
 writeln(Ant_k)
 ).

% INCOMPLETE RULES (below)

local_search(Ant_routes,Ant_routes_LS).	%TO DO
update_trails(Ro,Ant_routes_LS).		%TO DO
show_solution.							%TO DO

choose_next_movement(_Pos_0,Clients_0,_Partial_sol_0,Next):-
 length(Clients_0,L),
 random(N), Ancho is (2^31-1)/L, N1 is N/Ancho, ceiling(N1,Nextf), integer(Nextf,Next). % MODIFICAR

move(Partial_sol_0,Partial_sol_1,Clients_0,Clients_1,Next,Pos_1):-	% MODIFICAR
 ith(Next,Clients_0,Pos_1),
 append(Partial_sol_0,[Pos_1],Partial_sol_1),
 delete_meu(Pos_1,Clients_0,Clients_1).

% ---------------------------------------------

 substr(X,Y,Z):-
    (foreach(Xi,X),fromto(Y,Yin,Yout,Z) do
        delete_meu(Xi,Yin,Yout)
    ).

delete_meu(_,[],[]).
delete_meu(El,[H|T],X):-
    (El = H -> X = T;
               delete_meu(El,T,Y),
               append([H],Y,X)
    ).

ith(1,[H|_],H).
ith(N,[_|T],P):-
    ith(Naux,T,P),
    N is Naux+1.

ith2(1,[H|_],H2):-nonvar(H),H=H2.
ith2(N,[_|T],P):-
    ith2(Naux,T,P),
    N is Naux+1.