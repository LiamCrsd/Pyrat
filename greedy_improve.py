#Import
from collections import defaultdict
import random
import time
import heapq
import math
from collections import defaultdict
from queue import PriorityQueue

#Définition constantes et variables globales.
MOVE_DOWN = 'D'
MOVE_LEFT = 'L'
MOVE_RIGHT = 'R'
MOVE_UP = 'U'

time_liste = []
liste_mouvement = []
target = (0,0)
meta_graph = {}
liste_fromage = []

#Fonction utiles
def indice(liste,element):
    for i in range(len(liste)-1,-1,-1):
        if liste[i] == element:
            return i

def give_score(graph,current_vertex,neighbors):
    """Donne un score à chaque voisin d'un sommet d'un graphe

    Cette fonction donne un score à chaque sommet voisin du sommet concerné en fonction de sa distance

    Parameters
    ----------
    graph : Dictionnary
        graphe à étudier

    current_vertex : Couple of integer
        sommet du graphe à étudier

    neighbors : Liste of couple of integer
        liste des voisins dont on veut connaitre le score

    Returns
    -------
    Dictionnary
    	  dictionnaire ratachant à chaque sommet voisin son score
    """
    res = {}
    for neighbor in neighbors:
        try:
            res[neighbor] = graph[current_vertex][neighbor][0]
        except:
            res[neighbor] = graph[current_vertex][neighbor][0]
            print(current_vertex)
            print(neighbors)
            print(neighbor)
    return res

def greedy(graph,initial_vertex, vertices_to_visit):
    """Algorithme gourmand trassant le parcours du graphe pour récuperer tout les fromages du plus proche au plus proche

    Parameters
    ----------
    graph : Dictionnary
        meta graphe dont les sommets représentes les fromages et la position initiale

    initial_vertex : Couple of integer
            Sommet de départ

    vertices_to_visit :
        position des fromages à aller chercher

    Returns
    -------
    List of couple of integer
        Liste des positions à atteindre pour récuperer tout les fromages
    """
    res = [initial_vertex]
    while vertices_to_visit != []:
        scores = give_score(graph,res[-1],vertices_to_visit)
        minimum = math.inf,0
        for e in scores.keys():
            if scores[e] < minimum[0]:
                minimum = scores[e],e
        res.append(vertices_to_visit.pop(vertices_to_visit.index(minimum[1])))
    return res

def dijkstra(graphe:dict, start_vertex:tuple) -> (dict, dict):
    graphe = {**graphe}
    D = {e: math.inf for e in graphe.keys()}
    V = {}

    D[start_vertex] = 0
    V[start_vertex] = None

    pq = PriorityQueue()
    pq.put((0, start_vertex))

    while not pq.empty():
        dist, current_vertex = pq.get()
        try:
            liste = graphe[current_vertex].items()
            del graphe[current_vertex]
            for neighbor, distance in liste:

                old_cost : int = D[neighbor]
                new_cost : int = D[current_vertex] + distance

                if new_cost < old_cost:
                    pq.put((new_cost, neighbor))
                    V[neighbor] : tuple = current_vertex
                    D[neighbor] : int = new_cost
        except:
            pass
    return D, V

def create_FIFO():
    """
    Returns
    -------
    List
    """
    return []

def push_FIFO(structure,element):
    """
    Parameters
    ----------
    structure : FIFO

    element : *a
    """
    structure.append(element)

def pop_FIFO(structure):
    """
    Parameters
    ----------
    structure : FIFO

    Returns
    -------
    *a
    """
    return structure.pop(0)

def traversal(start_vertex,graph):
    """Trouve le chemin le plus court vers chaque sommet depuis un sommet initial sans prendre en compte le temp_poids

    Parameters
    ----------
    graph : Dictionnary
        Graphe étudié

    start_vertex : Couple of integer
        Sommet de départ

    Returns
    -------
    Couple of dictionnary
        res : arbre du parcours du graphe depuis le sommet de départ
        routing_table : dictionnaire attribuant à chaque sommet son parent
    """
    file = create_FIFO()#File des sommets en cours de visite
    push_FIFO(file,start_vertex)
    res = {}#Arbre
    routing_table = {}#Dictionnaire des parents
    while file != []:#On visite tout les sommets jusqu'en bas
        temp = pop_FIFO(file)
        temp_fils = graph[temp].keys()
        temp_new_fils = {}
        for fils in temp_fils:#On visite tout les voisins d'un sommet
            if fils not in res and fils not in file:#Si il ne sont pas déjà visité
                push_FIFO(file,fils)
                temp_new_fils[fils] = graph[temp][fils]
                routing_table[fils] = temp
        res[temp] = temp_new_fils
    return res,routing_table

def find_route(routing_table,source_location,target_location):
    """Renvoie le chemin a suivre pour aller entre deux positions

    Retourne le chemin le plus rapide selon la routing_table entre deux sommets

    Parameters
    ----------
    routing_table : Dictionnary
        dictionnaire des parents

    source_location : Couple of integer
        Sommet de départ

    target_location : Couple of integer
        Sommet d'arriver

    Returns
    -------
    List of couple of int
        Liste des positions à parcourir pour arriver à l'objectif [target_location,c,b,a]
    """
    route = []
    while source_location != target_location:
        route.append(target_location)
        target_location = routing_table[target_location]
    return route

def move_from_locations(source_location, target_location):
    """Faire correspondre a deux positions le mouvement a faire

    Parameters
    ----------
    source_location : Couple of integer
        Position 1

    target_location : Couple of integer
        Position 2

    Returns
    -------
    Char
        Mouvement à faire
    """
    difference = (target_location[0] - source_location[0], target_location[1] - source_location[1])
    if difference == (0, -1) :
        return MOVE_DOWN
    elif difference == (0, 1) :
        return MOVE_UP
    elif difference == (1, 0) :
        return MOVE_RIGHT
    elif difference == (-1, 0) :
        return MOVE_LEFT
    else :
        raise Exception("Impossible move")

def ajoute_sommet(meta_graph,position,liste,graphe):
    a,b = dijkstra(graphe,position)
    for i in range(len(liste)):
        temp_chemin = find_route(b,position,liste[i])
        temp_distance = a[liste[i]]
        if position in meta_graph:
            meta_graph[position][liste[i]] = temp_distance,temp_chemin
        else:
            meta_graph[position] = {liste[i]:(temp_distance,temp_chemin)}
    return meta_graph

def build_meta_graph(graphe,locations,pos_init):
    """Construit le meta graphe des fromages

    Parameters
    ----------
    graphe : Dictionnary
        graphe de départ

    locations : List of couple of integer
        liste des positions intéressantes

    pos_init : Couple of integer
        Position de départ

    Returns
    -------
    """
    locations = [pos_init] + locations
    n = len(locations)
    res = {}
    liste_temps = []
    for i in range(n):
        t1 = time.time()
        a,b = dijkstra(graphe,locations[i])
        liste_temps.append(time.time()-t1)
        for j in range(n):
            if j != i:
                temp_chemin = find_route(b,locations[i],locations[j])
                temp_distance = a[locations[j]]
                if locations[i] in res:
                    res[locations[i]][locations[j]] = temp_distance,temp_chemin
                else:
                    res[locations[i]] = {locations[j]:(temp_distance,temp_chemin)}
    #print("temps dijkstra :",sum(liste_temps)/len(liste_temps))
    return res

def tsp(graph, pos_init):
    """Trouve les chemin le plus court pour visiter tout les sommets en testant toutes les possibilités intéressantes

    Parameters
    ----------
    graph : Dictionnary
        graphe dont les sommets sont à visiter

    pos_init : Couple of integer
        position de départ

    Returns
    -------
    List of couple of integer
        liste ordonnée des sommets à visiters
    """
    chemins_visites = [[pos_init]]
    liste = []
    for e in graph.keys():
        if e != pos_init:
            liste.append(e)
    global minimum
    minimum = math.inf,[]
    def creuser(chemin,poids):
        global minimum
        if len(chemin) == len(liste) + 1:
            if poids < minimum[0]:
                minimum = poids,chemin
        elif poids >= minimum[0]:
            pass
        else:
            for e in liste:
                if not e in chemin:
                    temp_chemin = chemin + [e]
                    temp_poids = poids + graph[e][chemin[-1]][0]
                    if not temp_chemin in chemins_visites:
                        creuser(temp_chemin,temp_poids)
    creuser([pos_init],0)
    return minimum

def tfl_en_chemin(chemin,graph):
    """Revoie la liste des positions à visiter à partir de la liste ordonnée des lieux à visiter

    Parameters
    ----------
    chemin : List of couple of integer
        liste ordonné des lieux à visiter

    graph : Dictionnary
        dictionnaire renvoyant le chemin réel entre deux positons voisines dans le méta graphe
    """
    global liste_mouvement
    liste_mouvement = []
    for i in range(1,len(chemin)):
        truc = graph[chemin[i-1]][chemin[i]][1]
        liste_mouvement = truc + liste_mouvement

#Fonction pyrat
def preprocessing (maze_map, maze_width, maze_height, player_location, opponent_location, pieces_of_cheese, time_allowed):
    global liste_mouvement
    global meta_graph
    t1 = time.time()
    meta_graph = build_meta_graph(maze_map,pieces_of_cheese,player_location)
    t2 = time.time()
    truc = greedy(meta_graph,player_location,pieces_of_cheese)
    t3 = time.time()
    global liste_fromage
    liste_fromage = truc
    tfl_en_chemin(truc,meta_graph)
    global target
    target = truc[1]
    t4 = time.time()
    #print("build_meta_graph :",t2 - t1)
    #print("greedy :",t3 - t2)
    #print("tfl_en_chemin :",t4 - t3)

def turn (maze_map, maze_width, maze_height, player_location, opponent_location, player_score, opponent_score, pieces_of_cheese, time_allowed) :
    global meta_graph
    global liste_mouvement
    global liste_fromage
    global target
    try:
        if player_location == target:
            target = liste_fromage[liste_fromage.index(target)+1]
        if target in pieces_of_cheese:
            try:
                mouv = liste_mouvement.pop()
                mouv_turn = move_from_locations(player_location,mouv)
            except:
                truc = greedy(meta_graph,player_location,pieces_of_cheese)
                liste_fromage = truc
                tfl_en_chemin(truc,meta_graph)
                target = truc[1]
                mouv = liste_mouvement.pop()
                mouv_turn = move_from_locations(player_location,mouv)
        else:
            if not player_location in meta_graph:
                meta_graph = ajoute_sommet(meta_graph,player_location,pieces_of_cheese,maze_map)
            truc = greedy(meta_graph,player_location,pieces_of_cheese)
            liste_fromage = truc
            tfl_en_chemin(truc,meta_graph)
            target = truc[1]
            mouv = liste_mouvement.pop()
            mouv_turn = move_from_locations(player_location,mouv)
    except:
        if not player_location in meta_graph:
            meta_graph = ajoute_sommet(meta_graph,player_location,pieces_of_cheese,maze_map)
        truc = greedy(meta_graph,player_location,pieces_of_cheese)
        liste_fromage = truc
        tfl_en_chemin(truc,meta_graph)
        target = truc[1]
        mouv = liste_mouvement.pop()
        mouv_turn = move_from_locations(player_location,mouv)
    return mouv_turn


#A Faire
"""
- Remplacer greedy par un systeme de densité

- Faire en sorte de privilégier le centre

- Ne prendre en compte que les 21 fromages gagnants

- Prédire la cible de l'adversaire

- Utiliser le temps des tours prédéterminés

- Trouver une implémentation supplémentaire
"""
