# This file contains all the required routines to make an A* search algorithm.
#
__author__ = '1711491'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2023 - 2024
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    path_list = []

    # estació actual (última del camí)
    current_station = path.last

    # estacions adjacents (claus del diccionari)
    neighbours = map.connections[current_station].keys()

    for station in neighbours:
        # crear còpia del path
        new_path = Path(path.route.copy())

        # afegir nova estació
        new_path.add_route(station)

        path_list.append(new_path)

    return path_list


def remove_cycles(path_list):
    filtered_paths = []

    for path in path_list:
        # si l’última estació NO apareix abans al recorregut → no hi ha cicle
        if path.last not in path.route[:-1]:
            filtered_paths.append(path)

    return filtered_paths


def insert_depth_first_search(expand_paths, list_of_path):
    return expand_paths + list_of_path


def depth_first_search(origin_id, destination_id, map):
    paths = [Path([origin_id])]

    while paths:

        path = paths.pop(0)

        if path.last == destination_id:
            return path

        expanded = expand(path, map)
        expanded = remove_cycles(expanded)

        paths = insert_depth_first_search(expanded, paths)

    return None


def insert_breadth_first_search(expand_paths, list_of_path):
    # BFS: Els nous camins van al final (Cua/FIFO)
    return list_of_path + expand_paths


def breadth_first_search(origin_id, destination_id, map):
    paths = [Path([origin_id])]

    while paths:

        path = paths.pop(0)

        if path.last == destination_id:
            return path

        expanded = expand(path, map)
        expanded = remove_cycles(expanded)

        paths = insert_breadth_first_search(expanded, paths)

    return None


def calculate_cost(expand_paths, map, type_preference=0):
    for path in expand_paths:

        prev = path.route[-2]
        curr = path.route[-1]

        cost = 0

        # 0 → parades
        if type_preference == 0:
            cost = 1

        # 1 → temps
        elif type_preference == 1:
            cost = map.connections[prev][curr]

        # 2 → distància
        elif type_preference == 2:
            time = map.connections[prev][curr]
            velocity = map.velocity[map.stations[prev]['line']]
            cost = time * velocity

        # 3 → transbord
        elif type_preference == 3:
            if map.stations[prev]['line'] != map.stations[curr]['line']:
                cost = 1

        path.update_g(cost)

    return expand_paths


def insert_cost(expand_paths, list_of_path):
    all_paths = list_of_path + expand_paths
    all_paths.sort(key=lambda p: p.g)

    return all_paths


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    paths = [Path([origin_id])]

    while paths:

        path = paths.pop(0)

        if path.last == destination_id:
            return path

        expanded = expand(path, map)
        expanded = remove_cycles(expanded)
        expanded = calculate_cost(expanded, map, type_preference)

        paths = insert_cost(expanded, paths)

    return None

def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    for path in expand_paths:

        curr = path.last

        h = 0

        # 0 → parades
        if type_preference == 0:
            if curr == destination_id:
                h = 0
            else:
                h = 1

        # 1 → temps
        elif type_preference == 1:

            # Si el destí és -1 (destí final caminant)
            if destination_id == -1 or curr == -1:
                h = 0
            else:
                dist = euclidean_dist(
                    (map.stations[curr]['x'], map.stations[curr]['y']),
                    (map.stations[destination_id]['x'], map.stations[destination_id]['y'])
                )

                max_vel = max(map.velocity.values())
                h = dist / max_vel
        elif type_preference == 2:
            h = euclidean_dist(
                (map.stations[curr]['x'], map.stations[curr]['y']),
                (map.stations[destination_id]['x'], map.stations[destination_id]['y'])
            )

        # 3 → transbords
        elif type_preference == 3:
            h = 0 if map.stations[curr]['line'] == map.stations[destination_id]['line'] else 1

        path.update_h(h)

    return expand_paths


def update_f(expand_paths):
    for path in expand_paths:
        path.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    new_expanded = []

    for path in expand_paths:
        station = path.last
        cost = path.g

        # 1. Comprovem si el nou camí és millor que el que ja coneixíem
        if station not in visited_stations_cost or cost < visited_stations_cost[station]:
            visited_stations_cost[station] = cost
            new_expanded.append(path)

            # 2. AQUESTA ÉS LA CLAU: Eliminem camins antics que arribaven
            # a la mateixa estació amb un cost pitjor de la llista d'espera
            list_of_path = [p for p in list_of_path if p.last != station]

    return new_expanded, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    all_paths = list_of_path + expand_paths
    all_paths.sort(key=lambda p: p.f)

    return all_paths


def distance_to_stations(coord, map):
    distances = {}
    for station_id, station in map.stations.items():
        d = math.sqrt((coord[0] - station['x']) ** 2 + (coord[1] - station['y']) ** 2)
        distances[station_id] = d

    sorted_distances = dict(sorted(distances.items(), key=lambda item: item[1]))

    return sorted_distances


def Astar(origin_id, destination_id, map, type_preference=0):
    paths = [Path([origin_id])]
    visited_stations_cost = {origin_id: 0}

    while paths:

        path = paths.pop(0)

        if path.last == destination_id:
            return path

        expanded = expand(path, map)
        expanded = remove_cycles(expanded)

        expanded = calculate_cost(expanded, map, type_preference)
        expanded = calculate_heuristics(expanded, map, destination_id, type_preference)

        expanded = update_f(expanded)

        expanded, paths, visited_stations_cost = remove_redundant_paths(
            expanded, paths, visited_stations_cost
        )

        paths = insert_cost_f(expanded, paths)

    return None


def Astar_improved(origin_coords, dest_coords, map):
    # La velocitat caminant és 5
    WALKING_VELOCITY = 5

    # 1. Expand inicial: des del punt 0 (origen) podem anar a qualsevol estació
    # Calculem el temps de caminar fins a cada estació i inicialitzem els paths
    initial_paths = []
    for station_id, station in map.stations.items():
        dist = euclidean_dist(origin_coords, (station['x'], station['y']))
        time = dist / WALKING_VELOCITY

        p = Path([0, station_id])
        p.update_g(time)
        # L'heurística és el temps de caminar des de l'estació fins al destí
        dist_to_dest = euclidean_dist((station['x'], station['y']), dest_coords)
        p.update_h(dist_to_dest / WALKING_VELOCITY)
        p.update_f()
        initial_paths.append(p)

    # També considerem el cas de caminar directament de l'origen al destí
    dist_total = euclidean_dist(origin_coords, dest_coords)
    direct_path = Path([0, -1])
    direct_path.update_g(dist_total / WALKING_VELOCITY)
    direct_path.update_h(0)
    direct_path.update_f()
    initial_paths.append(direct_path)

    paths = initial_paths
    visited_stations_cost = {0: 0}  # Els costos s'aniran gestionant a l'expansió

    while paths:
        path = paths.pop(0)

        # Si l'última estació és -1, hem arribat al destí
        if path.last == -1:
            return path

        # Si és una estació de metro (no l'origen ni el destí), expandim
        if path.last > 0:
            # Expandir via metro
            expanded = expand(path, map)
            expanded = remove_cycles(expanded)
            expanded = calculate_cost(expanded, map, type_preference=1)  # Temps

            # Afegir opció de caminar fins al destí des de l'estació actual
            dist_to_dest = euclidean_dist((map.stations[path.last]['x'], map.stations[path.last]['y']), dest_coords)
            path_to_dest = Path(path.route + [-1])
            path_to_dest.update_g(path.g + (dist_to_dest / WALKING_VELOCITY))
            path_to_dest.update_h(0)
            path_to_dest.update_f()
            expanded.append(path_to_dest)

            # Heurística i actualització final per als nous camins
            expanded = calculate_heuristics(expanded, map, -1, type_preference=1)  # Temps
            expanded = update_f(expanded)

            paths = insert_cost_f(expanded, paths)

    return None