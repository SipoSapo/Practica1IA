from SearchAlgorithm import *
from SubwayMap import *
from utils import *

if __name__=="__main__":
    ROOT_FOLDER = './CityInformation/Barcelona_City/'
    map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    map.add_connection(connections)

    infoVelocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    map.add_velocity(infoVelocity_clean)



    ### BELOW HERE YOU CAN CALL ANY FUNCTION THAT yoU HAVE PROGRAMED TO ANSWER THE QUESTIONS OF THE EXAM ###
    ### this code is just for you, you won't have to upload it after the exam ###

    # ==========================================================
    # CONFIGURACIÓ DE L'EXAMEN
    # ==========================================================
    ORIGEN = 21
    DESTI = 16
    PREF = 2  # 0:Parades, 1:Temps, 2:Distància, 3:Transbord

    # ALGORISME
    # 1: BFS | 2: DFS | 3: UCS | 4: A* | 5: Cost d'una llista fixa
    OPCIO = 4

    res = None

    # ==========================================================
    # EXECUCIÓ
    # ==========================================================
    if OPCIO == 1:
        res = breadth_first_search(ORIGEN, DESTI, map)

    elif OPCIO == 2:
        res = depth_first_search(ORIGEN, DESTI, map)

    elif OPCIO == 3:
        res = uniform_cost_search(ORIGEN, DESTI, map, PREF)

    elif OPCIO == 4:
        res = Astar(ORIGEN, DESTI, map, PREF)

    elif OPCIO == 5:
        # Posa aquí la llista que et doni l'enunciat
        ruta_fixa = [5, 4, 3, 2, 1, 7, 8]
        res = Path(ruta_fixa)
        calculate_cost([res], map, type_preference=PREF)

    # ==========================================================
    # RESULTATS
    # ==========================================================
    if res:
        print(f"\n--- RESULTATS OPCIÓ {OPCIO} ---")
        print(f"Ruta: {res.route}")
        print(f"Cost G: {res.g}")
        print(f"Heurística H: {res.h}")
        print(f"Valor F (G+H): {res.f}")
    else:
        print("\n[!] No s'ha trobat cap resultat.")

    #this is an example of how to call some of the functions that you have programed
    #example_path=uniform_cost_search(9, 3, map, 1)
    #print_list_of_path_with_cost([example_path])

    # ==========================================================
    # CÀLCUL DE COST TOTAL PER A RUTES FIXES (PREGUNTA 2)
    # ==========================================================
    ruta_examen = [5, 4, 3, 2, 1, 7, 8]
    cost_total_distancia = 0

    for i in range(len(ruta_examen) - 1):
        p_id = ruta_examen[i]
        c_id = ruta_examen[i + 1]

        # 1. Obtenim el temps del tram
        temps_tram = map.connections[p_id][c_id]

        # 2. Obtenim la línia de l'estació origen del tram
        linea = map.stations[p_id]['line']

        # 3. Obtenim la velocitat d'aquesta línia
        velocitat = map.velocity[linea]

        # 4. Distància = Temps * Velocitat
        distancia_tram = temps_tram * velocitat
        cost_total_distancia += distancia_tram

    print(f"RESULTAT FINAL PREGUNTA 2: {cost_total_distancia}")