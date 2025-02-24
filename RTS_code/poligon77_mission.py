import multiprocessing.process
import threading
import multiprocessing
import time
import math

from omegabot_poligon77 import *
try:
    from obstacles_map import *
except:
    from RTS_code.obstacles_map import *


def det_position_garant(bot):
    pos_x, pos_y = 0, 0
    while True: #Получаем текущую позицию
                pos = bot.get_local_position_lps()
                if pos != None:
                    pos_x = pos[0]
                    pos_y = pos[1]
                    break
    return pos_x, pos_y


def build_bot_path(bot, target_point_x, target_point_y, map = map_55x55):   #Построение матршрута для бота относительно текущего положения
    #Строим маршрут
    x, y = det_position_garant(bot)   #Получаем текущую позицию с ожидаением её гарантированного получения
    x = recalc_cords_to_points(x)   #Пересчитываем координаты в точки для дальнейшей работы с картой
    y = recalc_cords_to_points(y)   #Пересчитываем координаты в точки для дальнейшей работы с картой
    print("curr-"+str(x)+" "+str(y))
    way_to_target = omegapath.find_shortest_path(map, (x, y), (target_point_x, target_point_y))  #Расчитываем кратчайший путь с объездом препятствий
    if way_to_target == None:
        return None #Если маршрут найти не удалось, возвращаем NONE
    if(len(way_to_target) > 1):
        del way_to_target[0]    # удаляем первую точку маршрута т.к мы и так там
    return way_to_target

def move_bot_path(bot, way_to_target, target_name):   #Движение по точкам маршрута
    i = 0
    for path_points in way_to_target:   #Двигаемся по точкам маршрута
                 i += 1
                 to_x, to_y = recalc_points_to_cords(path_points[0]), recalc_points_to_cords(path_points[1])
                 bot.go_to_local_point(to_x, to_y)
                 print("move to (" + target_name + " path point=" + str(i) + ") -> x" + str(to_x) + " y" + str(to_y))
                 pos = bot.get_local_position_lps()
                 while math.sqrt(pow(abs(to_x - pos[0]),2) + pow(abs(to_y - pos[1]),2)) > 0.5:    #ждем пока расстояние между текущей позицией и заданной точокй будет меньше указанного
                    pos = bot.get_local_position_lps()

def path_stop_before(way_to_target, stop_before = 3):  #Убрать несколько точек маршрута для недоезда до объекта интереса
    if(len(way_to_target) > stop_before):
        for a in range(1, stop_before+1):  
            if(len(way_to_target) > 1):
                del way_to_target[-a]


def bot_process(bot_ip, bot_name, targets = {}, line_obstacles=[], medium_obstacles=[], large_obstacles=[], home_point=[], sklads = {}, vzaimosv = {}):
    # bot_name - название бота для которого ищются точки интереса
    # targets - словарь целей
    # пример tatrgets - {"target1" : [x1, y1]}
    # line_obstacles - [[0.2, 1, 1.6, 2],[x1, y1, x2, y2]]
    # meduim_obstacles - [[x, y], [2.4, 3.2]]
    # large_obstacles - [[x, y], [2.6, 1.2]]
    #Все координаты в метрах
    #bot_ip - ip адрес робота, например 10.1.100.42
    #sklads - словарь {"skald1" : [0.2, -0.4], "name" : [x, y]} id - название склада, координаты
    #vzaimosv - словарь {"sklad" : [3, 5, 0]} - название склада, ассоциированые метки


    #Заполнение карты препятствиями из аргументов
    for obstacles in medium_obstacles:
        draw_obstacles_med(recalc_cords_to_points(obstacles[0]), recalc_cords_to_points(obstacles[1]))
    for obstacles in large_obstacles:
        draw_obstacles_big(recalc_cords_to_points(obstacles[0]), recalc_cords_to_points(obstacles[1]))
    for lines in line_obstacles:
        field_draw_line(recalc_cords_to_points(lines[0]), recalc_cords_to_points(lines[1]), recalc_cords_to_points(lines[2]), recalc_cords_to_points(lines[3]))

    #Загружаем координаты складов и ассоциации
    omegabot_sklad = {}#Очистка
    omegabot_aruco_vzaimosv = {}    #Очистка
    for sklad in sklads.keys(): #Заполнение
        omegabot_sklad[sklad] = [recalc_cords_to_points(sklads[sklad][0]), recalc_cords_to_points(sklads[sklad][1])]
    omegabot_aruco_vzaimosv = vzaimosv

    omegabot_targets = {} #Чистим массив определенный в obstacles map
    omegabot_targets[bot_name + "_home"] = [recalc_cords_to_points(home_point[0]), recalc_cords_to_points(home_point[1])] # Передаем точку дома
    i = 0
    for targ in targets.keys():
        i += 1
        omegabot_targets[bot_name + "_" + targ + str(i)] = [recalc_cords_to_points(targets[targ][0]), recalc_cords_to_points(targets[targ][1])]    # Передаем целевые точки

    #чистим грузы от препятствий (на всякий случай)
    for clear in targets.keys():
         draw_obstacles_med(recalc_cords_to_points(targets[clear][0]), recalc_cords_to_points(targets[clear][1]), 0)

    draw_obstacles_med(recalc_cords_to_points(home_point[0]), recalc_cords_to_points(home_point[1]), 0)
    
    print(home_point)
    print(omegabot_targets)

    bot_omegabot= Robot(bot_ip)
    #semi_final_bot_task(omegabot_bot, bot_name, targets = {}, line_obstacles=[], medium_obstacles=[], large_obstacles=[], home_point=[], sklads = {}, vzaimosv = {})

    bot_name_task = bot_name  #Обозначение бота для выбора соответствубщего задания
    print(omegabot_targets)
    for targ_key in omegabot_targets.keys():
        if(targ_key[0:4] == bot_name_task): 
            pass
        else:
            print("skip " + targ_key)
            continue

        if targ_key[5:] == "home":
            continue

        #Выводим текущую итерацию 
        print("cycle - >" + targ_key)


        #Начинаем задание связанное с разбором грузов
        #Поочередно разбираем грузы

        #Едем к грузу из текущей точки
        #Строим маршрут
        way_to_target = build_bot_path(bot_omegabot.bot, omegabot_targets[targ_key][0], omegabot_targets[targ_key][1])
        if way_to_target == None:
            print("path error ->" + targ_key) #Если путь построить не получилось, то переходим к следующей цели (например пути может не быть)
            continue
        print(way_to_target)

        #После построения пути до цели нужно проехать по точкам
        #Так как у нас есть задача найти аруко, а едем мы прямо в точку с кубиком, до него мы не доезжаем 3 путевых точки и начинаем искать аруко

        #Движемся до маркера не доезжая 3 путевых точки
        path_stop_before(way_to_target) #Удаляес несколько последних точек из маршрута
        move_bot_path(bot_omegabot.bot, way_to_target, targ_key) #Движемся до заданной точки по маршруту
        
        target_sklad = -1 #Сохраняем тут наиболее значимую метку
        aruco_size_targ = 0 #Размер метки которую мы видим (для того чтобы выбрать наибольшую из них)
        #Начинаем поиск метки
        time.sleep(1)   #Ждем немного чтобы проверить не обнаружим ли мы целевую метку прямо в том положении в котором остановились
        aruco_found = False #Маркер об обнаружении аруко метки
        recog_aruco_markers = {} #Сюда будут помещены найденные маркеры
        for a in range(3): #Несколько раз ищем метку
            if bot_omegabot.cv.is_aruco_visible():
                #мы налши метку, можно решать что делать
                recog_aruco_markers = bot_omegabot.cv.get_aruco_markers()
                for arucas in recog_aruco_markers.keys():
                    aruco_found = True
                    target_sklad = int(arucas)
                    break
                break
            time.sleep(1)

        if not aruco_found:
            yaw = bot_omegabot.bot.get_attitude()
            while yaw == None:
                yaw = bot_omegabot.bot.get_attitude()
            for a in range(6):
                a = (a - 2)*1
                bot_omegabot.bot.rotate(yaw + a)
                time.sleep(1)
                if bot_omegabot.cv.is_aruco_visible(): #Если видим аруку
                    aruco_found = True
                    recog_aruco_markers = bot_omegabot.cv.get_aruco_markers()
                    for arucas in recog_aruco_markers.keys(): #Выбираем к какой метке мы ближе всего, если их несколько
                        if recog_aruco_markers[arucas][2] > aruco_size_targ: #Если размер метки в кадре больше чем размер последней определенной, выбираем текущую как целевую
                            target_sklad = int(arucas) #временно сохраняем id метки для дальнейшей замены названием склада
                            aruco_size_targ = recog_aruco_markers[arucas][2] #Записываем размер наибольшей метки в кадре

        #тут мы нашли метку

        #Тут можно обратиться к функциям detect и get_box/drop_box при необходимости
        #Помните, что в метод detect нужно передавать строку, например "Stone_1", а найденное на текущий момент значение - id, например "3"
        # Необходим ассоциативный механизм или другой метод для корректной передачи значения в функцию
        
        #Когда значимая метка найдена, ассоциируем её айди с целевым складом
        print(str(target_sklad) + " " + bot_name_task)
        for sklads in omegabot_aruco_vzaimosv.keys():
            if target_sklad in omegabot_aruco_vzaimosv[sklads]:
                target_sklad = sklads   #Если арука ассоциируется со складом, выбираем склад как целевой
                break
        
        if target_sklad not in omegabot_sklad.keys():
            print("Can not detect target sklad")
            continue


        else:   #Если нашли метку и знаем куда её нести
            pass #Тут можно обратиться к функциям detect и get_box/drop_box при необходимости


        #Строим маршрут до целевого слада
        sklad_x, sklad_y = omegabot_sklad[target_sklad][0], omegabot_sklad[target_sklad][1] #Получаем координаты склада на карте
        way_to_target = build_bot_path(bot_omegabot.bot, sklad_x, sklad_y)

        move_bot_path(bot_omegabot.bot, way_to_target, target_sklad)
        
        #Доехали до склада
        #Тут можно обратиться к функциям detect и get_box/drop_box при необходимости

        #на этом моменте будет перемещение до следующей цели или выход из цикла
    
    #Двигаемся домой
    #Строим маршрут до дома
    home_x, home_y = omegabot_targets[bot_name_task+"_home"][0], omegabot_targets[bot_name_task+"_home"][1] #Получаем координаты склада на карте
    way_to_target = build_bot_path(bot_omegabot.bot, home_x, home_y)

    move_bot_path(bot_omegabot.bot, way_to_target, "home")
    
    #Приехали домой

    pass