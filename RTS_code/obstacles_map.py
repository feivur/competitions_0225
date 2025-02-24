from omegabot_poligon77 import *

# Двумерный массив - карта
map_55x55 =  [ [0] * 55 for _ in range(55)]
#Размер ячейки примерно 18 см


def draw_obstacles_med(point_x, point_y, type = 1, map = map_55x55):   #Наносит на карту map, в 9 точек 3x3 point_x, point_y клетку типа type(по умолчанию 1, т.е преграда)
    for x in range(point_x - 1, point_x+2):
        for y in range(point_y - 1, point_y+2):
            if(x >= 0 and x <= 55 and y >= 0 and y <= 55):
                map[x][y] = type

def draw_obstacles_big(point_x, point_y, type = 1, map = map_55x55):   #Наносит на карту map, в 25 точек 5x5 point_x, point_y клетку типа type(по умолчанию 1, т.е преграда)
    for x in range(point_x - 2, point_x+3):
        for y in range(point_y - 2, point_y+3):
            if(x >= 0 and x <= 55 and y >= 0 and y <= 55):
                map[x][y] = type

def field_draw_line(x1, y1, x2, y2, matrix = map_55x55):
    # Получаем размеры матрицы
    height = len(matrix)
    width = len(matrix[0])
    
    
    # Алгоритм Брезенхема для рисования линии
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    # Направление движения по оси X
    sx = 1 if x1 < x2 else -1
    
    # Направление движения по оси Y
    sy = 1 if y1 < y2 else -1
    
    err = dx - dy
    
    while True:
        matrix[y1][x1] = 1  # Рисуем точку
        
        if x1 == x2 and y1 == y2:
            break
        
        e2 = 2 * err
        
        if e2 > -dy:
            err -= dy
            x1 += sx
            
        if e2 < dx:
            err += dx
            y1 += sy

#Тут назначены базовые постоянные препятствия для бота, склады и точки дома

name_l = "bot2" #Обяательно 4 символа
name_r = "bot1" #Обяательно 4 символа

lines = [ # Передача линейных препятствий (для нанесения линии на карту робота нужно указать точку начала и конца)
            #Сюда можно запихнуть жд

        ]

obs_med = [ # Точки препятствий диаметром <= 0.54м
    #[x, y],
]

obs_large = [ # Точки препятствий диаметром < 0.9м
    #[x, y],
]

sklads_l = { #Координаты складов
    #"wood" : [x, y],
    #"stone" : [x, y],
}

sklads_r = { #Координаты складов
    #"wood" : [x, y],
    #"stone" : [x, y],
}

vzaimosv = {    #Информация о том, какие метки куда увозить (прописаны для правой и для левой стороны вместе)
    # "sklad name" : [id1, id2, id3, ...]
    "wood" : [0, 3],
    "stone" : [1, 4]
}

home_point_r = [0.00 , 0.00] #Точка дома прав, сюда бот вернется после выполнения миссии [x ,y]
home_point_l = [0.00 , 0.00] #Точка дома лев, сюда бот вернется после выполнения миссии [x, y]