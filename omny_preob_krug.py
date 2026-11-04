import math




# робот по -y смотрит датчиком на объект
# едет по оси +x (колесо R крутится с положительным направлением)
# по +z - векторное представление поворота робота по часовой стрелке в радианах 
# error = расстояние_нужное - расстояние_текущее (т.е. положительна когда робот слишком близко)
# vector[y] = - k * error 

r=0.057/2*1.1*0.9  # радиус колёс, м
B=0.102           # раиус от центра платормы до точки касания, м

radius = 1 # радиус вращения, м
kU = 0.25 # отношение угловой скорости оси мотора (в рад/с) к напряжению. Сказывается на поправке при ненулевом угле и при задании скорости

SPEED = 0.5 # линейная скорость, в м/с - если измерен kU
KP = 1 * 0.01 # в (м/с)/см


def get_vector(error) -> tuple[float,float,float]:
    
    vector2D = (1, KP*error)
    l = (vector2D[0]**2 + vector2D[1]**2)**(1/2)
    vector2D = tuple(coord / l for coord in vector2D)
    vector = vector2D + (1/radius,)
    vector = tuple(coord * SPEED for coord in vector)

    return vector        



def popravkaNaNenulevoeVremya(vector:tuple[float,float,float], t=0) -> tuple[float,float,float]:
    # задай ненулевое время если необходимо и узнай kU - т.к. я его не замеряла - а здесь оно влияет не только на скорость, но и на геометрию
    # если время нулевое - функция возвращает сам вектор
    # рассчитано на положительную компоненту z


    povorot_robota = vector[2] * t * kU * r / B
    print(povorot_robota)
    pril = (math.pi - povorot_robota) / 2
    povorot_traectorii = math.pi/2 - pril
    print(povorot_traectorii)
    return ( vector[0]*math.cos(povorot_traectorii) - vector[1]*math.sin(povorot_traectorii) ,    vector[1]*math.cos(povorot_traectorii) + vector[0]*math.sin(povorot_traectorii)  , vector[2] )




def get_Us(vector:tuple[float,float,float]) -> tuple[float,float,float]:
    ugolR = -math.pi/3 
    ugolL = math.pi/3 
    ugolB = math.pi 
                
    
    # единичные вектора направлений движения колёс
    nR = (math.cos(ugolR), math.sin(ugolR))
    nL = (math.cos(ugolL), math.sin(ugolL))
    nB = (math.cos(ugolB), math.sin(ugolB))



    S = vector[2] * B    
    det = ( nR[0] * (nL[1] - nB[1])
        - nL[0] * (nR[1] - nB[1])
        + nB[0] * (nR[1] - nL[1]) )

    vR = ( vector[0] * (nL[1] - nB[1])
        - nL[0]     * (vector[1] - nB[1] * S)
        + nB[0]     * (vector[1] - nL[1] * S) ) / det

    vL = ( nR[0]     * (vector[1] - nB[1] * S)
        - vector[0] * (nR[1] - nB[1])
        + nB[0]     * (nR[1] * S - vector[1]) ) / det

    vB = ( nR[0]     * (nL[1] * S - vector[1])
        - nL[0]     * (nR[1] * S - vector[1])
        + vector[0] * (nR[1] - nL[1]) ) / det

    UR = max(-100,min(vR / r / kU,100))
    UL = max(-100,min(vL / r / kU,100))
    UB = max(-100,min(vB / r / kU,100))

    return (UR, UL, UB) 


print(get_Us(popravkaNaNenulevoeVremya(get_vector(5), t=1)))
