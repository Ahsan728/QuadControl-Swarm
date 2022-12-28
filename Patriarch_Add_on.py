#!/usr/bin/env python3  
# -*- coding: utf-8 -*- 
# ----------------------------------------------------------------------------
# Created By  : Martin BOULIN
# Created for : LCIS Laboratory
# Version = '0.9.3 bis'
# ---------------------------------------------------------------------------
# Changelog :
#   Version 0.9.1 :
#       - Patriarch can now follow several bodies in the 6 DOF Euler mode
#       - It is now possible to receive data at a certain frequency
#   Version 0.9.2 :
#       - Improve overall performances (+ no more sleeping in the script)
#       - Now create a text file on the desktop if it does not exist (works for every computer). The user can choose the name of the file.
#       - Now all the data can be written as an argument when the script is called.
#       - Patriarch gives now the speed in addition to others parameters.
#   Version 0.9.3 :
#       - The speed calculus uses now Simpson's Rule, better accuracy expected.
# ---------------------------------------------------------------------------
# Import and variables

import socket

codec = 'ISO-8859-1'
error = 'strict'
# address_ip = '192.168.179.209'
address_ip = '172.20.10.7'

port_QTM = 22224
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket object

# ---------------------------------------------------------------------------
# Global variables, yeah that's bad but I don't care !

position_ant = [[0, 0, 0]]
position_ant_ant = [[0, 0, 0]]
vitesse_ant = [[0, 0, 0]]
temps1 = 0
compteur = 0
temps_init = 0


# ---------------------------------------------------------------------------
# Non-called functions

def ecriture_6DEuler(L, n_bodies, flag_speed):
    global temps1
    global compteur
    global temps_init
    L_fin = []

    if compteur == 0:
        getTimestamp()
        init_Var(n_bodies)
        temps1 = temps_init

    for j in range(n_bodies):
        L_temp = []
        index_list = 40 + j * 16
        for i in range(6):
            L_temp.append(bin_to_float(L, index_list, 4))
            index_list += 4
        L_fin.append(L_temp)

    dt = int.from_bytes(L[8:16], byteorder="big")
    dt -= temps1
    temps1 = int.from_bytes(L[8:16], byteorder="big")
    L_fin.append(temps1 - temps_init)

    if flag_speed == 1:
        for j in range(n_bodies):
            L_fin[j] += Euler(L_fin[j][0:3], j, dt)

    compteur += 1
    return L_fin


def init_Var(n_bodies):
    global vitesse_ant
    global position_ant
    global position_ant_ant

    vitesse_ant += (n_bodies - 1) * [[0, 0, 0]]
    position_ant += (n_bodies - 1) * [[0, 0, 0]]
    position_ant_ant += (n_bodies - 1) * [[0, 0, 0]]

    return True


def Euler(position, nb_body, dt):
    global position_ant
    vitesse = []
    for i in range(3):
        vitesse.append((position[i] - position_ant[nb_body][i]) * 1000 / dt)  # In m/s.
    position_ant[nb_body] = position[:3]
    return vitesse


def atoi(str):
    resultat = 0
    for i in range(len(str)):
        resultat = resultat * 10 + (ord(str[i]) - ord('0'))  # It is ASCII substraction
    return resultat


def bin_to_float(L, a, nb):
    bin_str = ""
    i = 0
    exp = 0
    frac = 1
    flt = 0
    for i in range(nb):
        bin_str += format(L[a + i], "08b")
    for i in range(1, 9):
        exp += atoi(bin_str[9 - i]) * 2 ** (i - 1)
    for i in range(9, 32):
        frac += atoi(bin_str[i]) * 2 ** (8 - i)
    flt = frac * 2 ** (exp - 127)
    if bin_str[0] == '1':
        flt *= -1
    return flt


def getInitialPosition(L):
    global position
    position = L


# ---------------------------------------------------------------------------
# Called functions

def connect_server(host=address_ip, port=port_QTM):
    s.connect((host, port))  # Connect to the server

    data = s.recv(1024)
    print(data[8:].decode(codec, error))

    # ------------------------------
    # Setting the version to 1.23 (highest version)
    # ------------------------------
    long = len("Version 1.23") + 9

    Mes_type = 1

    s.send(long.to_bytes(4, 'big'))
    s.send(Mes_type.to_bytes(4, 'big'))
    s.send("Version 1.23\0".encode())

    data = s.recv(1024)

    print(data[8:].decode(codec))

    getTimestamp()

    return (True)


## ---------------------------------------------------------------------------

def getTimestamp():
    global temps_init
    Mes = "GetCurrentFrame 6DEuler\0"

    len_mes = len(Mes) + 8
    Mes_type = 1

    s.send(len_mes.to_bytes(4, 'big'))
    s.send(Mes_type.to_bytes(4, 'big'))
    s.send(Mes.encode())

    data = s.recv(64)
    data_dec = data.decode(codec, error)
    L_int = [ord(c) for c in data_dec]
    if L_int[4:8] == [0, 0, 0, 0]:
        print(str(data_dec[2:]))
        exit()
    temps_init = int.from_bytes(L_int[8:16], byteorder="big")


## ---------------------------------------------------------------------------

def obtain_data(mod=0, Flag_Speed=0):
    # print("========== GetFrame 6D Euler ==========")

    Mes = "GetCurrentFrame 6DEuler\0"

    len_mes = len(Mes) + 8
    Mes_type = 1

    s.send(len_mes.to_bytes(4, 'big'))
    s.send(Mes_type.to_bytes(4, 'big'))
    s.send(Mes.encode())

    # ------------------------------
    # Beginning reception
    # ------------------------------

    data = s.recv(64)
    data_dec = data.decode(codec, error)
    L_int = [ord(c) for c in data_dec]
    if L_int[4:8] == [0, 0, 0, 0]:
        print(str(data_dec[2:]))
        print('Looks like there are some errors')
        exit()
    n_bodies = int.from_bytes(data[32:36], byteorder="big")
    # print(n_bodies)
    if n_bodies > 1:
        data = s.recv(24 * (n_bodies - 1))
        L_int += [ord(c) for c in data.decode(codec, error)]
    return ecriture_6DEuler(L_int, n_bodies, Flag_Speed)


## ---------------------------------------------------------------------------

def disconnect_server():
    s.close()  # close the connection
    print('\n========== Communication terminated ==========')
    return (True)
