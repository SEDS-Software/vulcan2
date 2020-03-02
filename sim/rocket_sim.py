#!/usr/bin/env python

import math

def atmospheric_model(z):
    # from http://www.braeunig.us/space/atmmodel.htm

    # mean sea level radius of earth (m)
    ro = 6356.766

    # specific gas constant (J/kg-K)
    R = 287.053

    if z < 86:
        h = ro * z / (ro + z)
        if h < 11:
            T = 288.15 - 6.5*h
            P = 101325.0 * (288.15 / T) ** (34.1632 / -6.5)
        elif h < 20:
            T = 216.65
            P = 22632.06 * math.exp(-34.1632 * (h - 11) / T)
        elif h < 32:
            T = 196.65 + h
            P = 5474.889 * (216.65 / T) ** 34.1632
        elif h < 47:
            T = 139.05 + 2.8*h
            P = 868.0187 * (228.65 / T) ** (34.1632 / 2.8)
        elif h < 51:
            T = 270.65
            P = 110.9063 * math.exp(-34.1632 * (h - 47) / T)
        elif h < 71:
            T = 413.45 - 2.8*h
            P = 66.93887 * (270.65 / T) ** (34.1632 / -2.8)
        else:
            T = 365.65 - 2*h
            P = 3.956420 * (214.65 / T) ** (34.1632 / -2)
        rho = P / (R * T)
    else:
        if z < 91:
            T = 186.8673
            P   = math.exp( 2.159582e-06*z**3 + -4.836957e-04*z**2 +  -0.1425192*z +  13.47530)
            rho = math.exp(-3.322622e-06*z**3 +  9.111460e-04*z**2 +  -0.2609971*z +  5.944694)
        elif z < 110:
            T = 263.1905 - 76.3232 * math.sqrt(1 - ((z - 91) / -19.9429) ** 2)
            if z < 100:
                P   = math.exp( 3.304895e-05*z**3 +  -0.009062730*z**2 +   0.6516698*z + -11.03037)
                rho = math.exp( 2.873405e-05*z**3 +  -0.008492037*z**2 +   0.6541179*z + -23.62010)
            else:
                P   = math.exp( 6.693926e-05*z**3 +   -0.01945388*z**2 +    1.719080*z + -47.75030)
                rho = math.exp(-1.240774e-05*z**4 +   0.005162063*z**3 +    -0.8048342*z**2 +    55.55996*z + -1443.338)
        elif z < 120:
            T = 240 + 12 * (z - 110)
            P   = math.exp(-6.539316e-05*z**3 +    0.02485568*z**2 +   -3.223620*z +  135.9355)
            rho = math.exp(-8.854164e-05*z**3 +    0.03373254*z**2 +   -4.390837*z +  176.5294)
        else:
            T = 1000 - 640 * math.exp(-0.01875 * ((z - 120) * (6356.766 + 120) / (6356.766 + z)))
            if z < 150:
                P   = math.exp( 2.283506e-07*z**4 + -1.343221e-04*z**3 +    0.02999016*z**2 +   -3.055446*z +  113.5764)
                rho = math.exp( 3.661771e-07*z**4 + -2.154344e-04*z**3 +    0.04809214*z**2 +   -4.884744*z +  172.3597)
            elif z < 200:
                P   = math.exp( 1.209434e-08*z**4 + -9.692458e-06*z**3 +   0.003002041*z**2 +  -0.4523015*z +  19.19151)
                rho = math.exp( 1.906032e-08*z**4 + -1.527799e-05*z**3 +   0.004724294*z**2 +  -0.6992340*z +  20.50921)
            elif z < 300:
                P   = math.exp( 8.113942e-10*z**4 + -9.822568e-07*z**3 +  4.687616e-04*z**2 +  -0.1231710*z +  3.067409)
                rho = math.exp( 1.199282e-09*z**4 + -1.451051e-06*z**3 +  6.910474e-04*z**2 +  -0.1736220*z + -5.321644)
            elif z < 500:
                P   = math.exp( 9.814674e-11*z**4 + -1.654439e-07*z**3 +  1.148115e-04*z**2 + -0.05431334*z + -2.011365)
                rho = math.exp( 1.140564e-10*z**4 + -2.130756e-07*z**3 +  1.570762e-04*z**2 + -0.07029296*z + -12.89844)
            elif z < 750:
                P   = math.exp(-7.835161e-11*z**4 +  1.964589e-07*z**3 + -1.657213e-04*z**2 +  0.04305869*z + -14.77132)
                rho = math.exp( 8.105631e-12*z**4 + -2.358417e-09*z**3 +  2.635110e-06*z**2 + -0.01562608*z + -20.02246)
            else: #z < 1000:
                P   = math.exp( 2.813255e-11*z**4 + -1.120689e-07*z**3 +  1.695568e-04*z**2 +  -0.1188941*z +  14.56718)
                rho = math.exp(-3.701195e-12*z**4 + -8.608611e-09*z**3 +  5.118829e-05*z**2 + -0.06600998*z + -6.137674)

    return (T, P, rho)

def main():
    # time step (s)
    dt = 0.001

    # gravitational constant (m^3 kg^-1 s^-2)
    G = 6.67408e-11

    # standard gravity
    g0 = 9.80665

    # mass of earth (kg)
    Me = 5.9722e24

    # radius of earth (m)
    re = 6378000

    # specific gas constant (J/kg-K)
    R = 287.053

    # rocket parameters
    # diameter (m)
    D_rocket = 0.2
    # length (m)
    L_rocket = 6.7
    # area (m**2)
    A_rocket = 3.14159*(D_rocket/2)**2

    # mass of structures (kg)
    Ms = 65
    # mass of helium (kg)
    Mh = 0.2
    # mass of fuel (kg)
    Mf = 16.5
    # dry mass (kg)
    Md = Ms+Mh

    # engine parameters
    # specific impulse (s)
    Isp = 259
    # thrust (N)
    th = 850*4.44822

    # pad altitude (m)
    A_pad = 1973/3.28

    # pressure drag coefficient for slender rocket
    Cdf = 0.25

    # Slender body wave drag coefficient (from Sears-Haak Body wiki)
    Cdw = (9*(3.14159**2)*(D_rocket/2)**2)/(2*L_rocket**2)

    # parachutes
    # Cd for main chute
    Cd_main = 2.2
    # diameter of main chute (m)
    D_main = 6.06
    A_main = 3.14159*(D_main/2)**2

    # Cd for drogue chute
    Cd_drogue = 1.6
    # diameter of drogue chute (m)
    D_drogue = 1.07
    A_drogue = 3.14159*(D_drogue/2)**2

    main_deploy = False
    drogue_deploy = False

    ay = 0
    vy = 0
    py = A_pad

    g_l = []
    rho_l = []
    c_l = []
    mach_l = []
    Fd_l = []

    ay_l = []
    vy_l = []
    py_l = []

    burn_time = 0
    drogue_deploy_time = 0
    main_deploy_time = 0

    ts = 0

    while True:

        # acceleration of gravity
        g = G*Me/(re+py)**2

        # air density calculation
        T, P, rho = atmospheric_model(py/1000)

        # speed of sound
        c = math.sqrt(1.400 * R * T)

        # mach number
        mach = vy/c

        # drag force
        if main_deploy:
            Cd = Cd_main
            A = A_main
        elif drogue_deploy:
            Cd = Cd_drogue
            A = A_drogue
        elif mach >= 0.8:
            Cd = Cdf + Cdw
            A = A_rocket
        else:
            Cd = Cdf
            A = A_rocket

        Fd = 0.5 * rho * Cd * A * vy**2

        # mass
        mass = Md + Mf

        if Mf > 0:
            Ft = th
            burn_time = ts*dt
        else:
            Ft = 0

        # store values
        g_l.append(g)
        rho_l.append(rho)
        c_l.append(c)
        mach_l.append(mach)
        Fd_l.append(Fd)

        ay_l.append(ay)
        vy_l.append(vy)
        py_l.append(py)

        # take a step
        ts += 1

        ay = Ft/mass - g - math.copysign(Fd/mass, vy)
        vy += ay*dt
        py += vy*dt

        Mf = max(0, Mf - th/(Isp*g0)*dt)

        if ts % 100 == 0:
            print("t={:.1f}s    py={:.2f}    vy={:.2f}    ay={:.2f}".format(ts*dt, py, vy, ay))

        if not drogue_deploy and vy <= 0:
            drogue_deploy_time = ts*dt
            drogue_deploy = True

        if not main_deploy and vy < 0 and py <= A_pad+333:
            main_deploy_time = ts*dt
            main_deploy = True

        if py <= A_pad:
            break

    print("Burn time: {} s".format(burn_time))
    print("Drogue deploy time: {} s".format(drogue_deploy_time))
    print("Main deploy time: {} s".format(main_deploy_time))
    print("Total impulse: {} N-s ({} lb-s)".format(th*burn_time, th*burn_time/4.44822))
    print("Max velocity: {} m/s".format(max(vy_l)))
    print("Max mach number: {}".format(max(mach_l)))
    print("Max alt: {} m".format(max(py_l)))

    if th*burn_time/4.44822 > 9208:
        print("Warning: over 9208 lb-sec class limit")

if __name__ == '__main__':
    main()

