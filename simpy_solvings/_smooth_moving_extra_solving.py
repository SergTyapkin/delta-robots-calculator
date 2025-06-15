from sympy import symbols, sqrt, atan, cos, sin, tan, pi, Symbol, solve, Eq, simplify
import math

# SymPy prettified
tan30 = symbols('tan30')
tan60 = symbols('tan60')
sin30 = symbols('sin30')
cos120 = -1 / 2
cosMinus120 = -1 / 2
sin120 = symbols('sqrt(3)') / 2
sinMinus120 = symbols('sqrt(3)') / 2

# Numbered
# tan30 = tan(30 / 180 * pi)
# tan60 = tan(60 / 180 * pi)
# sin30 = sin(30 / 180 * pi)

if __name__ == '__main__':
    F0 = 1
    E0 = 0.3
    Lf0 = 0.7
    Le0 = 1.2

    x0 = 0
    y0 = 0
    z0 = -1  # должно быть < 0 (вниз, под базой)

    F, E, LF, LE = symbols('F, E, Lf, Le')
    x, y, z = symbols('x, y, z')


    def calc(T, S, Vmax, Amax, j):
        t = Symbol('t')
        t1, t2, t3, t4, t5, t6 = symbols('t1, t2, t3, t4, t5, t6')

        varsForSolving = [t1, t2, t3, t4, t5, t6]
        if Vmax is None:
            Vmax = symbols('Vmax')
            varsForSolving.append(Vmax)
        if T is None:
            T = symbols('T')
            varsForSolving.append(T)
        if Amax is None:
            Amax = symbols('Amax')
            varsForSolving.append(Amax)
        if j is None:
            j = symbols('j')
            varsForSolving.append(j)

        # 1.	Увеличение ускорения до максимума;
        s1 = j * t**3 / 6
        ds1 = s1.diff(t)
        dds1 = ds1.diff(t)
        ddds1 = dds1.diff(t)

        # 2.	Набор скорости с макс. ускорением;
        s1t1 = s1.subs({'t': t1})
        tForS2 = t - t1
        s2 = s1t1 + ds1.subs({'t': t1}) * tForS2 + dds1.subs({'t': t1}) * tForS2**2 / 2
        ds2 = s2.diff(t)
        dds2 = ds2.diff(t)
        ddds2 = dds2.diff(t)

        # 3.	Достижение макс. скорости с плавным замедлением ускорения до 0;
        s2t2 = s2.subs({'t': t2})
        tForS3 = t - t2
        s3 = s2t2 + ds2.subs({'t': t2}) * tForS3 + dds2.subs({'t': t2}) * tForS3**2 / 2 - j * tForS3**3 / 6
        ds3 = s3.diff(t)
        dds3 = ds3.diff(t)
        ddds3 = dds3.diff(t)

        # 4.	Движение с макс. скоростью;
        s3t3 = s3.subs({'t': t3})
        tForS4 = t - t3
        s4 = s3t3 + ds3.subs({'t': t3}) * tForS4
        ds4 = s4.diff(t)
        dds4 = ds4.diff(t)
        ddds4 = dds4.diff(t)

        # 5.	Набор замедления до макс. замедления;
        s4t4 = s4.subs({'t': t4})
        tForS5 = t - t4
        s5 = s4t4 + ds4.subs({'t': t4}) * tForS5 - j * tForS5**3 / 6
        ds5 = s5.diff(t)
        dds5 = ds5.diff(t)
        ddds5 = dds5.diff(t)

        # 6.	Уменьшение скорости с макс. замедлением;
        s5t5 = s5.subs({'t': t5})
        tForS6 = t - t5
        s6 = s5t5 + ds5.subs({'t': t5}) * tForS6 - dds5.subs({'t': t5}) * tForS6**2 / 2
        ds6 = s6.diff(t)
        dds6 = ds6.diff(t)
        ddds6 = dds6.diff(t)

        # 7.	Уменьшение замедления и скорости до 0.
        s6t6 = s6.subs({'t': t6})
        tForS7 = t - t6
        s7 = s6t6 + ds6.subs({'t': t6}) * tForS7 - dds6.subs({'t': t6}) * tForS7**2 / 2 + j * tForS7**3 / 6
        ds7 = s7.diff(t)
        dds7 = ds7.diff(t)
        ddds7 = dds7.diff(t)

        print("\nℹ️ Paths:")
        print(simplify(s1), '\n', simplify(s2), '\n', simplify(s3), '\n', simplify(s4), '\n', simplify(s5), '\n', simplify(s6), '\n', simplify(s7))
        print("\nℹ️ First diffs (speeds):")
        print(ds1, '\n', ds2, '\n', ds3, '\n', ds4, '\n', ds5, '\n', ds6, '\n', ds7)
        print("\nℹ️ Second diffs (accelerations):")
        print(dds1, '\n', dds2, '\n', dds3, '\n', dds4, '\n', dds5, '\n', dds6, '\n', dds7)
        print("\nℹ️ Third diffs (j):")
        print(ddds1, '\n', ddds2, '\n', ddds3, '\n', ddds4, '\n', ddds5, '\n', ddds6, '\n', ddds7)

        checkStraightEqualities = [
            Eq(ds1.subs({'t': 0}), 0),
            Eq(ds1.subs({'t': t1}), ds2.subs({'t': t1})),
            Eq(ds2.subs({'t': t2}), ds3.subs({'t': t2})),
            Eq(ds3.subs({'t': t3}), ds4.subs({'t': t3})),
            Eq(ds4.subs({'t': t4}), ds5.subs({'t': t4})),
            Eq(ds5.subs({'t': t5}), ds6.subs({'t': t5})),
            Eq(ds6.subs({'t': t6}), ds7.subs({'t': t6})),

            # Eq(dds1.subs({'t': 0}), 0),
            # Eq(dds1.subs({'t': t1}), dds2.subs({'t': t1})),
            # Eq(dds2.subs({'t': t2}), dds3.subs({'t': t2})),
            # Eq(dds3.subs({'t': t3}), dds4.subs({'t': t3})),
            # Eq(dds4.subs({'t': t4}), dds5.subs({'t': t4})),
            # Eq(dds5.subs({'t': t5}), dds6.subs({'t': t5})),
            # Eq(dds6.subs({'t': t6}), dds7.subs({'t': t6})),
        ]

        print('\n')
        for eq in checkStraightEqualities:
            try:
                if bool(eq) is True:
                    continue
            except TypeError:
                pass
            print("❌ One of check equalities is not True!")
            print(eq)
            print("❗ Please check out your equalities")
            return
        print("✅ All checking equalities is OK")

        equalities = [
            Eq(ds7.subs({'t': T}), 0),
            Eq(s7.subs({'t': T}), S),
            Eq(ds4, Vmax),
            Eq(ds2, Amax),
        ]
        print("\nℹ️ EQS")
        for eq in equalities:
            print(eq)
        print("\n✅ Results")
        for solvings in solve(equalities, varsForSolving):
            for i in range(len(solvings)):
                print(varsForSolving[i], ':', solvings[i])
            print('---')


    T = Symbol('T')
    S = Symbol('S')
    a = Symbol('a')
    j = Symbol('j')
    Vmax = Symbol('Vmax')

    print("==== Fixed: T, S, j ====")
    calc(T, S, None, None, j)

    # print("==== Fixed: T, S, a, j ====")
    # calc(T, S, None, a, j)
    #
    # print("==== Fixed: S, Vmax, a, j ====")
    # calc(None, S, Vmax, a, j)
    #
    # print("==== Fixed: T, S, Vmax, j ====")
    # calc(T, S, Vmax, None, j)
    #
    # print("==== Fixed: T, S, Vmax, a ====")
    # calc(T, S, Vmax, a, None)
    #
    # print("==== Fixed: T, S ====")
    # calc(T, S, None, None, None)
