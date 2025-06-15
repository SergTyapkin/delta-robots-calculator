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
    t1, t2, t3 = symbols('t1, t2, t3')


    def calc(T, S, Vmax, a):
        t = Symbol('t')
        t1, t2 = symbols('t1, t2')
        varsForSolving = [t1, t2]

        if Vmax is None:
            Vmax = symbols('Vmax')
            varsForSolving.append(Vmax)
        if T is None:
            T = symbols('T')
            varsForSolving.append(T)
        if a is None:
            a = symbols('a')
            varsForSolving.append(a)

        s1 = a * t * t / 2
        ds1 = s1.diff(t)
        dds1 = ds1.diff(t)

        s1t1 = s1.subs({'t': t1})
        tForS1 = t - t1
        s2 = s1t1 + ds1.subs({'t': t1}) * tForS1
        ds2 = s2.diff(t)
        dds2 = ds2.diff(t)

        s2t2 = s2.subs({'t': t2})
        tForS2 = t - t2
        s3 = s2t2 + ds2.subs({'t': t2}) * tForS2 - a * tForS2 * tForS2 / 2
        ds3 = s3.diff(t)
        dds3 = ds3.diff(t)

        print("\nℹ️ Paths:")
        print(simplify(s1), '\n', simplify(s2), '\n', simplify(s3))
        print("\nℹ️ First diffs (speeds):")
        print(ds1, '\n', ds2, '\n', ds3)
        print("\nℹ️ Second diffs (accelerations):")
        print(dds1, '\n', dds2, '\n', dds3)

        checkStraightEqualities = [
            Eq(ds1.subs({'t': 0}), 0),
            Eq(ds1.subs({'t': t1}), ds2.subs({'t': t1})),
            Eq(ds2.subs({'t': t2}), ds3.subs({'t': t2})),
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
            Eq(ds3.subs({'t': T}), 0),
            Eq(s3.subs({'t': T}), S),
            Eq(ds2, Vmax),
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
    Vmax = Symbol('Vmax')

    print("==== Fixed: T, S, a ====")
    calc(T, S, None, a)

    print("==== Fixed: S, Vmax, a ====")
    calc(None, S, Vmax, a)

    print("==== Fixed: T, S, Vmax ====")
    calc(T, S, Vmax, None)

    # print("==== Fixed: T, S ====")
    # calc(T, S, None, None)
