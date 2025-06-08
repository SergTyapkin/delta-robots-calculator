from sympy import symbols, sqrt, atan, cos, sin, tan, pi, Symbol, solve, Eq
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


    def calc():
        a = Symbol('a')
        t = Symbol('t')
        # tmax = 10
        tmax = Symbol('tmax')
        # smax = 1
        smax = Symbol('smax')
        t1, t2 = symbols('t1, t2')
        vmax = Symbol('vmax')
        # vmax = a*t1

        s1 = a * t * t / 2
        ds1 = s1.diff(t)
        dds1 = ds1.diff(t)

        s1t1 = s1.subs({'t': t1})
        s2 = s1t1 + vmax * t
        ds2 = s2.diff(t)
        dds2 = ds2.diff(t)

        s2t2 = s2.subs({'t': t2})
        s3 = s2t2 + ds2.subs({'t': t2}) * t - a * t * t / 2
        ds3 = s3.diff(t)
        dds3 = ds3.diff(t)

        print("")
        print("First diffs:")
        print([ds1, ds2, ds3])
        print("Second diffs:")
        print([dds1, dds2, dds3])

        # print([
        #     Eq(ds1.subs({'t': 0}), 0),
        #     Eq(ds1.subs({'t': t1}), ds2.subs({'t': t1})), Eq(ds2.subs({'t': t2}), ds3.subs({'t': t2})),
        #     # Eq(ds3.subs({'t': tmax}), 0),
        #     Eq(s3.subs({'t': tmax}), smax),
        # ])

        # print(solve([
        #     Eq(ds1.subs({'t': 0}), 0),
        #     Eq(ds1.subs({'t': t1}), ds2.subs({'t': t1})), Eq(ds2.subs({'t': t2}), ds3.subs({'t': t2})),
        #     # Eq(ds3.subs({'t': tmax}), 0),
        #     Eq(s3.subs({'t': tmax}), smax),
        # ], [a, t1, t2]))

        print([
            Eq(ds1.subs({'t': 0}), 0),
            Eq(ds1.subs({'t': t1}), ds2.subs({'t': t1})),
            # Eq(ds3.subs({'t': tmax}), 0),
            Eq(s2.subs({'t': tmax}), smax),
        ])
        print(solve([
            Eq(ds1.subs({'t': 0}), 0),
            Eq(ds1.subs({'t': t1}), ds2.subs({'t': t1})),
            # Eq(ds3.subs({'t': tmax}), 0),
            Eq(s2.subs({'t': tmax}), smax),
        ], [a, t1]))

    calc()
