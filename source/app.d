import kalmand : StateSpaceModel, KalmanFilter;

void main()
{
    parameterEstimation();
}

void estimateAndFiltering()
{
    import std.stdio : writefln;

    StateSpaceModel!real model = {
        drift: 0.0,
        trend: 1.0,
        constant: 0.0,
        errorVariance: 1.0,
        stateVariance: 4.0,
        lastState: 4.0,
    };
    KalmanFilter!real kalmanFilter = {
        model: model,
        variance: 12.0,
    };

    immutable(real)[] data = [4.4, 4.0, 3.5, 4.6];
    foreach (d; data)
    {
        immutable es = kalmanFilter.estimateState;
        immutable ev = kalmanFilter.estimateVariance;
        immutable ey = kalmanFilter.estimateMeasurement(1.0);
        immutable v = kalmanFilter.estimateErrorVariance(1.0);
        immutable k = kalmanFilter.kalmanGain(1.0);
        kalmanFilter.filtering(1.0, d);
        immutable s = kalmanFilter.estimateState;
        immutable sv = kalmanFilter.variance;
        writefln("y: %s, es: %s, ev: %s, ey: %s, err: %s, v: %s, k: %s, s: %s, sv: %s",
            d, es, ev, ey, d - ey, v, k, s, sv);
    }
}

void parameterEstimation()
{
    import std.stdio : writeln, writefln;
    import lbfgsd.solver : SimpleSolver;

    immutable(real)[] data = [
        0.151,
        0.050,
        0.050,
        -0.352,
        -0.303,
        0.101,
        0.0,
        -0.202,
        -0.051,
        0.811,
        -0.805,
        0.254
    ];

    struct Func
    {
        T opCall(T)(in T[] params)
        {
            StateSpaceModel!T model = {
                drift: 0.0,
                trend: params[0],
                constant: 0.0,
                errorVariance: params[1],
                stateVariance: params[2],
                lastState: 0.1,
            };
            KalmanFilter!T kalmanFilter = {
                model: model,
                variance: 20.0,
            };

            auto logLikelihood = T(0.0);
            foreach (y; data)
            {
                logLikelihood += kalmanFilter.likelihood(T(1.0), T(y));
                kalmanFilter.filtering(T(1.0), T(y));
            }

            return logLikelihood;
        }
    }

    auto solver = new SimpleSolver!(real, 3);
    Func f;
    solver.setAutoDiffCost(f);

    real[] parameters = [0.5, 5.0, 3.0];
    solver.solve(parameters);
    parameters.writeln;

    StateSpaceModel!real model = {
        drift: 0.0,
        trend: 0.327438, //parameters[0],
        constant: 0.0,
        errorVariance: 4.155^^2, // parameters[1],
        stateVariance: 5.901^^2, // parameters[2],
        lastState: 0.1,
    };
    KalmanFilter!real kalmanFilter = {
        model: model,
        variance: 100.0,
    };

    foreach (d; data)
    {
        immutable es = kalmanFilter.estimateState;
        immutable ev = kalmanFilter.estimateVariance;
        immutable ey = kalmanFilter.estimateMeasurement(1.0);
        immutable v = kalmanFilter.estimateErrorVariance(1.0);
        immutable k = kalmanFilter.kalmanGain(1.0);
        kalmanFilter.filtering(1.0, d);
        immutable s = kalmanFilter.estimateState;
        immutable sv = kalmanFilter.variance;
        writefln("y: %s, es: %s, ev: %s, ey: %s, err: %s, v: %s, k: %s, s: %s, sv: %s",
            d, es, ev, ey, d - ey, v, k, s, sv);
    }
}

