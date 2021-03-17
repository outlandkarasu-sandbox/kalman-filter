import kalmand : KalmanFilter;

void main()
{
    estimateAndFiltering();
    //parameterEstimation();
}

void estimateAndFiltering()
{
    import std.stdio : writefln;

    alias KF = KalmanFilter!(real, real);
    alias Parameters = KF.Parameters;

    Parameters parameters = {
        drift: 0.0,
        tension: 1.0,
        cons: 0.0,
        measureVariance: 1.0,
        stateVariance: 4.0,
    };
    auto kalmanFilter = KF(parameters, 4.0, 12.0, 1.0);

    immutable(real)[] data = [4.4, 4.0, 3.5, 4.6];
    foreach (d; data)
    {
        auto ey = kalmanFilter.estimate(1.0);
        kalmanFilter.filtering(1.0, d);
        writefln("y: %s, ey: %s", d, ey);
    }
}

/+
void parameterEstimation()
{
    import std.stdio : writeln, writefln;

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
                errorVariance: exp(params[1]),
                stateVariance: exp(params[2]),
                lastState: 0.1,
            };
            KalmanFilter!T kalmanFilter = {
                model: model,
                variance: 100.0,
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

    Func f;

    real[] parameters = [0.5, 5.0, 3.0];
    parameters.writeln;

    StateSpaceModel!real model = {
        drift: 0.0,
        trend: 0.327438,// parameters[0],
        constant: 0.0,
        errorVariance: 4.155^^2.0, // log(parameters[1]),
        stateVariance: 5.901^^2.0, // log(parameters[2]),
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
        immutable ln = kalmanFilter.likelihood(1.0, d);
        kalmanFilter.filtering(1.0, d);
        immutable s = kalmanFilter.estimateState;
        immutable sv = kalmanFilter.variance;
        writefln("y: %s, es: %s, ev: %s, ey: %s, err: %s, v: %s, k: %s, s: %s, sv: %s, ln: %s",
            d, es, ev, ey, d - ey, v, k, s, sv, ln);
    }
}
+/
