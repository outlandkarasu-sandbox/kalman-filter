import kalmand : KalmanFilter;

void main()
{
    //estimateAndFiltering();
    parameterEstimation();
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

void parameterEstimation()
{
    import std.stdio : writeln, writefln;

    alias KF = KalmanFilter!(real, real);
    alias Parameters = KF.Parameters;

    Parameters parameters = {
        drift: 0.0,
        tension: 0.327438,
        cons: 0.0,
        measureVariance: 4.155^^2.0,
        stateVariance: 5.901^^2.0,
    };
    auto kalmanFilter = KF(parameters, 0.1, 100.0, 1.0);

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

    foreach (d; data)
    {
        auto ey = kalmanFilter.estimate(1.0);
        kalmanFilter.filtering(1.0, d);
        writefln("y: %s, ey: %s", d, ey);
    }
}
