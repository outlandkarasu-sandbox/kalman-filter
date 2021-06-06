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
    import std.array : appender;
    import std.stdio : writeln, writefln;
    import diffengine : Differentiable, constant, param, Parameter, zero, one, diffContext, evalContext;
    import karasux.linear_algebra : Matrix, inverseByLUDecomposition;

    alias KF = KalmanFilter!(const(Differentiable!real), const(Differentiable!real));
    alias Parameters = KF.Parameters;

    auto tension = param!real(0.327438);
    auto measureVariance = param!real(4.155^^2.0);
    auto stateVariance = param!real(5.901^^2.0); 
    auto zeroValue = zero!real();
    Parameters parameters = {
        drift: zeroValue,
        tension: tension,
        cons: zeroValue,
        measureVariance: measureVariance,
        stateVariance: stateVariance,
    };
    auto kalmanFilter = KF(
        parameters,
        constant!real(0.1),
        constant!real(100.0),
        constant!real(1.0));

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

    auto oneValue = one!real();
    foreach (d; data[0 .. 12])
    {
        auto ey = kalmanFilter.estimate(oneValue);
        kalmanFilter.filtering(oneValue, d.constant);
        writefln("y: %s, ey: %s", d, ey());
    }

    import std.algorithm : map;

    auto lf = kalmanFilter.likelyhood;
    auto variables = [tension, measureVariance, stateVariance];
    auto dVariables = appender!(const(Differentiable!real)[])();
    foreach (v; variables)
    {
        dVariables ~= lf.differentiate(v.diffContext);
    }

    auto hesseElements = appender!(const(Differentiable!real)[])();
    foreach (i, d; dVariables)
    {
        foreach (j; 0 .. i)
        {
            hesseElements ~= hesseElements[][j * variables.length + i];
        }

        foreach (j, v; variables[i .. $])
        {
            hesseElements ~= d.differentiate(v.diffContext);
        }
    }

    Matrix!(3, 3, real) hesseMatrix;
    auto currentParameters = Matrix!(3, 1, real).fromRows([
       [tension.value],
       [measureVariance.value],
       [stateVariance.value],
    ]);
    Matrix!(3, 1, real) score;
    Matrix!(3, 3, real) hesseMatrixInverse;
    Matrix!(3, 1, real) offset;

    foreach (n; 0 .. 20)
    {
        auto ec = evalContext!real();
        writefln("parameters: %s", currentParameters);
        writefln("likelihood: %s", ec.evaluate(lf));

        foreach (i, d; dVariables)
        {
            score[i, 0] = ec.evaluate(d);
        }

        foreach (i, e; hesseElements)
        {
            immutable row = i / variables.length;
            immutable column = i % variables.length;
            hesseMatrix[row, column] = ec.evaluate(e);
        }

        hesseMatrix.inverseByLUDecomposition(hesseMatrixInverse);
        offset.mul(hesseMatrixInverse, score);
        currentParameters -= offset;

        writefln("score: %s", score);
        writefln("hesse: %s", hesseMatrix);
        writefln("hesse^-1: %s", hesseMatrixInverse);
        writefln("offset: %s", offset);

        foreach (i; 0 .. currentParameters.columns)
        {
            variables[i].bind(currentParameters[i, 0]);
        }
    }
}

