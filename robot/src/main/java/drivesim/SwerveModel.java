package drivesim;

import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.exception.TooManyIterationsException;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresFactory;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.util.Pair;

import java.util.Arrays;

public class SwerveModel implements MultivariateJacobianFunction
{

    private static final double DIFF_DIST = 1e-8;

    private double wheelBaseLength, wheelBaseWidth, wheelBaseDiagonal;

    public SwerveModel(double wheelBaseLength, double wheelBaseWidth)
    {
        this.wheelBaseWidth = wheelBaseWidth;
        this.wheelBaseLength = wheelBaseLength;
        wheelBaseDiagonal = magnitude(wheelBaseWidth, wheelBaseLength);
    }

    public double[] getRobotVelocity(double... observationArr)
    {
        if (Arrays.stream(observationArr).skip(4).allMatch(d -> d == 0))
        {
            return new double[3];
        }
        for (int i = 0; i < 4; i++)
        {
            if (observationArr[i+4] < 0)
            {
                observationArr[i+4] = Math.abs(observationArr[i+4]);
                observationArr[i] += 180;
            }
            if (observationArr[i] < -180 || observationArr[i] > 180)
            {
                double angle = ((observationArr[i] % 360) + 360) % 360;
                if (angle > 180)
                {
                    angle = angle - 360;
                }
                observationArr[i] = angle;
            }
        }
        RealVector observation = new ArrayRealVector(observationArr);
        RealVector guess = new ArrayRealVector(new double[] { 1, 1, 1 });
        LeastSquaresProblem leastSquaresProblem = LeastSquaresFactory
            .create(this, observation, guess, null, 2000, 800);
        LeastSquaresOptimizer optimizer = new LevenbergMarquardtOptimizer();
        try
        {
            LeastSquaresOptimizer.Optimum optimum = optimizer.optimize(leastSquaresProblem);
            System.out.printf("Optimized with cost %.3f! {nIters: %d, nEvals: %d}\n", optimum.getRMS(),
                optimum.getIterations(), optimum.getEvaluations());
            return optimum.getPoint().toArray();
        }
        catch (TooManyEvaluationsException | TooManyIterationsException e)
        {
            e.printStackTrace();
            return new double[3];
        }
    }

    public double[] inverseKinematicsArr(double x, double y, double rotation)
    {
        double a = x - (rotation * wheelBaseLength / wheelBaseDiagonal);
        double b = x + (rotation * wheelBaseLength / wheelBaseDiagonal);
        double c = y - (rotation * wheelBaseWidth / wheelBaseDiagonal);
        double d = y + (rotation * wheelBaseWidth / wheelBaseDiagonal);

        // The white paper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        // Note: atan2(y, x) in java will take care of x being zero.
        //       If will return pi/2 for positive y and -pi/2 for negative y.
        double lfAngle = Math.toDegrees(Math.atan2(b, d));
        double rfAngle = Math.toDegrees(Math.atan2(b, c));
        double lrAngle = Math.toDegrees(Math.atan2(a, d));
        double rrAngle = Math.toDegrees(Math.atan2(a, c));

        // The white paper goes in order rf, lf, lr, rr. We like to do lf, rf, lr, rr.
        double lfPower = magnitude(b, d);
        double rfPower = magnitude(b, c);
        double lrPower = magnitude(a, d);
        double rrPower = magnitude(a, c);

        return new double[] { lfAngle, rfAngle, lrAngle, rrAngle, lfPower, rfPower, lrPower, rrPower };
    }

    public RealVector inverseKinematics(double x, double y, double rotation)
    {
        return new ArrayRealVector(inverseKinematicsArr(x, y, rotation));
    }

    public RealVector inverseKinematics(RealVector vector)
    {
        return inverseKinematics(vector.getEntry(0), vector.getEntry(1), vector.getEntry(2));
    }

    private double magnitude(double a, double b)
    {
        return Math.sqrt(a * a + b * b);
    }

    public Pair<RealVector, RealMatrix> value(RealVector point)
    {
        double x = point.getEntry(0);
        double y = point.getEntry(1);
        double rotation = point.getEntry(2);

        RealVector output = inverseKinematics(x, y, rotation);
        RealMatrix jacobianMatrix = computeApproximateJacobianMatrix(point);
        return new Pair<>(output, jacobianMatrix);
    }

    private RealMatrix computeApproximateJacobianMatrix(RealVector vector)
    {
        RealMatrix matrix = new Array2DRowRealMatrix(8, 3);
        for (int col = 0; col < 3; col++)
        {
            RealVector point1 = vector.copy();
            RealVector point2 = vector.copy();

            point1.addToEntry(col, -DIFF_DIST);
            point2.addToEntry(col, DIFF_DIST);

            RealVector col1 = inverseKinematics(point1);
            RealVector col2 = inverseKinematics(point2);

            RealVector difference = col2.subtract(col1);
            difference.mapDivideToSelf(2 * DIFF_DIST);
            matrix.setColumn(col, difference.toArray());
        }
        return matrix;
    }
}
