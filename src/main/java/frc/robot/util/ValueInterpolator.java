package frc.robot.util;

public class ValueInterpolator {

    private ValuePair[] m_pairs;

    public static class ValuePair {
        public final double first;
        public final double second;

        public ValuePair(double first, double second) {
            this.first = first;
            this.second = second;
        }
    }

    // Must pass in at least 2 pairs, and they must be in ascending order of the first value
    // The first value of the pair is the input, and the second is the output
    public ValueInterpolator(ValuePair... pairs) {
        if (pairs.length < 2) {
            throw new IllegalArgumentException("Must have at least 2 pairs");
        }
        m_pairs = pairs;
    }

    public double getInterpolatedValue(double input) {
        if (input < m_pairs[0].first) {
            double slope = (m_pairs[1].second - m_pairs[0].second) / (m_pairs[1].first - m_pairs[0].first);
            return m_pairs[0].second - (m_pairs[0].first - input) * slope;
        }
        for (int idx = 1; idx < m_pairs.length; idx++) {
            if (input < m_pairs[idx].first) {
                double slope = (m_pairs[idx].second - m_pairs[idx - 1].second) / (m_pairs[idx].first - m_pairs[idx - 1].first);
                return m_pairs[idx - 1].second + (input - m_pairs[idx - 1].first ) * slope;
            }
        }
        double slope = (m_pairs[m_pairs.length - 1].second - m_pairs[m_pairs.length - 2].second) / (m_pairs[m_pairs.length - 1].first - m_pairs[m_pairs.length - 2].first);
        return m_pairs[m_pairs.length - 1].second - (input - m_pairs[m_pairs.length - 1].first) * slope;
    }
}