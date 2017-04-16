package org.bazs.markovChain;

import java.util.AbstractMap.SimpleEntry;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;

public class MarkovChain<State extends Enum<State>>
{
   private final Map<State, RandomDecider<State>> _transitionTable;

   private State _currentState;

   @SuppressWarnings("unchecked")
   public MarkovChain(State initialState, Map<State, RandomDecider<State>> transitionTable)
   {
      if ((transitionTable.size() != EnumSet.allOf(initialState.getClass()).size()))
      {
         throw new IllegalArgumentException("The decision table is not complete.");
      }

      _currentState = initialState;
      _transitionTable = new HashMap<>();
      _transitionTable.putAll(transitionTable);
   }

   public State doTransition()
   {
      _currentState = _transitionTable.get(_currentState).nextDecision();
      return _currentState;
   }

   public void setState(State state)
   {
      _currentState = state;
   }

   /**
    * Calculates the stationary distribution of the Markov chain, if it has one.
    * <p>
    * Solves the linear equation {@code mu = mu * P}, where mu is the statinoary
    * distribution, and P is the transition table of the Markov chain, using the
    * additional constraint, that the sum of the elements of mu need to add up
    * to 1.
    * 
    * @return A Map between the States and their corresponding contribution to
    *         the stationary distribution.
    */
   public Map<State, Double> getStationaryDistribution()
   {
      List<List<Double>> probabilitiesPerState = _transitionTable.entrySet().stream()
            .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
            .map(e -> e.getValue().getDecisionProbabilities().entrySet().stream()
                  .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                  .map(entry -> entry.getValue())
                  .collect(Collectors.toList()))
            .collect(Collectors.toList());

      final int numStates = probabilitiesPerState.size();

      double[][] Pt = new double[numStates][];
      for (int row = 0; row < numStates; ++row)
      {
         Pt[row] = new double[numStates];
      }

      for (int row = 0; row < numStates; ++row)
      {
         List<Double> currRow = probabilitiesPerState.get(row);
         for (int col = 0; col < numStates; ++col)
         {
            // transpose the P matrix, so that the equation can take the form of
            // Pt * mut = mut
            Pt[col][row] = currRow.get(col);

            // Move the mut values into Pt, i.e subtract the 1 coefficient from
            // the main axis
            if (col == row)
            {
               Pt[col][row] -= 1.0;
            }
         }
      }

      // The additional constraint that sum(mut) = 1 can be written as an extra
      // row in the equation system, where all coefficients are 1, and the
      // expected result is also 1. This would make Pt non-square, which is not
      // accepted by jblas when getting exact solutions for linear equation
      // systems, so we move this row into the system by subtracting it from
      // the first row.
      for (int col = 0; col < numStates; ++col)
      {
         Pt[0][col] -= 1.0;
      }

      // Based on the above, the right side of the system is zeroes, except for
      // the first row, which is -1
      double[] b = new double[numStates];
      b[0] = -1.0;
      for (int coeffIdx = 1; coeffIdx < numStates; ++coeffIdx)
      {
         b[coeffIdx] = 0.0;
      }

      DoubleMatrix Amat = new DoubleMatrix(Pt);
      DoubleMatrix bMat = new DoubleMatrix(b.length, 1, b);

      DoubleMatrix stationaryDistributionMat = Solve.solve(Amat, bMat);

      double[] stationaryDistribution = stationaryDistributionMat.toArray();

      List<State> states = _transitionTable.keySet().stream()
            .sorted((k1, k2) -> k1.compareTo(k2))
            .collect(Collectors.toList());

      Map<State, Double> statDistMap = IntStream.range(0, states.size())
            .mapToObj(i -> new SimpleEntry<State, Double>(states.get(i), stationaryDistribution[i]))
            .collect(Collectors.toMap(e -> e.getKey(), e -> e.getValue()));

      return statDistMap;
   }
}
