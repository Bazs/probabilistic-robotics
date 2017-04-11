package org.bazs.markovChain;

import java.util.AbstractMap.SimpleEntry;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;

public class MarkovChain<State extends Enum<State>>
{
   private final Map<State, RandomDecider<State>> _transitionTable;

   private State _currentState;

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

   public Map<State, Double> getStationaryDistribution()
   {
      List<List<Double>> probabilitiesPerState = _transitionTable.entrySet().stream()
            .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
            .map(e -> e.getValue().getDecisionProbabilities().entrySet().stream()
                  .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                  .map(entry -> entry.getValue())
                  .collect(Collectors.toList()))
            .collect(Collectors.toList());

      List<List<Double>> pTranspose = IntStream.range(0, probabilitiesPerState.get(0).size())
            .mapToObj(i ->
            {
               return IntStream.range(0, probabilitiesPerState.size())
                     .mapToObj(j -> probabilitiesPerState.get(j).get(i))
                     .collect(Collectors.toList());

            })
            .collect(Collectors.toList());

      for (int row = 0; row < pTranspose.size(); ++row)
      {
         List<Double> currentRow = pTranspose.get(row);

         if (row == 0)
         {
            for (int col = 0; col < currentRow.size(); ++col)
            {
               currentRow.set(col, currentRow.get(col) - 1.0);
            }
         }

         currentRow.set(row, currentRow.get(row) - 1.0);
      }

      List<Double> B = DoubleStream.iterate(0.0, d -> d)
            .limit(probabilitiesPerState.size())
            .boxed()
            .collect(Collectors.toList());
      B.set(0, -1.0);

      double[] b = new double[B.size()];
      for (int idx = 0; idx < B.size(); ++idx)
      {
         b[idx] = B.get(idx);
      }

      double[][] A = new double[pTranspose.size()][];
      for (int row = 0; row < pTranspose.size(); ++row)
      {
         List<Double> currRow = pTranspose.get(row);
         A[row] = new double[currRow.size()];
         for (int col = 0; col < currRow.size(); ++col)
         {
            A[row][col] = currRow.get(col);
         }
      }

      DoubleMatrix Amat = new DoubleMatrix(A);
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
