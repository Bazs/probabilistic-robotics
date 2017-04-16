package org.bazs.markovChain;

import java.util.AbstractMap.SimpleEntry;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.atomic.DoubleAdder;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.bazs.mathUtils.MathUtils;

public class RandomDecider<Decision extends Enum<Decision>>
{
   private final List<Decision> _decisionsInAscendingProbability;
   private final List<Double> _decisionLowerLimits;

   private final Random _random;

   private static final double DOUBLE_TOLERANCE = 0.0001;

   public RandomDecider(Map<Decision, Double> decisionProbabilities)
   {
      _random = new Random();

      verifyDecisionProbabilities(decisionProbabilities);

      _decisionsInAscendingProbability = extractDecisionsInAscendingProbability(decisionProbabilities);
      _decisionLowerLimits = buildDecisionProbabilityCumulativeSums(decisionProbabilities,
            _decisionsInAscendingProbability);
   }

   private void verifyDecisionProbabilities(Map<Decision, Double> decisionProbabilities)
   {
      if (null == decisionProbabilities || decisionProbabilities.isEmpty())
      {
         throw new IllegalArgumentException("Empty or null decision probabilites.");
      }

      @SuppressWarnings("unchecked")
      Class<Decision> decisionClass = (Class<Decision>) decisionProbabilities.keySet().toArray()[0].getClass();

      if (decisionProbabilities.size() != EnumSet.allOf(decisionClass).size())
      {
         throw new IllegalArgumentException("The probability map does not describe a full decision space.");
      }

      Double probabilitySum = decisionProbabilities.values().stream().reduce(0.0, Double::sum);
      if (!MathUtils.doubleEquals(1.0, probabilitySum, DOUBLE_TOLERANCE))
      {
         throw new IllegalArgumentException("The probability map does not describe a full decision space.");
      }
   }

   private List<Decision> extractDecisionsInAscendingProbability(Map<Decision, Double> decisionProbabilities)
   {
      return decisionProbabilities.entrySet().stream()
            .sorted((e1, e2) -> Double.compare(e1.getValue(), e2.getValue()))
            .map(e -> e.getKey())
            .collect(Collectors.toList());
   }

   private List<Double> buildDecisionProbabilityCumulativeSums(Map<Decision, Double> decisionProbabilities,
         List<Decision> sortedDecisions)
   {
      List<Double> sortedProbabilities = IntStream.range(0, sortedDecisions.size())
            .mapToObj(i -> decisionProbabilities.get(sortedDecisions.get(i)))
            .collect(Collectors.toList());

      DoubleAdder adder = new DoubleAdder();
      return sortedProbabilities.stream()
            .map(p ->
            {
               adder.add(p);
               return adder.doubleValue();
            })
            .collect(Collectors.toList());
   }

   public Decision nextDecision()
   {
      double randomDouble = _random.nextDouble();

      int decisionIdx = IntStream.range(0, _decisionLowerLimits.size())
            .filter(i -> _decisionLowerLimits.get(i) > randomDouble)
            .findFirst()
            .getAsInt();

      return _decisionsInAscendingProbability.get(decisionIdx);
   }

   public Map<Decision, Double> getDecisionProbabilities()
   {
      Map<Decision, Double> decisionProbabilities = IntStream.range(1, _decisionsInAscendingProbability.size())
            .mapToObj(i -> new SimpleEntry<>(
                  _decisionsInAscendingProbability.get(i),
                  _decisionLowerLimits.get(i) - _decisionLowerLimits.get(i - 1)))
            .collect(Collectors.toMap(e -> e.getKey(), e -> e.getValue()));

      decisionProbabilities.put(_decisionsInAscendingProbability.get(0), _decisionLowerLimits.get(0));

      return decisionProbabilities;
   }
}
