package org.bazs.randomDecision;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class RandomDecider<Decision extends Enum<Decision>>
{
   private final Class<Decision> _decisionClass;
   private final Map<Decision, Double> _probabilityTable;

   private static final Random RANDOM = new Random();

   RandomDecider(Class<Decision> decisionClass, Map<Decision, Double> probabilityTable)
   {
      _decisionClass = decisionClass;

      Double probabilitySum = probabilityTable.values().stream().reduce(0.0, Double::sum);
      if (probabilitySum.
      {
         throw new IllegalArgumentException("The probability map does not describe a full decision space.");
      }

      _probabilityTable = new HashMap<>();
      _probabilityTable.putAll(probabilityTable);
   }

}
