package org.bazs.probRob.chapter2;

import static org.bazs.probRob.chapter2.WeatherSensor.Control.E_NO_CONTROL;
import static org.bazs.probRob.chapter2.WeatherSensor.Measurement.E_MEAS_CLOUDY;
import static org.bazs.probRob.chapter2.WeatherSensor.Measurement.E_MEAS_RAINY;
import static org.bazs.probRob.chapter2.WeatherSensor.Measurement.E_MEAS_SUNNY;
import static org.bazs.probRob.chapter2.WeatherSensor.State.E_CLOUDY;
import static org.bazs.probRob.chapter2.WeatherSensor.State.E_RAINY;
import static org.bazs.probRob.chapter2.WeatherSensor.State.E_SUNNY;

import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.bazs.bayesianFilter.DiscreteBayesianFilter;
import org.bazs.bayesianFilter.MeasurementCombination;
import org.bazs.bayesianFilter.PredictionCombination;

public class WeatherSensor
{
   private static final String NEWLINE = System.getProperty("line.separator");

   enum State
   {
      E_SUNNY, E_CLOUDY, E_RAINY
   }

   enum Measurement
   {
      E_MEAS_SUNNY, E_MEAS_CLOUDY, E_MEAS_RAINY
   }

   enum Control
   {
      E_NO_CONTROL
   }

   private static final Map<State, Double> INITIAL_BELIEFS;
   static
   {
      INITIAL_BELIEFS = new HashMap<>();
      INITIAL_BELIEFS.put(E_SUNNY, 1.0);
      INITIAL_BELIEFS.put(E_CLOUDY, 0.0);
      INITIAL_BELIEFS.put(E_RAINY, 0.0);
   }

   private static final Map<MeasurementCombination<Measurement, State>, Double> MEASUREMENT_PROBABILITIES;
   static
   {
      MEASUREMENT_PROBABILITIES = new HashMap<>();
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_SUNNY, E_SUNNY), 0.6);
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_SUNNY, E_CLOUDY), 0.3);
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_SUNNY, E_RAINY), 0.0);

      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_CLOUDY, E_SUNNY), 0.4);
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_CLOUDY, E_CLOUDY), 0.7);
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_CLOUDY, E_RAINY), 0.0);

      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_RAINY, E_SUNNY), 0.0);
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_RAINY, E_CLOUDY), 0.0);
      MEASUREMENT_PROBABILITIES.put(new MeasurementCombination<Measurement, State>(E_MEAS_RAINY, E_RAINY), 1.0);
   }

   private static final Map<PredictionCombination<State, Control>, Double> PREDICTION_PROBABILITIES;
   static
   {
      PREDICTION_PROBABILITIES = new HashMap<>();
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_SUNNY, E_NO_CONTROL, E_SUNNY), 0.8);
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_SUNNY, E_NO_CONTROL, E_CLOUDY), 0.4);
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_SUNNY, E_NO_CONTROL, E_RAINY), 0.2);

      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_CLOUDY, E_NO_CONTROL, E_SUNNY), 0.2);
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_CLOUDY, E_NO_CONTROL, E_CLOUDY), 0.4);
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_CLOUDY, E_NO_CONTROL, E_RAINY), 0.6);

      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_RAINY, E_NO_CONTROL, E_SUNNY), 0.);
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_RAINY, E_NO_CONTROL, E_CLOUDY), 0.2);
      PREDICTION_PROBABILITIES.put(new PredictionCombination<State, Control>(E_RAINY, E_NO_CONTROL, E_RAINY), 0.2);
   }

   public static void main(String[] args)
   {
      DiscreteBayesianFilter<State, Control, Measurement> filter = new DiscreteBayesianFilter<>(State.class,
            Control.class, Measurement.class);

      filter.setBeliefs(INITIAL_BELIEFS);

      filter.setMeasurementProbabilities(MEASUREMENT_PROBABILITIES);
      filter.setPredictionProbabilities(PREDICTION_PROBABILITIES);

      System.out.println("The fist day is " + E_SUNNY + " with full certainty in all of the following experiments.");

      // Answer to problem 3/a
      System.out.println("Forward propagation, plausible chain of measured weathers:" + NEWLINE);
      List<Measurement> measuredWeather = new ArrayList<>();
      measuredWeather.add(E_MEAS_CLOUDY);
      measuredWeather.add(E_MEAS_CLOUDY);
      measuredWeather.add(E_MEAS_RAINY);
      measuredWeather.add(E_MEAS_SUNNY);
      System.out.println(doAndPrintForwardIteration(filter, measuredWeather));

      // Answer to problem 3/b, with only forward propagation
      System.out.println(NEWLINE + "Forward propagation, implausible chain of measured weathers:" + NEWLINE);
      filter.setBeliefs(INITIAL_BELIEFS);
      measuredWeather.clear();
      measuredWeather.add(E_MEAS_SUNNY);
      measuredWeather.add(E_MEAS_SUNNY);
      measuredWeather.add(E_MEAS_RAINY);
      System.out.println(doAndPrintForwardIteration(filter, measuredWeather));

      System.out.println(NEWLINE + "Possible chains of weathers for an implausible chain of measurements:" + NEWLINE);
      filter.setBeliefs(INITIAL_BELIEFS);
      measuredWeather.clear();
      measuredWeather.add(E_MEAS_SUNNY);
      measuredWeather.add(E_MEAS_SUNNY);
      measuredWeather.add(E_MEAS_RAINY);

      // --- Calculate the belief for each day with knowing all future
      // measurements as well

      // Get the beliefs for each day by forward propagation incorporating the
      // measurements
      List<Map<State, Double>> beliefsPerDay = Stream.iterate(0, i -> i + 1)
            .peek(i -> filter.update(E_NO_CONTROL, measuredWeather.get(i)))
            .map(i ->
            {
               HashMap<State, Double> beliefs = new HashMap<>();
               beliefs.putAll(filter.getBeliefs());
               return beliefs;
            })
            .limit(measuredWeather.size())
            .collect(Collectors.toList());

      // Get all possible 3-long chains of days starting from a sunny day based
      // on the transition table only (not incorporating the measurements)
      List<List<State>> chains = getPossibleChains(E_SUNNY, PREDICTION_PROBABILITIES, measuredWeather.size());

      // Calculate the probability of each chain based on the beliefs we have
      // calculated incorporating the measurements
      Map<List<State>, Double> chainsWithProbabilities = new HashMap<>();
      for (List<State> chain : chains)
      {
         Double probabilityForChain = Stream.iterate(0, i -> i + 1)
               .map(i -> beliefsPerDay.get(i).get(chain.get(i)))
               .limit(beliefsPerDay.size())
               .reduce(1.0, (a, b) -> a * b);
         if (0.0 < probabilityForChain)
         {
            chainsWithProbabilities.put(chain, probabilityForChain);
         }
      }

      // Normalize the chain probabilities
      double chainProbSum = chainsWithProbabilities.values().stream()
            .reduce(0.0, Double::sum);
      for (Entry<List<State>, Double> chainWithProbability : chainsWithProbabilities.entrySet())
      {
         chainWithProbability.setValue(chainWithProbability.getValue() / chainProbSum);
      }

      // Collect the probabilities of each possible weather for each day, as
      // well as the most likely state for each day
      List<State> mostLikelyWeathers = new ArrayList<>();
      for (int dayIdx = 0; dayIdx < measuredWeather.size(); ++dayIdx)
      {
         final int currDay = dayIdx;
         Map<State, Double> statesAndProbabilitiesForDay = chainsWithProbabilities.entrySet().stream()
               .map(e -> new SimpleEntry<State, Double>(e.getKey().get(currDay), e.getValue()))
               .collect(Collectors.groupingBy(Entry::getKey))
               .entrySet().stream()
               .collect(Collectors.toMap(Entry::getKey, e -> e.getValue().stream()
                     .map(Entry::getValue).reduce(0.0, Double::sum)));

         mostLikelyWeathers.add(statesAndProbabilitiesForDay.entrySet().stream()
               .max((e1, e2) -> e1.getValue().compareTo(e2.getValue()))
               .get()
               .getKey());

         System.out.println("Day " + (currDay + 2) + ":" + NEWLINE + statesAndProbabilitiesForDay.entrySet().stream()
               .map(e -> e.getKey() + ": " + e.getValue())
               .collect(Collectors.joining(NEWLINE)));
      }

      // // Calculate the probability of the most likely chain happening in
      // // general, without having taken any measurements
      // filter.setBeliefs(INITIAL_BELIEFS);
      // Double probabilityOfMostLikelyChain = Stream.iterate(0, i -> i + 1)
      // .peek(i -> filter.update(E_NO_CONTROL, null))
      // .peek(i -> System.out.println("Day " + (i + 2) + ": most likely today:
      // " + mostLikelyWeathers.get(i)))
      // .map(i -> filter.getBeliefs().get(mostLikelyWeathers.get(i)))
      // .limit(mostLikelyWeathers.size())
      // .reduce(1.0, (a, b) -> a * b);

      // System.out.println("Probability of this most likely chain to happen in
      // general: " + probabilityOfMostLikelyChain);

   }

   private static String doAndPrintForwardIteration(DiscreteBayesianFilter<State, Control, Measurement> filter,
         List<Measurement> measuredWeather)
   {
      return Stream.iterate(0, i -> i + 1)
            .peek(i -> filter.update(E_NO_CONTROL, measuredWeather.get(i)))
            .map(i -> "Day " + (i + 2) + ":" + NEWLINE + filter)
            .limit(measuredWeather.size())
            .collect(Collectors.joining(NEWLINE));
   }

   private static List<List<State>> getPossibleChains(State currentState,
         Map<PredictionCombination<State, Control>, Double> transitionTable, int numOfDays)
   {
      List<List<State>> chains;
      Set<State> allStates = EnumSet.allOf((Class<State>) currentState.getClass());
      if (1 < numOfDays)
      {

         Set<State> possibleNextStates = allStates.stream()
               .filter(s -> transitionTable.get(new PredictionCombination<State, Control>(s, E_NO_CONTROL,
                     currentState)) > 0.0)
               .collect(Collectors.toSet());

         chains = new LinkedList<>();
         for (State possibleNextState : possibleNextStates)
         {
            chains.addAll(getPossibleChains(possibleNextState, transitionTable, numOfDays - 1).stream()
                  .peek(l -> l.add(0, possibleNextState))
                  .collect(Collectors.toList()));
         }
      }
      else if (1 == numOfDays)
      {
         Set<State> possibleNextStates = allStates.stream()
               .filter(s -> transitionTable.get(new PredictionCombination<State, Control>(s, E_NO_CONTROL,
                     currentState)) > 0.0)
               .collect(Collectors.toSet());

         chains = new ArrayList<>();
         for (State nextState : possibleNextStates)
         {
            List<State> chain = new LinkedList<>();
            chain.add(nextState);
            chains.add(chain);
         }
      }
      else
      {
         chains = new LinkedList<>();
      }

      return chains;
   }
}
