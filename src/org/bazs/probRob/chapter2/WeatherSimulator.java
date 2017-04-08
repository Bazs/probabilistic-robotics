package org.bazs.probRob.chapter2;

import static org.bazs.probRob.chapter2.WeatherSimulator.Weather.E_CLOUDY;
import static org.bazs.probRob.chapter2.WeatherSimulator.Weather.E_RAINY;
import static org.bazs.probRob.chapter2.WeatherSimulator.Weather.E_SUNNY;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import org.bazs.markovChain.MarkovChain;
import org.bazs.markovChain.RandomDecider;

public class WeatherSimulator
{
   enum Weather
   {
      E_SUNNY, E_CLOUDY, E_RAINY
   }

   private static final Map<Weather, Double> FROM_SUNNY_TABLE;
   static
   {
      FROM_SUNNY_TABLE = new HashMap<>();
      FROM_SUNNY_TABLE.put(E_SUNNY, 0.8);
      FROM_SUNNY_TABLE.put(E_CLOUDY, 0.2);
      FROM_SUNNY_TABLE.put(E_RAINY, 0.0);

   }

   private static final Map<Weather, Double> FROM_CLOUDY_TABLE;
   static
   {
      FROM_CLOUDY_TABLE = new HashMap<>();
      FROM_CLOUDY_TABLE.put(E_SUNNY, 0.4);
      FROM_CLOUDY_TABLE.put(E_CLOUDY, 0.4);
      FROM_CLOUDY_TABLE.put(E_RAINY, 0.2);
   }

   private static final Map<Weather, Double> FROM_RAINY_TABLE;
   static
   {
      FROM_RAINY_TABLE = new HashMap<>();
      FROM_RAINY_TABLE.put(E_SUNNY, 0.2);
      FROM_RAINY_TABLE.put(E_CLOUDY, 0.6);
      FROM_RAINY_TABLE.put(E_RAINY, 0.2);
   }

   private final static RandomDecider<Weather> SUNNY_DECIDER = new RandomDecider<Weather>(FROM_SUNNY_TABLE);
   private final static RandomDecider<Weather> CLOUDY_DECIDER = new RandomDecider<Weather>(FROM_CLOUDY_TABLE);
   private final static RandomDecider<Weather> RAINY_DECIDER = new RandomDecider<Weather>(FROM_RAINY_TABLE);

   private static final Map<Weather, RandomDecider<Weather>> TRANSITION_TABLE;
   static
   {
      TRANSITION_TABLE = new HashMap<>();
      TRANSITION_TABLE.put(E_SUNNY, SUNNY_DECIDER);
      TRANSITION_TABLE.put(E_CLOUDY, CLOUDY_DECIDER);
      TRANSITION_TABLE.put(E_RAINY, RAINY_DECIDER);
   }

   public static void main(String[] args)
   {
      MarkovChain<Weather> weatherChain = new MarkovChain<Weather>(E_SUNNY, TRANSITION_TABLE);

      Map<Weather, Integer> occurrences = new HashMap<>();
      occurrences.put(E_SUNNY, 0);
      occurrences.put(E_CLOUDY, 0);
      occurrences.put(E_RAINY, 0);

      int numberOfDays = 100000;

      for (int dayIdx = 0; dayIdx < numberOfDays; ++dayIdx)
      {
         Weather newWeather = weatherChain.doTransition();
         occurrences.put(newWeather, occurrences.get(newWeather) + 1);
      }

      // @foff
      System.out.println(
            occurrences.entrySet().stream()
            .map(e -> e.getKey().toString() + ": " + e.getValue() * 100.0 / numberOfDays + "%")
            .collect(Collectors.joining(System.getProperty("line.separator"))));
      // @fon
   }
}
