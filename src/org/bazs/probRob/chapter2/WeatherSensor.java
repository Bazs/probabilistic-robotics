package org.bazs.probRob.chapter2;

import static org.bazs.probRob.chapter2.WeatherSensor.Control.E_NO_CONTROL;
import static org.bazs.probRob.chapter2.WeatherSensor.Measurement.E_MEAS_CLOUDY;
import static org.bazs.probRob.chapter2.WeatherSensor.Measurement.E_MEAS_RAINY;
import static org.bazs.probRob.chapter2.WeatherSensor.Measurement.E_MEAS_SUNNY;
import static org.bazs.probRob.chapter2.WeatherSensor.State.E_CLOUDY;
import static org.bazs.probRob.chapter2.WeatherSensor.State.E_RAINY;
import static org.bazs.probRob.chapter2.WeatherSensor.State.E_SUNNY;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.bazs.bayesianFilter.DiscreteBayesianFilter;
import org.bazs.bayesianFilter.MeasurementCombination;
import org.bazs.bayesianFilter.PredictionCombination;

public class WeatherSensor
{
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

      List<Measurement> measuredWeather = new ArrayList<>();
      measuredWeather.add(E_MEAS_CLOUDY);
      measuredWeather.add(E_MEAS_CLOUDY);
      measuredWeather.add(E_MEAS_RAINY);
      measuredWeather.add(E_MEAS_SUNNY);

      System.out.println(filter + System.getProperty("line.separator"));

      for (Measurement measurement : measuredWeather)
      {
         filter.update(E_NO_CONTROL, measurement);
         System.out.println(filter + System.getProperty("line.separator"));
      }

   }
}
