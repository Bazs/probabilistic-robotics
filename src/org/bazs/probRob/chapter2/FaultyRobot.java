package org.bazs.probRob.chapter2;

import java.util.HashMap;
import java.util.Map;

import org.bazs.bayesianFilter.DiscreteBayesianFilter;
import org.bazs.bayesianFilter.MeasurementCombination;

public class FaultyRobot
{
   enum Measurement
   {
      E_ABOVE_1_M, E_BELOW_1_M
   }

   enum State
   {
      E_NOT_FAULTY, E_FAULTY
   }

   enum Control
   {
      E_CONTROL;
   }

   private static final Map<State, Double> INITIAL_BELIEFS;
   static
   {
      INITIAL_BELIEFS = new HashMap<State, Double>();
      INITIAL_BELIEFS.put(State.E_NOT_FAULTY, 0.999);
      INITIAL_BELIEFS.put(State.E_FAULTY, 0.001);
   }

   private static final Map<MeasurementCombination<Measurement, State>, Double> MEASUREMENT_PROBABILITIES;
   static
   {
      MEASUREMENT_PROBABILITIES = new HashMap<MeasurementCombination<Measurement, State>, Double>();
      MEASUREMENT_PROBABILITIES.put(
            new MeasurementCombination<Measurement, State>(Measurement.E_ABOVE_1_M, State.E_NOT_FAULTY), 2.0 / 3.0);
      MEASUREMENT_PROBABILITIES.put(
            new MeasurementCombination<Measurement, State>(Measurement.E_BELOW_1_M, State.E_NOT_FAULTY), 1.0 / 3.0);
      MEASUREMENT_PROBABILITIES
            .put(new MeasurementCombination<Measurement, State>(Measurement.E_ABOVE_1_M, State.E_FAULTY), 0.0);
      MEASUREMENT_PROBABILITIES
            .put(new MeasurementCombination<Measurement, State>(Measurement.E_BELOW_1_M, State.E_FAULTY), 1.0);
   }

   public static void main(String[] args)
   {
      DiscreteBayesianFilter<State, Control, Measurement> filter = new DiscreteBayesianFilter<State, Control, Measurement>(
            State.class, Control.class, Measurement.class);

      filter.setBeliefs(INITIAL_BELIEFS);
      filter.setMeasurementProbabilities(MEASUREMENT_PROBABILITIES);

      for (int step = 0; step < 10; ++step)
      {
         System.out.println("Step " + step + System.getProperty("line.separator"));
         filter.update(null, Measurement.E_BELOW_1_M);
         System.out.println(filter);
      }
   }

}
