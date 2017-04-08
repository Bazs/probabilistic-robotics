package org.bazs.bayesianFilter;

import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

public class DiscreteBayesianFilter<State extends Enum<State>, Control extends Enum<Control>, Measurement extends Enum<Measurement>>
{
   private final Class<State> _stateClass;
   private final Class<Control> _controlClass;
   private final Class<Measurement> _measurementClass;

   private final EnumSet<State> _allStates;
   private final EnumSet<Control> _allControls;
   private final EnumSet<Measurement> _allMeasurements;

   private final Map<PredictionCombination<State, Control>, Double> _predictionProbabilities;
   private final Map<MeasurementCombination<Measurement, State>, Double> _measurementProbabilities;

   private final Map<State, Double> _beliefs;

   private static final double DEFAULT_PRED_PROB = 0.5;
   private static final double DEFAULT_MEAS_PROB = 0.5;

   private static final double DEFAULT_BELIEF = 0.5;

   public DiscreteBayesianFilter(Class<State> stateClass, Class<Control> controlClass,
         Class<Measurement> measurementClass)
   {
      _stateClass = stateClass;
      _controlClass = controlClass;
      _measurementClass = measurementClass;

      _allStates = EnumSet.allOf(_stateClass);
      _allControls = EnumSet.allOf(_controlClass);
      _allMeasurements = EnumSet.allOf(_measurementClass);

      _predictionProbabilities = new HashMap<PredictionCombination<State, Control>, Double>();

      for (State state_t : _allStates)
      {
         for (State state_tm1 : _allStates)
         {
            for (Control control : _allControls)
            {
               _predictionProbabilities.put(new PredictionCombination<State, Control>(state_t, control, state_tm1),
                     DEFAULT_PRED_PROB);
            }
         }
      }

      _measurementProbabilities = new HashMap<MeasurementCombination<Measurement, State>, Double>();
      for (State state : _allStates)
      {
         for (Measurement measurement : _allMeasurements)
         {
            _measurementProbabilities.put(new MeasurementCombination<Measurement, State>(measurement, state),
                  DEFAULT_MEAS_PROB);
         }
      }

      _beliefs = new HashMap<State, Double>();
      for (State currState : EnumSet.allOf(_stateClass))
      {
         _beliefs.put(currState, DEFAULT_BELIEF);
      }
   }

   public void setBeliefs(Map<State, Double> beliefs)
   {
      _beliefs.putAll(beliefs);
   }

   public void setMeasurementProbabilities(
         Map<MeasurementCombination<Measurement, State>, Double> measurementProbabilities)
   {
      _measurementProbabilities.putAll(measurementProbabilities);
   }

   public void setPredictionProbabilities(Map<PredictionCombination<State, Control>, Double> predictionProbabilities)
   {
      _predictionProbabilities.putAll(predictionProbabilities);
   }

   public void update(Control control, Measurement measurement)
   {
      Map<State, Double> predictions = getPredictions(control);

      incorporateMeasurement(measurement, predictions);

      normalize();
   }

   private Map<State, Double> getPredictions(Control control)
   {
      Map<State, Double> predictions;

      if (null != control)
      {
         predictions = calculatePredictionsFromControl(control);
      }
      else
      {
         predictions = Collections.unmodifiableMap(_beliefs);
      }

      return predictions;
   }

   private void incorporateMeasurement(Measurement measurement, Map<State, Double> predictions)
   {
      if (null != measurement)
      {
         doMeasurementUpdate(measurement, predictions);
      }
      else
      {
         _beliefs.putAll(predictions);
      }
   }

   private Map<State, Double> calculatePredictionsFromControl(Control control)
   {
      Map<State, Double> predictions = new HashMap<State, Double>();

      for (State state_t : _allStates)
      {
         double prediction = 0.0;

         for (State state_tm1 : _allStates)
         {
            double predictionProbability = _predictionProbabilities
                  .get(new PredictionCombination<State, Control>(state_t, control, state_tm1));

            double belief_tm1 = _beliefs.get(state_tm1);

            prediction += predictionProbability * belief_tm1;
         }

         predictions.put(state_t, prediction);
      }

      return predictions;
   }

   private void doMeasurementUpdate(Measurement measurement, Map<State, Double> predictions)
   {
      for (State state : _allStates)
      {
         double measurementProbability = _measurementProbabilities
               .get(new MeasurementCombination<Measurement, State>(measurement, state));

         double prediction = predictions.get(state);

         _beliefs.put(state, measurementProbability * prediction);
      }
   }

   private void normalize()
   {
      double sum = _beliefs.values().stream().filter(Objects::nonNull).reduce(0.0, Double::sum);
      _beliefs.putAll(_beliefs.entrySet().stream().collect(Collectors.toMap(e -> e.getKey(), e -> e.getValue() / sum)));
   }

   @Override
   public String toString()
   {
      return _beliefs.entrySet().stream().map(e -> "Belief for state '" + e.getKey() + "':" + e.getValue())
            .collect(Collectors.joining(System.getProperty("line.separator")));
   }
}
