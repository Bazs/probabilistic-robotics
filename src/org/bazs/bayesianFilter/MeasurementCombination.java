package org.bazs.bayesianFilter;

public class MeasurementCombination<Measurement extends Enum<Measurement>, State extends Enum<State>>
{
   private Measurement _measurement;
   private State _state;

   public MeasurementCombination(Measurement measurement, State state)
   {
      _measurement = measurement;
      _state = state;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((_measurement == null) ? 0 : _measurement.hashCode());
      result = prime * result + ((_state == null) ? 0 : _state.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
      {
         return true;
      }
      if (obj == null)
      {
         return false;
      }
      if (getClass() != obj.getClass())
      {
         return false;
      }
      MeasurementCombination other = (MeasurementCombination) obj;
      if (_measurement == null)
      {
         if (other._measurement != null)
         {
            return false;
         }
      }
      else if (!_measurement.equals(other._measurement))
      {
         return false;
      }
      if (_state == null)
      {
         if (other._state != null)
         {
            return false;
         }
      }
      else if (!_state.equals(other._state))
      {
         return false;
      }
      return true;
   }
}
