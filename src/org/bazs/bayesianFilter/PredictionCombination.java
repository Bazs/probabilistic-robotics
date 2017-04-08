package org.bazs.bayesianFilter;

public class PredictionCombination<State extends Enum<State>, Control extends Enum<Control>>
{
   private State _state_t;
   private Control _control;
   private State _state_tm1;

   public PredictionCombination(State state_t, Control control, State state_tm1)
   {
      _state_t = state_t;
      _control = control;
      _state_tm1 = state_tm1;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((_control == null) ? 0 : _control.hashCode());
      result = prime * result + ((_state_t == null) ? 0 : _state_t.hashCode());
      result = prime * result + ((_state_tm1 == null) ? 0 : _state_tm1.hashCode());
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
      PredictionCombination other = (PredictionCombination) obj;
      if (_control == null)
      {
         if (other._control != null)
         {
            return false;
         }
      }
      else if (!_control.equals(other._control))
      {
         return false;
      }
      if (_state_t == null)
      {
         if (other._state_t != null)
         {
            return false;
         }
      }
      else if (!_state_t.equals(other._state_t))
      {
         return false;
      }
      if (_state_tm1 == null)
      {
         if (other._state_tm1 != null)
         {
            return false;
         }
      }
      else if (!_state_tm1.equals(other._state_tm1))
      {
         return false;
      }
      return true;
   }

}
